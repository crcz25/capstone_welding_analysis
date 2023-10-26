#!/usr/bin/env python

import sys
sys.path.extend([".", ".."])

from pathlib import Path

import argparse
import base64
import iconreader
import json
import numpy as np
import copy


def get_range_json(component, range_image, pixel_size, origin, float_to_uint16):
    new_pixel_size = copy.deepcopy(pixel_size)
    new_origin = copy.deepcopy(origin)

    if float_to_uint16:
        # we want to discretize to use as much as possible of the value space
        # but we also need to handle missing data (0 -> 0) minZ -> 1, maxZ -> (2^16-1)
        # lets keep it simple
        min_z = component.traits.world_range_traits["lower bound r"]
        max_z = component.traits.world_range_traits["upper bound r"]
        data16 = (((range_image - min_z) / (max_z - min_z)) * (2 ** 16 - 2) + 1).astype(np.uint16)
        data16[range_image == 0.0] = 0

        # we also get new pixelsizes and origins I assume.
        # sizeZ has to be a uint16 step now
        new_pixel_size["z"] = (max_z - min_z) / (2 ** 16 - 2)
        new_origin["z"] = min_z

    height, width = range_image.shape

    bytes_per_pixel = range_image.dtype.itemsize

    encoded = base64.b64encode(range_image.tobytes())
    rawdata = encoded.decode('ascii')
    pixels = {
        "numOfElems": height * width,
        "elemSz": bytes_per_pixel,
        "endian": "little",
        "elemTypes": [str(range_image.dtype)],
        "data": rawdata
    }

    data = {
        "ImageType": str(range_image.dtype),
        "Width": width,
        "Height": height,
        "Pixels": pixels
    }

    json_dict = {
        "class": "Image",
        "data": {
            "PixelSize": new_pixel_size,
            "Origin": new_origin,
            "HandleZeroPixels": "MissingData",
            "Data": data
        }
    }

    return json_dict


def get_intensity_json(component, pixel_size):

    intensity_image = component.get_intensity()

    new_pixel_size = {}
    new_pixel_size["x"] = pixel_size["x"]
    new_pixel_size["y"] = pixel_size["y"]
    new_pixel_size["z"] = 1.0

    origin = {"x": 0.0, "y": 0.0, "z": 0.0}

    height, width = intensity_image.shape

    encoded = base64.b64encode(intensity_image.tobytes())
    rawdata = encoded.decode('ascii')
    pixels = {
        "numOfElems": height * width,
        "elemSz": 1,
        "endian": "neutral",
        "elemTypes": ["uint8"],
        "data": rawdata
    }

    data = {
        "ImageType": "uint8",
        "Width": width,
        "Height": height,
        "Pixels": pixels
    }

    json_dict = {
        "class": "Image",
        "data": {
            "PixelSize": new_pixel_size,
            "Origin": origin,
            "HandleZeroPixels": "Default",
            "Data": data
        }
    }

    return json_dict


def get_scatter_json(component, pixel_size):

    scatter_image = component.get_scatter()
    im_type = str(scatter_image.dtype)
    bytes_per_pixel = scatter_image.dtype.itemsize
    endianess = "little" if scatter_image.dtype == np.uint16 else "neutral"

    height, width = scatter_image.shape

    origin = {"x": 0.0, "y": 0.0, "z": 0.0}

    new_pixel_size = {}
    new_pixel_size["x"] = pixel_size["x"]
    new_pixel_size["y"] = pixel_size["y"]
    new_pixel_size["z"] = 1.0

    encoded = base64.b64encode(scatter_image.tobytes())
    rawdata = encoded.decode('ascii')
    pixels = {
        "numOfElems": height * width,
        "elemSz": bytes_per_pixel,
        "endian": endianess,
        "elemTypes": [str(scatter_image.dtype)],
        "data": rawdata
    }

    data = {
        "ImageType": im_type,
        "Width": width,
        "Height": height,
        "Pixels": pixels
    }

    json_dict = {
        "class": "Image",
        "data": {
            "PixelSize": new_pixel_size,
            "Origin": origin,
            "HandleZeroPixels": "MissingData",
            "Data": data
            }
        }

    return json_dict


def save_appspace_json(component, range_image, pixel_size, origin, output_file_path, float_to_uint16):
    json_range = get_range_json(component, range_image, pixel_size, origin, float_to_uint16)
    json_arr = [json_range]

    if component.get_intensity() is not None:
        json_intensity = get_intensity_json(component, pixel_size)
        json_arr.append(json_intensity)

    if component.get_scatter() is not None:
        json_scatter = get_scatter_json(component, pixel_size)
        json_arr.append(json_scatter)

    with open(output_file_path, 'w') as f:
        json.dump(json_arr, f)


def process_file(args, xml_path):

    reader = iconreader.IConReader(xml_path)
    components = reader.load_image_components()

    if not reader.is_sensor_image:
        for name, component in components.items():
            file_name_no_ext = xml_path.stem
            new_file_name = f"{file_name_no_ext}_{name}"
            output_path = args.output / (new_file_name + ".json")

            sensor_range_traits = reader.traits[component.name].sensor_range_traits
            range_image = component.get_range()
            if range_image is not None:
                pixel_size, origin = reader.get_coordinate_system_traits(component, args.yres)
                if sensor_range_traits is not None:
                    mask = range_image == 0
                    if sensor_range_traits["scale z"] < 0:
                        range_image = 832 * 16 - \
                            (np.abs(np.asarray(sensor_range_traits['origin z'] / np.abs(sensor_range_traits['scale z']), dtype=np.uint16)) - range_image)
                    else:
                        range_image = 832 * 16 - \
                            (np.abs(np.asarray(sensor_range_traits['origin z'] / np.abs(sensor_range_traits['scale z']), dtype=np.uint16)) + range_image)
                    range_image[mask] = 0
                save_appspace_json(component, range_image, pixel_size, origin, output_path, args.convertFloatToUint)


def main():

    parser = argparse.ArgumentParser()

    parser.add_argument(
        '-i',
        "--input",
        action='store',
        type=Path,
        help='Path to the folder where the .dat/.xml pairs are located. All valid pairs will be converted')
    parser.add_argument(
        '-o',
        '--output',
        action='store',
        type=Path,
        required=True,
        help='Specify output folder where the converted data will be placed.')

    output_group = parser.add_argument_group('outputSettings')
    output_group.add_argument(
        '-y',
        '--yres',
        action='store',
        type=float,
        default=1.0,
        help='The y-resolution of the Icon image')

    output_group.add_argument(
        '-c',
        '--convertFloatToUint',
        action='store_true',
        default=False,
        help='Whether or not to convert float images to uint16 format instead, images are assumed to be rectified for this to work')

    args = parser.parse_args()

    args.output.mkdir(exist_ok=True)
    input_path = args.input
    if not input_path.exists():
        parser.error(f"Input path {input_path} does not exist.")

    if input_path.is_dir():
        files = list(input_path.glob("*.xml"))
        for i, xml_path in enumerate(files):
            print(f"Writing component {i + 1}/{len(files)}", end="\r")
            process_file(args, xml_path)
    else:
        process_file(args, input_path)

if __name__ == "__main__":
    main()
