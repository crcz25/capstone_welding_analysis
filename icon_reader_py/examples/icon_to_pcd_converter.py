#!/usr/bin/env python

import sys
sys.path.extend([".", ".."])

import argparse
import iconreader
import numpy as np
import open3d as o3d

from pathlib import Path
from typing import Tuple, Dict


def range_image_to_pointcloud(range_im, pixel_size, origin) -> o3d.geometry.PointCloud:
    point_list = []
    height, width = range_im.shape

    y_size = pixel_size["y"]
    x_size = pixel_size["x"]

    y_origin = origin["y"]
    x_origin = origin["x"]
    z_origin = origin["z"]

    x, y = np.meshgrid(np.arange(width), np.arange(height))
    x = x.flatten()
    y = y.flatten()
    range_im_flat = range_im.flatten()
    x = x[~np.isnan(range_im_flat)]
    y = y[~np.isnan(range_im_flat)]

    x_scaled = x.astype(np.float32) * x_size - x_origin
    y_scaled = y.astype(np.float32) * y_size - y_origin
    z = range_im[y.astype(np.int64), x]

    point_array = np.stack((x_scaled, y_scaled, z)).T
    pointcloud = o3d.geometry.PointCloud()
    pointcloud.points = o3d.utility.Vector3dVector(point_array)
    return pointcloud


def load_image(path: Path, y_res=float) -> Tuple[np.ndarray, Dict, Dict]:
    reader = iconreader.IConReader(path)
    image_component = reader.load_image_components()["Ranger3Range"]
    pixel_size, origin = reader.get_coordinate_system_traits(image_component, y_res=y_res)
    range_im = image_component.rescale_range_16bit_to_float()
    range_im[range_im == 0] = np.nan
    return range_im, pixel_size, origin


def process_file(args, xml_path: Path):
    range_im, pixel_size, origin = load_image(xml_path, args.yres)
    pointcloud = range_image_to_pointcloud(range_im, pixel_size, origin)
    output_path = args.output / (xml_path.stem + ".pcd")
    o3d.io.write_point_cloud(str(output_path), pointcloud)


def main():
    parser = argparse.ArgumentParser(description="Convert iCon images to PCD")

    parser.add_argument(
        "-i",
        "--input",
        action="store",
        type=Path,
        help="Path to the folder where the .dat/.xml pairs are located. All valid pairs will be converted")

    parser.add_argument(
        "-y",
        "--yres",
        type=float,
        default=1.0,
        help="The Y-resolution of the images, used if it's not available in the image metadata. Default is 1.0.")

    parser.add_argument(
        "-o",
        "--output",
        action="store",
        type=Path,
        required=True,
        help="Specify output folder where the converted data will be placed.")

    args = parser.parse_args()

    args.output.mkdir(exist_ok=True)
    input_path = args.input
    if not input_path.exists():
        parser.error(f"Input path {input_path} does not exist.")

    if input_path.is_dir():
        files = input_path.glob("*.xml")
        for xml_path in files:
            process_file(args, xml_path)
    else:
        process_file(args, input_path)


if __name__ == "__main__":
    main()

