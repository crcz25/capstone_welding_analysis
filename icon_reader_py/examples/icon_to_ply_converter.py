#!/usr/bin/env python

import sys
sys.path.extend([".", ".."])

from collections import defaultdict
from pathlib import Path

import argparse
import iconreader
import numpy as np


QNAN = -1000000


def is_valid(value):
    return value != 0 and value != QNAN


def write_ply_header(
        file,
        num_vertices,
        num_triangles,
        rgb,
        write_triangles):
    file.write("ply \n")
    file.write("format ascii 1.0 \n")
    file.write(f"element vertex {num_vertices} \n")
    file.write("property float x \n")
    file.write("property float y \n")
    file.write("property float z \n")
    if rgb:
        file.write("property uchar red \n")
        file.write("property uchar green \n")
        file.write("property uchar blue \n")
    else:
        file.write("property float Intensity \n")
    if write_triangles:
        file.write(f"element face {num_triangles} \n")
        file.write("property list uchar int vertex_index \n")
    file.write("end_header \n")


def write_ply(output_file_path, y_res, rgb, write_triangles, component):
    is_calibrated = component.traits.has_world_range_traits()
    is_uint = component.get_range().dtype == np.uint16
    if(is_calibrated and is_uint):
        range_image = component.rescale_range_16bit_to_float()
    else:
        range_image = component.get_range()
    x_positions = component.x_positions
    intensity_image = component.get_intensity()

    if range_image is not None:
        height, width = range_image.shape

        vertex_dict = defaultdict(dict)
        vertex_index = []
        triangle_index = []
        vertex_number = 0
        num_triangles = 0
        it = np.nditer([range_image, x_positions, intensity_image],
                        flags=['multi_index'])
        while not it.finished:  # Multi-iterators slightly more performant.
            ix = it.multi_index[0]
            iy = it.multi_index[1]
            y = it.multi_index[0] * y_res
            z = it[0]
            x = it[1]

            intensity = it[2]
            it.iternext()
            if is_valid(z):
                if rgb:
                    vertex_index.append(
                        f"{x} {y} {z} {intensity} {intensity} {intensity}\n")
                else:
                    c = intensity / 255.0
                    vertex_index.append(f"{x} {y} {z} {c}\n")
                vertex_dict[ix][iy] = vertex_number
                vertex_number += 1

        if write_triangles:
            for y in range(height):
                for x in range(width):
                    if is_valid(range_image[y, x]):
                        # Point is candidate for triangle
                        if (y != (height - 1) and x != (width - 1)):
                            if is_valid(range_image[y, x + 1]) and is_valid(range_image[y + 1, x]):
                                triangle_index.append(
                                    f"3 {vertex_dict[y][x]} {vertex_dict[y][x + 1]} {vertex_dict[y + 1][x]} \n")
                                num_triangles += 1
                        if y != 0 and x != 0:
                            if is_valid(range_image[y, x - 1]) and is_valid(range_image[y - 1, x]):
                                triangle_index.append(
                                    f"3 {vertex_dict[y][x]} {vertex_dict[y][x - 1]} {vertex_dict[y - 1][x]} \n")
                                num_triangles += 1

        num_vertices = np.count_nonzero(
            np.logical_and(
                range_image != 0,
                range_image != QNAN))
        with open(output_file_path, 'w') as f:
            write_ply_header(
                f,
                num_vertices,
                num_triangles,
                rgb,
                write_triangles)

            f.write(''.join(vertex_index))
            f.write(''.join(triangle_index))


def process_file(args, xml_path):
    reader = iconreader.IConReader(xml_path)
    components = reader.load_image_components()

    for name, component in components.items():
        file_name_no_ext = xml_path.stem
        new_file_name = f"{file_name_no_ext}_{name}"
        output_path = args.output / (new_file_name + ".ply")
        write_ply(output_path, args.yres, args.rgb, args.triangles, component)


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
        '-t',
        '--triangles',
        action='store_true',
        default=False,
        help='Whether or not to generate a mesh')

    output_group.add_argument(
        '-r',
        '--rgb',
        action='store_true',
        default=False,
        help='Whether to save with rgb color or not.')

    args = parser.parse_args()

    args.output.mkdir(exist_ok=True)
    input_path = args.input
    if not input_path.exists():
        parser.error(f"Input path {input_path} does not exist.")

    if input_path.is_dir():
        files = list(input_path.glob("*.xml"))
        for xml_path in files:
            process_file(args, xml_path)
    else:
        process_file(args, input_path)

if __name__ == "__main__":
    main()
