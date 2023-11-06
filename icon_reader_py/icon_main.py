#!/usr/bin/env python
import sys

sys.path.extend([".", ".."])

import datetime
import os
from pathlib import Path

import examples.icon_to_ply_converter as ipc
import iconreader
import matplotlib.pyplot as plt
import numpy as np
import open3d as o3d

# Get todays date in %Y-%m-%d-%H-%M-%S-%f format
today = datetime.datetime.now().strftime("%Y-%m-%#d")
# Set the directory of the data
script_dir = os.path.dirname(__file__)
data_dir = os.path.join(script_dir, "../data/section_coin")
# filter the files by extension (.xml) and sort them by date
files = [f for f in os.listdir(data_dir) if today in f]
xml_files = [f for f in files if f.endswith(".xml")]

res_rows, res_cols = (512, 1536)
res_rows_mm, res_cols_mm = (4.864, 14.592)


def extract_numeric_part(filename):
    return int(filename.split("_")[-1].split(".")[0])


def generate_plys():
    global xml_files
    xml_files = sorted(xml_files, key=extract_numeric_part)

    print(f"Data directory: {data_dir}, today: {today}")

    # Print the files to be processed
    print(f"To be processed: {xml_files}")

    # Loop through the files and generate a point cloud for each
    # use the script icon_to_ply_converter.py inside examples
    for file in xml_files:
        print(f"Processing file: {file}")
        # Get the path of the file
        cloud_path = Path(data_dir + "/" + file[:-4] + ".ply")
        # Check if the file has already been processed
        if cloud_path.is_file():
            print(f"File {file} already processed")
            continue
        # Read the file
        reader = iconreader.IConReader(Path(data_dir) / file)
        components = reader.load_image_components()
        # Iterate through the components and generate a point cloud for each
        for name, component in components.items():
            new_file_name = f"{file[:-4]}.ply"
            output_path = data_dir + "/" + new_file_name

            ipc.write_ply(Path(output_path), 0.0095, False, False, component)

def show_pointcloud(cloud_path):
    # Read the point cloud
    cloud = o3d.io.read_point_cloud(cloud_path)
    # Rotate the point cloud
    R = cloud.get_rotation_matrix_from_xyz((np.pi, 0, -np.pi / 2))
    cloud = cloud.rotate(R, center=(0, 0, 0))
    # Remove outliers from the point cloud
    cloud, ind = cloud.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)

    # Plot the point clouds subsecuently in the same window separated by 5 units in the x axis
    viewer = o3d.visualization.Visualizer()
    viewer.create_window(width=1280, height=720)
    viewer.add_geometry(cloud)
    opt = viewer.get_render_option()
    opt.show_coordinate_frame = True
    opt.background_color = np.asarray([0.5, 0.5, 0.5])
    viewer.run()
    viewer.destroy_window()

def main():
    # Generate the point clouds
    generate_plys()

    # Show the point cloud
    file_name = f"SCAN_2023-11-6-13-53-6-296_1.ply"
    cloud_path = data_dir + "/" + file_name
    print(f"Cloud path: {cloud_path}, file name: {file_name}")
    show_pointcloud(cloud_path)

    # Show the range and intensity images
    reader = iconreader.IConReader("data/section_coin/SCAN_2023-11-6-13-53-6-296_1.xml")
    components = reader.load_image_components()
    # Get the ranges
    range_image = components["Hi3D 1"].get_range()
    print(range_image)

    for name, component in components.items():
        if not reader.is_sensor_image:
            range_image = component.get_range()
            intensity_image = component.get_intensity()
            if range_image is not None and intensity_image is not None:
                # Plot the range image
                fig, ax = plt.subplots(2, 1)
                ax[0].set_title(f"Range image for {name}")
                ax[0].imshow(range_image, extent=[0, res_cols, res_rows, 0])
                ax[1].set_title(f"Intensity image for {name}")
                ax[1].imshow(intensity_image, extent=[0, res_cols, res_rows, 0])
                plt.show()

                # Plot the middle row of the range image as scatter plot
                fig, ax = plt.subplots()
                plt.title(f"Range image middle row for {name}")
                plt.plot(range_image[res_rows // 2, :])
                plt.show()


if __name__ == "__main__":
    main()
