#!/usr/bin/env python

import sys

sys.path.extend([".", ".."])

from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

import iconreader


def main():
    xml_path = Path(sys.argv[1])
    reader = iconreader.IConReader(xml_path)
    components = reader.load_image_components()

    for name, component in components.items():
        print(f"Plotting component {name}...")

        if reader.is_sensor_image:
            plt.figure(f"Sensor image for {name}")
            sensor_image = component.get_sensor_image()
            plt.imshow(sensor_image)
        else:
            plt.figure(f"Subcomponents for {name}")
            range_image = component.get_range()

            # Plot only the first row of the range image (first scan in time)
            row = range_image[0, :]
            x_axis = range(len(row))
            fig, ax = plt.subplots()
            ax.plot(x_axis, row)
            ax.set(xlabel="pixel", ylabel="range", title="Range image row")
            ax.grid()
            plt.show()

            # Plot 3d image of the range image
            x = range_image.shape[1]
            y = range_image.shape[0]
            fig, ax = plt.subplots(subplot_kw={"projection": "3d"})
            x_axis = range(x)
            y_axis = range(y)
            X, Y = np.meshgrid(x_axis, y_axis)
            ax.plot_surface(X, Y, range_image)
            ax.set(xlabel="x", ylabel="y", zlabel="range", title="Range image")
            plt.show()

            if range_image is not None:
                plt.subplot(1, 3, 1)
                plt.title("Range image")
                plt.imshow(range_image)

            intensity_image = component.get_intensity()
            if intensity_image is not None:
                plt.subplot(1, 3, 2)
                plt.imshow(intensity_image)
                plt.title("Intensity image")

            scatter_image = component.get_scatter()
            if scatter_image is not None:
                plt.subplot(1, 3, 3)
                plt.imshow(scatter_image)
                plt.title("Scatter image")

            mark = component.get_mark()
            if mark is not None:
                plt.figure(f"Mark information for {name}")
                plt.subplot(4, 2, 1)
                plt.plot(mark.frame_trigger_active)
                plt.title("Frame trigger active")

                plt.subplot(4, 2, 2)
                plt.plot(mark.encoder_a)
                plt.title("Encoder A")

                plt.subplot(4, 2, 3)
                plt.plot(mark.line_trigger_active)
                plt.title("Line trigger active")

                plt.subplot(4, 2, 4)
                plt.plot(mark.encoder_b)
                plt.title("Encoder B")

                plt.subplot(4, 2, 5)
                plt.plot(mark.encoder_reset_active)
                plt.title("Encoder reset active")

                plt.subplot(4, 2, 6)
                plt.plot(mark.overtrigger_count, "-r")
                plt.title("Overtrigger count")

                plt.subplot(4, 2, 7)
                plt.plot(mark.encoder_value, "r-")
                plt.title("Encoder value")

                plt.subplot(4, 2, 8)
                plt.plot(mark.sample_timestamp, "-r")
                plt.title("Sample timestamp")

                plt.tight_layout()

        plt.show()


if __name__ == "__main__":
    main()
