import time
import seaborn as sns
from collections import defaultdict

import matplotlib.pyplot as plt
import numpy as np
import rclpy
from interfaces.msg import Scan
from rclpy.node import Node

QNAN = -1000000


def is_valid(value):
    return value != 0 and value != QNAN


class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__("minimal_subscriber")
        # Create the subscriber. This subscriber will receive a Scan message and the window for plotting
        self.subscription_range = self.create_subscription(
            Scan, "range", self.listener_range_callback, 10
        )
        self.subscription_intensity = self.create_subscription(
            Scan, "intensity", self.listener_intensity_callback, 10
        )
        # Define timer to process the data
        self.timer = self.create_timer(0.01, self.process_data)
        # Variable to save the data
        self.range = np.array([])
        self.intensity = np.array([])
        self.ply_mat_range = np.array([])
        self.ply_mat_intensity = np.array([])

    def write_ply_header(
        self, file, num_vertices, num_triangles, write_triangles=False
    ):
        file.write("ply \n")
        file.write("format ascii 1.0 \n")
        file.write(f"element vertex {num_vertices} \n")
        file.write("property float x \n")
        file.write("property float y \n")
        file.write("property float z \n")
        file.write("property float Intensity \n")
        if write_triangles:
            file.write(f"element face {num_triangles} \n")
            file.write("property list uchar int vertex_index \n")
        file.write("end_header \n")

    def __get_x_scale(self, width):
        # <worldrangetraits>
        #     <parameter name="lower bound x" >0.0100641</parameter>
        #     <parameter name="lower bound r" >0.135948</parameter>
        #     <parameter name="upper bound x" >172.374</parameter>
        #     <parameter name="upper bound r" >89.5734</parameter>
        #     <parameter name="coordinate unit" ></parameter>
        # </worldrangetraits>

        lower_bound_x = 0.0100641
        upper_bound_x = 172.374

        return (upper_bound_x - lower_bound_x) / width

    def __generate_x_positions(self, range_image):
        height, width = range_image.shape
        data_x = np.empty_like(range_image, np.float64)
        x_scale = self.__get_x_scale(width)
        arr = np.linspace(0, width - 1, width) * x_scale
        for j in range(height):
            data_x[j][:] = arr
        return data_x

    def write_ply(self, range_image, x_positions, intensity_image):
        print(f"Range image shape: {range_image.shape}")
        print(f"X positions shape: {x_positions.shape}")
        print(f"Intensity image shape: {intensity_image.shape}")

        vertex_dict = defaultdict(dict)
        vertex_index = []
        triangle_index = []
        vertex_number = 0
        num_triangles = 0
        it = np.nditer(
            [range_image, x_positions, intensity_image], flags=["multi_index"]
        )

        y_res = 0.0095
        write_triangles = False

        height, width = range_image.shape

        while not it.finished:  # Multi-iterators slightly more performant.
            ix = it.multi_index[0]
            iy = it.multi_index[1]
            y = it.multi_index[0] * y_res
            z = it[0]
            x = it[1]

            intensity = it[2]
            it.iternext()
            if is_valid(z):
                c = intensity / 255.0
                vertex_index.append(f"{x} {y} {z} {c}\n")
                vertex_dict[ix][iy] = vertex_number
                vertex_number += 1

        if write_triangles:
            for y in range(height):
                for x in range(width):
                    if is_valid(range_image[y, x]):
                        # Point is candidate for triangle
                        if y != (height - 1) and x != (width - 1):
                            if is_valid(range_image[y, x + 1]) and is_valid(
                                range_image[y + 1, x]
                            ):
                                triangle_index.append(
                                    f"3 {vertex_dict[y][x]} {vertex_dict[y][x + 1]} {vertex_dict[y + 1][x]} \n"
                                )
                                num_triangles += 1
                        if y != 0 and x != 0:
                            if is_valid(range_image[y, x - 1]) and is_valid(
                                range_image[y - 1, x]
                            ):
                                triangle_index.append(
                                    f"3 {vertex_dict[y][x]} {vertex_dict[y][x - 1]} {vertex_dict[y - 1][x]} \n"
                                )
                                num_triangles += 1

        num_vertices = np.count_nonzero(
            np.logical_and(range_image != 0, range_image != QNAN)
        )
        time_now = time.time()
        with open(
            f"C:\\Users\\magok\\source\\repos\\crcz25\\capstone_welding_analysis\\test_{time_now}.ply",
            "w",
        ) as f:
            self.write_ply_header(f, num_vertices, num_triangles, write_triangles)

            f.write("".join(vertex_index))
            f.write("".join(triangle_index))

    def generate_ply(self, range_image, intensity_image):
        # Generate x positions
        x_positions = self.__generate_x_positions(range_image)
        # Generate the ply file
        self.write_ply(range_image, x_positions, intensity_image)
        return

    def listener_intensity_callback(self, msg):
        id = msg.id
        height = msg.height
        width = msg.width
        step = msg.step
        data = msg.data
        # Reshape the data to a 2D array
        data = np.array(data).reshape(height, width)
        # Save the data
        self.intensity = data
        return

    def listener_range_callback(self, msg):
        id = msg.id
        height = msg.height
        width = msg.width
        step = msg.step
        data = msg.data
        # Reshape the data to a 2D array
        data = np.array(data).reshape(height, width)
        # Save the data
        self.range = data
        return

    def process_data(self):
        # print("Processing data")
        pixel_size = {"x": 0.1122161041015625, "y": 1.0, "z": 1.0}
        origin: {"y": 0, "x": 0.0100641, "z": 0.0}
        # Get the calibrated data, is to be considered as a number of points (X, R) in the laser plane rather than an image.
        # The row (scan) number where an (X, R) point is located tells us the point’s position in the (X, Y, Z) real-world space.
        # R and Z are kept apart to indicate that R is parallel to the laser plane rather than perpendicular to the conveyor plane.

        range_image = np.array(self.range)
        intensity_image = np.array(self.intensity)
        # Plot the data as an image
        if len(range_image) > 0 and len(intensity_image) > 0:
            print(f"Range image shape: {range_image.shape}")
            print(f"Intensity image shape: {intensity_image.shape}")
            # Adjust the width of the plot based on the pixel size to get the plot with the correct
            # world coordinates
            # Normalize the x axis to the mesh width
            mesh_width = range_image.shape[1] * pixel_size["x"]
            x = np.linspace(0, mesh_width, range_image.shape[1])
            print(f"Mesh width: {mesh_width}")
            profile = range_image[0]

            # Calculate the dynamic filter of the profile
            # WINDOW_SIZE = 5  # Window size for the algorithm (adjustable)
            # PRECISION_THRESHOLD = 0.01  # Fixed precision threshold (adjustable)
            # thresholds = []  # For visualization
            # maxs = []  # For visualization
            # mins = []  # For visualization
            # step_times = []  # Array to store the times when steps were detected
            # sample_old = 0.0  # Initialize the previous sample
            # sample_new = 0.0  # Initialize the current sample
            # sample_result = 0.0  # Initialize the sample result
            # for i in range(len(profile) - WINDOW_SIZE):
            #     # Calculate the maximum and minimum acceleration in the window
            #     maximum = np.max(profile[i : i + WINDOW_SIZE])
            #     minimum = np.min(profile[i : i + WINDOW_SIZE])
            #     # Calculate the threshold for the window
            #     threshold = (maximum + minimum) / 2

            #     # Store the threshold for visualization
            #     thresholds.append(threshold)
            #     maxs.append(maximum)
            #     mins.append(minimum)

            #     sample_result = profile[i]
            #     if abs(sample_result - sample_new) > PRECISION_THRESHOLD:
            #         sample_new = sample_result
            #     else:
            #         sample_old = sample_new

            #     # A step is defined as happening if there is a negative slope of the acceleration plot (sample_new < sample_old) when the  acceleration curve crosses below the dynamic threshold.
            #     if sample_old >= threshold > sample_new and sample_new < sample_old:
            #         # print("Step detected at time " + str(time))
            #         step_times.append(time)

            # kernel = np.array([1, 4, 6, 4, 1]) / 16
            # profile = np.convolve(profile, kernel, mode="same")

            # Append the profiles to the array of profiles
            self.ply_mat_range = np.append(self.ply_mat_range, range_image)
            # Reshape the array to a 2D array
            self.ply_mat_range = self.ply_mat_range.reshape(-1, range_image.shape[1])
            print(f"PLY range mat shape: {self.ply_mat_range.shape}")

            # Append the profiles to the array of profiles
            self.ply_mat_intensity = np.append(self.ply_mat_intensity, intensity_image)
            # Reshape the array to a 2D array
            self.ply_mat_intensity = self.ply_mat_intensity.reshape(
                -1, intensity_image.shape[1]
            )
            print(f"PLY intensity mat shape: {self.ply_mat_intensity.shape}")

            # Create the figure
            fig, ax = plt.subplots()
            # Create the plot
            ax.set_title("Range image")
            ax.set_xlabel("X [mm]")
            ax.set_ylabel("Z [mm]")
            ax.set_xlim(0, mesh_width)
            ax.set_ylim(0, 100)
            ax.scatter(x, profile, s=1, c="black")
            # ax.plot(x[: len(x) - WINDOW_SIZE], thresholds, c="red")
            # ax.plot(x[: len(x) - WINDOW_SIZE], maxs, c="blue")
            # ax.plot(x[: len(x) - WINDOW_SIZE], mins, c="green")
            # plt.show()

            # Create a box plot of the range image
            # fig, ax = plt.subplots()
            # ax.boxplot(range_image[0])
            # plt.show()
            # sns.boxplot(x=range_image[0], color="skyblue", linewidth=2.5, width=0.5)

            # Plot the data
            # Adjust the plot limits to match the real world coordinates
            # self.ax.set_xlim(0, 172.374)
            # Plot the data
            # fig, ax = plt.subplots()
            # ax.imshow(range_image)
            # plt.show()
            # Itereate over the data as each row is one profile
            # for i in range(len(range_image)):
            #     # print(np.array(data[i]))
            #     # Update the plot
            #     self.line.set_ydata(np.array(range_image[i]))
            #     self.ax.relim()
            #     self.ax.autoscale_view()
            #     self.ax.draw_artist(self.ax.patch)
            #     self.ax.draw_artist(self.line)
            #     self.fig.canvas.update()
            #     self.fig.canvas.flush_events()
            #     # Wait 0.01 seconds to view the plot
            #     plt.pause(0.01)
            # Write the ply file
            if (self.ply_mat_range.shape[0] == 12800):
                self.generate_ply(self.ply_mat_range, self.ply_mat_intensity)
        # print(len(self.data_npz))

        # if id == 62:
        return


def main(args=None):
    try:
        # Initialize the node
        rclpy.init(args=args)
        # Create the node
        minimal_subscriber = MinimalSubscriber()
        # Spin the node
        rclpy.spin(minimal_subscriber)
    except KeyboardInterrupt:
        # Destroy the node
        minimal_subscriber.destroy_node()
        # Shutdown the ROS client library for Python
        rclpy.shutdown()


if __name__ == "__main__":
    main()
