import matplotlib.pyplot as plt
import numpy as np
import rclpy
from interfaces.msg import Scan
from rclpy.node import Node


class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__("minimal_subscriber")
        # Create the subscriber. This subscriber will receive a Scan message and the window for plotting
        self.subscription = self.create_subscription(
            Scan, "range", self.listener_callback, 10
        )
        self.subscription  # prevent unused variable warning
        # Variable to save the data in npz format
        self.data_npz = []
        # Initialize the matplotlib window
        self.fig, self.ax = plt.subplots()
        (self.line,) = self.ax.plot(np.random.randn(1536))
        plt.show(block=False)

    def listener_callback(self, msg):
        id = msg.id
        height = msg.height
        width = msg.width
        step = msg.step
        data = msg.data
        # Reshape the data to a 2D array of 512, whatever the width is
        data = np.array(data).reshape(height, width)
        # Print the data information
        self.get_logger().info(f"ID: {id}")
        self.get_logger().info(f"height: {height}, width: {width}, step: {step}")
        self.get_logger().info(f"size: {len(data)}")
        # Normalize the data
        data = data / np.max(data)

        # Plot the data as an image
        # fig, ax = plt.subplots()
        # ax.imshow(data, cmap="gray")
        # plt.show()
        # Itereate over the data as each row is one profile
        for i in range(len(data)):
            # print(np.array(data[i]))
            # Update the plot
            self.line.set_ydata(np.array(data[i]))
            self.ax.relim()
            self.ax.autoscale_view()
            self.ax.draw_artist(self.ax.patch)
            self.ax.draw_artist(self.line)
            self.fig.canvas.update()
            self.fig.canvas.flush_events()
            # Wait 0.01 seconds to view the plot
            plt.pause(0.01)
        # Save the data
        self.data_npz.append(data)
        # print(len(self.data_npz))


def main(args=None):
    try:
        # Initialize the node
        rclpy.init(args=args)
        # Create the node
        minimal_subscriber = MinimalSubscriber()
        # Spin the node
        rclpy.spin(minimal_subscriber)
    except KeyboardInterrupt:
        # Save the data in npz format
        # np.savez_compressed("data_136pc", minimal_subscriber.data_npz)
        # Destroy the node
        minimal_subscriber.destroy_node()
        # Shutdown the ROS client library for Python
        rclpy.shutdown()


if __name__ == "__main__":
    main()
