import matplotlib.pyplot as plt
import numpy as np
import rclpy
from interfaces.msg import Scan
from rclpy.node import Node


class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__("minimal_subscriber")
        self.subscription = self.create_subscription(
            Scan, "range", self.listener_callback, 10
        )
        self.subscription  # prevent unused variable warning
        # Variable to save the data in npz format
        self.data_npz = np.array([])

    def listener_callback(self, msg):
        id = msg.id
        height = msg.height
        width = msg.width
        step = msg.step
        data = msg.data
        # Reshape the data to a 2D array
        data = np.array(data).reshape(height, width)
        # Print the data information
        self.get_logger().info(f"ID: {id}")
        self.get_logger().info(f"height: {height}, width: {width}, step: {step}")
        self.get_logger().info(f"size: {len(data)}")
        # Plot the data as an image
        # fig, ax = plt.subplots()
        # ax.imshow(data, cmap="gray")
        # plt.show()
        # Save the data
        self.data_npz = np.append(self.data_npz, data)


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
        np.savez_compressed("data_coin", minimal_subscriber.data_npz)
        # Destroy the node
        minimal_subscriber.destroy_node()
        # Shutdown the ROS client library for Python
        rclpy.shutdown()


if __name__ == "__main__":
    main()
