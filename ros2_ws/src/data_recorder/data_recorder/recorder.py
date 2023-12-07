import numpy as np
import rclpy
from datetime import datetime
from time import strftime
from interfaces.msg import Scan
from std_msgs.msg import Header
from rclpy.node import Node


class Recorder(Node):
    def __init__(self):
        super().__init__("recorder")
        # Create the subscriber. This subscriber will receive a Scan message and the window for plotting
        self.subscription = self.create_subscription(
            Scan, "range", self.listener_callback, 10
        )
        self.subscription  # prevent unused variable warning
        # Variable to save the data in npy format
        self.data_npy = []
        self.timestamps = []

    def listener_callback(self, msg):
        id = msg.id
        height = msg.height
        width = msg.width
        step = msg.step
        data = msg.data
        timestamp = msg.header.stamp
        
        # Convert the time from ROS message to a single string value
        # and add leading zeros to nanoseconds
        timestamp = str(timestamp.sec) + str(timestamp.nanosec).zfill(9)
        self.timestamps.append(timestamp)
        # Reshape the data to a 2D array of 512, whatever the width is
        data = np.array(data).reshape(height, width)
        # Log the data information
        self.get_logger().info(f"ID: {id}")
        self.get_logger().info(f"height: {height}, width: {width}, step: {step}")
        self.get_logger().info(f"size: {len(data)}")

        self.data_npy.append(data)


def main(args=None):
    try:
        rclpy.init(args=args)
        recorder = Recorder()
        rclpy.spin(recorder)
    except KeyboardInterrupt:
        # Convert timestamp to human readable
        timestamp = datetime.fromtimestamp(float(recorder.timestamps[0]) // 1000000000)
        timestamp = timestamp.strftime('%d-%m-%Y-%H-%M-%S')
        timestamp_arr = np.asarray(recorder.timestamps, dtype=str)
        # TODO change save location
        np.savetxt(f"timestamps_ns_{timestamp}.csv", timestamp_arr, delimiter=',', fmt='%s')
        np.save(f"ranges_{timestamp}", recorder.data_npy)
        recorder.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

