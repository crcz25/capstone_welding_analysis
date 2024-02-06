import threading
from datetime import datetime
from time import strftime

import numpy as np
import rclpy
from interfaces.msg import Scan
from rclpy.node import Node
from std_msgs.msg import Header
import os


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
        # Lock to prevent the node from changing the data while accessing it
        self.lock = threading.Lock()

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
        with self.lock:
            self.timestamps.append(timestamp)
        # Reshape the data to a 2D array of 512, whatever the width is
        data = np.array(data).reshape(height, width)
        # Log the data information
        self.get_logger().info(f"ID: {id}")
        self.get_logger().info(f"height: {height}, width: {width}, step: {step}")
        self.get_logger().info(f"size: {len(data)}")

        with self.lock:
            self.data_npy.append(data)

    def save_data(self, file_path):
        print("Saving data")
        print(f"File path: {file_path}")
        timestamp = self.get_timestamp()

        # Create a folder for the ranges and timestamps
        os.mkdir(os.path.join(file_path, timestamp))

        # Generate timestamps in between message timestamps
        if len(self.timestamps) > 0:
            timestamps_generated = self.generate_timestamps(np.asarray(self.timestamps, dtype=int))
            timestamps_str = np.asarray(timestamps_generated, dtype=str)
        
        with self.lock:
            timestamp_path = os.path.join(file_path, timestamp, f"timestamps_ns_{timestamp}.csv")
            np.savetxt(
                timestamp_path,
                timestamps_str,
                delimiter=",",
                fmt="%s",
            )
            ranges_path = os.path.join(file_path, timestamp, f"ranges_{timestamp}.npy")
            shape = (np.shape(self.data_npy)[0] * np.shape(self.data_npy)[1], 1536)
            range_data = np.reshape(self.data_npy, shape)
            np.save(ranges_path, range_data)
            self.get_logger().info("Data saved")

    def get_timestamp(self) -> datetime:
        now = datetime.now()
        timestamp = datetime.fromtimestamp(
                    float(self.timestamps[0]) // 1000000000
                )
        # Convert timestamp to human readable
        timestamp = now.strftime("%d-%m-%Y-%H-%M-%S")
        return timestamp
    
    def generate_timestamps(self, timestamps: np.ndarray, cycle_time: float = 3000000) -> np.ndarray:
        """
        len(timestamps) > 0
        
        cycle_time in nanoseconds
        """
        new_timestamps = []
        for i in range(len(timestamps)-1):
            new_timestamps.append(np.linspace(timestamps[i], timestamps[i+1], num=512, dtype=float).astype(int))
        new_timestamps.append(np.linspace(timestamps[-1], timestamps[-1] + cycle_time*512, num=512, dtype=float).astype(int))
        new_timestamps = np.array(new_timestamps)
        new_timestamps = np.ravel(new_timestamps)
        return new_timestamps



def main(args=None):
    try:
        rclpy.init(args=args)
        recorder = Recorder()
        rclpy.spin(recorder)
    except KeyboardInterrupt:
        recorder.save_data(os.curdir)
        recorder.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
