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
    """
    A class that represents a data recorder node in ROS2.

    This node subscribes to a Scan message and records the received data.
    The recorded data can be saved to a file.

    Attributes:
        data_npy (list): A list to store the recorded data in npy format.
        timestamps (list): A list to store the timestamps of the received messages.
        lock (threading.Lock): A lock to prevent concurrent access to the data.

    Methods:
        __init__(): Initializes the Recorder node.
        listener_callback(msg): Callback function to process the received Scan message.
        save_data(file_path): Saves the recorded data to a file.
        get_timestamp() -> datetime: Returns the timestamp of the first received message.
        generate_timestamps(timestamps, cycle_time) -> np.ndarray: Generates timestamps between received messages.
    """
    def __init__(self):
        super().__init__("recorder")
        self.subscription = self.create_subscription(
            Scan, "range", self.listener_callback, 10
        )
        self.subscription
        self.data_npy = []
        self.timestamps = []
        self.lock = threading.Lock()

    def listener_callback(self, msg):
        """
        Callback function to process the received Scan message.

        Args:
            msg (Scan): The received Scan message.
        """
        id = msg.id
        height = msg.height
        width = msg.width
        step = msg.step
        data = msg.data
        timestamp = msg.header.stamp

        timestamp = str(timestamp.sec) + str(timestamp.nanosec).zfill(9)
        with self.lock:
            self.timestamps.append(timestamp)
        data = np.array(data).reshape(height, width)
        self.get_logger().info(f"ID: {id}")
        self.get_logger().info(f"height: {height}, width: {width}, step: {step}")
        self.get_logger().info(f"size: {len(data)}")

        with self.lock:
            self.data_npy.append(data)

    def save_data(self, file_path):
        """
        Saves the recorded data to a file.

        Args:
            file_path (str): The path to the directory where the data will be saved.
        """
        print("Saving data")
        print(f"File path: {file_path}")
        timestamp = self.get_timestamp()

        os.mkdir(os.path.join(file_path, timestamp))

        if len(self.timestamps) > 0:
            timestamps_generated = self.generate_timestamps(
                np.asarray(self.timestamps, dtype=np.int64)
            )
            timestamps_str = np.asarray(timestamps_generated, dtype=str)

        with self.lock:
            timestamp_path = os.path.join(
                file_path, timestamp, f"timestamps_ns_{timestamp}.csv"
            )
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
        """
        Returns the timestamp of the first received message.

        Returns:
            datetime: The timestamp of the first received message.
        """
        now = datetime.now()
        timestamp = datetime.fromtimestamp(float(self.timestamps[0]) // 1000000000)
        timestamp = now.strftime("%d-%m-%Y-%H-%M-%S")
        return timestamp

    def generate_timestamps(
        self, timestamps: np.ndarray, cycle_time: float = 3000000
    ) -> np.ndarray:
        """
        Generates timestamps between received messages.

        Args:
            timestamps (np.ndarray): The timestamps of the received messages.
            cycle_time (float): The cycle time in nanoseconds.

        Returns:
            np.ndarray: The generated timestamps.
        """
        new_timestamps = []
        for i in range(len(timestamps) - 1):
            new_timestamps.append(
                np.linspace(
                    timestamps[i], timestamps[i + 1], num=512, dtype=float
                ).astype(np.int64)
            )
        new_timestamps.append(
            np.linspace(
                timestamps[-1], timestamps[-1] + cycle_time * 512, num=512, dtype=float
            ).astype(np.int64)
        )
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
