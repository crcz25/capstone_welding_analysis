import threading
import time

import rclpy
from scan_demo.subscriber import MinimalSubscriber


class ROSNodeThread(threading.Thread):
    """
    A thread class for running a ROS node and saving data to a CSV file.

    Args:
        csv_file_path (str, optional): The file path to save the data as a CSV file.

    Attributes:
        ros_node (recorder): The ROS node object.
        csv_file_path (str): The file path to save the data as a CSV file.
        stop_event (threading.Event): An event to signal the thread to stop.

    Methods:
        set_ros_node: Sets up the ROS node object.
        run: The main method that runs the thread.
    """

    def __init__(self, csv_file_path=None):
        super().__init__()
        self.ros_node = None
        self.csv_file_path = csv_file_path
        self.stop_event = threading.Event()

    def set_ros_node(self):
        """
        Sets up the ROS node object.
        """
        self.ros_node = MinimalSubscriber()

    def run(self):
        """
        The main method that runs the thread.
        """
        try:
            # Run the thread until the stop event is set
            while not self.stop_event.is_set():
                # If the node is not set, set it
                if self.ros_node is None:
                    self.set_ros_node()
                # Spin the node once
                rclpy.spin_once(self.ros_node, timeout_sec=0.1)
                # Print the length of the data, we use a lock to prevent the node from changing the data while we are
                # printing it
                with self.ros_node.lock:
                    # We can do something with the data here
                    if len(self.ros_node.data_npz) > 0:
                        print(len(self.ros_node.data_npz))
                # If the node is not alive, break out of the loop
                if not self.is_alive():
                    break
        except KeyboardInterrupt:
            pass
        finally:
            # Close the node
            print("Closing ROS Node Thread")
            # We can also ddo something with the node before closing it (need to use lock inside the node)
            self.ros_node.save_data(self.csv_file_path)
            # Destroy the node if it is not None
            if self.ros_node is not None:
                self.ros_node.destroy_node()
