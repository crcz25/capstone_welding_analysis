import sys
from datetime import datetime
from pathlib import Path

import customtkinter as ctk
import numpy as np
import rclpy
from customtkinter import filedialog
from frames.InfoFrame import InfoFrame
from frames.MenuFrame import MenuFrame
from frames.NodeThread import ROSNodeThread
from frames.PlotControlFrame import PlotControlFrame
from frames.PlotFrame import PlotFrame
from frames.SettingsFrame import SettingsFrame

ctk.set_appearance_mode("System")
ctk.set_default_color_theme("dark-blue")


# --------------------------------------------------------GUI BASE--------------------------------------------------------#
class App(ctk.CTk):
    """
    The main application class for the GUI.

    Attributes:
    - menu_frame (MenuFrame): The frame for the menu options.
    - plot_frame (PlotFrame): The frame for displaying the plot.
    - info_frame (InfoFrame): The frame for displaying information.
    - plot_control_frame (PlotControlFrame): The frame for controlling the plot.
    - settings_frame (SettingsFrame): The frame for the settings.

    Methods:
    - __init__(): Initializes the App class.
    - change_appearance_mode_event(new_appearance_mode: str): Changes the appearance mode of the GUI.
    - change_scaling_event(new_scaling: str): Changes the scaling of the GUI.
    - change_button_text(button, text_1, text_2): Changes the text of a button.
    - change_console_text(text, tags=None): Logs text to the UI console.
    - menu_button(frame_type): Handles button clicks in the menu.
    - show_camera_frames(): Shows the frames related to camera settings.
    - show_default_frames(): Shows the default frames.
    - reset_imported_files(): Resets the imported files.
    - import_files(): Imports files into the application.
    """
    def __init__(self):
        super().__init__()

        self.title("GUI for SICK E55 Weld Monitoring")
        self.geometry(f"{1400}x{900}")
        self.minsize(1400, 900)

        self.grid_columnconfigure(1, weight=1)
        self.grid_columnconfigure((2, 3), weight=0)
        self.grid_rowconfigure((0, 1), weight=1)

        # Add Frames to the base
        self.menu_frame = MenuFrame(self)
        self.plot_frame = PlotFrame(self)
        self.info_frame = InfoFrame(self)
        # Create the thread for the node
        # thread = threading.Thread()
        self.plot_control_frame = PlotControlFrame(self)
        self.settings_frame = SettingsFrame(self)

        # Set default values
        self.menu_frame.appearance_mode_optionmenu.set("Dark")
        self.menu_frame.scaling_optionmenu.set("100%")
        self.plot_control_frame.slider.set(0)
        self.settings_frame.grid_remove()

        # These disable writing in the right frame textboxes
        self.info_frame.textbox_info.configure(state="disabled")
        self.info_frame.textbox_alerts.configure(state="disabled")

        # ROS2 node
        self.ros_node_thread = None
        self.scanning_in_progress = False

        # Import files
        self.directory = None
        self.range_file = None
        self.timestamp_file = None
        self.range_data = np.array([])
        self.timestamp_data = np.array([])

        # Visualization
        self.current_profile = 0
        self.max_profiles = 0

    # --------------------------------------------------------FUNCTIONALITY--------------------------------------------------------#
    def change_appearance_mode_event(self, new_appearance_mode: str):
        """
        Change the appearance mode of the application.

        Args:
            new_appearance_mode (str): The new appearance mode to set.

        Returns:
            None
        """
        ctk.set_appearance_mode(new_appearance_mode)
        self.show_default_frames()
        self.change_console_text(
            self
        )  # Make sure that text colors change when theme is changed

    def change_scaling_event(self, new_scaling: str):
        """
        Change the scaling of the widget and show default frames.

        Args:
            new_scaling (str): The new scaling value as a string with a percentage symbol (%).

        Returns:
            None
        """
        new_scaling_float = int(new_scaling.replace("%", "")) / 100
        ctk.set_widget_scaling(new_scaling_float)
        self.show_default_frames()

    def change_button_text(self, button, text_1, text_2):
        """
        Changes the text of a button based on its current state.

        Parameters:
        button (tkinter.Button): The button to change the text of.
        text_1 (str): The first text to be displayed on the button.
        text_2 (str): The second text to be displayed on the button.
        """
        current_state = button.cget("text")
        new_state = text_2 if current_state == text_1 else text_1
        # Check if it is time to start or stop the scan
        if current_state == "Take scan":
            self.change_console_text("Listening to scan topic /range", "INFORMATION")
            self.change_console_text(
                "Verify that ros2 node is publishing or the rosbag is playing",
                "INFORMATION",
            )
            self.start_scan()
        elif current_state == "Stop scan":
            self.change_console_text("Stopping scan", "INFORMATION")
            self.stop_scan()

        button.configure(text=new_state)

    def change_console_text(self, text, tags=None):
        """
        Function to log text to the UI console with different tags.

        Parameters:
        - text (text to be shown).
        - tag (INFO: white, ERROR: red, SUCCESS: green)
        """
        print(text)  # Kept this to print to terminal

        ap_mode = ctk.get_appearance_mode()

        # In light mode white color is not visible in the console
        # Create tags
        if ap_mode == "Dark":
            self.plot_control_frame.searchbox_entry.configure(text_color="white")
            self.plot_control_frame.console_entry.tag_config(
                "INFORMATION", foreground="white"
            )
        else:
            self.plot_control_frame.searchbox_entry.configure(text_color="black")
            self.plot_control_frame.console_entry.tag_config(
                "INFORMATION", foreground="black"
            )

        self.plot_control_frame.console_entry.tag_config("ERROR", foreground="red")
        self.plot_control_frame.console_entry.tag_config("SUCCESS", foreground="green")

        # Insert colored text to console
        self.plot_control_frame.console_entry.configure(state=ctk.NORMAL)
        self.plot_control_frame.console_entry.insert(
            ctk.END, tags + ": " + text + "\n", tags
        )
        self.plot_control_frame.console_entry.configure(state=ctk.DISABLED)

        # Keep focus on the end line
        self.plot_control_frame.console_entry.see("end")

    def menu_button(self, frame_type):
        """
        Handles the menu button click event and displays the corresponding frames.

        Args:
            frame_type (str): The type of frame to be displayed.

        Returns:
            None
        """
        if frame_type == "home":
            self.show_default_frames()
        elif frame_type == "settings":
            self.show_camera_frames()

    def show_camera_frames(self):
        """
        Hides the info frame, plot control frame, and plot frame, and shows the settings frame.
        """
        self.info_frame.grid_remove()
        self.plot_control_frame.grid_remove()
        self.plot_frame.grid_remove()
        self.settings_frame.grid()

    def show_default_frames(self):
        """
        Show the default frames in the GUI.

        This method hides the settings frame and shows the info frame, plot frame, and plot control frame.
        """
        self.settings_frame.grid_remove()
        self.info_frame.grid()
        self.plot_frame.grid()
        self.plot_control_frame.grid()

    def reset_imported_files(self):
        """
        Resets the imported files and associated data to their initial state.
        """
        self.range_file = None
        self.timestamp_file = None
        self.range_data = np.array([])
        self.timestamp_data = np.array([])
        self.current_profile = 0
        self.max_profiles = 0
        self.plot_control_frame.slider.set(0)
        self.plot_frame.clean_plot()
        self.update_info_frame()

    def import_files(self):
        """
        Import files from a selected directory.

        This method prompts the user to select a directory and imports the files with the extensions .npy and .csv
        from that directory. It performs various checks and validations on the imported files and updates the
        necessary data structures and UI elements accordingly.

        Returns:
            None
        """
        # If there is a scan already imported, replace it with the new one and clean the plot frame
        if self.range_file is not None and self.timestamp_file is not None:
            self.reset_imported_files()

        # Ask the user for the directory to import timestamps and ranges (.csv and .npy)
        self.directory = filedialog.askdirectory(
            title="Select directory to import",
            initialdir=f"{sys.path[0]}",
        )
        # Check if the user selected a directory
        if len(self.directory) == 0:
            self.change_console_text("No directory selected", "ERROR")
            return
        # Convert to absolute path
        self.directory = Path(self.directory).resolve()

        # If there is a scan already imported, replace it with the new one and clean the plot frame
        if self.range_file is not None and self.timestamp_file is not None:
            self.reset_imported_files()
            self.change_console_text("Replacing imported files", "INFORMATION")

        self.change_console_text(
            f"Directory to import: {self.directory}", "INFORMATION"
        )
        # Get the files in the directory and filter those that are .npy and .csv
        files = []
        for ext in ["*.npy", "*.csv"]:
            files.extend(self.directory.glob(ext))
        # Check that there are files in the directory
        if len(files) == 0:
            self.change_console_text("No files found, check the directory", "ERROR")
            return
        elif len(files) == 1:
            self.change_console_text(
                "Only one file found, check the directory", "ERROR"
            )
            return
        elif len(files) >= 2:
            # Print the files in the directory
            self.change_console_text(f"Files in directory: {files}", "INFORMATION")
            for f in files:
                if f.suffix == ".npy":
                    self.range_file = f
                elif f.suffix == ".csv":
                    self.timestamp_file = f
        # Open the npy file
        self.range_data = np.load(self.range_file)
        self.change_console_text(
            f"Range file to open: {self.range_file}", "INFORMATION"
        )
        self.change_console_text(
            f"Size of range data: {self.range_data.shape}", "INFORMATION"
        )
        # Make sure there are only two dimensions in the range data, if not, reshape it
        if len(self.range_data.shape) > 2:
            self.change_console_text(f"Reshaping range data", "INFORMATION")
            total_profiles = self.range_data.shape[0] * self.range_data.shape[1]
            self.range_data = self.range_data.reshape(
                total_profiles, self.range_data.shape[2]
            )

        # Set the current frame to 0
        self.current_profile = 0
        # Set the max frames
        self.max_profiles = self.range_data.shape[0] - 1
        # Set the slider to the max number of scans
        # Calculate number of steps in which the slider can be positioned (one step per profile)
        self.plot_control_frame.slider.configure(
            to=self.max_profiles,
            number_of_steps=self.max_profiles,
        )

        # open the csv file
        self.timestamp_data = np.genfromtxt(self.timestamp_file, delimiter=",")

        # Divide the timestamps for each profile
        self.timestamp_formatter()

        self.change_console_text(
            f"Timestamp file to open: {self.timestamp_file}", "INFORMATION"
        )
        self.change_console_text(
            f"Size of timestamp data: {self.timestamp_data.shape}", "INFORMATION"
        )

        # Correct the dimensions of the data based on the pixel size from the settings frame
        # Get the pixel size from the settings frame
        pixel_size_x, pixel_size_y, pixel_size_z = self.settings_frame.get_pixel_size()
        # Apply the pixel size to the range data
        self.range_data[:, 0] *= float(pixel_size_x)
        self.range_data[:, 1] *= float(pixel_size_y)
        self.range_data[:, 2] *= float(pixel_size_z)

        # Extrapolate the nan values in the range data with 0
        # self.range_data = np.nan_to_num(self.range_data, nan=0)

        # Update the plot
        self.plot_frame.create_figure(
            profile=self.current_profile, data=self.range_data
        )

        # Update the max profiles label of the slider
        self.plot_control_frame.end_position_label.configure(
            text=f"{self.max_profiles + 1}"
        )

        # Update the info frame textboxes
        self.update_info_frame()

    def timestamp_formatter(self):
        """
        Formats the csv file timestamps back to human readable format for the UI.

        Returns:
            None
        """
        self.formatted_timestamps = []
        for timestamp in self.timestamp_data:
            formatted_timestamp = datetime.fromtimestamp(timestamp / 1e9).strftime(
                "%H:%M:%S.%f"
            )
            self.formatted_timestamps.append(formatted_timestamp)

    def update_info_frame(self, new_limits=None):
        """
        Updates the information frame with relevant data.

        Args:
            new_limits (list, optional): New limits for the cursor. Defaults to None.
        """
        # Check if the data is loaded
        if self.range_file is not None and self.timestamp_file is not None:
            # Enable the textbox
            self.info_frame.textbox_info.configure(state="normal")
            # Delete the current text
            self.info_frame.textbox_info.delete("1.0", "end")
            # Insert the new text
            filename = self.range_file.name
            tstamp = f"{self.formatted_timestamps[self.current_profile]}"

            # Since the timestamps after profile 1536 are calculated with the cycle time, they have to be formatted differently
            if self.current_profile == 0:
                time_difference = 0
            elif self.current_profile <= 1536:
                time_difference = (
                    self.timestamp_data[self.current_profile]
                    - self.timestamp_data[self.current_profile - 1]
                ) // 1000000
            else:
                time_difference = (
                    self.timestamp_data[self.current_profile]
                    - self.timestamp_data[self.current_profile - 1]
                ) // 1000

            tstampdif = f"{time_difference:.2f} ms"
            profile = self.current_profile + 1
            # Get the limits of the cursor
            x_min = f"{self.plot_frame.cursor_limits['x_min']} mm"
            x_max = f"{self.plot_frame.cursor_limits['x_max']} mm"
            y_min = f"{self.plot_frame.cursor_limits['y_min']} mm"
            y_max = f"{self.plot_frame.cursor_limits['y_max']} mm"
            if len(self.plot_frame.cursor_limits) > 0 and new_limits is not None:
                x_min = f"{new_limits[0][0]} mm"
                x_max = f"{new_limits[0][1]} mm"
                y_min = f"{new_limits[1][0]} mm"
                y_max = f"{new_limits[1][1]} mm"
            inverted = self.plot_frame.invert_plot
            curr_filter = self.plot_control_frame.choice
            template = "Scan: {}\nTime: {}\nTime difference to previous: {}\nProfile: {}/{}\nX-Min: {}\nX-Max: {}\nY-Min: {}\nY-Max: {}\nInverted: {}\nFilter: {}".format(
                filename,
                tstamp,
                tstampdif,
                profile,
                self.max_profiles + 1,
                x_min,
                x_max,
                y_min,
                y_max,
                inverted,
                curr_filter,
            )
            self.info_frame.textbox_info.insert("0.0", template)
            # Disable the textbox
            self.info_frame.textbox_info.configure(state="disabled")
        else:
            self.change_console_text("Data is not loaded", "ERROR")

    def change_plot(self, change, profile=0):
        """
        Change the plot based on the given change and profile.

        Parameters:
        - change (int): The amount to change the current profile by.
        - profile (int): The profile to display on the plot.

        Returns:
        None
        """
        # Check if the data is loaded
        if self.range_data.size > 0 and self.timestamp_data.size > 0:
            self.current_profile += change
            # Check if the frame is out of bounds
            if self.current_profile < 0:
                self.current_profile = 0
            elif self.current_profile > self.max_profiles:
                self.current_profile = self.max_profiles
            # Check if the profile is out of bounds
            if profile < 0:
                profile = 0
            elif profile > self.max_profiles:
                profile = self.max_profiles
            # Reset alignment flags
            self.plot_frame.align_plot = False
            self.plot_frame.pt1 = None
            self.plot_frame.pt2 = None
            # Update the info frame textboxes
            self.update_info_frame()
            # Update the plot
            self.plot_frame.update_surface(
                profile=profile,
                data=self.range_data,
                choice=self.plot_control_frame.choice,
            )
        else:
            self.change_console_text("Data is not loaded", "ERROR")

    def call_scan(self):
        self.plot_control_frame.take_stop_scan()

    def start_scan(self):
        """
        Starts the scanning process.

        If scanning is not already in progress, it sets the scanning_in_progress flag to True,
        creates a ROSNodeThread instance, starts the thread, and updates the console text to indicate
        that the scan has started. If scanning is already in progress, it updates the console text
        to indicate that the scan did not start.
        """
        if not self.scanning_in_progress:
            self.scanning_in_progress = True
            self.ros_node_thread = ROSNodeThread()
            self.ros_node_thread.start()
            self.change_console_text("Scan started", "INFORMATION")
        else:
            self.change_console_text("Scan did not start", "INFORMATION")

    def stop_scan(self):
        """
        Stops the scanning process and saves the data.

        If scanning is in progress, it stops the node, saves the data to the specified directory,
        and updates the console text accordingly.
        """
        if self.scanning_in_progress:
            # Stop the node
            self.scanning_in_progress = False
            self.ros_node_thread.stop_event.set()
            self.ros_node_thread.join()
            self.change_console_text("Scan stopped", "INFORMATION")
            ## Save the data ##
            # Ask the user where to save the data
            self.file_path = filedialog.askdirectory()
            # Check if the user selected a directory
            if len(self.file_path) == 0:
                self.change_console_text("No directory selected", "ERROR")
                return
            # Convert to absolute path
            self.change_console_text(f"Saving data to {self.file_path}", "SUCCESS")
            self.ros_node_thread.save_data(self.file_path)


if __name__ == "__main__":
    try:
        rclpy.init(args=None)
        app = App()
        app.mainloop()
    except KeyboardInterrupt:
        app.destroy()
        rclpy.shutdown()
    finally:
        print("Exiting")
        rclpy.shutdown()
        exit()
