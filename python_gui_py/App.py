import sys
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
        ctk.set_appearance_mode(new_appearance_mode)
        self.show_default_frames()
        self.change_console_text(
            self
        )  # Make sure that text colors change when theme is changed

    def change_scaling_event(self, new_scaling: str):
        new_scaling_float = int(new_scaling.replace("%", "")) / 100
        ctk.set_widget_scaling(new_scaling_float)
        self.show_default_frames()

    def change_button_text(self, button, text_1, text_2):
        current_state = button.cget("text")
        new_state = text_2 if current_state == text_1 else text_1
        # Check if it is time to start or stop the scan
        if current_state == "Take scan":
            self.change_console_text("Taking scan", "INFORMATION")
            self.start_scan()
        elif current_state == "Stop scan":
            self.change_console_text("Stopping scan", "INFORMATION")
            self.stop_scan()

        button.configure(text=new_state)

    def change_console_text(self, text, tags=None):
        """
        Function to log text to the UI console.

        Parameters:
        - text (text to be shown).
        - tag (INFO: white, ERROR: red, SUCCESS: green)

        """
        print(text)  # Kept this to print to terminal

        ap_mode = ctk.get_appearance_mode()

        # In light mode white color is not visible in the console
        # Create tags
        if ap_mode == "Dark":
            self.plot_control_frame.console_entry.tag_config(
                "INFORMATION", foreground="white"
            )
        else:
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
        if frame_type == "home":
            self.show_default_frames()
        elif frame_type == "settings":
            self.show_camera_frames()

    def show_camera_frames(self):
        self.info_frame.grid_remove()
        self.plot_control_frame.grid_remove()
        self.plot_frame.grid_remove()
        self.settings_frame.grid()

    def show_default_frames(self):
        self.settings_frame.grid_remove()
        self.info_frame.grid()
        self.plot_frame.grid()
        self.plot_control_frame.grid()

    def reset_imported_files(self):
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

    def update_info_frame(self, new_limits=None):
        # Check if the data is loaded
        if self.range_file is not None and self.timestamp_file is not None:
            # Enable the textbox
            self.info_frame.textbox_info.configure(state="normal")
            # Delete the current text
            self.info_frame.textbox_info.delete("1.0", "end")
            # Insert the new text
            filename = self.range_file.name
            tstamp = "N/A"
            if self.current_profile < len(self.timestamp_data):
                tstamp = self.timestamp_data[self.current_profile]
            profile = self.current_profile + 1
            # Get the limits of the cursor
            x_min = self.plot_frame.cursor_limits["x_min"]
            x_max = self.plot_frame.cursor_limits["x_max"]
            y_min = self.plot_frame.cursor_limits["y_min"]
            y_max = self.plot_frame.cursor_limits["y_max"]
            if len(self.plot_frame.cursor_limits) > 0 and new_limits is not None:
                x_min = new_limits[0][0]
                x_max = new_limits[0][1]
                y_min = new_limits[1][0]
                y_max = new_limits[1][1]
            inverted = self.plot_frame.invert_plot
            curr_filter = self.plot_control_frame.choice
            template = "Scan: {}\nTime: {}\nProfile: {}/{}\nX-Min: {}\nX-Max: {}\nY-Min: {}\nY-Max: {}\nInverted: {}\nFilter: {}".format(
                filename,
                tstamp,
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
        if not self.scanning_in_progress:
            self.scanning_in_progress = True
            self.ros_node_thread = ROSNodeThread()
            self.ros_node_thread.start()
            self.change_console_text("Scan started", "INFORMATION")
        else:
            self.change_console_text("Scan did not start", "INFORMATION")

    def stop_scan(self):
        if self.scanning_in_progress:
            # Stop the node
            self.scanning_in_progress = False
            self.ros_node_thread.stop_event.set()
            self.ros_node_thread.join()
            self.change_console_text("Scan stopped", "INFORMATION")
            ## Save the data ##
            # Ask the user where to save the data
            self.csv_file_path = filedialog.askdirectory()
            self.change_console_text(f"Saving data to {self.csv_file_path}", "SUCCESS")
            self.ros_node_thread.save_data(self.csv_file_path)


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
