import tkinter

import customtkinter as ctk
import numpy as np
import open3d as o3d
import pandas as pd
from scipy.ndimage import gaussian_filter, median_filter, uniform_filter1d

from .ExportPLYFrame import ExportPLYWindow


# --------------------------------------------------------SLIDER/MAIN CONTROL FRAME--------------------------------------------------------#
class PlotControlFrame(ctk.CTkFrame):
    def __init__(self, master, **kwargs):
        super().__init__(master, bg_color="transparent", **kwargs)
        self.grid(row=1, column=1, padx=(10, 10), pady=(10, 10), sticky="nsew")

        # Configure weights for columns and rows
        self.grid_columnconfigure(0, weight=1)
        self.grid_columnconfigure(1, weight=1)
        self.grid_columnconfigure(2, weight=1)
        self.grid_columnconfigure(3, weight=1)
        self.grid_rowconfigure(0, weight=1)
        self.grid_rowconfigure(1, weight=1)
        self.grid_rowconfigure(2, weight=1)
        self.grid_rowconfigure(3, weight=1)

        # Default text color
        self.color = "white"

        # Create search box
        self.searchbox_label= ctk.CTkLabel(
            self, text="Search"
        )
        self.searchbox_label.grid(
            row=1, columnspan=2, padx=(10, 10), pady=(5, 5), sticky="w")

        self.searchbox_entry = ctk.CTkTextbox(
            self,height=25, text_color=self.color, activate_scrollbars=False,
        )
        self.searchbox_entry.grid(
            row=2, columnspan=2, padx=(10, 10), pady=(5, 5), sticky="ew")
        
        # Set initial text to searchbox
        self.searchbox_entry.insert("0.0","Search by profile <integer> or timestamp <hh:mm:ss.sss>")

        # Remove the original text after you click the searchbox
        self.searchbox_entry.bind("<Button-1>", lambda e: self.on_searchbox_click())

        # Bind the search function to the search box when enter is pressed
        self.searchbox_entry.bind("<Return>", lambda e: self.search_profile())

        # Bind the function to restore the original text when the entry loses focus
        self.searchbox_entry.bind("<FocusOut>", lambda e: self.restore_searchbox_text())


        # Create Slider
        self.start_position_label = ctk.CTkLabel(self, text="1")
        self.end_position_label = ctk.CTkLabel(self, text="512")
        self.slider_value = tkinter.IntVar(master=self.master, value=0)
        self.slider = ctk.CTkSlider(
            self,
            from_=0,
            to=512,
            number_of_steps=511,
            variable=self.slider_value,
            command=self.slider_event,
        )
        self.start_position_label.grid(row=0, column=0, padx=(10, 10), pady=(10, 5), sticky="ew")
        self.slider.grid(row=0, column=1, columnspan=2, padx=(10, 10), pady=(10, 5), sticky="ew")
        self.end_position_label.grid(row=0, column=3, padx=(10, 10), pady=(10, 5), sticky="ew")
        self.slider.set(0)

        # Create main control buttons
        self.scan_button = ctk.CTkButton(
            self, text="Take scan", command=self.take_stop_scan
        )
        self.previous_button = ctk.CTkButton(
            self, text="Previous", command=self.previous_plot
        )
        self.next_button = ctk.CTkButton(self, text="Next", command=self.next_plot)
        self.load_button = ctk.CTkButton(
            self, text="Import scan", command=self.import_scan
        )
        self.clear_point_of_interest = ctk.CTkButton(
            self, text="Reset plot", command=self.reset_cursors_and_plot
        )
        self.invert_button = ctk.CTkButton(
            self, text="Invert plot", command=self.invert
        )

        # Grid buttons
        self.scan_button.grid(row=3, column=0, padx=10, pady=5, sticky="new")
        self.previous_button.grid(row=3, column=1, padx=10, pady=5, sticky="new")
        self.next_button.grid(row=3, column=2, padx=10, pady=5, sticky="new")
        self.load_button.grid(row=3, column=3, padx=10, pady=5, sticky="new")
        self.clear_point_of_interest.grid(
            row=4, column=2, padx=10, pady=5, sticky="new"
        )
        self.invert_button.grid(row=4, column=1, padx=10, pady=5, sticky="new")

        # Dropdown menus
        self.filter_menu_dropdown = ctk.CTkOptionMenu(
            self,
            values=["No Filter", "Gaussian", "Median", "Linear"],
            anchor="center",
            command=self.filter_menu,
        )
        self.filter_menu_dropdown.grid(row=4, column=0, padx=10, pady=5, sticky="new")
        self.filter_menu_dropdown.set("No Filter")

        self.export_menu_dropdown = ctk.CTkOptionMenu(
            self, values=[".ply", ".npy"], anchor="center", command=self.export_menu
        )
        self.export_menu_dropdown.grid(row=4, column=3, padx=10, pady=5, sticky="new")
        self.export_menu_dropdown.set("Export")
        self.export_window = None

        # Console
        self.console_label = ctk.CTkLabel(self, text="Console")
        self.console_label.grid(
            row=5, column=0, padx=(10, 10), pady=(5, 5), sticky="w"
        )

        self.console_entry = ctk.CTkTextbox(
            self, height=150, text_color=self.color
        )
        self.console_entry.grid(
            row=6,
            column=0,
            columnspan=4,
            rowspan=4,
            padx=(10, 10),
            pady=(5, 10),
            sticky="nsew",
        )

        # Other variables
        self.choice = None
        self.invert_plot = False
        self.og_color = self.scan_button.cget("fg_color")

        # Bindings to filter functions
        self.filter_functions = {
            "No Filter": lambda row: row,
            "Gaussian": lambda row: gaussian_filter(row, sigma=1),
            "Median": lambda row: median_filter(row, size=3),
        }

    # --------------------------------------------------------FUNCTIONALITY--------------------------------------------------------#
    def slider_event(self, other=None):
        # Set the slider to the current profile
        self.master.current_profile = self.slider_value.get()
        self.master.change_plot(change=0, profile=self.slider_value.get())

    def take_stop_scan(self):
        self.master.change_button_text(self.scan_button, "Take scan", "Stop scan")

    def previous_plot(self):
        self.master.change_plot(change=-1, profile=self.master.current_profile - 1)
        # Set the slider to the current profile
        self.slider_value.set(self.master.current_profile)

    def next_plot(self):
        self.master.change_plot(change=1, profile=self.master.current_profile + 1)
        # Set the slider to the current profile
        self.slider_value.set(self.master.current_profile)

    def import_scan(self):
        self.master.import_files()

        # Default to no filter when files are imported
        self.filter_menu("No Filter")
        self.master.plot_frame.invert_plot = False
        self.invert_button.configure(fg_color=self.og_color, text_color="white")

    def invert(self):
        # Change the flag True/False
        self.master.plot_frame.invert_plot ^= True
        # Update the surface plot
        self.master.plot_frame.update_surface(
            profile=self.master.current_profile, choice=self.choice
        )
        # Change the background color of the button to indicate the state active/inactive
        if self.master.plot_frame.invert_plot:
            self.invert_button.configure(fg_color="Light Green", text_color="black")
        else:
            self.invert_button.configure(fg_color=self.og_color, text_color="white")

    def interpolate_and_filter(self, row, choice):
        """
        Interpolate and filter.

        Args:
            choice: filter type.
            row: profile to interpolate and filter.
        """
        np.set_printoptions(threshold=np.inf)
        # Interpolate missing values
        data_filtered = pd.Series(
            np.where(row == 0, np.nan, row)
        ).interpolate().ffill().bfill()

        # Apply filters
        if choice == "Gaussian":
            data_filtered_smoothed = gaussian_filter(
                data_filtered, sigma=10, mode="nearest"
            )
        elif choice == "Median":
            data_filtered_smoothed = median_filter(
                data_filtered, size=10, mode="nearest"
            )
        elif choice == "Linear":
            # Find the indices where the weld is
            if self.master.plot_frame.invert_plot:
                i_weld = np.where(row < np.percentile(row, 90))[0]
            else:
                i_weld = np.where(row > np.percentile(row, 10))[0]
            if i_weld.size == 0:
                return data_filtered
            data_filtered_smoothed = np.interp(np.arange(0, len(row)), i_weld, row[i_weld])
        else:
            return data_filtered

        return data_filtered_smoothed

    def filter_menu(self, choice):
        """
        Filter menu logic. Calls interpolate_and_filter()

        Args:
            choice: filter type.
        """
        # Set the dropdown menu to the selected choice
        self.choice = choice
        self.master.plot_frame.update_surface(
            profile=self.master.current_profile, choice=choice
        )

    def reset_cursors_and_plot(self):
        """
        Calls plot frame reset_cursors()

        """
        # try:
        if self.choice == "No filter" or self.choice is None:
            self.master.plot_frame.reset_cursors(
                profile=self.master.current_profile,
                data=self.master.range_data,
                choice=None,
            )
        else:
            self.master.plot_frame.reset_cursors(
                profile=self.master.current_profile,
                data=self.master.range_data,
                choice=self.choice,
            )
        # except Exception:
        # self.master.change_console_text("Data is not loaded", "ERROR")

    def create_pcd_array(self, pixel_size=(1.0, 1.0, 1.0)):
        # Get data from master
        ranges = self.master.range_data
        # Reshape data to be (x, 1536) where x is the number of frames
        if len(ranges.shape) == 3:
            ranges = ranges.reshape(-1, ranges.shape[2])
            # Generate mesh grid
            mesh_x, mesh_y = np.meshgrid(
                np.arange(0, ranges.shape[1]) * pixel_size[0],
                np.arange(0, ranges.shape[0]) * pixel_size[1],
            )
        else:
            mesh_x, mesh_y = np.meshgrid(
                np.arange(0, ranges.shape[1]) * pixel_size[0],
                np.arange(0, ranges.shape[0]) * pixel_size[1],
            )
        # Create point cloud array
        xyz = np.zeros((np.size(mesh_x), 3))
        xyz[:, 0] = np.reshape(mesh_x, -1)
        xyz[:, 1] = np.reshape(mesh_y, -1)
        xyz[:, 2] = np.reshape(ranges, -1)
        return xyz

    def write_ply(self, file_name, pixel_size=(1.0, 1.0, 1.0), remove_outliers=False):
        # Create point cloud array
        xyz = self.create_pcd_array(pixel_size)
        # Create point cloud
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(xyz)
        # Remove outliers
        # if remove_outliers:
            # print("Removing outliers")
            # pcd, _ = pcd.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)
        # Write point cloud to file
        o3d.io.write_point_cloud(file_name, pcd)

    def write_npy(self, file_name, pixel_size=(1.0, 1.0, 1.0)):
        # Create point cloud array
        xyz = self.create_pcd_array(pixel_size)
        # Write point cloud to file
        np.save(file_name, xyz)

    def export_menu(self, choice):
        self.export_menu_dropdown.set("Export")
        try:
            # Check if there is data to export
            if self.master.range_data.size > 0:
                self.master.change_console_text("Exporting point cloud", "INFO")
                # Create the window to export the point cloud, if it doesn't exist
                if self.export_window is None or not self.export_window.winfo_exists():
                    self.export_window = ExportPLYWindow(self.master, choice)
                    # Set the focus to the window and on top of all other windows
                    self.export_window.focus()
                    self.export_window.grab_set()
            else:
                self.master.change_console_text("Data is not loaded", "ERROR")
        except Exception as e:
            print("Error exporting")
            print(e)

    def on_searchbox_click(self):
        # Ensure the entry widget is clicked and delete intial text
        self.searchbox_entry.focus_set()
        self.searchbox_entry.delete("1.0", ctk.END)

    def restore_searchbox_text(self):
        # Insert default text back again
        self.searchbox_entry.delete("1.0", ctk.END)
        self.searchbox_entry.insert("0.0", "Search by profile <integer> or timestamp <hh:mm:ss.sss>")

        # Ensure that the focus is not on the searchbox
        self.master.focus_set()

    def timestamp_to_seconds(self, timestamp_str):
        try:
            # Convert timestamp string to seconds
            hours, minutes, seconds = map(float, timestamp_str.split(':'))
            return hours * 3600 + minutes * 60 + seconds
        except ValueError as ve:
            print(f"Error converting timestamp: {ve}")
            return None

    def find_closest_timestamp(self, target_timestamp):
        try:
            # Find the closest timestamp from the times list
            if self.master.formatted_timestamps:
                # Convert each timestamp in the list to seconds
                formatted_timestamps_seconds = [self.timestamp_to_seconds(ts) for ts in self.master.formatted_timestamps]

                # Find the closest timestamp based on total seconds
                closest_index = min(range(len(formatted_timestamps_seconds)), key=lambda i: abs(formatted_timestamps_seconds[i] - target_timestamp))
                closest_timestamp = self.master.formatted_timestamps[closest_index]

                return closest_index,closest_timestamp

        except Exception as e:
            print(f"Error in find_closest_timestamp: {e}")
            return None
        

    def search_profile(self):
        # Get the search term from the search box
        search_term = self.searchbox_entry.get("1.0", "end-1c").strip()
        
        try:
            if self.master.range_file is not None:
                if search_term.startswith("profile"):
                    profile_number_str = search_term[len("profile"):].strip()
                    if profile_number_str.isdigit():
                        profile_number = int(profile_number_str) - 1
                        self.handle_search_result(profile_number)

                elif search_term.startswith("timestamp"):
                    timestamp_str = search_term[len("timestamp"):].strip()
                    total_seconds = self.timestamp_to_seconds(timestamp_str)

                    if total_seconds is not None:
                        closest_index, closest_timestamp = self.find_closest_timestamp(total_seconds)

                        if closest_timestamp is not None:
                            self.handle_search_result(closest_index, closest_timestamp)
                        else:
                            self.master.change_console_text("No formatted timestamps available.", "ERROR")

                else:
                    self.master.change_console_text(f"Invalid search format: {search_term}. Use 'profile <integer>' or 'timestamp <hh:mm:ss.sss>'.", "ERROR")

                # Clear the search box after processing the search
                self.restore_searchbox_text()

        except Exception as e:
            self.master.change_console_text("Data is not loaded", "ERROR")


    def handle_search_result(self, profile_number, timestamp=None):
        if 0 <= profile_number < self.master.max_profiles + 1:
            self.slider_value.set(profile_number)
            self.master.current_profile = self.slider_value.get()
            self.master.change_plot(change=0, profile=self.slider_value.get())

            if timestamp is not None:
                # Display to the user which profile was selected (closest to search)
                self.master.change_console_text(
                    f"Found the closest timestamp: {timestamp}, profile: {profile_number + 1}.", "INFORMATION"
                )
            else:
                self.master.change_console_text(
                    f"Found profile: {profile_number + 1}.", "INFORMATION"
                )
        else:
            self.master.change_console_text(
                f"Profile number out of range (1 - {self.master.max_profiles + 1} ).", "ERROR"
        )


