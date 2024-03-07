import tkinter
from typing import List, Tuple

import customtkinter as ctk
import numpy as np
import open3d as o3d
import pandas as pd
from scipy.ndimage import gaussian_filter, median_filter, uniform_filter1d

from .AutoCompleteEntry import AutocompleteEntry
from .ExportFrame import ExportWindow


# --------------------------------------------------------SLIDER/MAIN CONTROL FRAME--------------------------------------------------------#
class PlotControlFrame(ctk.CTkFrame):
    """
    A class representing the plot control frame in the GUI.

    This frame contains various controls and buttons for interacting with the plot.

    Attributes:
        color (str): The default text color.
        search_type (tkinter.StringVar): The variable for the search type.
        searchbox_label (ctk.CTkLabel): The label for the search box.
        searchbox_entry (AutocompleteEntry): The entry for the search box.
        start_position_label (ctk.CTkLabel): The label for the start position of the slider.
        end_position_label (ctk.CTkLabel): The label for the end position of the slider.
        slider_value (tkinter.IntVar): The variable for the slider value.
        slider (ctk.CTkSlider): The slider control.
        checkbox_frame (ctk.CTkFrame): The frame for the action buttons.
        checkboxPoints (ctk.CTkCheckBox): The checkbox for point selection.
        checkboxInvert (ctk.CTkCheckBox): The checkbox for inverting the plot.
        scan_button (ctk.CTkButton): The button for taking or stopping a scan.
        previous_button (ctk.CTkButton): The button for navigating to the previous plot.
        next_button (ctk.CTkButton): The button for navigating to the next plot.
        load_button (ctk.CTkButton): The button for importing a scan.
        clear_point_of_interest (ctk.CTkButton): The button for resetting the plot.
        rotate_button (ctk.CTkButton): The button for aligning the plot.
        filter_menu_dropdown (ctk.CTkOptionMenu): The dropdown menu for selecting a filter.
        export_menu_dropdown (ctk.CTkOptionMenu): The dropdown menu for selecting an export option.
        console_label (ctk.CTkLabel): The label for the console.
        console_entry (ctk.CTkTextbox): The textbox for displaying the console output.
        choice (None): The choice variable.
        invert_plot (bool): The flag indicating whether the plot is inverted.
        profile_search_option (str): The search option for searching by profile.
        timestamp_search_option (str): The search option for searching by timestamp.
        selected_value (str): The currently selected search value.
        filter_functions (dict): A dictionary mapping filter names to filter functions.
    """

    def __init__(self, master, **kwargs):
        super().__init__(master, bg_color="transparent", **kwargs)
        self.grid(row=1, column=1, padx=(10, 10), pady=(10, 10), sticky="nsew")

        # Configure weights for columns and rows
        for i in range(4):
            self.grid_columnconfigure(i, weight=1)
            self.grid_rowconfigure(i, weight=1)

        # Default text color
        self.color = "white"

        # Default vars
        self.search_type = tkinter.StringVar()

        # Create the search box
        self.searchbox_label = ctk.CTkLabel(self, text="Search")
        self.searchbox_label.grid(
            row=1, columnspan=2, padx=(10, 10), pady=(5, 5), sticky="w"
        )
        self.searchbox_entry = AutocompleteEntry(
            self,
            variable=self.search_type,
            values=[
                "Search by profile <integer>",
                "Search by timestamp <hh:mm:ss.sss>",
            ],
            command=self.searchbox_selection,
        )
        self.searchbox_entry.grid(
            row=2, column=0, columnspan=2, padx=(10, 10), pady=(5, 5), sticky="ew"
        )
        self.searchbox_entry.set("Search by profile <integer>")

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
        self.start_position_label.grid(
            row=0, column=0, padx=(10, 10), pady=(10, 5), sticky="ew"
        )
        self.slider.grid(
            row=0, column=1, columnspan=2, padx=(10, 10), pady=(10, 5), sticky="ew"
        )
        self.end_position_label.grid(
            row=0, column=3, padx=(10, 10), pady=(10, 5), sticky="ew"
        )
        self.slider.set(0)

        # Create the subframe for the action buttons
        self.checkbox_frame = ctk.CTkFrame(
            self, bg_color="transparent", fg_color="transparent"
        )
        self.checkbox_frame.grid(row=1, column=1, columnspan=2, sticky="n")
        # Configure the grid
        for i in range(3):
            self.checkbox_frame.grid_columnconfigure(i, weight=1)
        self.checkbox_frame.grid_rowconfigure(0, weight=1)

        # Points
        self.pointsEnabled = ctk.StringVar(value="off")
        self.checkboxPoints = ctk.CTkCheckBox(
            self.checkbox_frame,
            text="Point Selection",
            command=self.checkbox_event_points,
            variable=self.pointsEnabled,
            onvalue="on",
            offvalue="off",
        )
        self.checkboxPoints.grid(row=0, column=0, padx=10, pady=10, sticky="new")

        # Invert
        self.invertEnabled = ctk.StringVar(value="off")
        self.checkboxInvert = ctk.CTkCheckBox(
            self.checkbox_frame,
            text="Invert Plot",
            command=self.invert,
            variable=self.invertEnabled,
            onvalue="on",
            offvalue="off",
        )
        self.checkboxInvert.grid(row=0, column=1, padx=10, pady=10, sticky="new")

        # Create the search box
        self.searchbox_label = ctk.CTkLabel(self, text="Search")
        self.searchbox_label.grid(
            row=1, columnspan=2, padx=(10, 10), pady=(5, 5), sticky="w"
        )
        self.searchbox_entry = AutocompleteEntry(
            self,
            variable=self.search_type,
            values=[
                "Search by profile <integer>",
                "Search by timestamp <hh:mm:ss.sss>",
            ],
            command=self.searchbox_selection,
        )
        self.searchbox_entry.grid(
            row=2, column=0, columnspan=2, padx=(10, 10), pady=(5, 5), sticky="ew"
        )
        self.searchbox_entry.set("Search by profile <integer>")

        # Remove the original text after you click the searchbox
        self.searchbox_entry.bind("<Button-1>", lambda e: self.on_searchbox_click())

        # Bind the search function to the search box when enter is pressed
        self.searchbox_entry.bind("<Return>", lambda e: self.search_profile())

        # Bind the function to restore the original text when the entry loses focus
        self.searchbox_entry.bind("<FocusOut>", lambda e: self.restore_searchbox_text())

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
        self.rotate_button = ctk.CTkButton(self, text="Align plot", command=self.rotate)

        # Grid buttons
        self.scan_button.grid(row=3, column=0, padx=10, pady=5, sticky="new")
        self.previous_button.grid(row=3, column=1, padx=10, pady=5, sticky="new")
        self.next_button.grid(row=3, column=2, padx=10, pady=5, sticky="new")
        self.load_button.grid(row=3, column=3, padx=10, pady=5, sticky="new")
        self.clear_point_of_interest.grid(
            row=4, column=2, padx=10, pady=5, sticky="new"
        )
        self.rotate_button.grid(row=4, column=1, padx=10, pady=5, sticky="new")

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
            self,
            values=[".ply", ".npy", "plot"],
            anchor="center",
            command=self.export_menu,
        )
        self.export_menu_dropdown.grid(row=4, column=3, padx=10, pady=5, sticky="new")
        self.export_menu_dropdown.set("Export")
        self.export_window = None

        # Console
        self.console_label = ctk.CTkLabel(self, text="Console")
        self.console_label.grid(row=5, column=0, padx=(10, 10), pady=(5, 5), sticky="w")

        self.console_entry = ctk.CTkTextbox(self, height=150, text_color=self.color)
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
        # self.og_color = self.scan_button.cget("fg_color")
        self.profile_search_option = "Search by profile <integer>"
        self.timestamp_search_option = "Search by timestamp <hh:mm:ss.sss>"
        self.selected_value = self.profile_search_option

        # Bindings to filter functions
        self.filter_functions = {
            "No Filter": lambda row: row,
            "Gaussian": lambda row: gaussian_filter(row, sigma=1),
            "Median": lambda row: median_filter(row, size=3),
        }

    def slider_event(self, other=None):
        """
        Event handler for the slider control.

        Updates the current profile and changes the plot accordingly.

        Args:
            other: Unused parameter.

        Returns:
            None
        """
        # Set the slider to the current profile
        self.master.current_profile = self.slider_value.get()
        self.master.change_plot(change=0, profile=self.slider_value.get())

    def take_stop_scan(self):
        """
        Event handler for the scan button.

        Changes the button text between "Take scan" and "Stop scan".

        Returns:
            None
        """
        self.master.change_button_text(self.scan_button, "Take scan", "Stop scan")

    def previous_plot(self):
        """
        Event handler for the previous button.

        Navigates to the previous plot and updates the slider value.

        Returns:
            None
        """
        self.master.change_plot(change=-1, profile=self.master.current_profile - 1)
        # Set the slider to the current profile
        self.slider_value.set(self.master.current_profile)
        if self.master.info_frame.defect_choice != "None":
            self.master.plot_frame.reset_guides_defects()
            self.master.info_frame.process_defects()

    def next_plot(self):
        """
        Event handler for the next button.

        Navigates to the next plot and updates the slider value.

        Returns:
            None
        """
        self.master.change_plot(change=1, profile=self.master.current_profile + 1)
        # Set the slider to the current profile
        self.slider_value.set(self.master.current_profile)
        if self.master.info_frame.defect_choice != "None":
            self.master.plot_frame.reset_guides_defects()
            self.master.info_frame.process_defects()

    def import_scan(self):
        """
        Event handler for the import button.

        Imports scan files and sets the default filter to "No Filter".

        Returns:
            None
        """
        self.master.import_files()

        # Default to no filter when files are imported
        self.filter_menu("No Filter")
        self.master.plot_frame.invert_plot = False

    def invert(self):
        """
        Event handler for the invert checkbox.

        Inverts the plot if the checkbox is checked.

        Returns:
            None
        """
        # Implement the invert functionality here
        pass
        # Change the flag True/False
        self.master.plot_frame.invert_plot ^= True
        # Update the surface plot
        self.master.plot_frame.update_surface(
            profile=self.master.current_profile, choice=self.choice
        )

    def rotate(self):
        """
        Rotate the plot by toggling the alignment of the reference line.

        If there is no reference line, the method does nothing.

        This method updates the plot surface based on the current profile and choice.

        Returns:
            None
        """
        if len(self.master.plot_frame.points) < 2:
            self.master.change_console_text(
                "Please select two points to align the plot", "ERROR"
            )
            return
        if len(self.master.plot_frame.points) > 2:
            self.master.change_console_text("Too many reference points. Please reset and select two points", "ERROR")
            return
        self.master.plot_frame.align_plot ^= True
        self.master.plot_frame.update_surface(
            profile=self.master.current_profile, choice=self.choice
        )

    def interpolate_and_filter(self, row, choice):
        """
        Interpolate and filter.

        Args:
            choice: filter type.
            row: profile to interpolate and filter.
        """
        np.set_printoptions(threshold=np.inf)
        # Interpolate missing values
        data_filtered = pd.Series(np.where(row == 0, np.nan, row)).ffill().bfill()
        # Convert to numpy array
        data_filtered = data_filtered.to_numpy()

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
            data_filtered_smoothed = np.interp(
                np.arange(0, len(row)), i_weld, row[i_weld]
            )
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
        """
        Create a point cloud array from range data.

        Args:
            pixel_size (tuple, optional): The size of each pixel in the x, y, and z dimensions. Defaults to (1.0, 1.0, 1.0).

        Returns:
            numpy.ndarray: The point cloud array with shape (N, 3), where N is the total number of points.
        """
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
        """
        Writes a point cloud to a PLY file.

        Args:
            file_name (str): The name of the output PLY file.
            pixel_size (tuple, optional): The pixel size in each dimension. Defaults to (1.0, 1.0, 1.0).
            remove_outliers (bool, optional): Whether to remove outliers from the point cloud. Defaults to False. Not implemented.
        """
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
        """
        Write a NumPy array representing a point cloud to a file in .npy format.

        Args:
            file_name (str): The name of the file to save the point cloud to.
            pixel_size (tuple, optional): The size of each pixel in the point cloud. Defaults to (1.0, 1.0, 1.0).
        """
        # Create point cloud array
        xyz = self.create_pcd_array(pixel_size)
        # Write point cloud to file
        np.save(file_name, xyz)

    def export_menu(self, choice):
        """
        Handles the export menu option.

        Args:
            choice (str): The export option chosen by the user.

        Returns:
            None
        """
        self.export_menu_dropdown.set("Export")
        try:
            # Check if there is data to export
            if self.master.range_data.size > 0:
                self.master.change_console_text("Exporting point cloud", "INFO")
                # Create the window to export the point cloud, if it doesn't exist
                if self.export_window is None or not self.export_window.winfo_exists():
                    self.export_window = ExportWindow(self.master, choice)
                    # Set the focus to the window and on top of all other windows
                    self.export_window.focus()
                    self.export_window.grab_set()
            else:
                self.master.change_console_text("Data is not loaded", "ERROR")
        except Exception as e:
            print("Error exporting")
            print(e)

    def searchbox_selection(self, choice):
        """
        Handles the selection made in the searchbox.

        Parameters:
        choice (str): The selected value from the searchbox.

        Returns:
        None
        """
        selected_value = self.searchbox_entry.get()
        self.selected_value = selected_value

        if self.selected_value == self.profile_search_option:
            self.master.change_console_text("Searching by profile", "INFO")
        elif self.selected_value == self.timestamp_search_option:
            self.master.change_console_text("Searching by timestamps", "INFO")

    def on_searchbox_click(self):
        """
        Handles the click event of the search box.

        Sets the default values for the search box based on the selected option.
        """
        if self.selected_value == self.profile_search_option:
            self.searchbox_entry.set("profile 1")
        elif self.selected_value == self.timestamp_search_option:
            self.searchbox_entry.set("timestamp 00:00:00.000")

    def restore_searchbox_text(self):
        """
        Restores the original text in the search box based on the selected value.

        If the selected value is the timestamp search option, the search box text will be set to "Search by timestamp <hh:mm:ss.sss>".
        Otherwise, the search box text will be set to "Search by profile <integer>".
        """
        if self.selected_value == self.timestamp_search_option:
            self.searchbox_entry.set("Search by timestamp <hh:mm:ss.sss>")
        else:
            self.searchbox_entry.set("Search by profile <integer>")

    def timestamp_to_seconds(self, timestamp_str):
        """
        Convert a timestamp string to seconds.

        Args:
            timestamp_str (str): The timestamp string in the format "HH:MM:SS".

        Returns:
            float: The equivalent number of seconds.

        Raises:
            ValueError: If the timestamp string is not in the correct format.

        """
        try:
            hours, minutes, seconds = map(float, timestamp_str.split(":"))
            return hours * 3600 + minutes * 60 + seconds
        except ValueError as ve:
            print(f"Error converting timestamp: {ve}")
            return None

    def find_closest_timestamp(self, target_timestamp):
        """
        Finds the closest timestamp to the target timestamp from the list of formatted timestamps.

        Args:
            target_timestamp (float): The target timestamp to find the closest match for.

        Returns:
            tuple: A tuple containing the index of the closest timestamp and the closest timestamp itself.
                   Returns None if an error occurs.
        """
        try:
            # Find the closest timestamp from the times list
            if self.master.formatted_timestamps:
                # Convert each timestamp in the list to seconds
                formatted_timestamps_seconds = [
                    self.timestamp_to_seconds(ts)
                    for ts in self.master.formatted_timestamps
                ]

                # Find the closest timestamp based on total seconds
                closest_index = min(
                    range(len(formatted_timestamps_seconds)),
                    key=lambda i: abs(
                        formatted_timestamps_seconds[i] - target_timestamp
                    ),
                )
                closest_timestamp = self.master.formatted_timestamps[closest_index]

                return closest_index, closest_timestamp

        except Exception as e:
            print(f"Error in find_closest_timestamp: {e}")
            return None

    def search_profile(self):
        """
        Searches for a profile or timestamp based on the selected search option.

        The search term is obtained from the search box and processed based on the selected search option.
        If the search term starts with "profile" and the selected search option is "profile", it extracts the profile number
        and calls the handle_search_result method with the profile number as an argument.

        If the search term starts with "timestamp" and the selected search option is "timestamp", it converts the timestamp
        to total seconds and finds the closest timestamp in the data. Then, it calls the handle_search_result method with
        the closest index and closest timestamp as arguments.

        If the search term does not match the expected format or the selected search option is invalid, it displays an error message.

        After processing the search, it clears the search box.

        Raises:
            Exception: If the data is not loaded.
        """
        # Get the search term from the search box
        search_term = self.searchbox_entry.get().strip()

        try:
            if self.master.range_file is not None:
                if (
                    search_term.startswith("profile")
                    and self.selected_value == self.profile_search_option
                ):
                    profile_number_str = search_term[len("profile") :].strip()
                    if profile_number_str.isdigit():
                        profile_number = int(profile_number_str) - 1
                        self.handle_search_result(profile_number)

                elif (
                    search_term.startswith("timestamp")
                    and self.selected_value == self.timestamp_search_option
                ):
                    timestamp_str = search_term[len("timestamp") :].strip()
                    total_seconds = self.timestamp_to_seconds(timestamp_str)

                    if total_seconds is not None:
                        closest_index, closest_timestamp = self.find_closest_timestamp(
                            total_seconds
                        )

                        if closest_timestamp is not None:
                            self.handle_search_result(closest_index, closest_timestamp)
                        else:
                            self.master.change_console_text(
                                "No formatted timestamps available.", "ERROR"
                            )

                else:
                    self.master.change_console_text(
                        f"Invalid search format: {search_term} or search option. Use 'profile <integer>' or 'timestamp <hh:mm:ss.sss>'.",
                        "ERROR",
                    )

                # Clear the search box after processing the search
                self.restore_searchbox_text()

        except Exception as e:
            self.master.change_console_text("Data is not loaded", "ERROR")

    def handle_search_result(self, profile_number, timestamp=None):
        """
        Handles the search result by updating the GUI elements and displaying the selected profile information.

        Args:
            profile_number (int): The profile number to be displayed.
            timestamp (str, optional): The timestamp of the selected profile. Defaults to None.
        """
        if 0 <= profile_number < self.master.max_profiles + 1:
            self.slider_value.set(profile_number)
            self.master.current_profile = self.slider_value.get()
            self.master.change_plot(change=0, profile=self.slider_value.get())

            if timestamp is not None:
                # Display to the user which profile was selected (closest to search)
                self.master.change_console_text(
                    f"Found the closest timestamp: {timestamp}, profile: {profile_number + 1}.",
                    "INFORMATION",
                )
            else:
                self.master.change_console_text(
                    f"Found profile: {profile_number + 1}.", "INFORMATION"
                )
        else:
            self.master.change_console_text(
                f"Profile number out of range (1 - {self.master.max_profiles + 1} ).",
                "ERROR",
            )

    def checkbox_event_points(self):
        """
        Toggles the visibility of points on the plot.

        This method updates the `pointsEnabled` attribute of the `plot_frame` object
        and calls the `update_surface` method to refresh the plot with the updated settings.

        Args:
            self: The current instance of the `PlotControlFrame` class.

        Returns:
            None
        """
        self.master.plot_frame.pointsEnabled ^= True
        if len(self.master.plot_frame.points) > 0:
            self.master.plot_frame.points = []
        self.master.plot_frame.update_surface(
            profile=self.master.current_profile, choice=self.choice
        )


    def lower_data(
        self, data: List, point1: Tuple[float, float], point2: Tuple[float, float]
    ) -> np.array:
        """
        Lowers the y-values of the data.

        Points 1 and 2 form a line and that line is the new height 0 (new x-axis).

        Args:
            data: Data to lower.
            point1: (x, y).
            point2: (x, y).

        Returns:
            Lowered data.
        """
        x1 = point1[0]
        y1 = point1[1]
        x2 = point2[0]
        y2 = point2[1]

        if x2 == x1:
            return np.array(data)

        slope = (y2 - y1) / (x2 - x1)
        intercept = y1 - slope * x1

        lowered_data = np.array(data) - (np.arange(len(data)) * slope + intercept)
        return lowered_data
