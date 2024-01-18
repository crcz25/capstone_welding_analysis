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
            row=0, columnspan=4, padx=(10, 10), pady=(10, 5), sticky="w")

        self.searchbox_entry = ctk.CTkTextbox(
            self,height=25, text_color=self.color, activate_scrollbars=False,
        )
        self.searchbox_entry.grid(
            row=1, columnspan=2, padx=(10, 10), pady=(5, 5), sticky="ew")
        
        # Set initial text to searchbox
        self.searchbox_entry.insert("0.0","Search by timestamp or profile...")

        # Remove the original text after you click the searchbox
        self.searchbox_entry.bind("<Button-1>", lambda e: self.on_searchbox_click())

        # Bind the search function to the search box when enter is pressed
        self.searchbox_entry.bind("<Return>", lambda e: self.search_profile())

        # Bind the function to restore the original text when the entry loses focus
        self.searchbox_entry.bind("<FocusOut>", lambda e: self.restore_searchbox_text())


        # Create Slider
        self.slider_value = tkinter.IntVar(master=self.master, value=0)
        self.slider = ctk.CTkSlider(
            self,
            from_=0,
            to=512,
            number_of_steps=511,
            variable=self.slider_value,
            command=self.slider_event,
        )
        self.slider.grid(row=2, columnspan=4, padx=(10, 10), pady=(5, 5), sticky="ew")
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
            values=["No Filter", "Gaussian", "Median"],
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
        self.master.plot_frame.update_surface(
            profile=self.master.current_profile, choice=self.choice
        )
        # Change the background color of the button to indicate the state active/inactive
        if self.master.plot_frame.invert_plot:
            self.invert_button.configure(fg_color="Light Green", text_color="black")
        else:
            self.invert_button.configure(fg_color=self.og_color, text_color="white")

    def interpolate_and_filter(self, first_row, choice):
        """
        Interpolate and filter.

        Args:
            choice: filter type.
            first_row: first points
        """
        np.set_printoptions(threshold=np.inf)

        # Interpolate missing values
        data_filtered = pd.Series(
            np.where(first_row == 0, np.nan, first_row)
        ).interpolate()

        # Apply filters
        if choice == "Gaussian":
            data_filtered_smoothed = gaussian_filter(
                data_filtered, sigma=10, mode="nearest"
            )
        elif choice == "Median":
            data_filtered_smoothed = median_filter(
                data_filtered, size=10, mode="nearest"
            )
        else:
            return first_row

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
        # Create point cloud array
        xyz = np.zeros((np.size(mesh_x), 3))
        xyz[:, 0] = np.reshape(mesh_x, -1)
        xyz[:, 1] = np.reshape(mesh_y, -1)
        xyz[:, 2] = np.reshape(ranges, -1)
        return xyz

    def write_ply(self, file_name, pixel_size=(1.0, 1.0, 1.0)):
        # Create point cloud array
        xyz = self.create_pcd_array(pixel_size)
        # Create point cloud
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(xyz)
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
        self.searchbox_entry.insert("0.0", "Search by timestamp or profile...")

        # Ensure that the focus is not on the searchbox
        self.master.focus_set()

    def find_closest_timestamp_index(self, target_timestamp):
        # Find the closest timestamp index from the times list
        if self.master.profile_times:
            closest_index = min(range(len(self.master.profile_times)), key=lambda i: abs(self.master.profile_times[i] - target_timestamp))
            closest_timestamp = min(self.master.profile_times, key=lambda x: abs(x - target_timestamp))
            return closest_index, closest_timestamp
        else:
            return None
        
    def search_profile(self):
        # Get the search term from the search box
        search_term = self.searchbox_entry.get("1.0", "end-1c").strip()
        try:
            # Check if the search term is numeric int (profile number)
            if search_term.isdigit() and self.master.range_file is not None:
                profile_number = int(search_term) - 1

                # Check that the given profile number is in the range of profiles
                if 0 <= profile_number < self.master.max_profiles + 1:
                    # Searching by profile number --> set the slider and change plot
                    self.slider_value.set(profile_number)
                    self.master.current_profile = self.slider_value.get()
                    self.master.change_plot(change=0, profile=self.slider_value.get())
                    self.master.change_console_text(
                        f"Found profile: {self.slider_value.get() + 1}.", "INFORMATION"
                    )
                else:
                    self.master.change_console_text(
                        f"Profile number out of range (1 - {self.master.max_profiles + 1} ).", "ERROR"
                    )

            # Check if the search term is numeric float (ms)   
            elif search_term.replace('.', '', 1).isdigit() and self.master.range_file is not None:
                # Check if the search term is a float
                timestamp = float(search_term)
                
                # Find the closest timestamp in the list
                closest_timestamp_index, closest_timestamp = self.find_closest_timestamp_index(timestamp)
                self.slider_value.set(closest_timestamp_index)
                self.master.current_profile = self.slider_value.get()
                self.master.change_plot(change=0, profile=self.slider_value.get())

                # Display to the user which profile was selected (closest to search)
                self.master.change_console_text(
                    f"Found the closest timestamp: {closest_timestamp}, profile: {closest_timestamp_index + 1}.", "INFORMATION"
                    )

            else:
                self.master.change_console_text(f"Could not find profile or timestamp: {search_term}, try again.", "ERROR")

            # Clear the search box after processing the search
            self.restore_searchbox_text()

        except Exception:
            self.master.change_console_text(f"Data is not loaded", "ERROR")

