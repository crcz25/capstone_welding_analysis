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
        self.grid_columnconfigure(3, weight=1)
        self.grid_rowconfigure(4, weight=1)

        # PlotFrame intance
        self.plot_frame = self.master.plot_frame

        # Default text color
        self.color = "white"

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
        self.slider.grid(
            row=0, columnspan=4, padx=(10, 10), pady=(10, 10), sticky="nsew"
        )
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

        self.scan_button.grid(row=1, column=0, padx=10, pady=10, sticky="ew")
        self.previous_button.grid(row=1, column=1, padx=10, pady=10, sticky="ew")
        self.next_button.grid(row=1, column=2, padx=10, pady=10, sticky="ew")
        self.load_button.grid(row=1, column=3, padx=10, pady=10, sticky="w")

        # Dropdown menus
        self.filter_label = ctk.CTkLabel(self, text="Apply filter:")
        self.filter_label.grid(
            row=2, column=0, padx=(10, 10), pady=(10, 10), sticky="we"
        )
        self.filter_menu = ctk.CTkOptionMenu(
            self,
            values=["Gaussian", "Median"],
            anchor="center",
            command=self.filter_menu,
        )
        self.filter_menu.grid(row=2, column=1, padx=10, pady=10, sticky="w")
        self.filter_menu.set("Filter")

        self.export_menu = ctk.CTkOptionMenu(
            self, values=[".ply", ".npy"], anchor="center", command=self.export_menu
        )
        self.export_menu.grid(row=2, column=2, padx=10, pady=10, sticky="w")
        self.export_menu.set("Export")
        self.export_window = None

        # Console
        self.console_label = ctk.CTkLabel(self, text="Console")
        self.console_label.grid(
            row=3, column=0, padx=(10, 10), pady=(10, 10), sticky="ws"
        )

        self.console_entry = ctk.CTkTextbox(
            self, width=250, height=150, text_color=self.color
        )
        self.console_entry.grid(
            row=4,
            column=0,
            columnspan=4,
            rowspan=4,
            padx=(10, 10),
            pady=(10, 10),
            sticky="nsew",
        )

    # --------------------------------------------------------FUNCTIONALITY--------------------------------------------------------#
    def slider_event(self, other=None):
        print(f"Slider event: {self.slider_value.get()}")
        self.master.change_plot(change=0, profile=self.slider_value.get())

    def take_stop_scan(self):
        self.master.change_button_text(self.scan_button, "Take scan", "Stop scan")

    def previous_plot(self):
        self.master.change_plot(-1)

    def next_plot(self):
        self.master.change_plot(1)

    def import_scan(self):
        self.master.import_files()

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
            return None  # Do nothing if nothing selected

        return data_filtered_smoothed

    def filter_menu(self, choice):
        """
        Filter menu logic. Calls interpolate_and_filter()

        Args:
            choice: filter type.
        """
        try:
            # Get the current opened .npy
            file_path = self.master.range_file

            # Extract data
            data = np.load(file_path[0])

            self.max_frames = data.shape[0]
            self.max_profiles = data.shape[1]

            current_frame = self.master.current_frame
            profile = int(self.master.plot_control_frame.slider.get())

            print(profile, self.max_profiles)

            # Loop through frames
            if current_frame < self.max_frames:
                # Loop through profiles
                if profile < self.max_profiles:
                    frames_data = data[current_frame, profile, :]

                    data_filtered_smoothed = self.interpolate_and_filter(
                        frames_data, choice
                    )

                    if data_filtered_smoothed is not None:
                        self.plot_frame.create_line_plot_figure(
                            current_frame=current_frame,
                            profile=profile,
                            data=data_filtered_smoothed,
                            choice=choice,
                        )
        except Exception:
            self.master.change_console_text("Data is not loaded", "ERROR")

    def export_menu(self, choice):
        self.export_menu.set("Export")
        try:
            if choice == ".ply":
                # Create the window to export the point cloud, if it doesn't exist
                if self.export_window is None or not self.export_window.winfo_exists():
                    self.export_window = ExportPLYWindow(self.master)
                    # Set the focus to the window and on top of all other windows
                    self.export_window.focus()
                    self.export_window.attributes("-topmost", True)

                self.master.change_console_text("Exporting point cloud", "INFO")
                # Set the pixel size
                pixel_size = {"x": 0.1122161041015625, "y": 1.0, "z": 1.0}
                origin: {"y": 0, "x": 0.0100641, "z": 0.0}
                # Get data
                ranges = self.master.range_data[0]
                # Reshape data to be (x, 1536) where x is the number of frames
                if len(ranges.shape) == 3:
                    ranges = ranges.reshape(-1, ranges.shape[2])
                # Generate mesh grid
                mesh_x, mesh_y = np.meshgrid(
                    np.arange(0, ranges.shape[1]) * pixel_size["x"],
                    np.arange(0, ranges.shape[0]) * pixel_size["y"],
                )
                # Flatten mesh grid
                # mesh_x = mesh_x.flatten()
                # mesh_y = mesh_y.flatten()
                # ranges = ranges.flatten()
                # Create point cloud
                # xyz = np.vstack((mesh_x, mesh_y, ranges)).T
                xyz = np.zeros((np.size(mesh_x), 3))
                xyz[:, 0] = np.reshape(mesh_x, -1)
                xyz[:, 1] = np.reshape(mesh_y, -1)
                xyz[:, 2] = np.reshape(ranges, -1)
                # Create point cloud
                pcd = o3d.geometry.PointCloud()
                pcd.points = o3d.utility.Vector3dVector(xyz)
                # Export point cloud
                if choice == ".ply":
                    # Ask for the file name to save the point cloud
                    file_name = tkinter.filedialog.asksaveasfilename(
                        title="Save point cloud as",
                        filetypes=(("PLY files", "*.ply"), ("All files", "*.*")),
                    )
                    # Verify the extension is .ply
                    if not file_name.endswith(".ply"):
                        file_name += ".ply"
                    # Print the file name and path
                    self.master.change_console_text(
                        f"Saving point cloud as {file_name}", "INFO"
                    )
                    # Export point cloud
                    o3d.io.write_point_cloud(file_name, pcd)
                    # Print success message
                    self.master.change_console_text("PLY file generated", "SUCCESS")
            elif choice == ".npy":
                # Ask for the file name to save the point cloud
                file_name = tkinter.filedialog.asksaveasfilename(
                    title="Save point cloud as",
                    filetypes=(("NPY files", "*.npy"), ("All files", "*.*")),
                )
                # Verify the extension is .npy
                if not file_name.endswith(".npy"):
                    file_name += ".npy"
                # Print the file name and path
                self.master.change_console_text(
                    f"Saving point cloud as {file_name}", "INFO"
                )
                # Export point cloud
                np.save(file_name, xyz)
            else:
                return None  # Do nothing if nothing selected

        except Exception as e:
            print("Error exporting")
            print(e)
