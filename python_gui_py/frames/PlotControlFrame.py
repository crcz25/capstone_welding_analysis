import tkinter

import customtkinter as ctk
import numpy as np
from scipy.ndimage import median_filter, gaussian_filter, uniform_filter1d
import matplotlib.pyplot as plt
import pandas as pd
import seaborn as sns

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

        # Create Slider
        self.slider_value = tkinter.IntVar(master=self.master, value=0)
        self.slider = ctk.CTkSlider(self, from_=0, to=512, number_of_steps=511, variable=self.slider_value, command=self.slider_event)
        self.slider.grid(row=0, columnspan=4, padx=(10, 10), pady=(10, 10), sticky="ew")
        self.slider.set(0)

        # Create main control buttons
        self.scan_button = ctk.CTkButton(self, text="Take scan", command=self.take_stop_scan)
        self.previous_button = ctk.CTkButton(self, text="Previous", command=self.previous_plot)
        self.next_button = ctk.CTkButton(self, text="Next", command=self.next_plot)
        self.load_button = ctk.CTkButton(self, text="Import scan", command=self.import_scan)
        self.clear_point_of_interest = ctk.CTkButton(self, text="Reset cursors", command=self.reset_cursors_and_plot)
        self.invert_button = ctk.CTkButton(self, text="Invert plot", command=self.invert)

        # Grid buttons
        self.scan_button.grid(row=1, column=0, padx=10, pady=10, sticky="new")
        self.previous_button.grid(row=1, column=1, padx=10, pady=10, sticky="new")
        self.next_button.grid(row=1, column=2, padx=10, pady=10, sticky="new")
        self.load_button.grid(row=1, column=3, padx=10, pady=10, sticky="new")
        self.clear_point_of_interest.grid(row=2, column=2, padx=10, pady=10, sticky="new")
        self.invert_button.grid(row=2, column=1, padx=10, pady=10, sticky="new")


        # Dropdown menus
        self.filter_menu_dropdown = ctk.CTkOptionMenu(self, values=["No Filter","Gaussian", "Median"], anchor="center", command=self.filter_menu)
        self.filter_menu_dropdown.grid(row=2, column=0, padx=10, pady=10, sticky="new")
        self.filter_menu_dropdown.set("No Filter")

        self.export_menu_dropdown = ctk.CTkOptionMenu(self, values=[".ply", ".npy"], anchor="center", command=self.export_menu)
        self.export_menu_dropdown.grid(row=2, column=3, padx=10, pady=10, sticky="new")
        self.export_menu_dropdown.set("Export")

        # Console
        self.console_label = ctk.CTkLabel(self, text="Console")
        self.console_label.grid(row=3, column=0, padx=(20, 10), pady=(10, 10), sticky="w")

        self.console_entry = ctk.CTkTextbox(self, width=250, height=150, text_color=self.color)
        self.console_entry.grid(row=4, column=0,columnspan=4,rowspan=4,padx=(10, 10),pady=(10, 10),sticky="nsew")

        # Other variables
        self.choice = None
               
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
        
        # Default to no filter when files are imported
        self.filter_menu("No Filter")
    
    def invert(self):
        # Change the flag True/False
        self.master.plot_frame.invert_plot ^= True
        
        # Default plot
        if self.choice == "No Filter":
            self.master.plot_frame.create_figure(self.master.current_frame, int(self.master.plot_control_frame.slider.get()), np.load(self.master.range_file[0]), self.choice)

    def interpolate_and_filter(self, first_row, choice):
        """
        Interpolate and filter.

        Args:
            choice: filter type.
            first_row: first points
        """

        np.set_printoptions(threshold=np.inf)

        # Interpolate missing values
        data_filtered = pd.Series(np.where(first_row == 0, np.nan, first_row)).interpolate()

        # Apply filters
        if choice == "Gaussian":
            data_filtered_smoothed = gaussian_filter(data_filtered, sigma=10, mode='nearest')
        elif choice == "Median":
            data_filtered_smoothed = median_filter(data_filtered, size=10, mode='nearest')
        elif choice == "No Filter":
            return None

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

            if current_frame < self.max_frames and profile < self.max_profiles:
                # Current profile
                self.frames_data = data[current_frame, profile, :]

                # If filter selected then apply filter
                if choice is not None and choice != "No Filter":
                    # Recompute frames_data based on the current choice
                    self.frames_data = self.interpolate_and_filter(self.frames_data, choice)
                else:
                    self.frames_data = data

                self.master.plot_frame.create_figure(
                    current_frame=current_frame, 
                    profile=profile, 
                    data=self.frames_data, 
                    choice=choice
                )
                self.choice = choice

        except Exception as e:
            self.master.change_console_text(f"Data is not loaded", "ERROR")


    def reset_cursors_and_plot(self):
        """
        Calls plot frame reset_cursors()

        """

        try:
            if self.choice == "No filter" or self.choice is None:
                self.master.plot_frame.reset_cursors(self.master.current_frame, int(self.master.plot_control_frame.slider.get()), np.load(self.master.range_file[0]))
            else:
                self.master.plot_frame.reset_cursors(self.master.current_frame, int(self.master.plot_control_frame.slider.get()), self.frames_data, self.choice)
        except Exception:
            self.master.change_console_text("Data is not loaded", 'ERROR')

         
    def export_menu(self, choice):
        self.export_menu.set("Export")
