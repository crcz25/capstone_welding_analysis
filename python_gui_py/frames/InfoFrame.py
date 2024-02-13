import datetime
import json
import threading
from pathlib import Path

import customtkinter as ctk
from util.Defects import WeldDefectsReport, find_defect


# --------------------------------------------------------RIGHT INFO/DEFECTS FRAME--------------------------------------------------------#
class InfoFrame(ctk.CTkFrame):
    def __init__(self, master, **kwargs):
        super().__init__(master, corner_radius=0, **kwargs)

        # Create 2 textboxes for info/defects
        self.grid(row=0, column=2, rowspan=4, padx=(0, 0), pady=(0, 0), sticky="nsew")
        self.grid_columnconfigure(2, weight=1)
        self.grid_rowconfigure(4, weight=1)

        self.scaling_label_info = ctk.CTkLabel(self, text="Information", anchor="w")
        self.scaling_label_info.grid(row=0, column=0, padx=10, pady=(10, 0))
        self.textbox_info = ctk.CTkTextbox(self, width=220)
        self.textbox_info.grid(
            row=1, column=0, padx=(10, 10), pady=(10, 0), sticky="nsew"
        )

        self.scaling_label_alerts = ctk.CTkLabel(self, text="Defects", anchor="w")
        self.scaling_label_alerts.grid(row=2, column=0, padx=10, pady=(10, 0))
        self.textbox_alerts = ctk.CTkTextbox(self, width=220, activate_scrollbars=False)
        self.textbox_alerts.grid(
            row=3, column=0, padx=(10, 10), pady=(10, 10), sticky="nsew"
        )
        # create CTk scrollbar
        ctk_textbox_scrollbar = ctk.CTkScrollbar(
            self, command=self.textbox_alerts.yview
        )
        ctk_textbox_scrollbar.grid(row=3, column=1, sticky="ns")
        # connect textbox scroll event to CTk scrollbar
        self.textbox_alerts.configure(yscrollcommand=ctk_textbox_scrollbar.set)

        # Defects
        labels_dropdown_frame = ctk.CTkFrame(self, bg_color="transparent")
        labels_dropdown_frame.grid(
            row=4, column=0, padx=(10, 10), pady=(10, 10), sticky="nsew"
        )

        self.data = self.open_json_file("python_gui_py\data\defects.json")

        # Initialize defects_found_text
        self.defects_found_text = ctk.StringVar()

        # Labels
        defect_settings_label = ctk.CTkLabel(
            labels_dropdown_frame, text="Defect settings"
        )
        defect_settings_label.grid(
            row=0, column=0, columnspan=2, padx=(10, 10), pady=(10, 10), sticky="nsew"
        )
        # Dropdown for defect type
        self.dropdown = ctk.CTkOptionMenu(
            labels_dropdown_frame,
            values=["None", "Excessive", "Sagging"],
            anchor="center",
            command=self.change_defects_found,
        )
        self.dropdown.grid(
            row=1, column=0, columnspan=2, padx=(10, 10), pady=(10, 10), sticky="ns"
        )
        self.dropdown.set("None")

        weld_settings_label = ctk.CTkLabel(labels_dropdown_frame, text="Weld settings")
        weld_settings_label.grid(
            row=2, column=0, columnspan=2, padx=(10, 10), pady=(10, 10), sticky="nsew"
        )

        # Settings for weld
        self.work_piece_thickness = ctk.DoubleVar(value=0.0)
        self.height_of_weld = ctk.DoubleVar(value=0.0)
        self.x_position_of_weld = ctk.DoubleVar(value=0.0)

        # Create labels and entry widgets
        work_piece_thickness_label = ctk.CTkLabel(
            labels_dropdown_frame, text="Work Piece Thickness (mm):"
        )
        work_piece_thickness_label.grid(
            row=3, column=0, padx=(10, 10), pady=(5, 5), sticky="we"
        )

        work_piece_thickness_entry = ctk.CTkEntry(
            labels_dropdown_frame, textvariable=self.work_piece_thickness
        )
        work_piece_thickness_entry.grid(
            row=3, column=1, padx=(10, 10), pady=(5, 5), sticky="we"
        )

        # width_of_weld_label = ctk.CTkLabel(labels_dropdown_frame, text="Width of Weld (mm):")
        # width_of_weld_label.grid(row=4, column=0, padx=(10,10), pady=(5,5), sticky="we")

        # width_of_weld_entry = ctk.CTkEntry(labels_dropdown_frame, textvariable=self.width_of_weld)
        # width_of_weld_entry.grid(row=4, column=1, padx=(10,10), pady=(5,5), sticky="we")

        # height_of_weld_label = ctk.CTkLabel(labels_dropdown_frame, text="Height of Weld (mm):")
        # height_of_weld_label.grid(row=5, column=0, padx=(10,10), pady=(5,5), sticky="we")

        # height_of_weld_entry = ctk.CTkEntry(labels_dropdown_frame, textvariable=self.height_of_weld)
        # height_of_weld_entry.grid(row=5, column=1, padx=(10,10), pady=(5,5), sticky="we")

        # Button to find defects
        find_defects_button = ctk.CTkButton(
            labels_dropdown_frame, text="Find defects", command=self.process_defects
        )
        find_defects_button.grid(
            row=6, column=0, padx=(50, 50), pady=(20, 20), sticky="we"
        )

        # Button to process all
        process_all_button = ctk.CTkButton(
            labels_dropdown_frame,
            text="Find all defects",
            command=self.process_all_defects,
        )
        process_all_button.grid(
            row=6, column=1, padx=(50, 50), pady=(20, 20), sticky="we"
        )

        self.update_in_progress = False

        # For defect detection
        self.defect_choice = "None"
        self.template_string = "Profile: {}\nTimestamp: {}\nDefect type: {} class\nHeight of weld: {} mm\n x position of weld: {} mm\n"
        self.export_path = Path(__file__).parent.parent / "data"
        # --------------------------------------------------------FUNCTIONALITY--------------------------------------------------------#

    def change_defects_found(self, choice):
        """
        Change the data in the defects panel according to what is selected from the defect type menu.

        Parameters:
        - choice (str).

        """
        self.defect_choice = choice
        if choice == "None":
            self.work_piece_thickness.set(0)
            self.master.plot_frame.reset_guides_defects()

    def find_height(self, profile, x_min, x_max, work_piece_thickness, defect_choice):
        try:
            # Crop the data (indexes based on the Xs cursors)
            cropped_data = profile[x_min:x_max]

            # Get the correct height according to the defect, if it is excessive, use maximum
            # if it is sagging, use minimum
            if defect_choice == "Excessive":
                # Calculate the height of the weld based on the maximum value
                height_of_weld = cropped_data.max() - work_piece_thickness
                x_position = cropped_data.argmax()
            elif defect_choice == "Sagging":
                # Calculate the height of the weld based on the minimum value
                height_of_weld = work_piece_thickness - cropped_data.min()
                x_position = cropped_data.argmin()
            else:
                self.master.change_console_text(
                    f"Incorrect input: Verify the dimensions to be evaluated.", "ERROR"
                )
                return

            # Before evaluating the defect, check if the height of the weld is non-negative
            if height_of_weld < 0:
                self.master.change_console_text(
                    f"Incorrect input: The height of the weld must be non-negative. Verify the dimensions to be evaluated..",
                    "ERROR",
                )
                return

            return height_of_weld, x_position
        except Exception as e:
            self.master.change_console_text(f"Verify there is data imported.", "ERROR")
            print(e)

    def process_defects(self):
        """
        Find defects in current weld.

        """
        print("Finding defects...")
        # Save settings for the weld
        work_piece_thickness = self.work_piece_thickness.get()
        # Validate the input (non-negative numbers)
        if work_piece_thickness < 0:
            self.master.change_console_text(
                f"Incorrect input: Work piece thickness must be non-negative and greater than zero.",
                "ERROR",
            )
            return

        # Get the current profile shown in the plot
        data = self.master.data_filtered.values
        # Get the current cursors to crop the data from the surface plot
        cursors = self.master.plot_frame.cursor_limits
        x_min = int(cursors["x_min"])
        x_max = int(cursors["x_max"])

        # Find the defect
        height_of_weld, x_position = self.find_height(
            data, x_min, x_max, work_piece_thickness, self.defect_choice
        )

        # Update the values in PlotFrame
        self.height_of_weld.set(height_of_weld)
        self.work_piece_thickness.set(work_piece_thickness)
        self.x_position_of_weld.set(x_position)

        # Find defects
        type_found = find_defect(
            self.defect_choice, work_piece_thickness, height_of_weld
        )

        # Write in console
        self.master.change_console_text(f"{type_found}", "INFORMATION")

        # Format the defects found
        formatted_defects = self.template_string.format(
            self.master.current_profile + 1,
            self.master.timestamp_data[self.master.current_profile],
            f"{self.defect_choice} - {type_found}",
            height_of_weld,
            x_position,
        )

        # Add text to the textbox
        self.textbox_alerts.configure(state="normal")
        self.textbox_alerts.delete("0.0", ctk.END)
        self.textbox_alerts.insert(ctk.END, formatted_defects)
        self.textbox_alerts.configure(state="disabled")

        # Update the surface plot
        self.master.plot_frame.update_surface(
            profile=self.master.current_profile,
            choice=self.master.plot_control_frame.choice,
        )

    def process_all_defects(self):
        """
        Find all defects in the ranges.
        """
        print("Finding all defects...")
        try:
            # Save settings for the weld
            work_piece_thickness = self.work_piece_thickness.get()
            # Validate the input (non-negative numbers)
            if work_piece_thickness < 0:
                self.master.change_console_text(
                    f"Incorrect input: Work piece thickness must be non-negative and greater than zero.",
                    "ERROR",
                )
                return

            ranges = self.master.range_data
            timestamps = self.master.timestamp_data
            filter = self.master.plot_control_frame.choice
            report = WeldDefectsReport()
            for idx, (range, timestamp) in enumerate(zip(ranges, timestamps)):
                filtered_data = self.master.plot_frame.apply_filter(range, filter).values
                # Find the defect
                height_of_weld, x_position = self.find_height(
                    filtered_data,
                    0,
                    ranges.shape[1],
                    work_piece_thickness,
                    self.defect_choice,
                )
                print(f"Work piece thickness: {work_piece_thickness}, height of weld: {height_of_weld}, x position of weld: {x_position}")
                # Find defects
                type_found = find_defect(
                    self.defect_choice, work_piece_thickness, height_of_weld
                )
                report.add_defect(self.defect_choice, idx, timestamp, type_found, height_of_weld)
            json_output = report.serialize()
            print(json_output)
            # Save the report to a file
            curr_datetime = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
            filename = f"defects_report_{curr_datetime}.json"
            with open(self.export_path / filename, "w") as file:
                file.write(json_output)
        except Exception as e:
            self.master.change_console_text(f"Verify there is data imported.", "ERROR")
            print(e)

    def open_json_file(self, file_path):
        # TODO: Check if you can actually display something or not (display only after analysis done)
        """
        Open and read a JSON file.

        Parameters:
        - file_path (str): The path to the JSON file.

        Returns:
        - dict: The parsed JSON data.
        """
        try:
            with open(file_path, "r") as file:
                json_data = json.load(file)
            return json_data

        except FileNotFoundError:
            print(f"Error: File not found at path '{file_path}'.")

        except json.JSONDecodeError as e:
            print(f"Error decoding JSON: {e}")

        except Exception as e:
            print(f"An unexpected error occurred: {e}")

        if json_data:
            print("JSON file successfully loaded:")
            print(json_data)
