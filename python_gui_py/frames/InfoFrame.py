import json
import threading

import customtkinter as ctk
from util.Defects import find_defect


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
        self.textbox_alerts = ctk.CTkTextbox(self, width=220)
        self.textbox_alerts.grid(
            row=3, column=0, padx=(10, 10), pady=(10, 10), sticky="nsew"
        )

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
        self.dropdown.grid(row=1, column=0, columnspan=2, padx=(50, 50), pady=(20, 20), sticky="nsew")
        self.dropdown.set("None")

        weld_settings_label = ctk.CTkLabel(labels_dropdown_frame, text="Weld settings")
        weld_settings_label.grid(
            row=2, column=0, columnspan=2, padx=(10, 10), pady=(10, 10), sticky="nsew"
        )

        # Settings for weld
        self.work_piece_thickness = ctk.DoubleVar(value=0.0)
        self.width_of_weld = ctk.DoubleVar(value=0.0)
        self.height_of_weld = ctk.DoubleVar(value=0.0)

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
            row=6, column=0, columnspan=2, padx=(50, 50), pady=(20, 20), sticky="we"
        )

        self.update_in_progress = False

        # For defect detection
        self.defect_choice = None
        self.template_string = "Profile: {}\nTimestamp: {}\nDefect type: {}\nHeight of weld: {} mm\n x position of weld: {} mm\n"
        # --------------------------------------------------------FUNCTIONALITY--------------------------------------------------------#

    def change_defects_found(self, choice):
        """
        Change the data in the defects panel according to what is selected from the defect type menu.

        Parameters:
        - choice (str).

        """
        self.defect_choice = choice
        if choice == "None":
            self.master.plot_frame.work_piece_thickness = 0
            self.master.plot_frame.height_of_weld = 0
            self.master.plot_frame.x_position_of_weld = 0
            self.work_piece_thickness.set(0)

    def process_defects(self):
        """
        Find defects in the weld.

        """
        try:
            print("Finding defects...")
            # Save settings for the weld
            work_piece_thickness = self.work_piece_thickness.get()
            # Validate the input (non-negative numbers)
            if work_piece_thickness < 0:
                self.master.change_console_text(
                    f"Incorrect input: Work piece thickness be non-negative numbers.",
                    "ERROR",
                )
                return

            # Get the current profile shown in the plot
            data = self.master.data_filtered
            # Get the current cursors to crop the data from the surface plot
            cursors = self.master.plot_frame.cursor_limits
            x_min = int(cursors["x_min"])
            x_max = int(cursors["x_max"])
            # Crop the data (indexes based on the Xs cursors)
            cropped_data = data[x_min:x_max]

            # Get the correct height according to the defect, if it is excessive, use maximum
            # if it is sagging, use minimum
            if self.defect_choice == "Excessive":
                # Calculate the height of the weld based on the maximum value
                height_of_weld = cropped_data.max() - work_piece_thickness
            elif self.defect_choice == "Sagging":
                # Calculate the height of the weld based on the minimum value
                height_of_weld = work_piece_thickness - cropped_data.min()
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

            # Find the x position of the height of the weld
            x_position = (
                cropped_data.argmax()
                if self.defect_choice == "Excessive"
                else cropped_data.argmin()
            )

            # Update the values in PlotFrame
            self.master.plot_frame.work_piece_thickness = work_piece_thickness
            self.master.plot_frame.height_of_weld = height_of_weld
            self.master.plot_frame.x_position_of_weld = x_position

            print(
                f"Work piece thickness: {work_piece_thickness}, height of weld: {height_of_weld}, x position of weld: {x_position}"
            )

            # Update the surface plot
            self.master.plot_frame.update_surface(
                profile=self.master.current_profile,
                choice=self.master.plot_control_frame.choice,
            )

            # Find defects
            type_found = find_defect(self.defect_choice, work_piece_thickness, height_of_weld)
            # Write in console
            self.master.change_console_text(
                    f"{type_found}", "INFORMATION"
            )

            # Format the defects found
            formatted_defects = self.template_string.format(
                self.master.current_profile+1,
                self.master.timestamp_data[self.master.current_profile],
                type_found,
                height_of_weld,
                x_position,
            )

            # Add text to the textbox
            self.textbox_alerts.configure(state="normal")
            self.textbox_alerts.delete("0.0", ctk.END)
            self.textbox_alerts.insert(ctk.END, formatted_defects)
            self.textbox_alerts.configure(state="disabled")
        except Exception as e:
            self.master.change_console_text(
                f"Verify there is data imported.", "ERROR"
            )


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
