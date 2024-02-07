import json
import threading

import customtkinter as ctk


#--------------------------------------------------------RIGHT INFO/DEFECTS FRAME--------------------------------------------------------#
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
        self.textbox_info.grid(row=1, column=0, padx=(10, 10), pady=(10, 0), sticky="nsew")

        self.scaling_label_alerts = ctk.CTkLabel(self, text="Defects", anchor="w")
        self.scaling_label_alerts.grid(row=2, column=0, padx=10, pady=(10, 0))
        self.textbox_alerts = ctk.CTkTextbox(self, width=220)
        self.textbox_alerts.grid(row=3, column=0, padx=(10, 10), pady=(10, 10), sticky="nsew")

        # Defects
        labels_dropdown_frame = ctk.CTkFrame(self, bg_color="transparent")
        labels_dropdown_frame.grid(row=4, column=0, padx=(10, 10), pady=(10, 10), sticky="nsew")

        self.data = self.open_json_file("python_gui_py\data\defects.json")

        # Initialize defects_found_text
        self.defects_found_text = ctk.StringVar()

        # Labels
        defect_settings_label = ctk.CTkLabel(labels_dropdown_frame, text="Defect settings")
        defect_settings_label.grid(row=0, column=0, columnspan=2, padx=(10, 10), pady=(10, 10), sticky="nsew")

        weld_settings_label = ctk.CTkLabel(labels_dropdown_frame, text="Weld settings")
        weld_settings_label.grid(row=2, column=0, columnspan=2, padx=(10, 10), pady=(10, 10), sticky="nsew")

        # Settings for weld
        self.work_piece_thickness = ctk.DoubleVar(value=0.0)
        self.width_of_weld = ctk.DoubleVar(value=0.0)
        self.height_of_weld = ctk.DoubleVar(value=0.0)

        # Create labels and entry widgets
        work_piece_thickness_label = ctk.CTkLabel(labels_dropdown_frame, text="Work Piece Thickness (mm):")
        work_piece_thickness_label.grid(row=3, column=0, padx=(10,10) , pady=(5,5), sticky="we")

        work_piece_thickness_entry = ctk.CTkEntry(labels_dropdown_frame, textvariable=self.work_piece_thickness)
        work_piece_thickness_entry.grid(row=3, column=1, padx=(10,10), pady=(5,5), sticky="we")

        width_of_weld_label = ctk.CTkLabel(labels_dropdown_frame, text="Width of Weld (mm):")
        width_of_weld_label.grid(row=4, column=0, padx=(10,10), pady=(5,5), sticky="we")

        width_of_weld_entry = ctk.CTkEntry(labels_dropdown_frame, textvariable=self.width_of_weld)
        width_of_weld_entry.grid(row=4, column=1, padx=(10,10), pady=(5,5), sticky="we")

        height_of_weld_label = ctk.CTkLabel(labels_dropdown_frame, text="Height of Weld (mm):")
        height_of_weld_label.grid(row=5, column=0, padx=(10,10), pady=(5,5), sticky="we")

        height_of_weld_entry = ctk.CTkEntry(labels_dropdown_frame, textvariable=self.height_of_weld)
        height_of_weld_entry.grid(row=5, column=1, padx=(10,10), pady=(5,5), sticky="we")

        # Button to find defects
        find_defects_button = ctk.CTkButton(labels_dropdown_frame, text="Find defects", command=self.find_defects)
        find_defects_button.grid(row=6, column=0, columnspan=2, padx=(50,50), pady=(20,20), sticky="we")

        self.update_in_progress = False

        # --------------------------------------------------------FUNCTIONALITY--------------------------------------------------------#

        def change_defects_found(choice):
            """
            Change the data in the defects panel according to what is selected from the defect type menu.

            Parameters:
            - choice (str).

            """
            for defect_category in self.data.get("weld_defects", []):
                for defect in defect_category.get("defects", []):
                    if defect.get("name") == choice:
                        defects_found = defect.get("defects_found", [])
            
            # Format the read JSON data
            formatted_defects = "\n".join([f"ID: {defect['id']}\nTimestamp: {defect['timestamp']}\n" for defect in defects_found])

            # Set data to the textbox
            self.textbox_alerts.configure(state="normal")
            self.textbox_alerts.delete("0.0", ctk.END)
            self.textbox_alerts.insert(ctk.END, formatted_defects)
            self.textbox_alerts.configure(state="disabled")

        # Generate labels and options from the given data
        for i, defect_category in enumerate(self.data.get("weld_defects", [])):
            label_text = defect_category.get("type", "")
            label = ctk.CTkLabel(labels_dropdown_frame, text=label_text, anchor="w")
            label.grid(row=i+1, column=0, padx=10, pady=(10, 10))

            options_data = [defect["name"] for defect in defect_category.get("defects", [])]
            self.dropdown_var = ctk.StringVar(labels_dropdown_frame)

            dropdown = ctk.CTkComboBox(labels_dropdown_frame, state="readonly", values=options_data, command=change_defects_found)
            dropdown['textvariable'] = self.dropdown_var
            dropdown.grid(row=i+1, column=1, padx=(10, 10), pady=(10, 10), sticky="w")
    
    
    def find_defects(self):
        """
        Find defects in the weld.

        """
        print("Finding defects...")
        # Save settings for the weld
        work_piece_thickness = self.work_piece_thickness.get()
        width_of_weld = self.width_of_weld.get()
        height_of_weld = self.height_of_weld.get()
        # Validate the input (non-negative numbers)
        if work_piece_thickness < 0 or width_of_weld < 0 or height_of_weld < 0:
            self.master.change_console_text(
                f"Incorrect input: Work piece thickness, width of weld and height of weld must be non-negative numbers.", "ERROR"
            )
            return
        # Update the values in PlotFrame
        self.master.plot_frame.work_piece_thickness = work_piece_thickness
        self.master.plot_frame.width_of_weld = width_of_weld
        self.master.plot_frame.height_of_weld = height_of_weld
        # Update the surface plot
        self.master.plot_frame.update_surface(
            profile=self.master.current_profile, choice=self.master.plot_control_frame.choice
        )

    def open_json_file(self, file_path):
        #TODO: Check if you can actually display something or not (display only after analysis done)
        """
        Open and read a JSON file.

        Parameters:
        - file_path (str): The path to the JSON file.

        Returns:
        - dict: The parsed JSON data.
        """
        try:
            with open(file_path, 'r') as file:
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