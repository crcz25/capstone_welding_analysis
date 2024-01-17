import tkinter

import customtkinter as ctk


class ExportPLYWindow(ctk.CTkToplevel):
    def __init__(self, master, choice, **kwargs):
        super().__init__(master, **kwargs)
        self.title("Export PLY")

        # Get the chosen option
        self.choice = choice

        # - Pixel size (x, y, z) The scale/size of the pixel in the x, y and z directions.
        # Get the pixel sizes from the settings window and apply them to the export window
        pixel_size_x, pixel_size_y, pixel_size_z = self.master.settings_frame.get_pixel_size()
        self.pixel_size_x = ctk.DoubleVar(value=pixel_size_x)
        self.pixel_size_y = ctk.DoubleVar(value=pixel_size_y)
        self.pixel_size_z = ctk.DoubleVar(value=pixel_size_z)
        # - File Path: Dialog to select the path and name of the PLY file.
        # - Export and Cancel buttons

        # Create a frame for the widgets
        self.frame = ctk.CTkFrame(self, bg_color="transparent")
        self.frame.grid(row=0, column=0, padx=(10, 10), pady=(10, 10), sticky="nsew")
        # Configure the grid
        for i in range(6):
            self.frame.grid_columnconfigure(i, weight=1)
        self.grid_rowconfigure(0, weight=1)
        self.grid_rowconfigure(1, weight=1)

        # Create subframe for pixel size
        self.pixel_size_frame = ctk.CTkFrame(self.frame, bg_color="transparent")
        self.pixel_size_frame.grid(
            row=0, column=0, padx=(10, 10), pady=(10, 10), sticky="nsew"
        )
        self.grid_columnconfigure(5, weight=1)
        self.grid_rowconfigure(1, weight=1)

        self.pixel_label = ctk.CTkLabel(self.pixel_size_frame, text="Pixel Size:")
        self.pixel_label.grid(
            row=0, column=0, padx=(10, 10), pady=(10, 10), sticky="nsew"
        )
        self.pixel_size_x_label = ctk.CTkLabel(self.pixel_size_frame, text="X:")
        self.pixel_size_x_label.grid(
            row=1, column=0, padx=(10, 10), pady=(10, 10), sticky="nsew"
        )
        self.pixel_size_x_entry = ctk.CTkEntry(
            self.pixel_size_frame, textvariable=self.pixel_size_x
        )
        self.pixel_size_x_entry.grid(
            row=1, column=1, padx=(10, 10), pady=(10, 10), sticky="nsew"
        )
        self.pixel_size_y_label = ctk.CTkLabel(self.pixel_size_frame, text="Y:")
        self.pixel_size_y_label.grid(
            row=1, column=2, padx=(10, 10), pady=(10, 10), sticky="nsew"
        )
        self.pixel_size_y_entry = ctk.CTkEntry(
            self.pixel_size_frame, textvariable=self.pixel_size_y
        )
        self.pixel_size_y_entry.grid(
            row=1, column=3, padx=(10, 10), pady=(10, 10), sticky="nsew"
        )
        self.pixel_size_z_label = ctk.CTkLabel(self.pixel_size_frame, text="Z:")
        self.pixel_size_z_label.grid(
            row=1, column=4, padx=(10, 10), pady=(10, 10), sticky="nsew"
        )
        self.pixel_size_z_entry = ctk.CTkEntry(
            self.pixel_size_frame, textvariable=self.pixel_size_z
        )
        self.pixel_size_z_entry.grid(
            row=1, column=5, padx=(10, 10), pady=(10, 10), sticky="nsew"
        )

        # Create subframe for file path and check button to export current scan
        self.file_path_frame = ctk.CTkFrame(self.frame, bg_color="transparent")
        self.file_path_frame.grid(
            row=1, column=0, padx=(10, 10), pady=(10, 10), sticky="nsew"
        )
        # Configure the grid
        self.grid_rowconfigure(0, weight=1)
        for i in range(4):
            self.file_path_frame.grid_columnconfigure(i, weight=1)

        self.file_path_label = ctk.CTkLabel(self.file_path_frame, text="File Path:")
        self.file_path_label.grid(
            row=0, column=0, padx=(10, 10), pady=(10, 10), sticky="e"
        )
        self.file_path_entry = ctk.CTkEntry(self.file_path_frame)
        self.file_path_entry.grid(
            row=0, column=1, columnspan=2, padx=(10, 10), pady=(10, 10), sticky="we"
        )
        self.file_path_button = ctk.CTkButton(
            self.file_path_frame, text="Browse", command=self.browse_file_path
        )
        self.file_path_button.grid(
            row=0, column=3, padx=(10, 10), pady=(10, 10), sticky="e"
        )
        # self.check_var = ctk.StringVar(value="off")
        # self.check_button = ctk.CTkCheckBox(
        #     self.file_path_frame,
        #     text="Export current Time Stamp",
        #     command=self.checkbox_event,
        #     variable=self.check_var,
        #     onvalue="on",
        #     offvalue="off",
        # )
        # self.check_button.grid(
        #     row=0, column=3, padx=(10, 10), pady=(10, 10), sticky="nsew"
        # )

        # Create subframe for export and cancel buttons
        self.export_cancel_frame = ctk.CTkFrame(self.frame, bg_color="transparent")
        self.export_cancel_frame.grid(
            row=2, column=0, padx=(10, 10), pady=(10, 10), sticky="nsew"
        )
        # Configure the grid
        self.export_cancel_frame.grid_rowconfigure(0, weight=1)
        for i in range(3):
            self.export_cancel_frame.grid_columnconfigure(i, weight=1)

        self.export_button = ctk.CTkButton(
            self.export_cancel_frame, text="Export", command=self.export
        )
        self.export_button.grid(
            row=0, column=2, padx=(10, 10), pady=(10, 10), sticky="e"
        )
        self.cancel_button = ctk.CTkButton(
            self.export_cancel_frame,
            text="Cancel",
            command=self.cancel,
            hover_color="DarkRed",
        )
        self.cancel_button.grid(
            row=0, column=3, padx=(10, 10), pady=(10, 10), sticky="e"
        )
        # Position the buttons to the right of the frame

        self.file_name = None

    # --------------------------------------------------------FUNCTIONALITY--------------------------------------------------------#

    def browse_file_path(self):
        # Check what option was selected if npy or ply
        if self.choice == ".ply":
            # Ask for the file name to save the point cloud
            self.file_name = tkinter.filedialog.asksaveasfilename(
                title="Save point cloud as",
                filetypes=(("PLY files", "*.ply"), ("All files", "*.*")),
            )

            # Verify the name is not empty
            if self.file_name == "":
                self.master.change_console_text("Invalid file name", "ERROR")
                return
            # Verify the extension is .ply
            if not self.file_name.endswith(".ply"):
                self.file_name += ".ply"
        elif self.choice == ".npy":
            # Ask for the file name to save the point cloud
            self.file_name = tkinter.filedialog.asksaveasfilename(
                title="Save point cloud as",
                filetypes=(("Numpy files", "*.npy"), ("All files", "*.*")),
            )
            # Verify the name is not empty
            if self.file_name == "":
                self.master.change_console_text("Invalid file name", "ERROR")
                return
            # Verify the extension is .npy
            if not self.file_name.endswith(".npy"):
                self.file_name += ".npy"
        # Update the entry widget with the file name
        self.file_path_entry.delete(0, tkinter.END)
        self.file_path_entry.insert(0, self.file_name)
        # Print the file name and path
        self.master.change_console_text(
            f"Saving point cloud as {self.file_name}", "INFO"
        )

    def export(self):
        # Check if the file name is valid
        if self.file_name is None:
            self.master.change_console_text("Invalid file name", "ERROR")
            return
        # Check if the pixel size is valid
        if (
            self.pixel_size_x.get() <= 0
            or self.pixel_size_y.get() <= 0
            or self.pixel_size_z.get() <= 0
        ):
            self.master.change_console_text("Invalid pixel size", "ERROR")
            return
        # Check what option was selected if npy or ply
        if self.choice == ".ply":
            # Export point cloud
            self.master.plot_control_frame.write_ply(
                file_name=self.file_name,
                pixel_size=(
                    self.pixel_size_x.get(),
                    self.pixel_size_y.get(),
                    self.pixel_size_z.get(),
                ),
            )
        elif self.choice == ".npy":
            # Export point cloud
            self.master.plot_control_frame.write_npy(
                file_name=self.file_name,
                pixel_size=(
                    self.pixel_size_x.get(),
                    self.pixel_size_y.get(),
                    self.pixel_size_z.get(),
                ),
            )
        # Print the file name and path
        self.master.change_console_text(
            f"Point cloud saved as {self.file_name}", "SUCCESS"
        )
        # Print the file name and path
        self.destroy()

    # def checkbox_event(self, event=None):
    #     value = self.check_var.get()
    #     print("checkbox toggled, current value:", value)

    def cancel(self):
        self.destroy()
