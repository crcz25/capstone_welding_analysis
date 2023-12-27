import tkinter

import customtkinter as ctk


class ExportPLYWindow(ctk.CTkToplevel):
    def __init__(self, master, **kwargs):
        super().__init__(master, **kwargs)
        self.title("Export PLY")
        # self.geometry("400x300")
        # self.resizable(False, False)

        # - Pixel size (x, y, z) The scale/size of the pixel in the x, y and z directions.
        self.pixel_size_x = ctk.DoubleVar(value=0.1122161041015625)
        self.pixel_size_y = ctk.DoubleVar(value=1.0)
        self.pixel_size_z = ctk.DoubleVar(value=1.0)

        # - Pixel origin (x, y, z) The origin of the pixel in the x, y and z directions.
        self.pixel_origin_x = ctk.DoubleVar(value=0.0100641)
        self.pixel_origin_y = ctk.DoubleVar(value=0.0)
        self.pixel_origin_z = ctk.DoubleVar(value=0.0)
        # - File Path: Dialog to select the path and name of the PLY file.
        # - Export and Cancel buttons

        # Create a frame for the widgets
        self.frame = ctk.CTkFrame(self, bg_color="transparent")
        self.frame.grid(row=0, column=0, padx=(10, 10), pady=(10, 10), sticky="nsew")
        self.grid_columnconfigure(1, weight=1)
        self.grid_rowconfigure(3, weight=1)

        # Create subframe for pixel size
        self.pixel_size_frame = ctk.CTkFrame(self.frame, bg_color="transparent")
        self.pixel_size_frame.grid(
            row=0, column=0, padx=(10, 10), pady=(10, 10), sticky="nsew"
        )
        self.grid_columnconfigure(6, weight=1)
        self.grid_rowconfigure(2, weight=1)

        self.pixel_label = ctk.CTkLabel(self.pixel_size_frame, text="Pixel Size:")
        self.pixel_label.grid(
            row=0, column=0, padx=(10, 10), pady=(10, 10), sticky="ew"
        )
        self.pixel_size_x_label = ctk.CTkLabel(self.pixel_size_frame, text="X:")
        self.pixel_size_x_label.grid(
            row=1, column=0, padx=(10, 10), pady=(10, 10), sticky="w"
        )
        self.pixel_size_x_entry = ctk.CTkEntry(
            self.pixel_size_frame, textvariable=self.pixel_size_x
        )
        self.pixel_size_x_entry.grid(
            row=1, column=1, padx=(10, 10), pady=(10, 10), sticky="w"
        )
        self.pixel_size_y_label = ctk.CTkLabel(self.pixel_size_frame, text="Y:")
        self.pixel_size_y_label.grid(
            row=1, column=2, padx=(10, 10), pady=(10, 10), sticky="w"
        )
        self.pixel_size_y_entry = ctk.CTkEntry(
            self.pixel_size_frame, textvariable=self.pixel_size_y
        )
        self.pixel_size_y_entry.grid(
            row=1, column=3, padx=(10, 10), pady=(10, 10), sticky="w"
        )
        self.pixel_size_z_label = ctk.CTkLabel(self.pixel_size_frame, text="Z:")
        self.pixel_size_z_label.grid(
            row=1, column=4, padx=(10, 10), pady=(10, 10), sticky="ew"
        )
        self.pixel_size_z_entry = ctk.CTkEntry(
            self.pixel_size_frame, textvariable=self.pixel_size_z
        )
        self.pixel_size_z_entry.grid(
            row=1, column=5, padx=(10, 10), pady=(10, 10), sticky="w"
        )

        # Create subframe for pixel origin
        self.pixel_origin_frame = ctk.CTkFrame(self.frame, bg_color="transparent")
        self.pixel_origin_frame.grid(
            row=1, column=0, padx=(10, 10), pady=(10, 10), sticky="nsew"
        )
        self.grid_columnconfigure(6, weight=1)
        self.grid_rowconfigure(2, weight=1)

        self.pixel_origin_label = ctk.CTkLabel(
            self.pixel_origin_frame, text="Pixel Origin:"
        )
        self.pixel_origin_label.grid(
            row=0, column=0, padx=(10, 10), pady=(10, 10), sticky="ew"
        )
        self.pixel_origin_x_label = ctk.CTkLabel(self.pixel_origin_frame, text="X:")
        self.pixel_origin_x_label.grid(
            row=1, column=0, padx=(10, 10), pady=(10, 10), sticky="w"
        )
        self.pixel_origin_x_entry = ctk.CTkEntry(
            self.pixel_origin_frame, textvariable=self.pixel_origin_x
        )
        self.pixel_origin_x_entry.grid(
            row=1, column=1, padx=(10, 10), pady=(10, 10), sticky="w"
        )
        self.pixel_origin_y_label = ctk.CTkLabel(self.pixel_origin_frame, text="Y:")
        self.pixel_origin_y_label.grid(
            row=1, column=2, padx=(10, 10), pady=(10, 10), sticky="w"
        )
        self.pixel_origin_y_entry = ctk.CTkEntry(
            self.pixel_origin_frame, textvariable=self.pixel_origin_y
        )
        self.pixel_origin_y_entry.grid(
            row=1, column=3, padx=(10, 10), pady=(10, 10), sticky="w"
        )
        self.pixel_origin_z_label = ctk.CTkLabel(self.pixel_origin_frame, text="Z:")
        self.pixel_origin_z_label.grid(
            row=1, column=4, padx=(10, 10), pady=(10, 10), sticky="ew"
        )
        self.pixel_origin_z_entry = ctk.CTkEntry(
            self.pixel_origin_frame, textvariable=self.pixel_origin_z
        )
        self.pixel_origin_z_entry.grid(
            row=1, column=5, padx=(10, 10), pady=(10, 10), sticky="w"
        )

        # Create subframe for file path
        self.file_path_frame = ctk.CTkFrame(self.frame, bg_color="transparent")
        self.file_path_frame.grid(
            row=2, column=0, padx=(10, 10), pady=(10, 10), sticky="nsew"
        )
        self.grid_columnconfigure(2, weight=1)
        self.grid_rowconfigure(1, weight=1)

        self.file_path_label = ctk.CTkLabel(self.file_path_frame, text="File Path:")
        self.file_path_label.grid(
            row=0, column=0, padx=(10, 10), pady=(10, 10), sticky="ew"
        )
        self.file_path_entry = ctk.CTkEntry(self.file_path_frame)
        self.file_path_entry.grid(
            row=0, column=1, padx=(10, 10), pady=(10, 10), sticky="ew"
        )
        self.file_path_button = ctk.CTkButton(
            self.file_path_frame, text="Browse", command=self.browse_file_path
        )
        self.file_path_button.grid(
            row=0, column=2, padx=(10, 10), pady=(10, 10), sticky="ew"
        )

        # Create subframe for export and cancel buttons
        self.export_cancel_frame = ctk.CTkFrame(self.frame, bg_color="transparent")
        self.export_cancel_frame.grid(
            row=3, column=0, padx=(10, 10), pady=(10, 10), sticky="nsew"
        )
        self.grid_columnconfigure(2, weight=1)
        self.grid_rowconfigure(1, weight=1)

        self.export_button = ctk.CTkButton(
            self.export_cancel_frame, text="Export", command=self.export
        )
        self.export_button.grid(
            row=0, column=0, padx=(10, 10), pady=(10, 10), sticky="nsew"
        )
        self.cancel_button = ctk.CTkButton(
            self.export_cancel_frame, text="Cancel", command=self.cancel
        )
        self.cancel_button.grid(
            row=0, column=1, padx=(10, 10), pady=(10, 10), sticky="nsew"
        )

    # --------------------------------------------------------FUNCTIONALITY--------------------------------------------------------#

    def browse_file_path(self):
        print("Browse file path")
        # Get data from master
        file_path = tkinter.filedialog.askdirectory()
        print(file_path)

    def export(self):
        print("Export")
        ranges = self.master.range_data[0]
        print(ranges)

    def cancel(self):
        print("Cancel")
        self.destroy()
