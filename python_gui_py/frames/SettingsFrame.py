import customtkinter as ctk


# --------------------------------------------------------SETTINGS FRAME--------------------------------------------------------#
class SettingsFrame(ctk.CTkFrame):
    """
    A class representing the settings frame.

    This frame contains various settings related to camera, ROS2, and import settings.
    It provides functionality to save settings, validate pixel size, set pixel size, and get pixel size.

    Attributes:
        pixel_size_x (ctk.StringVar): The variable for the x-axis pixel size.
        pixel_size_y (ctk.StringVar): The variable for the y-axis pixel size.
        pixel_size_z (ctk.StringVar): The variable for the z-axis pixel size.
        settings_frame (ctk.CTkFrame): The frame containing the import settings.
        pixel_size_frame (ctk.CTkFrame): The subframe for pixel size.

    Methods:
        save_settings: Saves the settings to a file.
        validate_pixel_size: Validates the pixel size input.
        set_pixel_size: Sets the pixel size for x, y, and z dimensions.
        get_pixel_size: Gets the pixel size in the x, y, and z dimensions.
    """

    def __init__(self, master, **kwargs):
        super().__init__(master, bg_color="transparent", **kwargs)
        self.grid(row=0, column=1, padx=(10, 10), pady=(10, 10), sticky="nsew")

        # Divide the frame into 9 columns and 4 rows
        for i in range(9):
            self.grid_columnconfigure(i, weight=1)
        for i in range(2):
            self.grid_rowconfigure(i, weight=1)
        # Last row should occupy the entire width of the frame
        # self.grid_rowconfigure(3, weight=1)

        # --------------- Camera settings ---------------
        # self.camera_title_label = ctk.CTkLabel(self, text="Camera settings")
        # self.camera_title_label.grid(row=0, column=0, columnspan=3, padx=(10, 10), pady=(10, 10), sticky="we")

        # IP Address
        # self.ip_label = ctk.CTkLabel(self, text="IP Address:")
        # self.ip_label.grid(row=1, column=0, padx=(10, 10), pady=(10, 10), sticky="ew")

        # self.ip_entry = ctk.CTkEntry(self)
        # self.ip_entry.grid(row=1, column=1, padx=(10, 10), pady=(10, 10), sticky="ew")

        # Parameter file
        # self.paramfile_label = ctk.CTkLabel(self, text="Parameter file:")
        # self.paramfile_label.grid(row=2, column=0, padx=(10, 10), pady=(10, 10), sticky="ew")

        # self.parameter_file_button = ctk.CTkButton(self, text="Import parameter file",
        #    command=master.import_files)
        # self.parameter_file_button.grid(row=2, column=2, padx=(10, 10), pady=(10, 10), sticky="ew")

        # Connect Button
        # self.connect_disconnect_button = ctk.CTkButton(self, text="Connect",
        #    command=lambda: master.change_button_text(self.connect_disconnect_button, "Connect", "Disconnect"))
        # self.connect_disconnect_button.grid(row=1, column=2, pady=(10, 10), sticky="ew")

        # --------------- ROS2 settings ---------------
        # self.ros_title_label = ctk.CTkLabel(self, text="ROS2 settings")
        # self.ros_title_label.grid(row=0, column=4, columnspan=3, padx=(10, 10), pady=(10, 10), sticky="we")

        # Topic name
        # self.topic_label = ctk.CTkLabel(self, text="Topic:")
        # self.topic_label.grid(row=1, column=4, padx=(10, 10), pady=(10, 10), sticky="ew")

        # self.topic_entry_ros = ctk.CTkEntry(self)
        # self.topic_entry_ros.grid(row=1, column=5, padx=(10, 10), pady=(10, 10), sticky="ew")

        # Set topic button
        # self.set_topic_button = ctk.CTkButton(self, text="Set",
        #    command=lambda: master.change_button_text(self.set_topic_button, "Set", "Unset"))
        # self.set_topic_button.grid(row=1, column=6, pady=(10, 10), sticky="ew")

        # Create settings frame
        self.settings_frame = ctk.CTkFrame(self, bg_color="transparent")
        self.settings_frame.grid(
            row=0, column=0, padx=(10, 10), pady=(10, 10), sticky="nsew"
        )
        # Configure the grid
        for i in range(2):
            self.settings_frame.grid_columnconfigure(i, weight=1)
        for i in range(4):
            self.settings_frame.grid_rowconfigure(i, weight=1)

        # --------------- Import settings ---------------
        # - Pixel size (x, y, z) The scale/size of the pixel in the x, y and z directions.
        self.pixel_size_x = ctk.StringVar(master=self, value=0.1122161041015625)
        self.pixel_size_y = ctk.StringVar(master=self, value=1.0)
        self.pixel_size_z = ctk.StringVar(master=self, value=1.0)
        # Create subframe for pixel size
        self.pixel_size_frame = ctk.CTkFrame(
            self.settings_frame, bg_color="transparent", fg_color="transparent"
        )
        self.pixel_size_frame.grid(
            row=0, column=0, padx=(10, 10), pady=(10, 10), sticky="nsew"
        )
        for i in range(2):
            self.pixel_size_frame.grid_columnconfigure(i, weight=1)
        for i in range(4):
            self.pixel_size_frame.grid_rowconfigure(i, weight=0)

        # Create widgets
        self.pixel_label = ctk.CTkLabel(self.pixel_size_frame, text="Pixel Size:")
        self.pixel_label.grid(
            row=0, column=0, padx=(10, 10), pady=(10, 10), sticky="nsew"
        )
        self.pixel_size_x_label = ctk.CTkLabel(self.pixel_size_frame, text="X:")
        self.pixel_size_x_label.grid(
            row=1, column=0, padx=(10, 10), pady=(10, 10), sticky="nse"
        )
        self.pixel_size_x_entry = ctk.CTkEntry(
            self.pixel_size_frame, textvariable=self.pixel_size_x
        )
        self.pixel_size_x_entry.grid(
            row=1, column=1, padx=(10, 10), pady=(10, 10), sticky="nsew"
        )
        self.pixel_size_y_label = ctk.CTkLabel(self.pixel_size_frame, text="Y:")
        self.pixel_size_y_label.grid(
            row=2, column=0, padx=(10, 10), pady=(10, 10), sticky="nse"
        )
        self.pixel_size_y_entry = ctk.CTkEntry(
            self.pixel_size_frame, textvariable=self.pixel_size_y
        )
        self.pixel_size_y_entry.grid(
            row=2, column=1, padx=(10, 10), pady=(10, 10), sticky="nsew"
        )
        self.pixel_size_z_label = ctk.CTkLabel(self.pixel_size_frame, text="Z:")
        self.pixel_size_z_label.grid(
            row=3, column=0, padx=(10, 10), pady=(10, 10), sticky="nse"
        )
        self.pixel_size_z_entry = ctk.CTkEntry(
            self.pixel_size_frame, textvariable=self.pixel_size_z
        )
        self.pixel_size_z_entry.grid(
            row=3, column=1, padx=(10, 10), pady=(10, 10), sticky="nsew"
        )

        # --------------- Action Frame ---------------
        # Create a frame for the widgets in the last row of the main frame
        self.action_frame = ctk.CTkFrame(
            self, bg_color="transparent", fg_color="transparent"
        )
        self.action_frame.grid(
            row=3, column=0, padx=(10, 10), pady=(10, 10), sticky="nsew", columnspan=9
        )
        # Configure the grid
        for i in range(3):
            self.action_frame.grid_columnconfigure(i, weight=1)
        self.action_frame.grid_rowconfigure(0, weight=1)

        # Create widgets
        self.save_settings_button = ctk.CTkButton(
            self.action_frame, text="Save settings", command=self.save_settings
        )
        self.save_settings_button.grid(
            row=0, column=3, padx=(10, 10), pady=(10, 10), sticky="nse"
        )

    # --------------------------------------------------------FUNCTIONALITY--------------------------------------------------------#
    def save_settings(self):
        # Save the settings to a file
        self.master.change_console_text("Saving settings", "INFORMATION")
        # Set the new pixel sizes
        self.set_pixel_size(
            self.pixel_size_x.get(), self.pixel_size_y.get(), self.pixel_size_z.get()
        )

    def validate_pixel_size(self, x, y, z):
        """
        Validates the pixel size input.

        Args:
            x (float): The x-axis pixel size.
            y (float): The y-axis pixel size.
            z (float): The z-axis pixel size.

        Returns:
            bool: True if the pixel size is valid, False otherwise.
        """
        # Verify the input was a number and not a string
        try:
            x = float(x)
            y = float(y)
            z = float(z)
        except ValueError:
            return False
        # Verify the input is positive
        if x <= 0 or y <= 0 or z <= 0:
            return False
        return True

    def set_pixel_size(self, x, y, z):
        """
        Sets the pixel size for x, y, and z dimensions.

        Args:
            x (float): The new pixel size for the x dimension.
            y (float): The new pixel size for the y dimension.
            z (float): The new pixel size for the z dimension.

        Returns:
            None
        """
        # Get the new values
        new_x = x
        new_y = y
        new_z = z
        # Verify the input was a number and not a string
        if not self.validate_pixel_size(new_x, new_y, new_z):
            self.master.change_console_text(
                "Pixel size is not a number or is negative", "INFORMATION"
            )
            return
        else:
            self.master.change_console_text("New pixel size set", "SUCCESS")
            print(f"New pixel size: {new_x}, {new_y}, {new_z}")
            # Set the pixel size
            self.pixel_size_x.set(float(new_x))
            self.pixel_size_y.set(float(new_y))
            self.pixel_size_z.set(float(new_z))

    def get_pixel_size(self):
        """
        Get the pixel size in the x, y, and z dimensions.

        Returns:
            tuple: A tuple containing the pixel size in the x, y, and z dimensions.
        """
        return self.pixel_size_x.get(), self.pixel_size_y.get(), self.pixel_size_z.get()
