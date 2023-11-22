import customtkinter as ctk

#--------------------------------------------------------SETTINGS FRAME--------------------------------------------------------#
class SettingsFrame(ctk.CTkFrame):
    def __init__(self, master, **kwargs):
        super().__init__(master, bg_color="transparent", **kwargs)
        self.grid(row=0, column=1, padx=(10, 10), pady=(10, 10), sticky="nsew")

        # Divide the frame into 9 columns and 4 rows
        for i in range(9):
            self.grid_columnconfigure(i, weight=1)
        for i in range(4):
            self.grid_rowconfigure(i, weight=1)

        # --------------- Camera settings ---------------
        self.camera_title_label = ctk.CTkLabel(self, text="Camera settings")
        self.camera_title_label.grid(row=0, column=0, columnspan=3, padx=(10, 10), pady=(10, 10), sticky="we")

        # IP Address
        self.ip_label = ctk.CTkLabel(self, text="IP Address:")
        self.ip_label.grid(row=1, column=0, padx=(10, 10), pady=(10, 10), sticky="ew")

        self.ip_entry = ctk.CTkEntry(self)
        self.ip_entry.grid(row=1, column=1, padx=(10, 10), pady=(10, 10), sticky="ew")

        # Parameter file
        self.paramfile_label = ctk.CTkLabel(self, text="Parameter file:")
        self.paramfile_label.grid(row=2, column=0, padx=(10, 10), pady=(10, 10), sticky="ew")

        self.parameter_file_button = ctk.CTkButton(self, text="Import parameter file")
        self.parameter_file_button.grid(row=2, column=2, padx=(10, 10), pady=(10, 10), sticky="ew")

        # Connect Button
        self.connect_disconnect_button = ctk.CTkButton(self, text="Connect",
                                                       command=lambda: master.change_button_text(self.connect_disconnect_button, "Connect", "Disconnect"))
        self.connect_disconnect_button.grid(row=1, column=2, pady=(10, 10), sticky="ew")

        # --------------- ROS2 settings ---------------
        self.ros_title_label = ctk.CTkLabel(self, text="ROS2 settings")
        self.ros_title_label.grid(row=0, column=4, columnspan=3, padx=(10, 10), pady=(10, 10), sticky="we")

        # Topic name
        self.topic_label = ctk.CTkLabel(self, text="Topic:")
        self.topic_label.grid(row=1, column=4, padx=(10, 10), pady=(10, 10), sticky="ew")

        self.topic_entry_ros = ctk.CTkEntry(self)
        self.topic_entry_ros.grid(row=1, column=5, padx=(10, 10), pady=(10, 10), sticky="ew")

        # Set topic button
        self.set_topic_button = ctk.CTkButton(self, text="Set",
                                                       command=lambda: master.change_button_text(self.set_topic_button, "Set", "Unset"))
        self.set_topic_button.grid(row=1, column=6, pady=(10, 10), sticky="ew")
        
    #--------------------------------------------------------FUNCTIONALITY--------------------------------------------------------#