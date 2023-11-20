import customtkinter as ctk

#--------------------------------------------------------CAMERA SETTINGS FRAME--------------------------------------------------------#
class CameraSettingsFrame(ctk.CTkFrame):
    def __init__(self, master, **kwargs):
        super().__init__(master, bg_color="transparent", **kwargs)
        self.grid(row=0, column=1, padx=(10, 10), pady=(10, 10), sticky="nsew")
        self.grid_columnconfigure(3, weight=1)
        self.grid_rowconfigure(3, weight=1)

        # IP Address
        self.ip_label = ctk.CTkLabel(self, text="IP Address:")
        self.ip_label.grid(row=0, column=0, padx=(10, 10), pady=(10, 10), sticky="ew")

        self.ip_entry = ctk.CTkEntry(self)
        self.ip_entry.grid(row=0, column=1, padx=(10, 10), pady=(10, 10), sticky="ew")

        # Connect Button
        self.connect_disconnect_button = ctk.CTkButton(self, text="Connect",
                                                       command=lambda: master.change_button_text(self.connect_disconnect_button, "Connect", "Disconnect"))
        self.connect_disconnect_button.grid(row=0, column=2, pady=(10, 10), sticky="ew")

    #--------------------------------------------------------FUNCTIONALITY--------------------------------------------------------#