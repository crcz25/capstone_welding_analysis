import customtkinter as ctk

#--------------------------------------------------------RIGHT INFO/ALERT FRAME--------------------------------------------------------#
class InfoFrame(ctk.CTkFrame):
    def __init__(self, master, **kwargs):
        super().__init__(master, bg_color="transparent", corner_radius=0, **kwargs)

        # Create 2 textboxes for info/alerts
        self.grid(row=0, column=2, rowspan=4, padx=(0, 0), pady=(0, 0), sticky="nsew")

        self.scaling_label_info = ctk.CTkLabel(self, text="Information:", anchor="w")
        self.scaling_label_info.grid(row=0, column=0, padx=10, pady=(10, 0))
        self.textbox_info = ctk.CTkTextbox(self, width=220)
        self.textbox_info.grid(row=1, column=0, padx=(10, 10), pady=(10, 0), sticky="nsew")

        self.scaling_label_alerts = ctk.CTkLabel(self, text="Alerts:", anchor="w")
        self.scaling_label_alerts.grid(row=2, column=0, padx=10, pady=(10, 0))
        self.textbox_alerts = ctk.CTkTextbox(self, width=220)
        self.textbox_alerts.grid(row=3, column=0, padx=(10, 10), pady=(10, 0), sticky="nsew")