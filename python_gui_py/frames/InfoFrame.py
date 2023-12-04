import customtkinter as ctk

#--------------------------------------------------------RIGHT INFO/DEFECTS FRAME--------------------------------------------------------#
class InfoFrame(ctk.CTkFrame):
    def __init__(self, master, **kwargs):
        super().__init__(master, bg_color="transparent", corner_radius=0, **kwargs)

        # Create 2 textboxes for info/defects
        self.grid(row=0, column=2, rowspan=4, padx=(0, 0), pady=(0, 0), sticky="nsew")

        self.scaling_label_info = ctk.CTkLabel(self, text="Information:", anchor="w")
        self.scaling_label_info.grid(row=0, column=0, padx=10, pady=(10, 0))
        self.textbox_info = ctk.CTkTextbox(self, width=220)
        self.textbox_info.grid(row=1, column=0, padx=(10, 10), pady=(10, 0), sticky="nsew")

        self.scaling_label_alerts = ctk.CTkLabel(self, text="Defects:", anchor="w")
        self.scaling_label_alerts.grid(row=2, column=0, padx=10, pady=(10, 0))
        self.textbox_alerts = ctk.CTkTextbox(self, width=220)
        self.textbox_alerts.grid(row=3, column=0, padx=(10, 10), pady=(10, 0), sticky="nsew")

        # Defects
        labels_dropdown_frame = ctk.CTkFrame(self)
        labels_dropdown_frame.grid(row=3, column=0, padx=(10, 10), pady=(10, 0), sticky="nsew")
        labels_and_dropdowns = []

        # Just example data (TODO: Move to JSON etc)
        data = {
            "Undercut": {"options": ["Data A", "Data B", "Data C"]},
            "Cracks": {"options": ["Data X", "Data Y", "Data Z"]},
        }

        # Generate labels and options from given data
        for i, (label_text, _) in enumerate(data.items()):
            label = ctk.CTkLabel(labels_dropdown_frame, text=label_text, anchor="w")
            label.grid(row=i, column=0, padx=10, pady=(10, 0))

            options_data = data.get(label_text, {}).get("options", [])
            dropdown_var = ctk.StringVar(labels_dropdown_frame)

            dropdown = ctk.CTkComboBox(labels_dropdown_frame, state="readonly" ,values=options_data)
            dropdown['variable'] = dropdown_var
            dropdown.grid(row=i, column=1, padx=(10, 10), pady=(10, 0), sticky="w")


            labels_and_dropdowns.append((label, dropdown, dropdown_var))


    #--------------------------------------------------------FUNCTIONALITY--------------------------------------------------------#