import customtkinter as ctk

#--------------------------------------------------------SLIDER/MAIN CONTROL FRAME--------------------------------------------------------#
class PlotControlFrame(ctk.CTkFrame):
    def __init__(self, master, **kwargs):
        super().__init__(master, bg_color="transparent", **kwargs)
        self.grid(row=1, column=1, padx=(10, 10), pady=(10, 10), sticky="nsew")
        self.grid_columnconfigure(3, weight=1)
        self.grid_rowconfigure(1, weight=0)

        # Create Slider
        self.slider = ctk.CTkSlider(self, from_=0, to=100)
        self.slider.grid(row=0, columnspan=4, padx=(10, 10), pady=(10, 10), sticky="nsew")

        # Create main control buttons
        scan_button = ctk.CTkButton(self, text="Take scan", command=master.take_scan)
        previous_button = ctk.CTkButton(self, text="Previous", command=master.previous_plot)
        next_button = ctk.CTkButton(self, text="Next", command=master.next_plot)
        load_button = ctk.CTkButton(self, text="Import scan", command=master.import_scan)

        scan_button.grid(row=1, column=0, padx=10, pady=10, sticky="ew")
        previous_button.grid(row=1, column=1, padx=10, pady=10, sticky="ew")
        next_button.grid(row=1, column=2, padx=10, pady=10, sticky="ew")
        load_button.grid(row=1, column=3, padx=10, pady=10, sticky="w")

        # Dropdown menus
        scan_type_menu = ctk.CTkOptionMenu(self, values=["Single scan", "Continuous scan"], anchor="center",
                                           command=master.filter_menu)
        scan_type_menu.grid(row=2, column=0, padx=10, pady=10, sticky="w")

        filter_menu = ctk.CTkOptionMenu(self, values=["Gaussian"], anchor="center", command=master.filter_menu)
        filter_menu.grid(row=2, column=1, padx=10, pady=10, sticky="w")