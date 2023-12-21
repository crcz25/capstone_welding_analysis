import tkinter

import customtkinter as ctk


# --------------------------------------------------------SLIDER/MAIN CONTROL FRAME--------------------------------------------------------#
class PlotControlFrame(ctk.CTkFrame):
    def __init__(self, master, **kwargs):
        super().__init__(master, bg_color="transparent", **kwargs)
        self.grid(row=1, column=1, padx=(10, 10), pady=(10, 10), sticky="nsew")
        self.grid_columnconfigure(3, weight=1)
        self.grid_rowconfigure(4, weight=1)

        # Default text color
        self.color = "white"

        # Create Slider
        self.slider_value = tkinter.IntVar(master=self.master, value=0)
        self.slider = ctk.CTkSlider(
            self,
            from_=0,
            to=512,
            number_of_steps=511,
            variable=self.slider_value,
            command=self.slider_event,
        )
        self.slider.grid(
            row=0, columnspan=4, padx=(10, 10), pady=(10, 10), sticky="nsew"
        )
        self.slider.set(0)

        # Create main control buttons
        self.scan_button = ctk.CTkButton(
            self, text="Take scan", command=self.take_stop_scan
        )
        self.previous_button = ctk.CTkButton(
            self, text="Previous", command=self.previous_plot
        )
        self.next_button = ctk.CTkButton(self, text="Next", command=self.next_plot)
        self.load_button = ctk.CTkButton(
            self, text="Import scan", command=self.import_scan
        )

        self.scan_button.grid(row=1, column=0, padx=10, pady=10, sticky="ew")
        self.previous_button.grid(row=1, column=1, padx=10, pady=10, sticky="ew")
        self.next_button.grid(row=1, column=2, padx=10, pady=10, sticky="ew")
        self.load_button.grid(row=1, column=3, padx=10, pady=10, sticky="w")

        # Dropdown menus
        self.filter_label = ctk.CTkLabel(self, text="Apply filter:")
        self.filter_label.grid(
            row=2, column=0, padx=(10, 10), pady=(10, 10), sticky="we"
        )
        self.filter_menu = ctk.CTkOptionMenu(
            self, values=["Gaussian"], anchor="center", command=self.filter_menu
        )
        self.filter_menu.grid(row=2, column=1, padx=10, pady=10, sticky="w")
        self.filter_menu.set("Filter")

        self.export_menu = ctk.CTkOptionMenu(
            self, values=[".ply", ".npy"], anchor="center", command=self.export_menu
        )
        self.export_menu.grid(row=2, column=2, padx=10, pady=10, sticky="w")
        self.export_menu.set("Export")

        # Console
        self.console_label = ctk.CTkLabel(self, text="Console")
        self.console_label.grid(
            row=3, column=0, padx=(10, 10), pady=(10, 10), sticky="ws"
        )

        self.console_entry = ctk.CTkTextbox(self, width=250, height=150, text_color=self.color)
        self.console_entry.grid(row=4, column=0,columnspan=4,rowspan=4,
            padx=(10, 10),
            pady=(10, 10),
            sticky="nsew",
        )

    # --------------------------------------------------------FUNCTIONALITY--------------------------------------------------------#
    def slider_event(self, other=None):
        print(f"Slider event: {self.slider_value.get()}")
        self.master.change_plot(change=0, profile=self.slider_value.get())

    def take_stop_scan(self):
        self.master.change_button_text(self.scan_button, "Take scan", "Stop scan")

    def previous_plot(self):
        self.master.change_plot(-1)

    def next_plot(self):
        self.master.change_plot(1)

    def import_scan(self):
        self.master.import_files()

    def filter_menu(self, choice):
        pass

    def export_menu(self, choice):
        self.export_menu.set("Export")
