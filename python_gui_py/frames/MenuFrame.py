import customtkinter as ctk


# --------------------------------------------------------LEFT SIDEBAR FRAME--------------------------------------------------------#
class MenuFrame(ctk.CTkFrame):
    """
    A custom frame class for the menu section of the GUI.

    This class inherits from ctk.CTkFrame and provides functionality for displaying the menu buttons,
    appearance mode options, and UI scaling options.

    Attributes:
        logo_label (ctk.CTkLabel): The label displaying the dashboard logo.
        sidebar_button_1 (ctk.CTkButton): The button for the "Home" menu option.
        sidebar_button_2 (ctk.CTkButton): The button for the "Settings" menu option.
        appearance_mode_label (ctk.CTkLabel): The label for the appearance mode option.
        appearance_mode_optionmenu (ctk.CTkOptionMenu): The option menu for selecting the appearance mode.
        scaling_label (ctk.CTkLabel): The label for the UI scaling option.
        scaling_optionmenu (ctk.CTkOptionMenu): The option menu for selecting the UI scaling.

    Methods:
        __init__(self, master, **kwargs): Initializes the MenuFrame object.
    """
    def __init__(self, master, **kwargs):
        super().__init__(master, width=140, corner_radius=0, **kwargs)
        self.grid(row=0, column=0, rowspan=4, sticky="nsew")
        self.grid_rowconfigure(4, weight=1)

        self.logo_label = ctk.CTkLabel(
            self, text="Dashboard", font=ctk.CTkFont(size=20, weight="bold")
        )
        self.logo_label.grid(row=0, column=0, padx=20, pady=(20, 10))

        self.sidebar_button_1 = ctk.CTkButton(
            self, text="Home", command=lambda: master.menu_button("home")
        )
        self.sidebar_button_1.grid(row=1, column=0, padx=20, pady=10)
        self.sidebar_button_2 = ctk.CTkButton(
            self, text="Settings", command=lambda: master.menu_button("settings")
        )
        self.sidebar_button_2.grid(row=2, column=0, padx=20, pady=10)

        self.appearance_mode_label = ctk.CTkLabel(
            self, text="Appearance Mode", anchor="w"
        )
        self.appearance_mode_label.grid(row=5, column=0, padx=20, pady=(10, 0))
        self.appearance_mode_optionmenu = ctk.CTkOptionMenu(
            self,
            values=["Light", "Dark"],
            command=master.change_appearance_mode_event,
        )
        self.appearance_mode_optionmenu.grid(row=6, column=0, padx=20, pady=(10, 10))

        self.scaling_label = ctk.CTkLabel(self, text="UI Scaling", anchor="w")
        self.scaling_label.grid(row=7, column=0, padx=20, pady=(10, 0))
        self.scaling_optionmenu = ctk.CTkOptionMenu(
            self,
            values=["80%", "90%", "100%", "110%"],
            command=master.change_scaling_event,
        )
        self.scaling_optionmenu.grid(row=8, column=0, padx=20, pady=(10, 20))

    # --------------------------------------------------------FUNCTIONALITY--------------------------------------------------------#
