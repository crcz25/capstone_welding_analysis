import customtkinter as ctk

#--------------------------------------------------------PLOT FRAME--------------------------------------------------------#
class PlotFrame(ctk.CTkFrame):
    def __init__(self, master, **kwargs):
        super().__init__(master, bg_color="transparent", **kwargs)
        self.grid(row=0, column=1, padx=(10, 10), pady=(10, 10), sticky="nsew")

        
    #--------------------------------------------------------FUNCTIONALITY--------------------------------------------------------#