import customtkinter as ctk
import matplotlib.pyplot as plt
import numpy as np
import seaborn as sns
from matplotlib.backend_bases import key_press_handler
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure


# --------------------------------------------------------PLOT FRAME--------------------------------------------------------#
class PlotFrame(ctk.CTkFrame):
    def __init__(self, master, **kwargs):
        super().__init__(master, bg_color="transparent", **kwargs)
        self.grid(row=0, column=1, padx=(10, 10), pady=(10, 10), sticky="nsew")

        # Create the plot
        self.fig, self.ax = plt.subplots()
        self.canvas = FigureCanvasTkAgg(self.fig, master=self)

    # --------------------------------------------------------FUNCTIONALITY--------------------------------------------------------#

    def create_figure(self, current_frame=0, profile=0, data=None):
        if current_frame is None and data is None:
            current_frame = self.master.current_frame
            data = self.master.data
        section = data[current_frame, profile, :]
        print(f"Section shape: {section.shape}")
        print(f"Section: {section}")
        self.ax.plot(section)
        self.ax.set_title(f"Frame {current_frame + 1}")
        self.ax.set_axis_off()

        self.fig.tight_layout()
        self.update_window()

    def update_window(self):
        self.canvas.draw()
        self.canvas.get_tk_widget().grid(
            row=0, column=0, padx=(10, 10), pady=(10, 10), sticky="nsew"
        )
        super().update()

    def update_surface(self, current_frame=0, profile=0, data=None):
        self.ax.clear()
        self.ax.set_title(f"Frame {current_frame + 1}, Profile {profile + 1}")
        section = data[current_frame, profile, :]
        print(f"Section shape: {section.shape}")
        self.ax.plot(section)
        self.master.plot_control_frame.slider.set(profile)
        self.update_window()
