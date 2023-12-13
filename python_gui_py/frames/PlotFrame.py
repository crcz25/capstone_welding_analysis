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
        super().__init__(master, bg_color="transparent", fg_color="darkblue", **kwargs)
        self.grid(row=0, column=1, padx=(10, 10), pady=(10, 10), sticky="nsew")

        # Create the plot
        self.fig, self.ax = plt.subplots()
        self.canvas = FigureCanvasTkAgg(self.fig, master=self)

    # --------------------------------------------------------FUNCTIONALITY--------------------------------------------------------#

    def create_figure(self, current_frame=None, data=None):
        if current_frame is None:
            current_frame = self.master.current_frame
        if data is None:
            data = self.master.data

        self.ax.imshow(data[current_frame], cmap="gray", vmin=0, vmax=255)
        self.ax.set_title(f"Frame {current_frame}")
        self.ax.set_axis_off()

        self.fig.tight_layout()
        self.update_window()

    def update_window(self):
        self.canvas.draw()
        self.canvas.get_tk_widget().grid(
            row=0, column=0, padx=(10, 10), pady=(10, 10), sticky="nsew"
        )
        super().update()

    def update_surface(self, surface):
        self.ax.clear()
        self.ax.imshow(surface, cmap="gray", vmin=0, vmax=255)
        self.update_window()
