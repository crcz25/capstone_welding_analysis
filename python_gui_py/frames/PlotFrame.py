import customtkinter as ctk
import matplotlib.pyplot as plt
import numpy as np
import seaborn as sns
from matplotlib.backend_bases import key_press_handler
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure


# --------------------------------------------------------PLOT FRAME--------------------------------------------------------#
class PlotFrame(ctk.CTkFrame):
    """
    A custom frame for plotting data.

    Args:
        master: The parent widget.
        **kwargs: Additional keyword arguments to pass to the parent widget.

    Attributes:
        fig: The figure object for the plot.
        ax: The axes object for the plot.
        canvas: The canvas object for displaying the plot.

    Methods:
        create_figure: Creates a new plot figure.
        update_window: Updates the plot window.
        update_surface: Updates the plot surface.

    """

    def __init__(self, master, **kwargs):
        super().__init__(master, bg_color="transparent", **kwargs)
        self.grid(row=0, column=1, padx=(10, 10), pady=(10, 10), sticky="nsew")

        # Create the plot
        self.fig, self.ax = plt.subplots()
        self.canvas = FigureCanvasTkAgg(self.fig, master=self)

    # --------------------------------------------------------FUNCTIONALITY--------------------------------------------------------#

    def create_figure(self, current_frame=0, profile=0, data=None):
        """
        Creates a new plot figure.

        Args:
            current_frame: The index of the current frame.
            profile: The index of the current profile.
            data: The data to plot.
        """
        
        # Get the section to plot
        section = data[current_frame, profile, :]
        print(f"Section shape: {section.shape}")
        print(f"Section: {section}")
        # Add the section to the plot
        self.ax.plot(section)
        # Set the plot title
        self.ax.set_title(f"Frame {current_frame + 1}")
        self.update_window()
    
    def create_line_plot_figure(self, current_frame=0, profile=0, data=None, choice=None):
        """
        Displays a line plot in the UI.

        Args:
            current_frame: The index of the current frame.
            profile: The index of the current profile.
            data: The data to plot.
            choice: Filter type

        """
        self.ax.clear()
        self.ax.set_ylim(0, 70)
        self.ax.set_title(f"Frame {current_frame + 1}, Profile {profile + 1}, Filter {choice}")
        self.ax.plot(data)
        self.update_window()


    def update_window(self):
        """
        Updates the plot window.

        """
        # Draw the plot
        self.canvas.draw()
        # Set the plot position to fill the frame and expand to fill the frame
        self.canvas.get_tk_widget().pack(side="top", fill="both", expand=True)
        # Update the frame
        super().update()

    def update_surface(self, current_frame=0, profile=0, data=None):
        """
        Updates the plot surface.

        Args:
            current_frame: The index of the current frame.
            profile: The index of the current profile.
            data: The data to plot.

        """
        # Get the section to plot
        section = data[current_frame, profile, :]
        print(f"Section shape: {section.shape}")
        print(f"Section: {section}")
        # Clear the plot
        self.ax.clear()
        # Set the plot title
        self.ax.set_title(f"Frame {current_frame + 1}, Profile {profile + 1}")
        # Add the section to the plot
        self.ax.plot(section)
        # Set the slider position to the current profile
        self.master.plot_control_frame.slider.set(profile)
        # Update the plot window
        self.update_window()

    def clean_plot(self):
        """
        Clean the plot.

        """
        self.ax.clear()
        self.update_window()