import customtkinter as ctk
import matplotlib.pyplot as plt
import numpy as np
import seaborn as sns
from matplotlib.backend_bases import key_press_handler
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
from frames.cursors import PlotCursor


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

        # Store references to lines and cursors
        self.lines = []
        self.cursor_limits = {"x_min": 0, "x_max": 1600, "y_min": 0, "y_max": 70}
        self.create_cursors()

    # --------------------------------------------------------FUNCTIONALITY--------------------------------------------------------#
    def create_cursors(self):
        self.y_1 = PlotCursor(self.ax, "y", self.cursor_limits["y_max"], "Y - Max")
        self.y_2 = PlotCursor(self.ax, "y", self.cursor_limits["y_min"], "Y - Min")
        self.x_1 = PlotCursor(self.ax, "x", self.cursor_limits["x_min"], "X - Min")
        self.x_2 = PlotCursor(self.ax, "x", self.cursor_limits["x_max"], "X - Max")
        print(self.x_1.line,self.x_2,self.y_1,self.y_2)
        self.update_cursor_limits()

        

    def update_cursor_limits(self):
        self.cursor_limits["x_min"] = self.x_1.axis
        self.cursor_limits["x_max"] = self.x_2.axis
        self.cursor_limits["y_min"] = self.y_2.axis
        self.cursor_limits["y_max"] = self.y_1.axis
        print(self.cursor_limits)


    def create_figure(self, current_frame=0, profile=0, data=None):
        """
        Creates a new plot figure.

        Args:
            current_frame: The index of the current frame.
            profile: The index of the current profile.
            data: The data to plot.
        """
        
        # Get the section to plot
        self.ax.clear()
        self.create_cursors()
        section = data[current_frame, profile, :]
        print(f"Section shape: {section.shape}")
        print(f"Section: {section}")
        # Add the section to the plot
        self.ax.plot(section)
        # Set the plot title
        self.ax.set_title(f"Frame {current_frame + 1}")
        self.update_cursor_limits()
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
        self.create_cursors()
        self.ax.set_ylim(0, 70)
        self.ax.set_title(f"Frame {current_frame + 1}, Profile {profile + 1}, Filter {choice}")
        self.ax.plot(data)
        self.update_cursor_limits()
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
        self.update_cursor_limits()
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
        # Clear the plot
        self.ax.clear()
        self.create_cursors()
        section = data[current_frame, profile, :]
        print(f"Section shape: {section.shape}")
        print(f"Section: {section}")
        # Set the plot title
        self.ax.set_title(f"Frame {current_frame + 1}, Profile {profile + 1}")
        # Add the section to the plot
        self.ax.plot(section)
        # Set the slider position to the current profile
        self.master.plot_control_frame.slider.set(profile)
        self.update_cursor_limits()
        # Update the plot window
        self.update_window()
