import customtkinter as ctk
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.patches import Arc

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
        # Add guide lines to the plot
        self.add_guides(current_frame, profile, data)
        # Set the plot title
        self.ax.set_title(f"Frame {current_frame + 1}")
        self.update_window()
    
    def add_guides(self, current_frame=0, profile=0, data=None):
        """
        Adds guide lines to the figure.

        Args:
            current_frame: The index of the current frame.
            profile: The index of the current profile.
            data: The data to plot.
        """

        # Get the section to plot
        section = data[current_frame, profile, :]
        
        # Weld width lines

        # Find the indices where the weld is
        # TODO: needs adjusting
        z_weld = np.where((section > np.percentile(section, 5)) & (section < np.percentile(section, 95)))[0] 
        
        # Find the start and end points of the weld
        weld_start = z_weld[0]
        weld_end = z_weld[-1]
        
        # Plot a vertical line at the start and end points of the weld
        self.ax.axvline(x=weld_start, color='red', linestyle='--')
        self.ax.axvline(x=weld_end, color='red', linestyle='--')

        # Weld height lines
        
        # Calculate the top and bottom position of the weld
        # TODO: needs adjusting
        weld_top = np.mean(section[z_weld])
        weld_bot = np.min(np.delete(section, z_weld)) # surface height

        # Plot a horizontal line at the top and bottom points of the weld
        self.ax.axhline(y=weld_top, color='red', linestyle='--')
        self.ax.axhline(y=weld_bot, color='red', linestyle='--')

        # Calculate the angle of the weld in relation to the center point of the weld
        weld_width = weld_end - weld_start
        weld_height = weld_top - weld_bot
        angle_rad = np.arctan(weld_height / (weld_width / 2)) # radians
        angle_deg = angle_rad * 180 / np.pi # degrees
        
        # Define the arc radius for drawing the angle
        arc_radius = weld_width / 10 # recommended max value is weld_width / 2 

        # Left line
        left_01 = [weld_start, weld_bot]
        left_02 = [weld_start + np.cos(angle_rad) * arc_radius, weld_bot + np.sin(angle_rad) * arc_radius]
        
        # Right line
        right_01 = [weld_end, weld_bot]
        right_02 = [weld_end - np.cos(angle_rad) * arc_radius, weld_bot + np.sin(angle_rad) * arc_radius]
        
        # Draw the angle lines, i.e. hypotenuses
        x_values = [[left_01[0], right_01[0]], [left_02[0], right_02[0]]]
        y_values = [[left_01[1], right_01[1]], [left_02[1], right_02[1]]]
        self.ax.plot(x_values, y_values, color='red', linestyle='--')

        # Draw the angle on both sides
        self.ax.add_patch(Arc(left_01, width=2*arc_radius, height=2*arc_radius, angle=0,
                              theta1=0, theta2=angle_deg, color='red', linestyle='--', zorder=2))
        self.ax.add_patch(Arc(right_01, width=2*arc_radius, height=2*arc_radius, angle=180-angle_deg,
                              theta1=0, theta2=angle_deg, color='red', linestyle='--', zorder=2))
        
        # Draw the angle value
        self.ax.text(left_02[0], left_02[1]*1.1, f'{angle_deg:.2f} degrees', 
            horizontalalignment='center', color='black', fontsize=12, zorder=2)
        self.ax.text(right_02[0], right_02[1]*1.1, f'{angle_deg:.2f} degrees', 
            horizontalalignment='center', color='black', fontsize=12, zorder=2)
        
        # Print the information
        print(f'The weld starts at x = {weld_start:.2f} and ends at x = {weld_end:.2f}.')
        print(f'The weld starts at z = {weld_bot:.2f} and ends at z = {weld_top:.2f}.')
        print(f'The weld angle is alpha = {angle_deg:.2f} degrees.')

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
        # Add guide lines to the plot
        self.add_guides(current_frame, profile, data)
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