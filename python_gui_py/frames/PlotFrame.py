import threading
import time

import customtkinter as ctk
import matplotlib.pyplot as plt
import numpy as np
import seaborn as sns
from frames.cursors import PlotCursor
from matplotlib.backend_bases import key_press_handler
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
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
        initial_cursor_limits: Stores the original min,max values for each axis
        cursor_limits: Stores the current min,max values
        stop_event: Safe thread start/stop
        y_1 = Y - max cursor
        y_2 = Y - min cursor
        x_1 = X - min cursor
        x_2 = X - max cursor
        invert_plot: Flag to invert the plot


    Methods:
        create_figure: Creates a new plot figure.
        update_window: Updates the plot window.
        update_surface: Updates the plot surface.
        update_info_frame: Updates the textbox every x second
        start_update_info_frame: Starts the thread and runs the update_info_frame method
        stop_update_info_frame: Stops the thread running the update_info_frame method
        create_cursors: Creates 4 cursors for the plot
        reset_cursors: Resets the cursors and plot to the initial way
        update_cursor_limits: Updates the cursor limits
        set_axes_limits: Update the axes limits of the plot

    """

    def __init__(self, master, **kwargs):
        super().__init__(master, bg_color="transparent", **kwargs)
        self.grid(row=0, column=1, padx=(10, 10), pady=(10, 10), sticky="nsew")

        # Create the plot
        self.fig, self.ax = plt.subplots()
        self.canvas = FigureCanvasTkAgg(self.fig, master=self)

        # Default cursor limits
        self.initial_cursor_limits = {
            "x_min": 0,
            "x_max": 1600,
            "y_min": 0,
            "y_max": 70,
        }
        # Current cursor limits
        self.cursor_limits = self.initial_cursor_limits.copy()
        # Inital cursors
        self.create_cursors()

        # Ensure safe thread stop
        self.stop_event = threading.Event()
        # Flag for inverting the plot
        self.invert_plot = False

    # --------------------------------------------------------FUNCTIONALITY--------------------------------------------------------#
    def update_info_frame(self):
        """

        Update the info frame with the newest info every second

        """
        while not self.stop_event.is_set():
            # Check if the update is in progress
            if not self.master.info_frame.update_in_progress:
                try:
                    # Set the update flag to True
                    self.master.info_frame.update_in_progress = True

                    # Enable the textbox
                    self.master.info_frame.textbox_info.configure(state="normal")

                    # Delete the current text
                    self.master.info_frame.textbox_info.delete("1.0", "end")

                    # Insert the new text to the textbox
                    txt = f"""Scan: {self.master.range_file} \
                        # \nTime: {self.master.timestamp_data[self.master.current_profile]} \
                        # \nFrame: {self.master.current_profile + 1} \
                        \nProfile: {int(self.master.current_profile) + 1} \
                        \nX-Min: {self.x_1.line.get_xdata()[0]} \
                        \nX-Max: {self.x_2.line.get_xdata()[0]} \
                        \nY-Min: {self.y_2.line.get_ydata()[0]} \
                        \nY-Max: {self.y_1.line.get_ydata()[0]} \
                        \nInverted: {self.invert_plot}"""
                    self.master.info_frame.textbox_info.insert("0.0", txt)

                    # Disable the textbox
                    self.master.info_frame.textbox_info.configure(state="disabled")

                finally:
                    # Set the update flag to False
                    self.master.info_frame.update_in_progress = False

            time.sleep(1.0)

    def start_update_info_frame(self):
        """

        Start the function update_info_frame() on a thread

        """
        # Clear the event
        self.stop_event.clear()
        # Create a separate thread for the update loop
        self.update_thread = threading.Thread(
            target=self.update_info_frame, daemon=True
        )
        # Start the thread
        self.update_thread.start()

    def stop_update_info_frame(self):
        """

        Stop the thread

        """

        # Set the event
        self.stop_event.set()
        # Kill the thread
        self.update_thread.join()

    def create_cursors(self):
        """

        Creates 4 cursors. Calls the PlotCursor class

        Args:
            profile: The index of the current profile.
            data: The data to plot.
        """
        # Initialize cursors with the stored initial limits
        self.y_1 = PlotCursor(self.ax, "y", self.cursor_limits["y_max"], "Y - Max")
        self.y_2 = PlotCursor(self.ax, "y", self.cursor_limits["y_min"], "Y - Min")
        self.x_1 = PlotCursor(self.ax, "x", self.cursor_limits["x_min"], "X - Min")
        self.x_2 = PlotCursor(self.ax, "x", self.cursor_limits["x_max"], "X - Max")

    def reset_cursors(self, profile, data, choice):
        """

        Resets the cursors to their initial limits.

        """
        print(f"Resetting cursors for profile {profile + 1}...")
        try:
            # Check if the cursor limits have change
            self.update_cursor_limits()
            # Update the filter menu to the current choice
            if self.cursor_limits != self.initial_cursor_limits:
                # Stop the updating of the infobox
                # self.stop_update_info_frame()

                # Update the cursor limits with the initial limits
                self.cursor_limits = self.initial_cursor_limits.copy()

                # Update cursors based on the initial limits
                self.y_1.line.set_ydata(
                    [self.cursor_limits["y_max"], self.cursor_limits["y_max"]]
                )
                self.y_2.line.set_ydata(
                    [self.cursor_limits["y_min"], self.cursor_limits["y_min"]]
                )
                self.x_1.line.set_xdata(
                    [self.cursor_limits["x_min"], self.cursor_limits["x_min"]]
                )
                self.x_2.line.set_xdata(
                    [self.cursor_limits["x_max"], self.cursor_limits["x_max"]]
                )

                # Redraw the canvas
                self.canvas.draw_idle()

                # Reset the figure to the original one
                self.create_figure(profile, data, choice)

                self.master.change_console_text(
                    f"Plot and cursors reset frame: profile, {profile + 1}",
                    "INFORMATION",
                )
            else:
                self.master.change_console_text(
                    "Cursors are already in their initial positions", "ERROR"
                )

        except Exception as e:
            self.master.change_console_text(
                f"Error during cursor reset: {str(e)}", "ERROR"
            )

    def update_cursor_limits(self):
        """

        Updates the cursors limits with each adjustment.

        """

        # Update cursor limits by getting current axis value
        self.cursor_limits["x_min"] = self.x_1.line.get_xdata()[0]
        self.cursor_limits["x_max"] = self.x_2.line.get_xdata()[0]
        self.cursor_limits["y_max"] = self.y_1.line.get_ydata()[0]
        self.cursor_limits["y_min"] = self.y_2.line.get_ydata()[0]

    def create_figure(self, profile=0, data=None, choice=None):
        """

        Creates a new plot figure.

        Args:
            profile: The index of the current profile.
            data: The data to plot.
            choice: Filter type

        """

        # Clear the previous plot
        self.ax.clear()

        # Update cursor limits, create them and start the info_frame thread
        self.create_cursors()
        self.update_cursor_limits()
        # self.start_update_info_frame()

        # Depending on the choice plot differently
        section = data[profile, :]
        self.ax.plot(section)
        plot_title = f"Profile {profile + 1}, Filter {choice}"

        # Set the axes limits based on cursor positions
        self.set_axes_limits()

        # Add guide lines to the plot
        self.add_guides(profile, data)
        # Set the plot title
        self.ax.set_title(plot_title)
        # Update the plot window
        self.update_window()

    def add_guides(self, profile=0, data=None):
        """
        Adds guide lines to the figure.

        Args:
            current_profile: The index of the current frame.
            profile: The index of the current profile.
            data: The data to plot.
        """

        # Get the section to plot
        section = data[profile, :]

        # Weld width lines

        # Find the indices where the weld is
        # TODO: needs adjusting
        z_weld = np.where(
            (section > np.percentile(section, 5))
            & (section < np.percentile(section, 95))
        )[0]

        # Find the start and end points of the weld
        weld_start = z_weld[0]
        weld_end = z_weld[-1]

        # Plot a vertical line at the start and end points of the weld
        self.ax.axvline(x=weld_start, color="red", linestyle="--")
        self.ax.axvline(x=weld_end, color="red", linestyle="--")

        # Weld height lines

        # Calculate the top and bottom position of the weld
        # TODO: needs adjusting
        weld_top = np.mean(section[z_weld])
        weld_bot = np.min(np.delete(section, z_weld))  # surface height

        # Plot a horizontal line at the top and bottom points of the weld
        self.ax.axhline(y=weld_top, color="red", linestyle="--")
        self.ax.axhline(y=weld_bot, color="red", linestyle="--")

        # Calculate the angle of the weld in relation to the center point of the weld
        weld_width = weld_end - weld_start
        weld_height = weld_top - weld_bot
        angle_rad = np.arctan(weld_height / (weld_width / 2))  # radians
        angle_deg = angle_rad * 180 / np.pi  # degrees

        # Define the arc radius for drawing the angle
        arc_radius = weld_width / 10  # recommended max value is weld_width / 2

        # Left line
        left_01 = [weld_start, weld_bot]
        left_02 = [
            weld_start + np.cos(angle_rad) * arc_radius,
            weld_bot + np.sin(angle_rad) * arc_radius,
        ]

        # Right line
        right_01 = [weld_end, weld_bot]
        right_02 = [
            weld_end - np.cos(angle_rad) * arc_radius,
            weld_bot + np.sin(angle_rad) * arc_radius,
        ]

        # Draw the angle lines, i.e. hypotenuses
        x_values = [[left_01[0], right_01[0]], [left_02[0], right_02[0]]]
        y_values = [[left_01[1], right_01[1]], [left_02[1], right_02[1]]]
        self.ax.plot(x_values, y_values, color="red", linestyle="--")

        # Draw the angle on both sides
        self.ax.add_patch(
            Arc(
                left_01,
                width=2 * arc_radius,
                height=2 * arc_radius,
                angle=0,
                theta1=0,
                theta2=angle_deg,
                color="red",
                linestyle="--",
                zorder=2,
            )
        )
        self.ax.add_patch(
            Arc(
                right_01,
                width=2 * arc_radius,
                height=2 * arc_radius,
                angle=180 - angle_deg,
                theta1=0,
                theta2=angle_deg,
                color="red",
                linestyle="--",
                zorder=2,
            )
        )

        # Draw the angle value
        self.ax.text(
            left_02[0],
            left_02[1] * 1.1,
            f"{angle_deg:.2f} degrees",
            horizontalalignment="center",
            color="black",
            fontsize=12,
            zorder=2,
        )
        self.ax.text(
            right_02[0],
            right_02[1] * 1.1,
            f"{angle_deg:.2f} degrees",
            horizontalalignment="center",
            color="black",
            fontsize=12,
            zorder=2,
        )

        # Print the information
        print(
            f"The weld starts at x = {weld_start:.2f} and ends at x = {weld_end:.2f}."
        )
        print(f"The weld starts at z = {weld_bot:.2f} and ends at z = {weld_top:.2f}.")
        print(f"The weld angle is alpha = {angle_deg:.2f} degrees.")

    def set_axes_limits(self):
        """
        Update the axes limits of the plot

        """

        self.ax.set_xlim(self.cursor_limits["x_min"], self.cursor_limits["x_max"])
        self.ax.set_ylim(self.cursor_limits["y_min"], self.cursor_limits["y_max"])

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

    def apply_filter(self, data=None, choice=None):
        print(f"Applying filter {choice}...")
        # Get the section to plot
        section = data
        # Invert the plot if the flag is set
        if self.invert_plot:
            inverted_section = -section
            # Move the inverted data back to zero
            min_value = np.min(inverted_section)
            section = inverted_section - min_value

        # Depending on the choice filter differently
        if choice != None or choice != "No Filter":
            section = self.master.plot_control_frame.interpolate_and_filter(
                section, choice
            )

        return section

    def update_surface(self, profile=0, data=None, choice=None):
        """

        Updates the plot surface.

        Args:
            current_profile: The index of the current frame.
            profile: The index of the current profile.
            data: The data to plot.

        """
        print(f"\nUpdating plot surface for profile {profile}...")
        data = self.master.range_data
        # Get the choice from the control frame
        print(f"Choice: {choice}")
        # Apply the filter
        section = self.apply_filter(data[profile, :], choice)
        # Recreate the figure
        # Default color
        default_color = plt.rcParams["axes.prop_cycle"].by_key()["color"][0]
        # Clean the plot
        self.ax.clear()
        # Remove and re-add the cursors to the plot
        for cursor in [self.y_1, self.y_2, self.x_1, self.x_2]:
            if cursor.line is not None:
                cursor.line.remove()
                cursor.line = None
        # Create the cursors
        self.create_cursors()
        # Update cursor limits
        self.update_cursor_limits()
        # Set the axes limits based on cursor positions
        self.set_axes_limits()
        # Plot the data
        self.ax.plot(section, color=default_color)
        # Add guide lines to the plot
        self.add_guides(profile, data)
        # Set the title of the plot
        plot_title = f"Profile {profile + 1}, Filter {choice}"
        self.ax.set_title(plot_title)
        # Redraw the canvas
        self.canvas.draw_idle()
        # Update the plot window
        self.update_window()

    def clean_plot(self):
        """
        Clean the plot.

        """
        self.ax.clear()
        self.update_window()
