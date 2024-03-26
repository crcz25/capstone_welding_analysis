import threading
import time

import customtkinter as ctk
import matplotlib.pyplot as plt
import numpy as np
from frames.cursors import PlotCursor
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.lines import Line2D
from matplotlib.patches import Arc


# --------------------------------------------------------PLOT FRAME--------------------------------------------------------#
class PlotFrame(ctk.CTkFrame):
    """
    A class representing a plot frame.

    Attributes:
        fig (matplotlib.figure.Figure): The figure object for the plot.
        ax (matplotlib.axes.Axes): The axes object for the plot.
        canvas (matplotlib.backends.backend_tkagg.FigureCanvasTkAgg): The canvas object for the plot.
        points (list): A list to store the clicked points.
        initial_cursor_limits (dict): The initial cursor limits.
        cursor_limits (dict): The current cursor limits.
        stop_event (threading.Event): An event to signal the thread to stop.
        invert_plot (bool): A flag for inverting the plot.
        work_piece_thickness (float): The thickness of the work piece.
        height_of_weld (float): The height of the weld.
        x_position_of_weld (float): The x position of the weld.
        row_filtered (None or ndarray): The filtered row data.
        pointsEnabled (bool): A flag to enable points for clicked guide lines.
        align_plot (bool): A flag for aligning the plot.
        pt1 (None or tuple): The first point for aligning the plot.
        pt2 (None or tuple): The second point for aligning the plot.

    Methods:
        update_info_frame: Update the info frame with the newest info every second.
        start_update_info_frame: Start the function update_info_frame() on a thread.
        stop_update_info_frame: Stop the thread.
        create_cursors: Creates 4 cursors. Calls the PlotCursor class.
        reset_cursors: Resets the cursors to their initial limits.
        update_cursor_limits: Updates the cursors limits with each adjustment.
        create_figure: Creates a new plot figure.
        add_lines_on_click: Draws lines between clicked points.
        add_guides: Adds guide lines to the figure.
        reset_guides_defects: Resets the guide lines for defects.
        add_guides_defects: Adds a guide line for the thickness of the work piece.
        set_axes_limits: Update the axes limits of the plot.
        update_window: Updates the plot window.
        apply_filter: Apply a filter to the given data and return the filtered section.
        update_surface: Updates the plot surface.
        clean_plot: Clean the plot.
        save_plot: Save the plot as a .png file.
    """

    def __init__(self, master, **kwargs):
        super().__init__(master, bg_color="transparent", **kwargs)
        self.grid(row=0, column=1, padx=(10, 10), pady=(10, 10), sticky="nsew")

        # Create the plot
        self.fig, self.ax = plt.subplots()
        self.canvas = FigureCanvasTkAgg(self.fig, master=self)
        self.x_axis = np.arange(0, 1600, 1)

        # Connect the mouse click event to the add_lines_on_click function
        self.canvas.mpl_connect("button_press_event", self.add_lines_on_click)

        # Initialize an empty list to store the clicked points
        self.points = []

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

        # For weld defects guide lines
        self.work_piece_thickness = 0.0
        self.height_of_weld = 0.0
        self.x_position_of_weld = 0.0
        # For weld defects processing
        self.row_filtered = None

        # Enable points for clicked guide lines
        self.pointsEnabled = False

        # Flag for aligning the plot
        self.align_plot = False

        # Plot margins of 5% area of the plot for the cursor limits
        self.margin_cursor = 0.01

    # --------------------------------------------------------FUNCTIONALITY--------------------------------------------------------#
    def update_plot_style(self, plot_title, x_label, y_label):
        """
        Applies a custom light or dark theme to the plot
        """
        ap_mode = ctk.get_appearance_mode()

        # Set plot custom style
        if ap_mode == "Light":
            self.fig.patch.set_facecolor('#f2f2f2')
            self.ax.set_facecolor('#f2f2f2')
            plt.style.use(r'python_gui_py\\styles\\custom_light_style.mplstyle')
            self.line_text_color = 'black'

        elif ap_mode == "Dark":
            self.fig.patch.set_facecolor('#1a1a1a')
            self.ax.set_facecolor('#1a1a1a')
            plt.style.use(r'python_gui_py\\styles\\custom_dark_style.mplstyle')
            self.line_text_color = 'white'

        # Plot edge color
        self.ax.tick_params(axis='both', colors=self.line_text_color, labelsize=10)
        self.ax.spines['bottom'].set_color(self.line_text_color)
        self.ax.spines['top'].set_color(self.line_text_color)
        self.ax.spines['right'].set_color(self.line_text_color)
        self.ax.spines['left'].set_color(self.line_text_color)

        # Plot titles & labels color
        self.ax.set_title(plot_title, color=self.line_text_color, fontsize=12)
        self.ax.set_xlabel(x_label, color=self.line_text_color, fontsize=10)
        self.ax.set_ylabel(y_label, color=self.line_text_color, fontsize=10)

        self.canvas.draw_idle()


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
        self.y_1 = PlotCursor(
            self.master, self.ax, "y", self.cursor_limits["y_max"], "Y - Max"
        )
        self.y_2 = PlotCursor(
            self.master, self.ax, "y", self.cursor_limits["y_min"], "Y - Min"
        )
        self.x_1 = PlotCursor(
            self.master, self.ax, "x", self.cursor_limits["x_min"], "X - Min"
        )
        self.x_2 = PlotCursor(
            self.master, self.ax, "x", self.cursor_limits["x_max"], "X - Max"
        )

    def reset_cursors(self, profile, data, choice):
        """
        Resets the cursors to their initial limits.
        """
        try:
            # Check if the cursor limits have changed
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

            else:
                self.master.change_console_text(
                    "Cursors are already in their initial positions", "Information"
                )
            # Reset the thickness and height of the weld
            self.work_piece_thickness = 0.0
            self.height_of_weld = 0.0
            self.x_position_of_weld = 0.0

            # Reset the defect choice
            self.master.info_frame.defect_choice = "None"
            self.master.info_frame.dropdown.set("None")
            self.master.info_frame.work_piece_thickness.set(0.0)

            # Reset the lower data
            self.align_plot = False
            self.points = []

            # Add guide lines for defects
            self.add_guides_defects()

            # Redraw the canvas
            self.canvas.draw_idle()

            # Reset the figure to the original one
            self.create_figure(profile, data, choice)

            self.master.change_console_text(
                f"Plot and cursors reset frame: profile, {profile + 1}",
                "INFORMATION",
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
        # Apply the filter
        section = self.apply_filter(data[profile, :], choice)
        # Define the plot title and axis labels
        plot_title = f"Profile {profile + 1}, Filter {choice}"
        x_label = "Width [mm]"
        y_label = "Height [mm]"
        # Set the plot style
        self.update_plot_style(plot_title, x_label, y_label)
        # Plot the data with the coordinates used after setting the pixel size
        self.ax.plot(self.x_axis, section)
        # Set the axes limits based on cursor positions
        self.set_axes_limits()
        # Set the plot title and axis labels
        self.ax.set_title(plot_title)
        self.ax.set_xlabel(x_label)
        self.ax.set_ylabel(y_label)
        # Add guide lines to the plot
        # self.add_guides(profile, data)

        # Update the plot window
        self.update_window()

    def add_lines_on_click(self, event):
        """
        Draws lines between clicked points.

        Args:
            event: The button press event.
        """
        # Check if the points are enabled
        if not self.pointsEnabled:
            return

        # Get the x and y coordinates of the clicked point
        x = event.xdata
        y = event.ydata

        # Check the position of the clicked point, we calculate the bounds by using a margin of 15%. The offset is always internal as it is a percentage of the plot area and not the window area.
        width = self.cursor_limits["x_max"] - self.cursor_limits["x_min"]
        height = self.cursor_limits["y_max"] - self.cursor_limits["y_min"]
        offset_x = width * self.margin_cursor
        offset_y = height * self.margin_cursor
        lower_x = self.cursor_limits["x_min"] + offset_x
        upper_x = self.cursor_limits["x_max"] - offset_x
        lower_y = self.cursor_limits["y_min"] + offset_y
        upper_y = self.cursor_limits["y_max"] - offset_y
        print(f"X: {x}, Y: {y}")
        print(f"Lower X: {lower_x}, Upper X: {upper_x}")
        print(f"Lower Y: {lower_y}, Upper Y: {upper_y}")

        if (
            x < lower_x
            or x > upper_x
            or y < lower_y
            or y > upper_y
            or x == None
            or y == None
        ):
            # Add a legend in the top center of the plot to inform the user
            self.ax.text(
                self.cursor_limits["x_max"] / 2,
                self.cursor_limits["y_max"] * 0.5,
                "Point out of bounds",
                horizontalalignment="center",
                fontsize=10,
                zorder=2,
            )
            self.points = []
            return

        # Append the point to the list
        self.points.append((x, y))

        # Add a legend in the top center of the plot
        self.add_points_legend(event=event)

        # Check that list has at least 2 points and that all points have a pair
        if len(self.points) < 2 or len(self.points) % 2 != 0:
            return

        # Get the last two points to draw the newest line
        p1 = self.points[-2]
        p2 = self.points[-1]

        # Check none values
        if p1[0] == None or p2[0] == None:
            return

        # Plot a line segment connecting them
        self.ax.plot([p1[0], p2[0]], [p1[1], p2[1]], color="red", linestyle="-")

        # Calculate the angle of the line
        width = abs(p1[0] - p2[0])
        height = abs(p1[1] - p2[1])
        angle_rad = np.arctan(height / width)  # radians
        angle_deg = angle_rad * 180 / np.pi  # degrees

        # Define the arc radius for drawing the angle
        if width < height:
            arc_radius = height / 2
        else:
            arc_radius = width / 2

        # Define the arc angle
        if p2[0] < p1[0] and p2[1] > p1[1]:
            arc_angle = 180 - angle_deg
        elif p2[0] < p1[0] and p2[1] < p1[1]:
            arc_angle = 180
        elif p2[0] > p1[0] and p2[1] < p1[1]:
            arc_angle = -angle_deg
        else:  # p2[0] > p1[0] and p2[1] > p1[1]:
            arc_angle = 0

        # Define points for texts
        if p2[0] < p1[0]:
            x_left = p2[0]
            x_right = p1[0]
        else:
            x_left = p1[0]
            x_right = p2[0]
        if p2[1] < p1[1]:
            y_upper = p1[1]
            y_lower = p2[1]
        else:
            y_upper = p2[1]
            y_lower = p1[1]

        # Draw the angle
        self.ax.add_patch(
            Arc(
                p1,
                width=2 * arc_radius,
                height=2 * arc_radius,
                angle=arc_angle,
                theta1=0,
                theta2=angle_deg,
                color="red",
                linestyle="--",
                zorder=2,
            )
        )

        # Draw the angle value
        self.ax.text(
            (p1[0] + p2[0]) / 2,
            y_lower - 1,
            f"a={angle_deg:.2f}°",
            horizontalalignment="center",
            fontsize=10,
            zorder=2,
        )

        # Draw width value
        self.ax.text(
            (p1[0] + p2[0]) / 2,
            y_upper,
            f"w={width:.2f}",
            horizontalalignment="center",
            fontsize=10,
            zorder=2,
        )

        # Draw height value
        self.ax.text(
            x_right,
            (p1[1] + p2[1]) / 2,
            f"h={height:.2f}",
            horizontalalignment="center",
            fontsize=10,
            zorder=2,
        )

        # Draw the plot
        self.canvas.draw_idle()

    def add_points_legend(self, event=None):
        """
        Adds a legend in the top center of the plot when setting points for aligning the plot.
        """
        if len(self.points) % 2 == 0:
            # Remove the previous text
            for text in self.ax.texts:
                if text.get_text() in ["Set point 1", "Set point 2"]:
                    text.remove()
            self.ax.text(
                self.cursor_limits["x_max"] / 2,
                self.cursor_limits["y_max"] * 0.9,
                "Set point 1",
                horizontalalignment="center",
                fontsize=10,
                zorder=2,
            )
        else:
            # Remove the previous text
            for text in self.ax.texts:
                if text.get_text() in ["Set point 1", "Set point 2"]:
                    text.remove()
            self.ax.text(
                self.cursor_limits["x_max"] / 2,
                self.cursor_limits["y_max"] * 0.9,
                "Set point 2",
                horizontalalignment="center",
                fontsize=10,
                zorder=2,
            )

        self.canvas.draw_idle()

    def reset_points_alignment(self):
        self.align_plot = False
        self.points = []

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
        i_weld = np.where(section > np.percentile(section, 10))[0]

        # Find the start and end points of the weld
        weld_start = i_weld[0]
        weld_end = i_weld[-1]

        # Plot a vertical line at the start and end points of the weld
        self.ax.axvline(x=weld_start, color="red", linestyle="--")
        self.ax.axvline(x=weld_end, color="red", linestyle="--")

        # Weld height lines

        # Calculate the top and bottom position of the weld
        if self.invert_plot:
            # Inverted
            weld_top = np.max(section) - np.mean(section[i_weld])
        else:
            weld_top = np.mean(section[i_weld])
        weld_bot = np.min(np.delete(section, i_weld))  # surface height

        # Plot a horizontal line at the top and bottom points of the weld
        self.ax.axhline(y=weld_top, color="red", linestyle="--")
        self.ax.axhline(y=weld_bot, color="red", linestyle="--")

        # Print the information
        txt = (
            f"\nThe weld starts at x = {weld_start:.2f} and ends at x = {weld_end:.2f}."
        )
        txt += (
            f"\nThe weld starts at z = {weld_bot:.2f} and ends at z = {weld_top:.2f}."
        )
        self.master.change_console_text(txt, "INFORMATION")

    def reset_guides_defects(self):
        """
        Resets the guide lines for defects.
        """
        # remove previous guide lines
        for line in self.ax.lines:
            if line.get_label() in ["thickness", "height"]:
                line.remove()
        for child in self.ax.get_children():
            if isinstance(child, Line2D) and child.get_label() in [
                "thickness",
                "height",
            ]:
                child.remove()
        # Remove the text
        for text in self.ax.texts:
            text.remove()
        # Reset the work piece thickness and height of the weld
        self.work_piece_thickness = 0.0
        self.height_of_weld = 0.0
        self.x_position_of_weld = 0.0
        # Redraw the canvas
        self.update_window()

    def add_guides_defects(self):
        """
        Adds a guide line for the thickness of the work piece.

        Args:
            y_position: The y position of the guide line.

        """
        self.work_piece_thickness = self.master.info_frame.work_piece_thickness.get()
        self.height_of_weld = self.master.info_frame.height_of_weld.get()
        self.x_position_of_weld = self.master.info_frame.x_position_of_weld.get()

        if self.work_piece_thickness > 0 and self.height_of_weld > 0:
            # Plot the guide line of the work piece thickness
            line = Line2D(
                [0, self.cursor_limits["x_max"]],
                [self.work_piece_thickness, self.work_piece_thickness],
                color=self.line_text_color,
                linestyle="--",
                label="thickness",
            )
            self.ax.add_line(line)
            print(f"X position of weld: {self.x_position_of_weld}")
            print(f"Height of weld: {self.height_of_weld}")
            print(f"Work piece thickness: {self.work_piece_thickness}")
            # Plot the guide line of the height of the weld using the thickness of the work piece as its x axis position
            if self.master.info_frame.defect_choice == "Excessive":
                y_min = self.work_piece_thickness
                y_max = self.work_piece_thickness + self.height_of_weld
                line = Line2D(
                    [self.x_position_of_weld, self.x_position_of_weld],
                    [y_min, y_max],
                    color=self.line_text_color,
                    linestyle="--",
                    label="height",
                )
                self.ax.add_line(line)
                self.ax.text(
                    self.x_position_of_weld,
                    y_max * 1.1,
                    f"H={self.height_of_weld:.2f}",
                    horizontalalignment="center",
                    fontsize=10,
                    zorder=2,
                )
            elif self.master.info_frame.defect_choice == "Sagging":
                y_min = self.work_piece_thickness - self.height_of_weld
                y_max = self.work_piece_thickness
                line = Line2D(
                    [self.x_position_of_weld, self.x_position_of_weld],
                    [y_min, y_max],
                    color=self.line_text_color,
                    linestyle="--",
                    label="height",
                )
                self.ax.add_line(line)
                self.ax.text(
                    self.x_position_of_weld,
                    y_min * 0.9,
                    f"H={self.height_of_weld:.2f}",
                    horizontalalignment="center",
                    fontsize=10,
                    zorder=2,
                )
            else:
                self.master.change_console_text(
                    "Plot error: Unknown defect type", "ERROR"
                )

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

    def apply_filter(self, data=None, choice=None):
        """
        Apply a filter to the given data and return the filtered section.

        Parameters:
        - data: The input data to be filtered (default: None)
        - choice: The choice of filter to be applied (default: None)

        Returns:
        - section: The filtered section of the data
        """
        # Interpolate (and possibly filter) the data first
        section = self.master.plot_control_frame.interpolate_and_filter(data, choice)

        # Invert the plot if the flag is set
        if self.invert_plot:
            inverted_section = -section
            # Move the inverted data back to zero
            min_value = np.min(inverted_section)
            section = inverted_section - min_value

        # Align the data
        if self.align_plot and len(self.points) == 2:
            section = self.master.plot_control_frame.lower_data(
                section, self.points[-2], self.points[-1]
            )

        # Set the data filtered
        self.row_filtered = section

        return section

    def update_surface(self, profile=0, data=None, choice=None):
        """
        Updates the plot surface.

        Args:
            current_profile: The index of the current frame.
            profile: The index of the current profile.
            data: The data to plot.
        """
        data = self.master.range_data
        try:
            # Apply the filter
            section = self.apply_filter(data[profile, :], choice)
            # Recreate the figure
            # Default color
            default_color = plt.rcParams["axes.prop_cycle"].by_key()["color"][0]
            # Save the positions of the cursors to recreate them later
            x_1_pos = self.x_1.line.get_xdata()[0]
            x_2_pos = self.x_2.line.get_xdata()[0]
            y_1_pos = self.y_1.line.get_ydata()[0]
            y_2_pos = self.y_2.line.get_ydata()[0]
            # Clean the plot
            self.ax.clear()
            # Plot style change
            # Define plot_title and axis labels
            plot_title = f"Profile {profile + 1}, Filter {choice}"
            x_label = "Width [mm]"
            y_label = "Height [mm]"
            self.update_plot_style(plot_title, x_label, y_label)
            # Remove and re-add the cursors to the plot
            for cursor in [self.y_1, self.y_2, self.x_1, self.x_2]:
                if cursor.line is not None:
                    cursor.line.remove()
                    cursor.line = None
            # Create the cursors in their previous positions
            self.y_1 = PlotCursor(self.master, self.ax, "y", y_1_pos, "Y - Max")
            self.y_2 = PlotCursor(self.master, self.ax, "y", y_2_pos, "Y - Min")
            self.x_1 = PlotCursor(self.master, self.ax, "x", x_1_pos, "X - Min")
            self.x_2 = PlotCursor(self.master, self.ax, "x", x_2_pos, "X - Max")
            # Create the cursors
            # self.create_cursors()
            # Update cursor limits
            self.update_cursor_limits()
            # Set the axes limits based on cursor positions
            self.set_axes_limits()
            # Plot the data
            # self.ax.plot(section, color=default_color)
            self.ax.plot(self.x_axis, section, color=default_color)
            # Set the plot title and axis labels
            self.ax.set_title(plot_title)
            self.ax.set_xlabel(x_label)
            self.ax.set_ylabel(y_label)
            # Add guide lines to the plot
            # self.add_guides(profile, data)
            # Add the guide line for the work piece thickness
            self.add_guides_defects()
            # Add legend to the plot saying select point 1 and 2
            if self.pointsEnabled:
                self.add_points_legend()
            # Remove points for clicked guide lines
            # self.points = []
            # Redraw the canvas
            self.canvas.draw_idle()
            # Update the plot window
            self.update_window()
            # Update the info frame
            self.master.update_info_frame()
        except Exception as e:
            self.master.change_console_text(
                f"Error during plot update: verify there is data imported and try again.",
                "ERROR",
            )
            print(e)

    def clean_plot(self):
        """
        Clean the plot.
        """
        self.ax.clear()
        self.update_window()

    def save_plot(self, file_name):
        """
        Save the plot as a .png file.
        """
        self.fig.savefig(file_name, dpi=300, bbox_inches="tight")

    def set_axis_dimensions(self, data_dict):
        """
        Set the axis dimensions of the plot.
        """
        self.initial_cursor_limits["x_max"] = data_dict["x_max"]
        self.initial_cursor_limits["y_max"] = data_dict["z_max"]
        self.cursor_limits["x_max"] = data_dict["x_max"]
        self.cursor_limits["y_max"] = data_dict["z_max"]
        self.x_axis = data_dict["x_axis"]
