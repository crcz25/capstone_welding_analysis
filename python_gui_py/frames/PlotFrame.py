import customtkinter as ctk
import matplotlib.pyplot as plt
import numpy as np
import seaborn as sns
from matplotlib.backend_bases import key_press_handler
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
from frames.cursors import PlotCursor
import threading
import time


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

        # Default cursor limits
        self.initial_cursor_limits = {"x_min": 0, "x_max": 1600, "y_min": 0, "y_max": 70}
        self.cursor_limits = self.initial_cursor_limits.copy()

        # Inital cursors
        self.y_1 = PlotCursor(self.ax, "y", self.cursor_limits["y_max"], "Y - Max")
        self.y_2 = PlotCursor(self.ax, "y", self.cursor_limits["y_min"], "Y - Min")
        self.x_1 = PlotCursor(self.ax, "x", self.cursor_limits["x_min"], "X - Min")
        self.x_2 = PlotCursor(self.ax, "x", self.cursor_limits["x_max"], "X - Max")

        self.not_stopped_thread = True
        self.data = None
        

    # --------------------------------------------------------FUNCTIONALITY--------------------------------------------------------#
    def update_info_frame(self):
        """

        Update the info frame with the newest info every second

        """
        while self.not_stopped_thread:
            # Check if the update is in progress and acquire the lock
            if not self.master.info_frame.update_in_progress and self.master.info_frame.update_lock.acquire(blocking=False):
                try:
                    # Set the update flag to True
                    self.master.info_frame.update_in_progress = True

                    # Enable the textbox
                    self.master.info_frame.textbox_info.configure(state="normal")

                    # Check if there is existing text
                    if len(self.master.info_frame.textbox_info.get("1.0", "end-1c")) > 0:
                        # Delete the current text
                        self.master.info_frame.textbox_info.delete("1.0", "end")

                    # Insert the new text
                    txt = f"""Scan: {self.master.range_file[0].split("/")[-1]} \
                        \nTime: {self.master.timestamp_data[self.master.current_frame]} \
                        \nFrame: {self.master.current_frame + 1} \
                        \nProfile: {int(self.master.plot_control_frame.slider.get()) + 1} \
                        \nX-Min: {self.x_1.line.get_xdata()[0]} \
                        \nX-Max: {self.x_2.line.get_xdata()[0]} \
                        \nY-Min: {self.y_2.line.get_ydata()[0]} \
                        \nY-Max: {self.y_1.line.get_ydata()[0]}"""
                    self.master.info_frame.textbox_info.insert("0.0", txt)

                    # Disable the textbox
                    self.master.info_frame.textbox_info.configure(state="disabled")

                finally:
                    # Release the lock
                    self.master.info_frame.update_lock.release()

                    # Set the update flag to False
                    self.master.info_frame.update_in_progress = False

            time.sleep(1.0)


    def start_update_info_frame(self):
        """

        Start the function update_info_frame() on a thread

        """
        self.not_stopped_thread = True
        # Create a separate thread for your update loop
        self.update_thread = threading.Thread(target=self.update_info_frame)

        # Start the thread
        self.update_thread.start()
    

    def stop_update_info_frame(self):
        """

        Stop the thread

        """

        self.not_stopped_thread = False
        self.update_thread.join()
        time.sleep(0.25)

    def create_cursors(self):
        """

        Creates 4 cursors. Calls the PlotCursor class

        Args:
            current_frame: the current frame.
            axis: to which axis the cursor is plotted.
            limits: cursor limits.
            label: cursor name.
        """

        # Initialize cursors with the stored initial limits
        self.y_1 = PlotCursor(self.ax, "y", self.cursor_limits["y_max"], "Y - Max")
        self.y_2 = PlotCursor(self.ax, "y", self.cursor_limits["y_min"], "Y - Min")
        self.x_1 = PlotCursor(self.ax, "x", self.cursor_limits["x_min"], "X - Min")
        self.x_2 = PlotCursor(self.ax, "x", self.cursor_limits["x_max"], "X - Max")


    def reset_cursors(self, current_frame, profile, data, choice):
        """
        Resets the cursors to their initial limits.

        """
        self.update_cursor_limits()
        if self.cursor_limits != self.initial_cursor_limits:
            # Update the cursor limits with the initial limits
            self.cursor_limits = self.initial_cursor_limits.copy()

            # Update cursors based on the initial limits
            self.y_1.line.set_ydata([self.cursor_limits["y_max"], self.cursor_limits["y_max"]])
            self.y_2.line.set_ydata([self.cursor_limits["y_min"], self.cursor_limits["y_min"]])
            self.x_1.line.set_xdata([self.cursor_limits["x_min"], self.cursor_limits["x_min"]])
            self.x_2.line.set_xdata([self.cursor_limits["x_max"], self.cursor_limits["x_max"]])

            # Redraw the canvas
            self.canvas.draw_idle()
            self.stop_update_info_frame()
            self.create_figure(current_frame, profile, data)
            self.start_update_info_frame()
            self.master.change_console_text("Plot and cursors reset", 'INFORMATION')
        else:
            self.master.change_console_text("Cursors are already in their initial positions", 'ERROR')
    

    def update_cursor_limits(self):
        """

        Updates the cursors limits with each adjustment.

        """  

        self.cursor_limits["x_min"] = self.x_1.line.get_xdata()[0]
        self.cursor_limits["x_max"] = self.x_2.line.get_xdata()[0]
        self.cursor_limits["y_max"] = self.y_1.line.get_ydata()[0]
        self.cursor_limits["y_min"] = self.y_2.line.get_ydata()[0]
    

    def create_figure(self, current_frame=0, profile=0, data=None, choice=None):
        """
        Creates a new plot figure.

        Args:
            current_frame: The index of the current frame.
            profile: The index of the current profile.
            data: The data to plot.
            choice: Filter type
        """
        self.ax.clear()
        self.create_cursors()
        self.start_update_info_frame()

        if choice is None or choice == "No Filter":
            section = data[current_frame, profile, :]
            self.ax.plot(section)
            plot_title = f"Frame {current_frame + 1}, Profile {profile + 1}"
        else:
            section = data
            self.ax.plot(section)
            plot_title = f"Frame {current_frame + 1}, Profile {profile + 1}, Filter {choice}"
        print(f"Section shape: {section.shape}")
        print(f"Section: {section}")

        # Set the plot title
        self.ax.set_title(plot_title)
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
        choice = self.master.plot_control_frame.choice

        # Get the first color in the default color cycle (keep the color always the default blue)
        default_color = plt.rcParams['axes.prop_cycle'].by_key()['color'][0]

        # Remove the old plot line without clearing the axes
        for line in self.ax.lines:
            if line.get_label() == '_line0' or line.get_label().startswith("_child"):
                line.remove()

        # Remove the old cursor lines from the plot
        for cursor in [self.y_1, self.y_2, self.x_1, self.x_2]:
            if cursor.line is not None:
                cursor.line.remove()

        if choice is None or choice == "No Filter":
            plot_title = f"Frame {current_frame + 1}, Profile {profile + 1}"
            section = data[current_frame, profile, :]
            self.ax.plot(section, label='_line0', color=default_color)
        else:
            section = data[current_frame, profile, :]
            data_filtered_smoothed = self.master.plot_control_frame.interpolate_and_filter(section, choice)
            plot_title = f"Frame {current_frame + 1}, Profile {profile + 1}, Filter {choice}"
            self.ax.plot(data_filtered_smoothed, label='_line0', color=default_color)


        self.update_cursor_limits()

        # Add the cursor lines back to the plot
        for cursor in [self.y_1, self.y_2, self.x_1, self.x_2]:
            if cursor.line is not None:
                self.ax.add_line(cursor.line)

        # Set the slider position to the current profile
        self.master.plot_control_frame.slider.set(profile)

        # Set the plot title
        self.ax.set_title(plot_title)

        # Redraw the canvas
        self.canvas.draw_idle()

        # Update the plot window
        self.update_window()
