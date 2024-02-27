import customtkinter as ctk
import matplotlib.lines as lines
import matplotlib.pyplot as plt


# --------------------------------------------------------CURSORS--------------------------------------------------------#
class PlotCursor(ctk.CTkFrame):
    """
    A class representing a cursor on a plot.

    Parameters:
    - master: The master widget.
    - ax: The matplotlib axes object on which the cursor will be displayed.
    - axis_name: The name of the axis ('x' or 'y') along which the cursor will move.
    - axis: The initial position of the cursor along the specified axis.
    - label: The label of the cursor.
    - minimum_x: The minimum value for the x-axis.
    - minimum_y: The minimum value for the y-axis.
    - maximum_x: The maximum value for the x-axis.
    - maximum_y: The maximum value for the y-axis.
    - **kwargs: Additional keyword arguments to be passed to the ctk.CTkFrame constructor.
    """
    def __init__(
        self,
        master,
        ax,
        axis_name,
        axis,
        label,
        minimum_x=0,
        minimum_y=0,
        maximum_x=1600,
        maximum_y=70,
        **kwargs,
    ):
        super().__init__(master, bg_color="transparent", **kwargs)
        # Get canvas and assign basic variables
        self.ax = ax
        self.canvas = ax.get_figure().canvas
        self.axis_name = axis_name
        self.axis = axis
        self.minimum_x, self.minimum_y = minimum_x, minimum_y
        self.maximum_x, self.maximum_y = maximum_x, maximum_y

        # Define axis size
        if axis_name == "y":
            self.x = [self.minimum_x, self.maximum_x]
            self.y = [axis, axis]
        elif axis_name == "x":
            self.x = [axis, axis]
            self.y = [self.minimum_y, self.maximum_y]

        # Create lines on the plot
        self.label = f"{label}"
        self.line = lines.Line2D(
            self.x, self.y, picker=10, color="red", linewidth=2, label=label
        )
        self.ax.add_line(self.line)
        self.canvas.draw_idle()
        self.line_id = self.canvas.mpl_connect("pick_event", self.click_on_line)
        self.follower = None
        self.releaser = None

    # --------------------------------------------------------FUNCTIONALITY--------------------------------------------------------#
    def click_on_line(self, event):
        """
        Create follower, releaser for the clicked line
        """
        if event.artist == self.line:
            print("Selected line:", self.line)
            self.follower = self.canvas.mpl_connect(
                "motion_notify_event", self.follow_mouse
            )
            self.releaser = self.canvas.mpl_connect(
                "button_press_event", self.release_on_click
            )
        else:
            self.disconnect_and_redraw()

    def disconnect_and_redraw(self):
        """
        Disconnect the line and then update the plot
        """
        if hasattr(self, "releaser"):
            self.canvas.mpl_disconnect(self.releaser)
        if hasattr(self, "follower"):
            self.canvas.mpl_disconnect(self.follower)

        self.remove_label()
        self.canvas.draw_idle()

    def remove_label(self):
        """
        Removes the text label associated with the cursor, if it exists.
        """
        if hasattr(self, "text_label") and self.text_label is not None:
            self.text_label.remove()
            self.text_label = None

    def follow_mouse(self, event):
        """
        Line follows the mouse position and updates the text label with the current position.
        """
        # Check if the mouse position is valid
        if event.xdata is None or event.ydata is None:
            return

        self.remove_label()

        if self.axis_name == "y":
            ydata = event.ydata
            ydata = max(
                0, min(ydata, self.maximum_y)
            )  # Ensure ydata stays within plot limits

            self.text_label = self.ax.text(
                self.x[0],
                ydata,
                f"{self.line.get_label()} (mm): {ydata:.2f}",
                color="black",
                fontsize=8,
                ha="center",
                va="center",
            )

            # Set the line at the new mouse position
            self.line.set_ydata([ydata, ydata])
            label_x = (self.ax.get_xlim()[0] + self.ax.get_xlim()[1]) / 2
            label_y = ydata  # Updated to use the new mouse position
        else:
            xdata = event.xdata
            xdata = max(
                0, min(xdata, self.maximum_x)
            )  # Ensure xdata stays within plot limits

            self.text_label = self.ax.text(
                xdata,
                self.y[0],
                f"{self.line.get_label()} (mm): {xdata:.2f}",
                color="black",
                fontsize=8,
                ha="center",
                va="center",
            )

            # Set the line at the new mouse position
            self.line.set_xdata([xdata, xdata])
            label_x = xdata  # Updated to use the new mouse position
            label_y = (self.ax.get_ylim()[0] + self.ax.get_ylim()[1]) / 2

        # Update the position of the text label
        self.text_label.set_position((label_x, label_y))
        # Update info frame
        self.canvas.draw_idle()

    def release_on_click(self, event):
        """
        When released, plot limits are changed and lines are moved to the current mouse position.
        """
        # Assuming self.line is the line that was clicked on
        if self.axis_name == "y":
            ydata = self.line.get_ydata()

            # Set Y - axis maximum
            if self.line in self.ax.get_lines():
                if self.line.get_label() == "Y - Max":
                    if self.minimum_y < ydata[0] <= self.maximum_y:
                        self.ax.set_ylim(self.ax.get_ylim()[0], ydata[0])
                    else:
                        self.reset_cursors()

                # Set Y - axis minimum
                elif self.line.get_label() == "Y - Min":
                    if self.maximum_y > ydata[0] >= self.minimum_y:
                        self.ax.set_ylim(ydata[0], self.ax.get_ylim()[1])
                    else:
                        self.reset_cursors()
        else:
            xdata = self.line.get_xdata()

            if self.line in self.ax.get_lines():
                # Set X - axis minimum
                if (
                    self.line.get_label() == "X - Min"
                ):  # Fourth line is for "x1 minimum"
                    if self.maximum_x > xdata[0] >= self.minimum_x:
                        self.ax.set_xlim(xdata[0], self.ax.get_xlim()[1])
                    else:
                        self.reset_cursors()

                # Set X - axis maximum
                elif (
                    self.line.get_label() == "X - Max"
                ):  # Third line is for "x2 maximum"
                    if self.maximum_x >= xdata[0] > self.minimum_x:
                        self.ax.set_xlim(self.ax.get_xlim()[0], xdata[0])
                    else:
                        self.reset_cursors()

        self.disconnect_and_redraw()

        # Update info frame
        new_limits = [self.ax.get_xlim(), self.ax.get_ylim()]
        self.master.update_info_frame(new_limits=new_limits)

    def reset_cursors(self):
        """
        Plot and lines go to their initial positions/values.
        """
        # Reset to the initial position
        if self.axis_name == "y":
            y_min, y_max = self.ax.get_ylim()
            self.ax.set_ylim(max(self.minimum_y, y_min), min(self.maximum_y, y_max))

            # Update cursor positions
            self.line.set_ydata([y_max, y_max])
        else:
            x_min, x_max = self.ax.get_xlim()
            self.ax.set_xlim(max(self.minimum_x, x_min), min(self.maximum_x, x_max))

            # Update cursor positions
            self.line.set_xdata([x_min, x_min])

        self.disconnect_and_redraw()
