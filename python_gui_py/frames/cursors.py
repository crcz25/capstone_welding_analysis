import matplotlib.pyplot as plt
import matplotlib.lines as lines

# --------------------------------------------------------CURSORS--------------------------------------------------------#
class PlotCursor():
    def __init__(self, ax, axis_name, axis, label):
        # Get canvas and assign basic variables
        self.ax = ax
        self.canvas = ax.get_figure().canvas
        self.axis_name = axis_name
        self.axis = axis

        # Define axis size
        if axis_name == "y":
            self.x = [0, 1600]
            self.y = [axis, axis]
        elif axis_name == "x":
            self.x = [axis, axis]
            self.y = [0, 70]

        # Create lines on the plot
        self.label = (f"{label}, {axis}")
        self.text_label = self.ax.text(self.x[0], self.y[0], self.label, color="black", fontsize=8, ha='center', va='center')
        self.line = lines.Line2D(self.x, self.y, picker=5, color="red", linewidth=2, label=label)
        self.ax.add_line(self.line)
        self.canvas.draw_idle()
        self.sid = self.canvas.mpl_connect('pick_event', self.clickonline)
    

    def clickonline(self, event):
        if event.artist == self.line:
            print("Selected line:", event.artist)
            print(self.line)
            self.follower = self.canvas.mpl_connect("motion_notify_event", self.followmouse)
            self.releaser = self.canvas.mpl_connect("button_press_event", self.releaseonclick)

    def followmouse(self, event):
        
        # Remove the previous text label and create a new one
        if hasattr(self, 'text_label') and self.text_label is not None:
            self.text_label.remove()

        self.text_label = self.ax.text(self.x[0], self.y[0], self.label, color="black", fontsize=8, ha='center', va='center')

        # Set the line and label at new mouse position
        if self.axis_name == "y":
            self.line.set_ydata([event.ydata, event.ydata])
            label_x = (self.line.get_xdata()[0] + self.line.get_xdata()[1]) / 2
            label_y = self.line.get_ydata()[0]
        else:
            self.line.set_xdata([event.xdata, event.xdata])
            label_x = self.line.get_xdata()[0]
            label_y = (self.line.get_ydata()[0] + self.line.get_ydata()[1]) / 2

        # Update the position attributes
        if self.axis_name == "y":
            self.axis = event.ydata
        else:
            self.axis = event.xdata

        # Update the position of the text label
        self.text_label.set_position((label_x, label_y))
        # Update info frame
        self.canvas.draw_idle()

    def releaseonclick(self, event):
        print(self.axis_name)
        self.minimum_x = 0
        self.minimum_y = 0
        self.maximum_x = 1600
        self.maximum_y = 70

        # Assuming self.line is the line that was clicked on

        if self.axis_name == "y":
            ydata = self.line.get_ydata()
            print(ydata)

            # Set Y - axis maximum
            if self.line in self.ax.get_lines():
                if self.line is self.ax.get_lines()[0]:
                    if self.minimum_y <= ydata[0] <= self.maximum_y:
                        self.ax.set_ylim(self.ax.get_ylim()[0], ydata[0])
                    else:
                        self.reset_cursors()

                # Set Y - axis minimum
                elif self.line is self.ax.get_lines()[1]:
                    if self.maximum_y >= ydata[0] >= self.minimum_y:
                        self.ax.set_ylim(ydata[0], self.ax.get_ylim()[1])
                    else:
                        self.reset_cursors()
        else:
            xdata = self.line.get_xdata()
            print(xdata)

            if self.line in self.ax.get_lines():  # Check if the line is in the list of lines
                if self.line is self.ax.get_lines()[2]:  # Fourth line is for "x1 minimum"
                    if self.maximum_x >= xdata[0] >= self.minimum_x:
                        self.ax.set_xlim(xdata[0], self.ax.get_xlim()[1])
                    else:
                        self.reset_cursors()

                elif self.line is self.ax.get_lines()[3]:  # Third line is for "x2 maximum"
                    if self.maximum_x >= xdata[0] >= self.minimum_x:
                        self.ax.set_xlim(self.ax.get_xlim()[0], xdata[0])
                    else:
                        self.reset_cursors()
        self.canvas.mpl_disconnect(self.releaser)
        self.canvas.mpl_disconnect(self.follower)
        self.canvas.draw_idle()

    def reset_cursors(self):
        # Reset to the initial position
        if self.axis_name == "y":
            y_min, y_max = self.ax.get_ylim()
            self.ax.set_ylim(max(0, y_min), min(70, y_max))
        else:
            x_min, x_max = self.ax.get_xlim()
            self.ax.set_xlim(max(0, x_min), min(1600, x_max))
        self.canvas.draw_idle()