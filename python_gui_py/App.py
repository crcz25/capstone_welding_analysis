import customtkinter as ctk
import numpy as np
import rclpy
from customtkinter import filedialog
from frames.InfoFrame import InfoFrame
from frames.MenuFrame import MenuFrame
from frames.NodeThread import ROSNodeThread
from frames.PlotControlFrame import PlotControlFrame
from frames.PlotFrame import PlotFrame
from frames.SettingsFrame import SettingsFrame

ctk.set_appearance_mode("System")
ctk.set_default_color_theme("dark-blue")


# --------------------------------------------------------GUI BASE--------------------------------------------------------#
class App(ctk.CTk):
    def __init__(self):
        super().__init__()

        self.title("GUI for SICK E55 Weld Monitoring")
        self.geometry(f"{1400}x{900}")
        self.minsize(1400, 900)

        self.grid_columnconfigure(1, weight=1)
        self.grid_columnconfigure((2, 3), weight=0)
        self.grid_rowconfigure((0, 1), weight=1)

        # Add Frames to the base
        self.menu_frame = MenuFrame(self)
        self.plot_frame = PlotFrame(self)
        self.info_frame = InfoFrame(self)
        # Create the thread for the node
        # thread = threading.Thread()
        self.plot_control_frame = PlotControlFrame(self)
        self.settings_frame = SettingsFrame(self)

        # Set default values
        self.menu_frame.appearance_mode_optionmenu.set("Dark")
        self.menu_frame.scaling_optionmenu.set("100%")
        self.plot_control_frame.slider.set(0)
        self.settings_frame.grid_remove()

        # These disable writing in the right frame textboxes
        self.info_frame.textbox_info.configure(state="disabled")
        self.info_frame.textbox_alerts.configure(state="disabled")

        # ROS2 node
        self.ros_node_thread = None
        self.scanning_in_progress = False

        # Import files
        self.files = None
        self.range_file = None
        self.timestamp_file = None
        self.range_data = np.array([])
        self.timestamp_data = np.array([])

        # Visualization
        self.current_frame = 0
        self.max_frames = 0

    # --------------------------------------------------------FUNCTIONALITY--------------------------------------------------------#
    def change_appearance_mode_event(self, new_appearance_mode: str):
        ctk.set_appearance_mode(new_appearance_mode)
        self.show_default_frames() 
        self.change_console_text(self) # Make sure that text colors change when theme is changed

    def change_scaling_event(self, new_scaling: str):
        new_scaling_float = int(new_scaling.replace("%", "")) / 100
        ctk.set_widget_scaling(new_scaling_float)
        self.show_default_frames()

    def change_button_text(self, button, text_1, text_2):
        current_state = button.cget("text")
        new_state = text_2 if current_state == text_1 else text_1
        print(f"Changing button text from {current_state} to {new_state}")
        # Check if it is time to start or stop the scan
        if current_state == "Take scan":
            print("Starting scan")
            self.start_scan()
        elif current_state == "Stop scan":
            print("Stopping scan")
            self.stop_scan()

        button.configure(text=new_state)
    
    def change_console_text(self, text, tags=None):
        """
            Function to log text to the UI console.

            Parameters:
            - text (text to be shown).
            - tag (INFO: white, ERROR: red, SUCCESS: green)

        """
        print(text) # Kept this to print to terminal

        ap_mode = ctk.get_appearance_mode()

        # In light mode white color is not visible in the console
        # Create tags
        if ap_mode == "Dark":
            self.plot_control_frame.console_entry.tag_config("INFORMATION", foreground="white")
        else:
            self.plot_control_frame.console_entry.tag_config("INFORMATION", foreground="black")


        self.plot_control_frame.console_entry.tag_config("ERROR", foreground="red")
        self.plot_control_frame.console_entry.tag_config("SUCCESS", foreground="green")
        
        # Insert colored text to console
        self.plot_control_frame.console_entry.configure(state=ctk.NORMAL)
        self.plot_control_frame.console_entry.insert(ctk.END, tags + ": " + text + "\n", tags)
        self.plot_control_frame.console_entry.configure(state=ctk.DISABLED)

        # Keep focus on the end line
        self.plot_control_frame.console_entry.see("end")
    

    def menu_button(self, frame_type):
        if frame_type == "home":
            self.show_default_frames()
        elif frame_type == "settings":
            self.show_camera_frames()

    def show_camera_frames(self):
        self.info_frame.grid_remove()
        self.plot_control_frame.grid_remove()
        self.plot_frame.grid_remove()
        self.settings_frame.grid()

    def show_default_frames(self):
        self.settings_frame.grid_remove()
        self.info_frame.grid()
        self.plot_frame.grid()
        self.plot_control_frame.grid()

    def import_files(self):
        # Ask the user for the files to import timestamps and ranges (.csv and .npy)
        self.files = filedialog.askopenfilenames(
            title="Select files to import",
            filetypes=(
                ("all files", "*.*"),
                ("CSV files", "*.csv"),
                ("Numpy files", "*.npy"),
            ),
        )

        self.change_console_text(f"Files to import: {self.files}", 'INFORMATION')

        # Separate the files into timestamps and ranges
        # ranges are the npy files
        self.range_file = [f for f in self.files if f.endswith(".npy")]
        # open the npy file
        self.range_data = np.load(self.range_file[0])
        self.change_console_text(f"Range file to open: {self.range_file}", 'INFORMATION')
        self.change_console_text(f"Size of range data: {self.range_data.shape}", 'INFORMATION')

        # Set the max frames
        self.max_frames = self.range_data.shape[0] - 1
        # Set the slider to the max number of scans
        self.plot_control_frame.slider.configure(to=self.range_data.shape[1] - 1)

        # timestamps are the csv files
        self.timestamp_file = [f for f in self.files if f.endswith(".csv")]
        # open the csv file
        self.timestamp_data = np.genfromtxt(self.timestamp_file[0], delimiter=",")
        self.change_console_text(f"Timestamp file to open: {self.timestamp_file}", 'INFORMATION')
        self.change_console_text(f"Size of timestamp data: {self.timestamp_data.shape}", 'INFORMATION')

        # Update the plot
        self.plot_frame.create_figure(
            current_frame=self.current_frame, profile=0, data=self.range_data
        )


    def change_plot(self, change, profile=0):
        # Check if the data is loaded
        if self.range_file is not None and self.timestamp_file is not None:
            print(f"Current frame: {self.current_frame}, Max frames: {self.max_frames}")
            print(f"Change: {change}")
            self.current_frame += change
            # Check if the frame is out of bounds
            if self.current_frame < 0:
                self.current_frame = 0
            elif self.current_frame > self.max_frames:
                self.current_frame = self.max_frames
            print(f"New frame: {self.current_frame}")
            # Update the plot surface
            self.plot_frame.update_surface(
                current_frame=self.current_frame, profile=profile, data=self.range_data
            )
        else:
            self.change_console_text("Data is not loaded", 'ERROR')

    def call_scan(self):
        self.plot_control_frame.take_stop_scan()

    def start_scan(self):
        if not self.scanning_in_progress:
            self.scanning_in_progress = True
            self.ros_node_thread = ROSNodeThread()
            self.ros_node_thread.start()
            self.change_console_text("Scan started", 'INFORMATION')
        else:
            self.change_console_text("Scan did not start", 'INFORMATION')

    def stop_scan(self):
        if self.scanning_in_progress:
            # Stop the node
            self.scanning_in_progress = False
            self.ros_node_thread.stop_event.set()
            self.ros_node_thread.join()
            self.change_console_text("Scan stopped", 'INFORMATION')
            ## Save the data ##
            # Ask the user where to save the data
            self.csv_file_path = filedialog.askdirectory()
            self.change_console_text(f"Saving data to {self.csv_file_path}", 'SUCCESS')
            self.ros_node_thread.save_data(self.csv_file_path)


if __name__ == "__main__":
    try:
        rclpy.init(args=None)
        app = App()
        app.mainloop()
    except KeyboardInterrupt:
        app.destroy()
        rclpy.shutdown()
    finally:
        print("Exiting")
        rclpy.shutdown()
        exit()
