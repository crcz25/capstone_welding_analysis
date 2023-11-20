import customtkinter as ctk
import datetime

from frames.MenuFrame import MenuFrame
from frames.PlotFrame import PlotFrame
from frames.InfoFrame import InfoFrame
from frames.PlotControlFrame import PlotControlFrame
from frames.CameraSettingsFrame import CameraSettingsFrame

ctk.set_appearance_mode("System")
ctk.set_default_color_theme("dark-blue")

#--------------------------------------------------------GUI BASE--------------------------------------------------------#
class App(ctk.CTk):
    def __init__(self):
        super().__init__()

        self.title("GUI for SICK E55 Weld Monitoring")
        self.geometry(f"{1200}x{600}")
        self.minsize(1200, 600)

        self.grid_columnconfigure(1, weight=1)
        self.grid_columnconfigure((2, 3), weight=0)
        self.grid_rowconfigure((0, 1), weight=1)

        # Add Frames to the base
        self.menu_frame = MenuFrame(self)
        self.plot_frame = PlotFrame(self)
        self.info_frame = InfoFrame(self)
        self.plot_control_frame = PlotControlFrame(self)
        self.settings_frame = CameraSettingsFrame(self)

        # Set default values
        self.menu_frame.appearance_mode_optionmenu.set("Dark")
        self.menu_frame.scaling_optionmenu.set("100%")  
        self.plot_control_frame.slider.set(0)
        self.settings_frame.grid_remove()

        # These are here for example now
        scan_name = "Test1"
        scan_time = datetime.datetime.now()

        # Insert data to textboxes
        self.info_frame.textbox_info.insert("0.0", f"Scan: {scan_name}" + '\n' + f"Time: {scan_time}")
        self.info_frame.textbox_alerts.insert("0.0", "Defects:")

        # These disable writing in the right frame textboxes
        self.info_frame.textbox_info.configure(state="disabled")
        self.info_frame.textbox_alerts.configure(state="disabled")


    #--------------------------------------------------------FUNCTIONALITY--------------------------------------------------------#
    def change_appearance_mode_event(self, new_appearance_mode: str):
        ctk.set_appearance_mode(new_appearance_mode)
        self.show_default_frames()

    def change_scaling_event(self, new_scaling: str):
        new_scaling_float = int(new_scaling.replace("%", "")) / 100
        ctk.set_widget_scaling(new_scaling_float)
        self.show_default_frames()

    def change_button_text(self, button, text_1, text_2):
        current_state = button.cget("text")
        new_state = text_2 if current_state == text_1 else text_1
        button.configure(text=new_state)

    def menu_button(self, frame_type):
        if frame_type == "home":
            self.show_default_frames()
        elif frame_type == "camera":
            self.show_camera_frames()

    def show_camera_frames(self):
        self.info_frame.grid_remove()
        self.plot_control_frame.grid_remove()
        self.plot_frame.grid_remove()
        self.settings_frame.grid()

    def show_default_frames(self):
        self.settings_frame.grid_remove()
        self.info_frame.grid()
        self.plot_control_frame.grid()    


if __name__ == "__main__":
    app = App()
    app.mainloop()
