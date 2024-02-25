import tkinter
import customtkinter as ctk

class AutocompleteEntry(ctk.CTkComboBox):
    def __init__(self, *args, **kwargs):
        ctk.CTkComboBox.__init__(self, *args, **kwargs)
        self.autocomplete_list = ["profile 1", "timestamp 00:00:00.000"]
        # With keyrelease autofill
        self.bind("<KeyRelease>", lambda event: self.handle_keyrelease(event))

    def set_completion_list(self, completion_list):
        self.autocomplete_list = sorted(completion_list)

    def autocomplete(self, delta=0):
        prefix = self.get().strip()
        completion = [item for item in self.autocomplete_list if item.startswith(prefix)]

        if completion:
            self.set(completion[0])

    def handle_keyrelease(self, event):
        if event.keysym in ('BackSpace', 'Left', 'Right', 'Up', 'Down'):
            return
        if event.keysym == "Return":
            self.autocomplete(delta=1)
        else:
            self.autocomplete(delta=0)