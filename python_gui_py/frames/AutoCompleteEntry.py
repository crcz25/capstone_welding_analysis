import tkinter

import customtkinter as ctk


class AutocompleteEntry(ctk.CTkComboBox):
    """
    A custom combobox widget that provides autocomplete functionality.

    Attributes:
        autocomplete_list (list): The list of items used for autocompletion.

    Methods:
        set_completion_list(completion_list): Sets the completion list for autocompletion.
        autocomplete(delta=0): Autocompletes the entry based on the current input.
        handle_keyrelease(event): Handles the key release event for autocompletion.
    """

    def __init__(self, *args, **kwargs):
        ctk.CTkComboBox.__init__(self, *args, **kwargs)
        self.autocomplete_list = ["profile 1", "timestamp 00:00:00.000"]
        # With keyrelease autofill
        self.bind("<KeyRelease>", lambda event: self.handle_keyrelease(event))

    def set_completion_list(self, completion_list):
        """
        Sets the completion list for autocompletion.

        Args:
            completion_list (list): The list of items to be used for autocompletion.
        """
        self.autocomplete_list = sorted(completion_list)

    def autocomplete(self, delta=0):
        """
        Autocompletes the entry based on the current input.

        Args:
            delta (int): The direction of autocompletion. 0 for forward, 1 for backward.
        """
        prefix = self.get().strip()
        completion = [
            item for item in self.autocomplete_list if item.startswith(prefix)
        ]

        if completion:
            self.set(completion[0])

    def handle_keyrelease(self, event):
        """
        Handles the key release event for autocompletion.

        Args:
            event (Event): The key release event.
        """
        if event.keysym in ("BackSpace", "Left", "Right", "Up", "Down"):
            return
        if event.keysym == "Return":
            self.autocomplete(delta=1)
        else:
            self.autocomplete(delta=0)
