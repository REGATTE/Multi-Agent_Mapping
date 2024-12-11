from omni.ui import Button, Frame, Alignment
from omni.kit.window.filepicker import FilePickerDialog


class FileManager:
    """
    Handles file selection using a folder/file picker icon.
    """

    def __init__(self, on_file_selected_callback):
        """
        Initializes the file manager.

        Args:
            on_file_selected_callback (function): Callback function to handle selected files.
        """
        self.on_file_selected_callback = on_file_selected_callback

    def add_file_picker(self, parent_ui):
        """
        Adds a file picker button to the UI.

        Args:
            parent_ui: The parent UI container where the picker will be added.
        """
        def on_click_fn(filename, path):
            # Callback when a file is selected
            if filename and filename.endswith(".usd"):
                print(f"[FileManager] USD File selected: {filename}")
                self.on_file_selected_callback([path])
            else:
                print("[FileManager] No valid USD file selected.")

        # Add the picker button to the parent UI
        self._add_folder_picker_icon(
            on_click_fn=on_click_fn,
            parent_ui=parent_ui,
            dialog_title="Select Robot USD File",
            button_title="Open File",
        )

    def _add_folder_picker_icon(
        self,
        on_click_fn,
        parent_ui,
        dialog_title="Select Folder",
        button_title="Select File",
    ):
        """
        Internal function to create the file picker button and its dialog.

        Args:
            on_click_fn: Callback function triggered when a file is selected.
            parent_ui: The parent UI container.
            dialog_title: Title for the dialog box.
            button_title: Title for the confirmation button.
        """
        def open_file_picker():
            def on_selected(filename, path):
                on_click_fn(filename, path)
                file_picker.hide()

            def on_canceled(a, b):
                print("[FileManager] File selection canceled.")
                file_picker.hide()

            file_picker = FilePickerDialog(
                dialog_title,
                allow_multi_selection=False,
                apply_button_label=button_title,
                click_apply_handler=lambda a, b: on_selected(a, b),
                click_cancel_handler=lambda a, b: on_canceled(a, b),
                enable_versioning_pane=True,
            )

        # Add the file picker button to the parent UI
        with Frame(parent=parent_ui, tooltip=button_title):
            Button(
                name="IconButton",
                width=100,
                height=25,
                clicked_fn=open_file_picker,
                text="Browse",  # Text for the button
                alignment=Alignment.LEFT_CENTER,
            )
