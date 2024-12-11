from omni.kit.window.file_importer import get_file_importer

class FileManager:
    """
    Handles file selection using the File Importer extension.
    """

    def __init__(self, on_file_selected_callback):
        """
        Initializes the file manager.

        Args:
            on_file_selected_callback (function): Callback function to handle selected files.
        """
        self.on_file_selected_callback = on_file_selected_callback

    def open_file_picker(self):
        """
        Opens the file picker dialog to select a USD file.
        """
        file_importer = get_file_importer()  # Get the file importer instance

        # Open the file picker dialog with a simple configuration
        file_importer.show_window()  # Show the file picker dialog

        # Set the callback to handle file selection
        file_importer.set_on_files_selected_fn(self.on_file_selected_callback)


