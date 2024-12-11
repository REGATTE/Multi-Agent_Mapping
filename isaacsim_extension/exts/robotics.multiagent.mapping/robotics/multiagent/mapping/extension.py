import omni.ext
import omni.ui as ui
from .spawn import RobotSpawner
from .file_manager import FileManager

class RoboticsMultiagentMappingExtension(omni.ext.IExt):
    """
    Main class for the Multi-Agent Robotic Mapping extension.
    Manages the UI and file selection logic.
    """

    def on_startup(self, ext_id):
        print("[robotics.multiagent.mapping] Extension startup")

        # Initialize file manager
        self.file_manager = FileManager(self.on_file_selected)

        # Create UI window
        self._window = ui.Window("Multi-Agent Robotic Mapping", width=400, height=200)
        with self._window.frame:
            with ui.VStack(spacing=10):
                self.setup_file_picker()  # Setup the file picker
                self.setup_robot_controls()  # Setup robot controls

    def setup_file_picker(self):
        """
        Sets up the file picker UI components.
        """
        ui.Label("Robot USD File:", height=20)
        with ui.HStack(height=30):
            # Display for selected file path
            self._file_path_label = ui.Label("No file selected", height=20)

            # Add file picker button
            self.file_manager.add_file_picker(parent_ui=ui.HStack())

    def on_file_selected(self, file_paths):
        """
        Callback triggered when a file is selected.

        Args:
            file_paths (list): List of selected file paths.
        """
        if file_paths and len(file_paths) > 0:
            self.selected_usd_path = file_paths[0]
            self._file_path_label.text = f"Selected: {self.selected_usd_path}"
            print(f"[Extension] File selected: {self.selected_usd_path}")
        else:
            print("[Extension] No file selected.")

    def setup_robot_controls(self):
        """
        Sets up the controls for entering robot count and spawning.
        """
        ui.Label("Enter number of robots to spawn:", height=20)
        self._robot_count_field = ui.IntField(width=200, height=25)
        self._robot_count_field.model.set_value(1)

        ui.Button(
            "Spawn Robots",
            height=30,
            clicked_fn=self.spawn_robots
        )

    def spawn_robots(self):
        """
        Spawns robots using the selected USD file.
        """
        if not hasattr(self, "selected_usd_path"):
            print("[Extension] Please select a USD file before spawning.")
            return

        num_of_robots = self._robot_count_field.model.get_value_as_int()
        print(f"Spawning {num_of_robots} robots using {self.selected_usd_path}")
