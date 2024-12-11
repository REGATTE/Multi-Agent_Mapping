import omni.ext
import omni.ui as ui
from .file_manager import FileManager
from .spawn import RobotSpawner


class RoboticsMultiagentMappingExtension(omni.ext.IExt):
    """
    Main class for the Multi-Agent Robotic Mapping extension.
    Manages the UI, user interactions, and connections to file management and robot spawning logic.
    """

    def on_startup(self, ext_id):
        """
        Called when the extension is enabled.
        Initializes UI components and sets up file manager and robot spawner.
        """
        print("[robotics.multiagent.mapping] Extension startup")

        # Initialize file manager and robot spawner
        self.file_manager = FileManager(self.on_file_selected)
        self.robot_spawner = None
        self.selected_usd_path = ""  # Path to the selected robot USD file

        # Create the main UI window
        self._window = ui.Window("Multi-Agent Robotic Mapping", width=400, height=200)
        with self._window.frame:
            with ui.VStack(spacing=10):
                self.setup_file_picker()  # Setup file picker section
                self.setup_robot_controls()  # Setup robot controls section

    def setup_file_picker(self):
        """
        Sets up the UI elements for selecting a robot USD file.
        """
        # File picker section with horizontal alignment
        with ui.HStack(height=30):
            ui.Label("Robot USD File:", width=ui.Percent(30), height=20)  # Label for the file picker
            self._file_path_label = ui.Label("No file selected", width=ui.Percent(50), height=20)  # Display selected file path
            ui.Button("Browse", width=ui.Percent(20), height=25, clicked_fn=self.file_manager.open_file_picker)  # Button to open file picker

    def setup_robot_controls(self):
        """
        Sets up the UI elements for specifying the number of robots and spawning them.
        """
        # Robot controls section with horizontal alignment
        with ui.HStack(height=30):
            ui.Label("Enter number of robots to spawn:", width=ui.Percent(50), height=20)  # Label for robot count input
            self._robot_count_field = ui.IntField(width=ui.Percent(50), height=25)  # Input field for robot count
            self._robot_count_field.model.set_value(1)  # Default value

        # Label to display feedback messages
        self._message_label = ui.Label("", height=20)

        # Spawn Robots button
        ui.Button("Spawn Robots", width=ui.Percent(100), height=30, clicked_fn=self.spawn_robots)

    def on_file_selected(self, file_paths):
        """
        Callback triggered when a file is selected using the file picker.

        Args:
            file_paths (list): List of selected file paths.
        """
        if file_paths and len(file_paths) > 0:
            self.selected_usd_path = file_paths[0]  # Take the first selected file
            self._file_path_label.text = f"Selected: {self.selected_usd_path}"  # Update the label
            self.robot_spawner = RobotSpawner(self._message_label)  # Initialize the robot spawner

    def spawn_robots(self):
        """
        Callback for the 'Spawn Robots' button.
        Validates input and triggers the robot spawning process.
        """
        if not self.robot_spawner or not self.selected_usd_path:
            self._message_label.text = "Please select a robot USD file before spawning."
            return

        num_of_robots = self._robot_count_field.model.get_value_as_int()
        self.robot_spawner.spawn_robots(num_of_robots, self.selected_usd_path)

    def on_shutdown(self):
        """
        Called when the extension is disabled.
        Cleans up resources if necessary.
        """
        print("[robotics.multiagent.mapping] Extension shutdown")
