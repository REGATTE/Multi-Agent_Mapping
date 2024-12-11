import os
import omni.ext
import omni.ui as ui
import omni.usd
from .spawn import RobotSpawner
from .file_manager import FileManager


class RoboticsMultiagentMappingExtension(omni.ext.IExt):
    """
    Main class for the Multi-Agent Robotic Mapping extension.
    Manages the UI and file selection logic.
    """

    def on_startup(self, ext_id):
        print("[robotics.multiagent.mapping] Extension startup")

        # Initialize file manager and robot spawner
        self.file_manager = FileManager(self.on_file_selected)
        self.robot_spawner = None

        # Create UI window
        self._window = ui.Window("Multi-Agent Robotic Mapping", width=400, height=300)
        with self._window.frame:
            with ui.VStack(spacing=0):  # Set vertical spacing to 0
                self.setup_file_picker()  # Setup the file picker
                self.setup_robot_controls()  # Setup robot controls

    def setup_file_picker(self):
        """
        Sets up the file picker UI components.
        """
        ui.Label("Robot USD File:", height=20)
        with ui.HStack(spacing=10):
            # Display for selected file path
            self._file_path_label = ui.Label("No file selected", height=20)

            # Add file picker button
            self.file_manager.add_file_picker(parent_ui=ui.HStack())

    def on_file_selected(self, file_path):
        """
        Callback triggered when a file is selected.

        Args:
            file_path (str): Full path of the selected file.
        """
        print(f"[Extension] Raw file path: {file_path}")  # Debugging

        # Validate the selected file
        if os.path.isfile(file_path) and file_path.endswith(".usd"):
            self.selected_usd_path = file_path
            self._file_path_label.text = f"Selected: {self.selected_usd_path}"
            self.robot_spawner = RobotSpawner(self._file_path_label)
            print(f"[Extension] Valid USD file selected: {self.selected_usd_path}")
        else:
            self._file_path_label.text = "Error: Please select a valid .usd file."
            print("[Extension] Invalid USD file selected.")

    def setup_robot_controls(self):
        """
        Sets up the controls for entering robot count, spawning robots, and resetting the world.
        """
        # Number input field beside the text
        with ui.HStack(height=30, spacing=10):  # Adjust spacing between label and field
            ui.Label("Enter number of robots to spawn:", height=20)
            self._robot_count_field = ui.IntField(width=100, height=25)
            self._robot_count_field.model.set_value(1)

        # Add buttons with proper spacing and fixed widths
        with ui.HStack(height=30, spacing=20):  # Adjust spacing between buttons
            ui.Button(
                "Spawn Robots",
                height=25,
                width=150,  # Fixed width to prevent overlap
                clicked_fn=self.spawn_robots
            )
            ui.Button(
                "Reset World",
                height=25,
                width=150,  # Fixed width to prevent overlap
                clicked_fn=self.reset_world
            )

    def spawn_robots(self):
        """
        Spawns robots using the selected USD file.
        """
        if not hasattr(self, "selected_usd_path"):
            print("[Extension] Please select a USD file before spawning.")
            self._file_path_label.text = "Error: No USD file selected."
            return

        num_of_robots = self._robot_count_field.model.get_value_as_int()
        if self.robot_spawner:
            self.robot_spawner.spawn_robots(num_of_robots, self.selected_usd_path)
        else:
            print("[Extension] Robot spawner not initialized.")

    def reset_world(self):
        """
        Resets the stage by removing all spawned robots and resetting the world.
        """
        try:
            print("[Extension] Resetting the world...")

            # Iterate through and remove all robots under /World
            stage = omni.usd.get_context().get_stage()
            world = stage.GetPrimAtPath("/World")

            if world.IsValid():
                for child in world.GetChildren():
                    print(f"[Extension] Removing {child.GetPath()}")
                    stage.RemovePrim(child.GetPath())

            self._file_path_label.text = "World reset successfully."
            print("[Extension] World reset complete.")
        except Exception as e:
            print(f"[Extension] Error resetting the world: {e}")
            self._file_path_label.text = "Error resetting the world."

    def on_shutdown(self):
        print("[robotics.multiagent.mapping] Extension shutdown")
