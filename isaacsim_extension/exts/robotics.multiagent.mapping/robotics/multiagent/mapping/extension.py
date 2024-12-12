import os
import omni.ext
import omni.ui as ui
import omni.usd
from omni.isaac.core.utils.stage import add_reference_to_stage
from .spawn import RobotSpawner
from .file_manager import FileManager


class RoboticsMultiagentMappingExtension(omni.ext.IExt):
    def on_startup(self, ext_id):
        print("[robotics.multiagent.mapping] Extension startup")
        self.file_manager = FileManager()
        self.robot_spawner = None
        self.robot_file_paths = []
        self.max_robots = 10

        # Create UI window
        self._window = ui.Window("Multi-Agent Robotic Mapping", width=500, height=400)
        with self._window.frame:
            with ui.VStack():
                self.setup_world_picker()
                self.setup_robot_ui()
                self.setup_controls()

    def setup_world_picker(self):
        """Sets up the file picker for the world file."""
        with ui.HStack():
            ui.Label("World USD File:", height=20)
            self._world_file_path_label = ui.Label("No file selected", height=20)
            self.file_manager.add_file_picker(
                parent_ui=ui.HStack(),
                dialog_title="Select World USD File",
                on_file_selected_callback=self.on_world_file_selected,
            )

    def setup_robot_ui(self):
        """Sets up the UI for dynamically adding robots."""
        self.robot_ui_frame = ui.ScrollingFrame(height=200)
        with self.robot_ui_frame:
            self.robot_ui_container = ui.VStack()
            self.add_robot_row()  # Add the first robot row

        with ui.HStack():
            ui.Button(
                "+",
                width=30,
                height=30,
                clicked_fn=self.add_robot_row,
                tooltip="Add another robot",
            )

    def add_robot_row(self):
        """Adds a new robot file picker row."""
        if len(self.robot_file_paths) >= self.max_robots:
            print("[Extension] Maximum robot limit reached.")
            return

        robot_index = len(self.robot_file_paths) + 1
        self.robot_file_paths.append(None)

        with self.robot_ui_container:
            with ui.HStack():
                ui.Label(f"Robot {robot_index} USD File:", width=150, height=20)
                robot_label = ui.Label("No file selected", height=20)
                self.file_manager.add_file_picker(
                    parent_ui=ui.HStack(),
                    dialog_title=f"Select Robot {robot_index} USD File",
                    on_file_selected_callback=lambda path, index=robot_index: self.on_robot_file_selected(path, index),
                )

    def on_robot_file_selected(self, file_path, robot_index):
        """Callback for selecting a robot file."""
        print(f"[Extension] Selected Robot {robot_index}: {file_path}")
        if os.path.isfile(file_path) and file_path.endswith(".usd"):
            self.robot_file_paths[robot_index - 1] = file_path
        else:
            print(f"[Extension] Invalid file selected for Robot {robot_index}.")

    def on_world_file_selected(self, file_path):
        """Callback for selecting the world file."""
        print(f"[Extension] World file selected: {file_path}")
        if os.path.isfile(file_path) and file_path.endswith(".usd"):
            self.selected_world_path = file_path
            self._world_file_path_label.text = f"Selected: {os.path.basename(file_path)}"
        else:
            print("[Extension] Invalid world file selected.")

    def setup_controls(self):
        """Sets up controls for spawning and resetting."""
        with ui.HStack():
            ui.Button(
                "Spawn Robots and World",
                clicked_fn=self.spawn_robots_and_world,
            )
            ui.Button(
                "Reset World",
                clicked_fn=self.reset_world,
            )

    def spawn_robots_and_world(self):
        """Spawns robots and loads the world."""
        if not hasattr(self, "selected_world_path"):
            print("[Extension] No world file selected.")
            return

        valid_robot_paths = [path for path in self.robot_file_paths if path]
        if not valid_robot_paths:
            print("[Extension] No robot files selected.")
            return

        try:
            stage = omni.usd.get_context().get_stage()
            add_reference_to_stage(self.selected_world_path, "/World/Environment")
            self.robot_spawner = RobotSpawner(None)
            for i, robot_path in enumerate(valid_robot_paths):
                self.robot_spawner.spawn_robots(i + 1, robot_path, "/World/Environment")
            print("[Extension] Robots and world spawned successfully.")
        except Exception as e:
            print(f"[Extension] Error during spawning: {e}")

    def reset_world(self):
        """Resets the world and clears all robots."""
        try:
            stage = omni.usd.get_context().get_stage()
            stage.RemovePrim("/World")
            print("[Extension] World reset.")
        except Exception as e:
            print(f"[Extension] Error resetting world: {e}")

    def on_shutdown(self):
        print("[robotics.multiagent.mapping] Extension shutdown")
