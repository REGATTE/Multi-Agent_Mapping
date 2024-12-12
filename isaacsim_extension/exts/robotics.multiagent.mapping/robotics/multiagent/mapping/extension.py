import os
import omni.ext
import omni.ui as ui
import omni.usd
from omni.isaac.core.utils.stage import add_reference_to_stage
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
        self.file_manager = FileManager()
        self.robot_spawner = None

        # Create UI window
        self._window = ui.Window("Multi-Agent Robotic Mapping", width=500, height=150, flags=ui.WINDOW_FLAGS_NO_RESIZE)
        with self._window.frame:
            with ui.VStack(spacing=0):  # Set vertical spacing to 0
                self.setup_file_picker()  # Setup the robot file picker
                self.setup_world_picker()  # Setup the world file picker
                self.setup_robot_controls()  # Setup robot controls

    def setup_file_picker(self):
        """
        Sets up the file picker UI components for robot files.
        """
        with ui.HStack():
            ui.Label("Robot USD File:", height=20, width=150)  # Static width for label
            self._file_path_label = ui.Label("No file selected", height=20, width=150)
            ui.Spacer()  # Pushes the browse button to the right
            self.file_manager.add_file_picker(
                parent_ui=ui.HStack(),  # Correct usage
                dialog_title="Select Robot USD File",
                on_file_selected_callback=self.on_file_selected,
            )

    def setup_world_picker(self):
        """
        Sets up the file picker UI components for world files.
        """
        with ui.HStack():
            ui.Label("World USD File:", height=20, width=150)  # Static width for label
            self._world_file_path_label = ui.Label("No file selected", height=20, width=150)
            ui.Spacer()  # Pushes the browse button to the right
            self.file_manager.add_file_picker(
                parent_ui=ui.HStack(),  # Correct usage
                dialog_title="Select World USD File",
                on_file_selected_callback=self.on_world_file_selected,
            )

    def on_file_selected(self, file_path):
        """
        Callback triggered when a robot file is selected.

        Args:
            file_path (str): Full path of the selected file.
        """
        print(f"[Extension] Raw file path: {file_path}")  # Debugging
        # Extract the file name from the path
        file_name = os.path.basename(file_path)

        # Validate the selected file
        if os.path.isfile(file_path) and file_path.endswith(".usd"):
            self.selected_usd_path = file_path
            self._file_path_label.text = f"Selected: {file_name}"
            self.robot_spawner = RobotSpawner(self._file_path_label)
            print(f"[Extension] Valid USD file selected: {self.selected_usd_path}")
        else:
            self._file_path_label.text = "Error: Please select a valid .usd file."
            print("[Extension] Invalid USD file selected.")

    def on_world_file_selected(self, file_path):
        """
        Callback triggered when a world file is selected.

        Args:
            file_path (str): Full path of the selected world file.
        """
        print(f"[Extension] Raw world file path: {file_path}")  # Debugging

        # Extract the file name from the path
        file_name = os.path.basename(file_path)

        # Validate the selected world file
        if os.path.isfile(file_path) and file_path.endswith(".usd"):
            self.selected_world_path = file_path
            self._world_file_path_label.text = f"Selected: {file_name}"
            print(f"[Extension] Valid world USD file selected: {self.selected_world_path}")
        else:
            self._world_file_path_label.text = "Error: Please select a valid .usd file."
            print("[Extension] Invalid world USD file selected.")

    def setup_robot_controls(self):
        """
        Sets up the controls for spawning robots and resetting the world.
        """
        with ui.HStack(height=30, spacing=5):  # Adjusted horizontal spacing
            ui.Label("Enter number of robots to spawn:", height=20, width=0)
            self._robot_count_field = ui.IntField(width=100, height=25)

        with ui.HStack(height=30, spacing=10):  # Standardized button spacing
            ui.Spacer()
            ui.Button(
                "Spawn Robots and World",
                height=25,
                width=150,
                clicked_fn=self.spawn_robots_and_world
            )
            ui.Button(
                "Reset to Spawn Position",
                height=25,
                width=150,
                clicked_fn=self.reset_to_initial_positions
            )
            ui.Button(
                "Reset World",
                height=25,
                width=150,
                clicked_fn=self.reset_world
            )
            ui.Spacer()

    def spawn_robots_and_world(self):
        """
        Loads the world file under /World/<world_name> and spawns robots under /World.
        """
        if not hasattr(self, "selected_world_path"):
            print("[Extension] Please select a world file before spawning.")
            self._world_file_path_label.text = "Error: No world file selected."
            return

        if not hasattr(self, "selected_usd_path"):
            print("[Extension] Please select a robot file before spawning.")
            self._file_path_label.text = "Error: No robot file selected."
            return

        try:
            # Load the world file
            stage = omni.usd.get_context().get_stage()

            # Ensure the /World prim exists
            world_prim = stage.GetPrimAtPath("/World")
            if not world_prim.IsValid():
                world_prim = stage.DefinePrim("/World", "Xform")

            # Use the world file name (without extension) as the parent prim
            world_file_name = os.path.splitext(os.path.basename(self.selected_world_path))[0]
            world_parent_path = f"/World/{world_file_name}"

            # Add the world file under /World/<world_name>
            add_reference_to_stage(self.selected_world_path, world_parent_path)
            print(f"[Extension] World loaded successfully under {world_parent_path}.")

            # Spawn robots under /World
            num_of_robots = self._robot_count_field.model.get_value_as_int()
            if self.robot_spawner:
                self.robot_spawner.spawn_robots(num_of_robots, self.selected_usd_path, world_parent_path)
                print(f"[Extension] Spawned {num_of_robots} robots successfully.")
            else:
                print("[Extension] Robot spawner not initialized.")
        except Exception as e:
            print(f"[Extension] Error during spawn: {e}")
            self._file_path_label.text = "Error during spawn."
    
    def reset_to_initial_positions(self):
        """
        Resets all robots to their initial spawn positions.
        Stops the simulation and resets the positions.
        """
        if not self.robot_spawner:
            print("[Extension] Robot spawner not initialized.")
            return

        try:
            print("[Extension] Stopping simulation and resetting to initial positions...")
            omni.timeline.get_timeline_interface().stop()  # Stop the simulation
            self.robot_spawner.reset_to_initial_positions()
            print("[Extension] Robots reset to initial positions.")
        except Exception as e:
            print(f"[Extension] Error resetting to initial positions: {e}")

    def reset_world(self):
        """
        Resets the stage by clearing the /World prim entirely.
        """
        try:
            print("[Extension] Resetting the world...")

            # Clear the /World prim
            stage = omni.usd.get_context().get_stage()
            world = stage.GetPrimAtPath("/World")
            if world.IsValid():
                stage.RemovePrim("/World")

            self._world_file_path_label.text = "World reset successfully."
            print("[Extension] World reset complete.")
        except Exception as e:
            print(f"[Extension] Error resetting the world: {e}")
            self._world_file_path_label.text = "Error resetting the world."

    def on_shutdown(self):
        print("[robotics.multiagent.mapping] Extension shutdown")