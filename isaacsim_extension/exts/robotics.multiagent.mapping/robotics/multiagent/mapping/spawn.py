import os
from pxr import Gf
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.prims import XFormPrim


class RobotSpawner:
    """
    Handles the logic for spawning robots in the simulation.
    """

    def __init__(self, message_label):
        """
        Initializes the robot spawner.

        Args:
            message_label (ui.Label): Label for displaying messages in the UI.
        """
        self.message_label = message_label

    def spawn_robots(self, num_robots, robot_usd_path):
        """
        Spawns multiple robots in the simulation.

        Args:
            num_robots (int): Number of robots to spawn.
            robot_usd_path (str): Path to the robot USD file.
        """
        try:
            # Validate USD file path
            if not os.path.exists(robot_usd_path):
                self.message_label.text = "Robot USD path does not exist."
                return

            if num_robots <= 0:
                self.message_label.text = "Please enter a positive number of robots."
                return

            # Base position for spawning robots
            base_position = Gf.Vec3d(0, 0, 0)

            # Spawn robots with unique names and positions
            for i in range(num_robots):
                robot_name = f"robot_{i+1}"
                robot_path = f"/World/{robot_name}"
                position = base_position + Gf.Vec3d(i * 2.0, 0, 0)  # Adjust spacing between robots

                # Add robot to the stage
                add_reference_to_stage(robot_usd_path, robot_path)

                # Set the robot's position
                xform_prim = XFormPrim(prim_path=robot_path)
                xform_prim.set_world_transform(translation=position)
                print(f"Spawned {robot_name} at {position}")

            # Update the UI with success message
            self.message_label.text = f"Spawned {num_robots} robots successfully!"
        except Exception as e:
            # Handle errors and display message in UI
            print(f"Error spawning robots: {e}")
            self.message_label.text = "Failed to spawn robots. Check logs for details."
