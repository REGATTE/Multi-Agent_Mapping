from pxr import Gf, UsdGeom, Usd
import omni.usd
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.prims import XFormPrim
import os, random
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
        self.initial_positions = {} # Dictionary to store initial robot positions

    def spawn_robots(self, num_robots, robot_usd_path, world_prim_path):
        """
        Spawns robots randomly within the bounds of the world map, ensuring equal spacing.

        Args:
            num_robots (int): Number of robots to spawn.
            robot_usd_path (str): Path to the robot USD file.
            world_prim_path (str): Path to the world prim.
        """
        try:
            if not os.path.exists(robot_usd_path):
                print(f"[RobotSpawner] Invalid USD file path: {robot_usd_path}")
                return

            if num_robots <= 0:
                print("[RobotSpawner] Invalid robot count.")
                return

            stage = omni.usd.get_context().get_stage()
            world_prim = stage.GetPrimAtPath(world_prim_path)
            if not world_prim.IsValid():
                print(f"[RobotSpawner] Invalid world prim path: {world_prim_path}")
                return

            # Correct construction of BBoxCache
            bbox_cache = UsdGeom.BBoxCache(
                Usd.TimeCode.Default(),
                [UsdGeom.Tokens.default_],
                useExtentsHint=True
            )
            bounding_box = bbox_cache.ComputeWorldBound(world_prim)
            bounds_min = bounding_box.GetRange().GetMin()
            bounds_max = bounding_box.GetRange().GetMax()

            positions = []
            spacing = 2.0  # Minimum spacing between robots

            for _ in range(num_robots):
                while True:
                    x = random.uniform(bounds_min[0], bounds_max[0])
                    y = random.uniform(bounds_min[1], bounds_max[1])
                    z = bounds_min[2]  # Keep robots on the ground
                    position = Gf.Vec3d(x, y, z)

                    if all((pos - position).GetLength() > spacing for pos in positions):
                        positions.append(position)
                        break

            for i, position in enumerate(positions):
                robot_name = f"robot_{i+1}"
                robot_path = f"/World/{robot_name}"
                add_reference_to_stage(robot_usd_path, robot_path)

                xform_prim = stage.GetPrimAtPath(robot_path)
                if xform_prim.IsValid():
                    xform_prim.GetAttribute("xformOp:translate").Set(position)
                    self.initial_positions[robot_name] = position  # Save initial position
                    print(f"[RobotSpawner] Positioned {robot_name} at {position}.")
                else:
                    print(f"[RobotSpawner] Failed to retrieve prim for {robot_name}.")

            print(f"[RobotSpawner] Spawned {num_robots} robots randomly!")
        except Exception as e:
            print(f"[RobotSpawner] Error spawning robots randomly: {e}")
    
    def reset_to_initial_positions(self):
        """
        Resets all robots to their initial positions.
        """
        stage = omni.usd.get_context().get_stage()
        for robot_name, position in self.initial_positions.items():
            robot_path = f"/World/{robot_name}"
            xform_prim = stage.GetPrimAtPath(robot_path)
            if xform_prim.IsValid():
                xform_prim.GetAttribute("xformOp:translate").Set(position)
                print(f"[RobotSpawner] Reset {robot_name} to initial position {position}.")
            else:
                print(f"[RobotSpawner] Prim {robot_name} not found for reset.")
