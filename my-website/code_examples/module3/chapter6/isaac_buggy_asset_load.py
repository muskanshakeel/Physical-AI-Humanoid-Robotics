from omni.isaac.kit import SimulationApp

# This script demonstrates a common error: attempting to load a non-existent USD asset.
# It requires Isaac Sim to be installed and the Python environment correctly configured.

CONFIG = {
    "headless": False,  # True for no UI, False for UI
    "width": 1280,
    "height": 720
}

simulation_app = SimulationApp(CONFIG)

from omni.isaac.core import World
from omni.isaac.core.articulations import Articulation

class IsaacBuggyAssetLoad:
    def __init__(self):
        self.world = World(stage_units_in_meters=1.0)
        self.world.scene.add_default_ground_plane()

        # Intentionally try to load a non-existent USD asset
        # This path is unlikely to exist and should cause an error
        buggy_asset_path = "/non_existent_path/my_buggy_robot.usd"
        
        try:
            # Attempt to add a robot from a USD path
            self.buggy_robot = self.world.scene.add(
                Articulation(
                    prim_path=buggy_asset_path,
                    name="buggy_robot",
                    position=[0.0, 0.0, 0.0]
                )
            )
            self.world.reset()
            print(f"Successfully loaded {buggy_asset_path} (unexpected).")
        except Exception as e:
            print(f"Caught expected error when loading asset: {e}")
            print("This demonstrates a common asset loading failure in Isaac Sim.")
            print("Please ensure your USD asset paths are correct and the assets exist.")
            self.world.reset() # Still need to reset the world even if asset loading fails to properly clean up

    def run(self):
        # Even if asset loading fails, we still need to run simulation_app.close()
        # For a buggy example, we just run for a short period
        for _ in range(100):
            self.world.step(render=False) # No need to render if we just show error
        print("Simulation application running (briefly) after asset load attempt.")


# Run the example
try:
    isaac_buggy_app = IsaacBuggyAssetLoad()
    isaac_buggy_app.run()
except Exception as e:
    print(f"An unexpected error occurred during run: {e}")
finally:
    simulation_app.close()
