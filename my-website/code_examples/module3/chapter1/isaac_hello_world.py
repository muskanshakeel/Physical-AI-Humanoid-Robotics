from omni.isaac.kit import SimulationApp

# This is a basic example of how to start Isaac Sim and add a simple object.
# It requires Isaac Sim to be installed and the Python environment correctly configured.

CONFIG = {
    "headless": False,  # True for no UI, False for UI
    "width": 1280,
    "height": 720
}

simulation_app = SimulationApp(CONFIG)

from omni.isaac.core import World
from omni.isaac.core.objects import DynamicCuboid

class IsaacHelloWorld:
    def __init__(self):
        self.world = World(stage_units_in_meters=1.0)
        self.world.scene.add_default_ground_plane()

        # Add a dynamic cuboid
        self.cube = self.world.scene.add(
            DynamicCuboid(
                prim_path="/World/Fancy_Cube",
                name="fancy_cube",
                position=[0.0, 0.0, 1.0],
                size=0.5,
                color=[0.0, 0.0, 1.0],
            )
        )
        self.world.reset()

    def run(self):
        self.world.run_physics()
        # You can add more complex simulation logic here
        # For a simple demo, just let it run for a bit
        for _ in range(500): # Run for 500 simulation steps
            self.world.step(render=True)
        print("Simulation finished.")

# Run the example
try:
    isaac_app = IsaacHelloWorld()
    isaac_app.run()
except Exception as e:
    print(f"An error occurred: {e}")
finally:
    simulation_app.close()
