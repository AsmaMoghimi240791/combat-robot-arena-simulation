from simulation import Simulation
import pybullet as p
import time

# Try: "empty", "offset_boxes", "cross", "corridor", "corners", "mixed"
sim = Simulation(render=True, arena_type="mixed")

sim.render_balloon()

p.resetDebugVisualizerCamera(
    cameraDistance=3.0,
    cameraYaw=50,
    cameraPitch=-42,
    cameraTargetPosition=[0, 0, 0]
)

print("Arena is running. Press Ctrl+C to stop.")

try:
    while True:
        sim.step_simulation()
        time.sleep(1 / 60)
except KeyboardInterrupt:
    print("Closing simulation...")
finally:
    sim.close()