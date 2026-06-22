from gym_wrapper import TwoWheeledRobotEnv
import time

env = TwoWheeledRobotEnv(render=True)

obs, info = env.reset()

for _ in range(2000):
    action = [0.0, 0.0]  # robot stays still
    obs, reward, terminated, truncated, info = env.step(action)
    env.render()
    print("Running...", terminated, truncated)
    time.sleep(0.02)

print("Simulation finished. Press Enter to close.")
input()

env.close()