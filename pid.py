from simple_pid import PID
from gym_wrapper import TwoWheeledRobotEnv
import numpy as np

# Create the environment
env = TwoWheeledRobotEnv(render=True)

# Create a mapping to map a PID yaw output of [-1, 1] to individual wheel velocities to turn the robot
def pid_to_wheel_velocities(pid_output):
    # Map the output to individual wheel velocities
    left_velocity = 0
    right_velocity = 0
    if pid_output < 0:
        left_velocity = pid_output
        right_velocity = -pid_output
    elif pid_output > 0:
        left_velocity = -pid_output
        right_velocity = pid_output
    return left_velocity, right_velocity

# calculate the angle between the robot and the target position obs[:3] -> x, y, yaw, obs[3:] -> x_target, y_target
def calculate_yaw(obs):
    x, y, theta, x_target, y_target = obs
    dx = x_target - x
    dy = y_target - y
    angle = np.arctan2(dy, dx)
    return angle

# get the distance between the robot and the target position
def calculate_distance(obs):
    x, y, _, x_target, y_target = obs
    dx = x_target - x
    dy = y_target - y
    return np.sqrt(dx ** 2 + dy ** 2)

# Reset the environment
obs, _ = env.reset()

# Create a PID controller
pid_yaw = PID(5, 0, 0.05, setpoint=calculate_yaw(obs))
pid_drive = PID(1, 0.1, 0.05, setpoint=0)



# Simulate the environment for 1000 steps
for _ in range(10000):
    # Get the current yaw angle
    theta = obs[2]
    # add 90 degrees to theta
    theta += np.pi / 2
    # Compute the control signal
    control_yaw = pid_yaw(theta)
    # convert the control signal to individual wheel velocities
    left_velocity, right_velocity = pid_to_wheel_velocities(control_yaw)
    # Get the distance between the robot and the target position
    distance = calculate_distance(obs)
    # Compute the control signal for driving
    control_drive = pid_drive(distance)
    # Apply the control signal for driving
    left_velocity += control_drive
    right_velocity += control_drive
    # Apply the control signal to the robot
    action = [left_velocity, right_velocity]
    # action = [0, 0]

    obs, reward, terminated, truncated, _ = env.step(action)
    if terminated or truncated:
        env.reset()
        pid_yaw.reset()
        obs, _ = env.reset()
        print("Resetting the environment")
        # set the setpoint of the PID controller to the new yaw angle
        pid_yaw.setpoint = calculate_yaw(obs)
    
        continue
    #convert theta to degrees
    theta = theta * 180 / np.pi

    print(f"Yaw: {theta:.2f}, Control: {control_yaw:.2f}, Reward: {reward:.2f}, Desired Yaw: {calculate_yaw(obs)* 180 / np.pi:.2f}")
