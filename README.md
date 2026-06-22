# Balloon Pop Simulation

## Overview
- `simulation.py` is a simulation environment built using PyBullet to train and test a two-wheeled robot that aims to pop a balloon. 
- `gym_wrapper.py`: A custom wrapper for the Gym environment.
- `pid.py`: A script to control the robot using a PID controller.
- `test_agent.py`: A script to verify the Gym wrapper functionality.


## Prerequisites
Before using this simulation, ensure you have the following installed:

- Python 3.8 or later
- PyBullet
- Stable-Baselines3
- Gymnasium
- NumPy
- Simple-PID

---

## Installation
Follow these steps to set up the environment:

1. Clone the Repository
2. Create a Python Virtual Environment
3. Install Required Packages
Install the necessary Python packages:
```bash
pip install -r requirements.txt
```
4. Test the Installation
Run the following script to verify the installation:
```bash
python test_agent.py
```
This will initialize the environment and spawn the robot and balloon.

---

## Usage

### Interacting with `simulation.py`

#### 1. Training a Model
Use the following script to train a reinforcement learning policy for the robot:
```python
from stable_baselines3 import PPO
from stable_baselines3.common.monitor import Monitor
from gym_wrapper import TwoWheeledRobotEnv

# Wrap the environment for monitoring
env = Monitor(TwoWheeledRobotEnv())

# Define the PPO model
model = PPO(
    "MlpPolicy",
    env,
    verbose=1,
    learning_rate=3e-4,
    n_steps=2048,
    batch_size=64,
    gamma=0.99,
    gae_lambda=0.95,
    ent_coef=0.01,
)

# Train the model
model.learn(total_timesteps=1000000, progress_bar=True)

# Save the trained model
model.save("model.zip")
```

#### 2. Testing the Trained Model
After training, use the saved model to test the robot:
```python
from stable_baselines3 import PPO
from gym_wrapper import TwoWheeledRobotEnv

# Load the trained model
model = PPO.load("model.zip")

# Create the environment
env = TwoWheeledRobotEnv()
obs, _ = env.reset()

done = False
while not done:
    action, _ = model.predict(obs, deterministic=True)
    obs, reward, done, _, _ = env.step(action)
    env.render()
```

### Interacting with `pid.py`
The `pid.py` script demonstrates how to control the robot using a PID controller:

#### Running the PID Controller Test
Run the following command to execute the script:
```bash
python pid.py
```

This script:
- Initializes the robot environment.
- Uses a PID controller to compute wheel velocities based on the robot's yaw angle and distance to the target.
- Prints debug information, including yaw angles, control signals, and rewards.

### Interacting with `test_agent.py`
The `test_agent.py` script verifies the Gym wrapper functionality:

#### Running the Gym Wrapper Test
Run the following command to execute the script:
```bash
python test_agent.py
```

This script:
- Initializes the Gym environment.
- Tests the action and observation spaces.
- Simulates a simple scenario where the robot executes constant wheel velocities.
