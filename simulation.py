import pybullet as p
import pybullet_data
import numpy as np
import time
import random


class Simulation:
    def __init__(
        self,
        render=False,
        arena_half_size=1.6,
        random_spawn=True,
        arena_type="offset_boxes"
    ):
        # Connect to PyBullet
        if render:
            self.physics_client = p.connect(p.GUI)
        else:
            self.physics_client = p.connect(p.DIRECT)

        p.setAdditionalSearchPath(pybullet_data.getDataPath())

        # Arena settings
        self.arena_half_size = arena_half_size
        self.wall_height = 0.45
        self.wall_thickness = 0.05
        self.random_spawn = random_spawn
        self.arena_type = arena_type

        # Predefined obstacle centers for spawn safety checks
        self.obstacle_centers = []

        # Load environment
        p.setGravity(0, 0, -9.8)
        self.plane_id = p.loadURDF("plane.urdf")

        self.create_battle_arena()
        self.create_arena_layout()

        # Robot spawn
        if self.random_spawn:
            self.robot_start_pos = self.get_random_robot_spawn()
            random_yaw = random.uniform(-np.pi, np.pi)
            self.robot_start_orientation = p.getQuaternionFromEuler([0, 0, random_yaw])
        else:
            self.robot_start_pos = [0, 0, 0.1]
            self.robot_start_orientation = p.getQuaternionFromEuler([0, 0, 0])

        # Load the robot
        self.robot_id = p.loadURDF(
            "puncture_prime.urdf",
            self.robot_start_pos,
            self.robot_start_orientation
        )

        # Retrieve and store joint indices
        self.left_wheel_index = None
        self.right_wheel_index = None

        num_joints = p.getNumJoints(self.robot_id)
        for i in range(num_joints):
            joint_info = p.getJointInfo(self.robot_id, i)
            if b'left_wheel_hinge' in joint_info[1]:
                self.left_wheel_index = i
            elif b'right_wheel_hinge' in joint_info[1]:
                self.right_wheel_index = i

        if self.left_wheel_index is None or self.right_wheel_index is None:
            raise ValueError("Wheel joints not found in the robot definition.")

        # Get the link index of the pin
        self.pin_index = None
        for i in range(num_joints):
            joint_info = p.getJointInfo(self.robot_id, i)
            if b'pin' in joint_info[12]:
                self.pin_index = i

        if self.pin_index is None:
            raise ValueError("Pin link not found in the robot definition.")

    def create_battle_arena(self):
        arena_half_size = self.arena_half_size
        wall_height = self.wall_height
        wall_thickness = self.wall_thickness

        horizontal_wall_collision = p.createCollisionShape(
            p.GEOM_BOX,
            halfExtents=[arena_half_size, wall_thickness, wall_height / 2]
        )
        horizontal_wall_visual = p.createVisualShape(
            p.GEOM_BOX,
            halfExtents=[arena_half_size, wall_thickness, wall_height / 2],
            rgbaColor=[0.18, 0.18, 0.18, 1]
        )

        vertical_wall_collision = p.createCollisionShape(
            p.GEOM_BOX,
            halfExtents=[wall_thickness, arena_half_size, wall_height / 2]
        )
        vertical_wall_visual = p.createVisualShape(
            p.GEOM_BOX,
            halfExtents=[wall_thickness, arena_half_size, wall_height / 2],
            rgbaColor=[0.18, 0.18, 0.18, 1]
        )

        self.walls = []

        # Top wall
        self.walls.append(
            p.createMultiBody(
                baseMass=0,
                baseCollisionShapeIndex=horizontal_wall_collision,
                baseVisualShapeIndex=horizontal_wall_visual,
                basePosition=[0, arena_half_size, wall_height / 2]
            )
        )

        # Bottom wall
        self.walls.append(
            p.createMultiBody(
                baseMass=0,
                baseCollisionShapeIndex=horizontal_wall_collision,
                baseVisualShapeIndex=horizontal_wall_visual,
                basePosition=[0, -arena_half_size, wall_height / 2]
            )
        )

        # Left wall
        self.walls.append(
            p.createMultiBody(
                baseMass=0,
                baseCollisionShapeIndex=vertical_wall_collision,
                baseVisualShapeIndex=vertical_wall_visual,
                basePosition=[-arena_half_size, 0, wall_height / 2]
            )
        )

        # Right wall
        self.walls.append(
            p.createMultiBody(
                baseMass=0,
                baseCollisionShapeIndex=vertical_wall_collision,
                baseVisualShapeIndex=vertical_wall_visual,
                basePosition=[arena_half_size, 0, wall_height / 2]
            )
        )

    def create_box_obstacle(self, position, size=0.12, color=(0.85, 0.15, 0.15, 1)):
        collision = p.createCollisionShape(
            p.GEOM_BOX,
            halfExtents=[size, size, size]
        )
        visual = p.createVisualShape(
            p.GEOM_BOX,
            halfExtents=[size, size, size],
            rgbaColor=list(color)
        )

        obstacle_id = p.createMultiBody(
            baseMass=0,
            baseCollisionShapeIndex=collision,
            baseVisualShapeIndex=visual,
            basePosition=[position[0], position[1], size]
        )
        return obstacle_id

    def create_cylinder_obstacle(self, position, radius=0.10, height=0.18, color=(0.2, 0.5, 0.9, 1)):
        collision = p.createCollisionShape(
            p.GEOM_CYLINDER,
            radius=radius,
            height=height * 2
        )
        visual = p.createVisualShape(
            p.GEOM_CYLINDER,
            radius=radius,
            length=height * 2,
            rgbaColor=list(color)
        )

        obstacle_id = p.createMultiBody(
            baseMass=0,
            baseCollisionShapeIndex=collision,
            baseVisualShapeIndex=visual,
            basePosition=[position[0], position[1], height]
        )
        return obstacle_id

    def create_arena_layout(self):
        self.obstacles = []
        self.obstacle_centers = []

        if self.arena_type == "empty":
            return

        elif self.arena_type == "offset_boxes":
            positions = [(0.45, 0.20), (-0.45, -0.20)]
            for pos in positions:
                self.obstacles.append(self.create_box_obstacle(pos, size=0.12))
                self.obstacle_centers.append(pos)

        elif self.arena_type == "cross":
            positions = [(0.0, 0.0), (0.45, 0.0), (-0.45, 0.0), (0.0, 0.45), (0.0, -0.45)]
            for pos in positions:
                self.obstacles.append(self.create_box_obstacle(pos, size=0.10, color=(0.9, 0.2, 0.2, 1)))
                self.obstacle_centers.append(pos)

        elif self.arena_type == "corridor":
            positions = [
                (-0.35, 0.45), (-0.35, 0.10), (-0.35, -0.25),
                (0.35, 0.25), (0.35, -0.10), (0.35, -0.45)
            ]
            for pos in positions:
                self.obstacles.append(self.create_box_obstacle(pos, size=0.11, color=(0.85, 0.3, 0.1, 1)))
                self.obstacle_centers.append(pos)

        elif self.arena_type == "corners":
            positions = [(0.75, 0.75), (-0.75, 0.75), (-0.75, -0.75), (0.75, -0.75)]
            for pos in positions:
                self.obstacles.append(self.create_cylinder_obstacle(pos, radius=0.12, height=0.16, color=(0.2, 0.55, 0.9, 1)))
                self.obstacle_centers.append(pos)

        elif self.arena_type == "mixed":
            box_positions = [(0.55, 0.25), (-0.55, -0.25)]
            cyl_positions = [(-0.10, 0.55), (0.10, -0.55)]

            for pos in box_positions:
                self.obstacles.append(self.create_box_obstacle(pos, size=0.11, color=(0.9, 0.2, 0.2, 1)))
                self.obstacle_centers.append(pos)

            for pos in cyl_positions:
                self.obstacles.append(self.create_cylinder_obstacle(pos, radius=0.10, height=0.15, color=(0.2, 0.55, 0.9, 1)))
                self.obstacle_centers.append(pos)

        else:
            # Fallback layout
            positions = [(0.45, 0.20), (-0.45, -0.20)]
            for pos in positions:
                self.obstacles.append(self.create_box_obstacle(pos, size=0.12))
                self.obstacle_centers.append(pos)

    def get_random_robot_spawn(self):
        margin = 0.45

        while True:
            x = random.uniform(-self.arena_half_size + margin, self.arena_half_size - margin)
            y = random.uniform(-self.arena_half_size + margin, self.arena_half_size - margin)

            valid = True
            for ox, oy in self.obstacle_centers:
                if np.linalg.norm(np.array([x, y]) - np.array([ox, oy])) < 0.38:
                    valid = False
                    break

            if valid:
                return [x, y, 0.1]

    def get_random_balloon_position(self):
        margin = 0.40

        while True:
            x = random.uniform(-self.arena_half_size + margin, self.arena_half_size - margin)
            y = random.uniform(-self.arena_half_size + margin, self.arena_half_size - margin)

            valid = True
            for ox, oy in self.obstacle_centers:
                if np.linalg.norm(np.array([x, y]) - np.array([ox, oy])) < 0.32:
                    valid = False
                    break

            if valid:
                return [x, y]

    def reset_robot(self):
        if self.random_spawn:
            self.robot_start_pos = self.get_random_robot_spawn()
            random_yaw = random.uniform(-np.pi, np.pi)
            self.robot_start_orientation = p.getQuaternionFromEuler([0, 0, random_yaw])

        p.resetBasePositionAndOrientation(
            self.robot_id,
            self.robot_start_pos,
            self.robot_start_orientation
        )
        p.resetBaseVelocity(self.robot_id, [0, 0, 0], [0, 0, 0])

        if hasattr(self, 'balloon_id'):
            p.removeBody(self.balloon_id)
            del self.balloon_id

    def set_wheel_velocity(self, left_velocity, right_velocity):
        p.setJointMotorControl2(
            self.robot_id,
            self.left_wheel_index,
            p.VELOCITY_CONTROL,
            targetVelocity=left_velocity
        )
        p.setJointMotorControl2(
            self.robot_id,
            self.right_wheel_index,
            p.VELOCITY_CONTROL,
            targetVelocity=right_velocity
        )

    def step_simulation(self):
        p.stepSimulation()
        self.check_collision()
        time.sleep(1 / 240)

    def get_robot_state(self):
        pos, orientation = p.getBasePositionAndOrientation(self.robot_id)
        euler_orientation = p.getEulerFromQuaternion(orientation)
        return np.array([pos[0], pos[1], euler_orientation[2]])

    def close(self):
        p.disconnect()

    def render_balloon(self, target_pos=None):
        balloon_radius = 0.15
        balloon_mass = 0.01

        if target_pos is None:
            target_pos = self.get_random_balloon_position()

        balloon_collision = p.createCollisionShape(
            p.GEOM_SPHERE,
            radius=balloon_radius
        )
        balloon_visual = p.createVisualShape(
            p.GEOM_SPHERE,
            radius=balloon_radius,
            rgbaColor=[1, 0, 0, 1]
        )

        self.balloon_id = p.createMultiBody(
            baseMass=balloon_mass,
            baseCollisionShapeIndex=balloon_collision,
            baseVisualShapeIndex=balloon_visual,
            basePosition=[target_pos[0], target_pos[1], balloon_radius],
            baseOrientation=[0, 0, 0, 1]
        )

    def check_collision(self):
        if hasattr(self, 'balloon_id') and self.pin_index is not None:
            contact_points = p.getContactPoints(
                bodyA=self.robot_id,
                bodyB=self.balloon_id,
                linkIndexA=self.pin_index
            )
            if contact_points:
                print("Balloon popped!")
                p.removeBody(self.balloon_id)
                del self.balloon_id

    def get_balloon_position(self):
        if hasattr(self, 'balloon_id'):
            pos, _ = p.getBasePositionAndOrientation(self.balloon_id)
            return np.array(pos[:2])
        return np.array([0.0, 0.0])