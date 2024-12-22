import pybullet as p
import pybullet_data
import time
import numpy as np
import os 
from python_robotics_middleware import Obstacle, Sphere, Cube, are_tuples_close



def visualize_collision_with_sphere(data, urdf, visualize=True, delay=True):
    """
    Visualize pre-computed joint configurations in PyBullet with collision detection.
    
    Args:
        data: List of dictionaries containing pre-computed joint angles for robot poses.
        urdf: Path to the robot URDF file.
        n: Number of poses to visualize.
        visualize: Whether to visualize the robot in PyBullet.
        delay: Whether to add a delay between visualizing poses.
    Author: Sai Coumar
    """
    if visualize:
        # Initialize PyBullet in GUI mode
        p.connect(p.GUI)
        p.setRealTimeSimulation(False)
        p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.81)
        p.setTimeStep(1/240.0)
        
        # Set up camera for better visualization
        yaw = 90
        pitch = -35
        dist = 2
        target = np.array([0, 0, 0])
        p.resetDebugVisualizerCamera(dist, yaw, pitch, target)

        # Load ground plane
        plane_id = p.loadURDF("plane.urdf", [0, 0, 0], useFixedBase=True)
        p.changeVisualShape(plane_id, -1, rgbaColor=[0.3, 0.3, 0.3, 1])

        # Load robot model
        robot_id = p.loadURDF(urdf, basePosition=[0, 0, 0], useFixedBase=True)
        
        sphere = Sphere(name="sphere_1",color=[1,0,0,1], position=[0.5,0.5,0.2],mass=0, radius=0.2)

        sphere_id = sphere.generate_pybullet_obstacle()

      

        # Keep the visualization running until manually exited
        cached_contact = None
        while True:
            contact_points_with_obstacle = p.getContactPoints(bodyA=robot_id, bodyB=sphere_id)

            # print(contact_points_with_obstacle)

            if contact_points_with_obstacle:
                for contact in contact_points_with_obstacle:
                    if are_tuples_close(contact[5], cached_contact, tolerance=1e-3):
                        print(f"Contact at position {contact[5]}")
                    cached_contact = contact[5]

            
            p.stepSimulation()
def visualize_collision_with_cube(data, urdf, visualize=True, delay=True):
    """
    Visualize pre-computed joint configurations in PyBullet with collision detection.
    
    Args:
        data: List of dictionaries containing pre-computed joint angles for robot poses.
        urdf: Path to the robot URDF file.
        n: Number of poses to visualize.
        visualize: Whether to visualize the robot in PyBullet.
        delay: Whether to add a delay between visualizing poses.
    Author: Sai Coumar
    """
    if visualize:
        # Initialize PyBullet in GUI mode
        p.connect(p.GUI)
        p.setRealTimeSimulation(False)
        p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.81)
        p.setTimeStep(1/240.0)

        # Set up camera for better visualization
        yaw = 90
        pitch = -35
        dist = 2
        target = np.array([0, 0, 0])
        p.resetDebugVisualizerCamera(dist, yaw, pitch, target)

        # Load ground plane
        plane_id = p.loadURDF("plane.urdf", [0, 0, 0], useFixedBase=True)
        p.changeVisualShape(plane_id, -1, rgbaColor=[0.3, 0.3, 0.3, 1])

        # Load robot model
        robot_id = p.loadURDF(urdf, basePosition=[0, 0, 0], useFixedBase=True)
        
        cube = Cube(name="cube_1",color=[1,1,0,0.5], position=[0.5,1.0,0.2],mass=100, side_length=0.2)

        cube_id = cube.generate_pybullet_obstacle()

      

        cached_contact = None
        while True:
            contact_points_with_obstacle = p.getContactPoints(bodyA=robot_id, bodyB=cube_id)

            # print(contact_points_with_obstacle)

            if contact_points_with_obstacle:
                for contact in contact_points_with_obstacle:
                    if are_tuples_close(contact[5], cached_contact, tolerance=1e-3):
                        print(f"Contact at position {contact[5]}")
                    cached_contact = contact[5]

            
            p.stepSimulation()

def visualize_collision_with_multiple_obstacles(data, urdf, obstacles, visualize=True, delay=False):
    """
    Visualize pre-computed joint configurations in PyBullet with collision detection
    involving multiple spheres and cubes.

    Args:
        data: List of dictionaries containing pre-computed joint angles for robot poses.
        urdf: Path to the robot URDF file.
        obstacles: List of obstacle definitions. Each obstacle is a dictionary with keys:
                   - type: "cube" or "sphere"
                   - position: [x, y, z] position
                   - size: Size parameter (side_length for cube, radius for sphere)
                   - color: [r, g, b, a] RGBA color
                   - mass: Mass of the obstacle
        visualize: Whether to visualize the robot in PyBullet.
        delay: Whether to add a delay between visualizing poses.
    Author: Sai Coumar
    """
    if visualize:
        # Initialize PyBullet in GUI mode
        p.connect(p.GUI)
        p.setRealTimeSimulation(False)
        p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.81)

        p.setTimeStep(1/240)

        # Set up camera for better visualization
        yaw = 90
        pitch = -35
        dist = 2
        target = np.array([0, 0, 0])
        p.resetDebugVisualizerCamera(dist, yaw, pitch, target)

        # Load ground plane
        plane_id = p.loadURDF("plane.urdf", [0, 0, 0], useFixedBase=True)
        p.changeVisualShape(plane_id, -1, rgbaColor=[0.3, 0.3, 0.3, 1])

        # Load robot model
        robot_id = p.loadURDF(urdf, basePosition=[0, 0, 0], useFixedBase=True)

        obstacle_ids = [obs.generate_pybullet_obstacle() for obs in obstacles]

        # Create and load obstacles
        cached_contact = None
        # Main simulation loop
        while True:
            for obstacle_id in obstacle_ids:
                contact_points = p.getContactPoints(bodyA=robot_id, bodyB=obstacle_id)

                if contact_points:
                    for contact in contact_points:
                        if are_tuples_close(contact[5], cached_contact, tolerance=1e-3):
                            print(f"Contact with obstacle {obstacle_id} at position {contact[5]}")
                        cached_contact = contact[5]

            p.stepSimulation()

            if delay:
                time.sleep(0.01)

def visualize_collision_with_loaded_obstacle(data, urdf, obstacle, visualize=True, delay=False):
    """
    Visualize pre-computed joint configurations in PyBullet with collision detection.

    Args:
        data: List of dictionaries containing pre-computed joint angles for robot poses.
        urdf: Path to the robot URDF file.
        obstacle: An instance of the Obstacle class or derived class (e.g., Sphere, Cube).
        n: Number of poses to visualize.
        visualize: Whether to visualize the robot in PyBullet.
        delay: Whether to add a delay between visualizing poses.
    Author: Sai Coumar
    """
    if visualize:
        # Initialize PyBullet in GUI mode
        p.connect(p.GUI)
        p.setRealTimeSimulation(False)
        p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.81)
        p.setTimeStep(1/240.0)
        # Set up camera for better visualization
        yaw = 90
        pitch = -35
        dist = 2
        target = np.array([0, 0, 0])
        p.resetDebugVisualizerCamera(dist, yaw, pitch, target)

        # Load ground plane
        plane_id = p.loadURDF("plane.urdf", [0, 0, 0], useFixedBase=True)
        p.changeVisualShape(plane_id, -1, rgbaColor=[0.3, 0.3, 0.3, 1])

        # Load robot model
        robot_id = p.loadURDF(urdf, basePosition=[0, 0, 0], useFixedBase=True)

        # Generate and load the obstacle from the Obstacle object
        obstacle_id = obstacle.generate_pybullet_obstacle()
        # p.resetBasePositionAndOrientation(obstacle_id, [0.5, 0.5, 0.75], [0, 0, 0, 1])
         # Load obstacle from SDF file
        print("Full sdf path:", obstacle.model_path)

        # Visualize robot poses and check for collisions
        cached_contact = None
        while True:
            contact_points_with_obstacle = p.getContactPoints(bodyA=robot_id, bodyB=obstacle_id)

            # print(contact_points_with_obstacle)

            if contact_points_with_obstacle:
                for contact in contact_points_with_obstacle:
                    if are_tuples_close(contact[5], cached_contact, tolerance=1e-3):
                        print(f"Contact at position {contact[5]}")
                    cached_contact = contact[5]

            
            p.stepSimulation()
if __name__ == "__main__":
    search_path = pybullet_data.getDataPath()

    # Define URDF and dataset
    urdf = "franka/fp3_franka_hand.urdf"
    data = [
        {'joint_angles': [0.0, -0.5, 0.5, -1.0, 0.0, 0.8, 0.0]},  # Example pose 1
        {'joint_angles': [0.2, -0.3, 0.7, -0.9, 0.1, 0.5, 0.0]},  # Example pose 2
        # Add more poses here...
    ]

    sdf_path = "example_sdfs/cinder_block/model.sdf"
    # sdf_path = "test_sdfs/table.sdf"
    full_path = os.path.abspath(sdf_path)

    obstacle = Obstacle(model_path=full_path, name = "cinder_block", color=None, search_path = "test_sdfs/cinder_block", position = [0.5, 0.5, 0.75], mass=0.001, pose=[0,0,0,1])
    print("Obstacle:", obstacle)

    x, y = 0.5, 0.5

    # Create a list of cinder blocks with different z positions
    cinder_blocks = []
    z_positions = [0.75 + i * 1.5 for i in range(5)]  # Increment z by 0.5 for each block

    for z in z_positions:
        obstacle = Obstacle(
            model_path=full_path,
            name="cinder_block",
            color=None,
            search_path="example_sdfs/cinder_block",
            position=[x, y, z],
            mass=1,
            pose=[0, 0, 0, 1]
        )
        cinder_blocks.append(obstacle)


    # visualize_collision_with_sphere(data=data, urdf=urdf)
    # visualize_collision_with_cube(data=data, urdf=urdf)
    # visualize_collision_with_loaded_obstacle(data=data, urdf=urdf, obstacle=obstacle)
    visualize_collision_with_multiple_obstacles(data=data, urdf=urdf, obstacles=cinder_blocks)
