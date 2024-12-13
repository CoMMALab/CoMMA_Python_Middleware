import pybullet as p

class Obstacle:
    def __init__(self, model_path, name, color, search_path, position=[0, 0, 0], mass=0, pose=None):
        self.model_path = model_path  # Model path for the obstacle
        self.name = name              # Name of the obstacle
        self.x = position[0]          # X position of the obstacle
        self.y = position[1]          # Y position of the obstacle
        self.z = position[2]          # Z position of the obstacle
        self.mass = mass              # Mass of the obstacle
        self.color = color            # Color of the obstacle
        self.search_path = search_path
        self.pose = pose if pose else ([self.x, self.y, self.z], p.getQuaternionFromEuler([0, 0, 0]))  # Default to no rotation

    def __repr__(self):
        return f"Obstacle(name={self.name}, position=({self.x}, {self.y}, {self.z}), mass={self.mass}, model_path={self.model_path}, color={self.color}, pose={self.pose})"

    def generate_pybullet_obstacle(self):
        p.setAdditionalSearchPath(self.search_path)
        
        obstacle_ids = p.loadSDF(self.model_path)
        if not obstacle_ids:
            raise ValueError(f"Failed to load obstacle from SDF file: {self.model_path}")
        final_obstacle_id = None
        for obstacle_id in obstacle_ids:
            _, name = p.getBodyInfo(obstacle_id)

            final_obstacle_id = obstacle_id
            if name.decode('utf-8') == self.name:
                p.changeDynamics(final_obstacle_id, -1, mass=self.mass)  # Make obstacle static
                p.resetBasePositionAndOrientation(final_obstacle_id, [self.x, self.y, self.z], self.pose)
                return final_obstacle_id
            
        raise Exception("No obstacle found")


class Sphere(Obstacle):
    def __init__(self, name, position, color, radius, mass, pose=None):
        super().__init__(model_path=None, name=name, color=color, position=position, mass=mass, pose=pose)  # Sphere doesn't have a model path
        self.radius = radius  # Radius of the sphere

    def generate_pybullet_obstacle(self):
        # Position and orientation (pose)
        sphere_position, sphere_orientation = self.pose  # Unpack the pose
        
        # Create collision shape for the sphere
        sphere_collision_shape = p.createCollisionShape(p.GEOM_SPHERE, radius=self.radius)
        
        # Create visual shape for the sphere
        sphere_visual_shape = p.createVisualShape(p.GEOM_SPHERE, radius=self.radius, rgbaColor=self.color)
        
        # Create the sphere body (static object)
        sphere_id = p.createMultiBody(
            baseMass=self.mass,  # Mass of the sphere
            baseCollisionShapeIndex=sphere_collision_shape,
            baseVisualShapeIndex=sphere_visual_shape,
            basePosition=sphere_position,
            baseOrientation=sphere_orientation
        )
        
        return sphere_id

    def __repr__(self):
        return f"Sphere(name={self.name}, position=({self.x}, {self.y}, {self.z}), radius={self.radius}, mass={self.mass}, color={self.color}, pose={self.pose})"


class Cube(Obstacle):
    def __init__(self, name, color, position, side_length, mass, pose=None):
        super().__init__(model_path=None, name=name, color=color, position=position, mass=mass, pose=pose)  # Cube doesn't have a model path
        self.side_length = side_length  # Side length of the cube

    def generate_pybullet_obstacle(self):
        # Position and orientation (pose)
        cube_position, cube_orientation = self.pose  # Unpack the pose
        
        # Half extents for the cube's side length
        half_extents = [self.side_length / 2, self.side_length / 2, self.side_length / 2]  # Size of the cube
        
        # Create collision shape for the cube
        cube_collision_shape = p.createCollisionShape(p.GEOM_BOX, halfExtents=half_extents)
        
        # Create visual shape for the cube
        cube_visual_shape = p.createVisualShape(p.GEOM_BOX, halfExtents=half_extents, rgbaColor=self.color)
        
        # Create the cube body (static object)
        cube_id = p.createMultiBody(
            baseMass=self.mass,  # Mass of the cube
            baseCollisionShapeIndex=cube_collision_shape,
            baseVisualShapeIndex=cube_visual_shape,
            basePosition=cube_position,
            baseOrientation=cube_orientation
        )
        
        return cube_id

    def __repr__(self):
        return f"Cube(name={self.name}, position=({self.x}, {self.y}, {self.z}), side_length={self.side_length}, mass={self.mass}, color={self.color}, pose={self.pose})"
