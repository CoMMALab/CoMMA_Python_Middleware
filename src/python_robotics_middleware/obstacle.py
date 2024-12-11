# obstacle.py
import pybullet as p

class Obstacle:
    def __init__(self, model_path, name, color, position=[0,0,0], mass=0):
        self.model_path = model_path  # Model path for the obstacle
        self.name = name              # Name of the obstacle
        self.x = position[0]                    # X position of the obstacle
        self.y = position[1]           # Y position of the obstacle
        self.z = position[2]                    # Z position of the obstacle
        self.mass = mass              # Mass of the obstacle
        self.color = color

    def __repr__(self):
        return f"Obstacle(name={self.name}, position=({self.x}, {self.y}, {self.z}), mass={self.mass}, model_path={self.model_path}, color={self.color})"


class Sphere(Obstacle):
    def __init__(self, name, position, color, radius, mass):
        super().__init__(model_path=None, name=name, color=color, position=position, mass=mass)  # Sphere doesn't have a model path
        self.radius = radius  # Radius of the sphere
    def generate_pybullet_obstacle(self):
        sphere_position = [self.x, self.y, self.z]  # Position of the sphere
        sphere_collision_shape = p.createCollisionShape(p.GEOM_SPHERE, radius=self.radius)
        sphere_visual_shape = p.createVisualShape(p.GEOM_SPHERE, radius=self.radius, rgbaColor=self.color)
        sphere_id = p.createMultiBody(
            baseMass=0,  # Static object
            baseCollisionShapeIndex=sphere_collision_shape,
            baseVisualShapeIndex=sphere_visual_shape,
            basePosition=sphere_position
        )
        return sphere_id

    def __repr__(self):
        return f"Sphere(name={self.name}, position=({self.x}, {self.y}, {self.z}), radius={self.radius}, mass={self.mass}, color={self.color})"


class Cube(Obstacle):
    def __init__(self, name, color, position, side_length, mass):
        super().__init__(model_path=None, name=name, color=color, position=position, mass=mass)  # # Cube doesn't have a model path
        self.side_length = side_length  # Side length of the cube

    def generate_pybullet_obstacle(self):
        # Position and size of the cube
        cube_position = [self.x, self.y, self.z]
        half_extents = [self.side_length,self.side_length,self.side_length]  # Size of the cube
        
        # Create collision shape for the cube
        cube_collision_shape = p.createCollisionShape(p.GEOM_BOX, halfExtents=half_extents)
        
        # Create visual shape for the cube
        cube_visual_shape = p.createVisualShape(p.GEOM_BOX, halfExtents=half_extents, rgbaColor=self.color)
        
        # Create the cube body (static object)
        cube_id = p.createMultiBody(
            baseMass=0,  # Static object
            baseCollisionShapeIndex=cube_collision_shape,
            baseVisualShapeIndex=cube_visual_shape,
            basePosition=cube_position
        )
        
        return cube_id

    def __repr__(self):
        return f"Cube(name={self.name}, position=({self.x}, {self.y}, {self.z}), side_length={self.side_length}, mass={self.mass}, color={self.color})"
