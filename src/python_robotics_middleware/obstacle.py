import pybullet as p
import os 
import xml.etree.ElementTree as ET

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
        self.mass = mass if mass is not None else self._get_mass_from_sdf()

    def __repr__(self):
        return f"Obstacle(name={self.name}, position=({self.x}, {self.y}, {self.z}), mass={self.mass}, model_path={self.model_path}, color={self.color}, pose={self.pose})"

    
    def _get_mass_from_sdf(self):
        """
        Extract the mass of the object from the SDF file.

        Returns:
            float: The mass of the object.
        """
        try:
            tree = ET.parse(self.model_path)
            root = tree.getroot()

            # Locate the mass element for the given object name
            for model in root.findall('model'):
                if model.get('name') == self.name:
                    inertial = model.find('link/inertial/mass')
                    if inertial is not None and inertial.text:
                        return float(inertial.text)
            
            print(f"Mass not found in SDF file for model '{self.name}'")
        except Exception as e:
            print(f"Failed to parse SDF file '{self.model_path}': {e}")


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
                texture_path = os.path.join(self.search_path, "materials/textures/cinder_block_diffuse.png")
                if os.path.exists(texture_path):
                    texture_id = p.loadTexture(texture_path)
                    p.changeVisualShape(final_obstacle_id, -1, textureUniqueId=texture_id)
            
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
