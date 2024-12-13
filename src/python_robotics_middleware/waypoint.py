import pybullet as p


class Waypoint:
    """
    Represents a visual waypoint in PyBullet.
    
    Attributes:
        name: Name of the waypoint.
        position: [x, y, z] position of the waypoint.
        color: [r, g, b, a] color of the waypoint.
        radius: Radius of the waypoint sphere.
    """
    def __init__(self, name, position, color=[0, 1, 0, 0.75], radius=0.05):
        self.name = name
        self.position = position
        self.color = color
        self.radius = radius
        self.met = False  # Property to indicate collision with a robot

    def get_position_as_tuple(self):
        return (self.position[0],self.position[1],self.position[2])

    def generate_pybullet_waypoint(self):
        """Generate the waypoint in PyBullet as a visual shape."""
        visual_shape_id = p.createVisualShape(
            shapeType=p.GEOM_SPHERE,
            radius=self.radius,
            rgbaColor=self.color
        )
        waypoint_id = p.createMultiBody(
            baseVisualShapeIndex=visual_shape_id,
            basePosition=self.position,
            baseCollisionShapeIndex=p.createCollisionShape(p.GEOM_SPHERE, radius=self.radius),  # No collision shape
            baseMass=0
        )
        return waypoint_id