from python_robotics_middleware import matrix_to_euler_angles
import torch

def compute_error(goal_tf, end_effector_tf):
    goal_pos = goal_tf.get_matrix()[..., :3, 3]
    end_effector_pos = end_effector_tf.get_matrix()[..., :3, 3]
    pos_error = torch.norm(goal_pos - end_effector_pos, dim=-1)

    goal_rot = matrix_to_euler_angles(goal_tf.get_matrix()[..., :3, :3], "XYZ")
    end_effector_rot = matrix_to_euler_angles(end_effector_tf.get_matrix()[..., :3, :3], "XYZ")
    rot_error = torch.norm(goal_rot - end_effector_rot, dim=-1)

    return pos_error, rot_error

def are_tuples_close(tuple1, tuple2, tolerance=1e-3):
    """
    Compare two tuples (including nested ones) element-wise within a given tolerance.

    Args:
        tuple1: First tuple of values (can be nested).
        tuple2: Second tuple of values (can be nested).
        tolerance: Maximum allowed difference for each element.

    Returns:
        True if all elements are within the tolerance, False otherwise.

    Author: Sai Coumar
    """
    if type(tuple1) != type(tuple2):
        return False

    if isinstance(tuple1, (tuple, list)) and isinstance(tuple2, (tuple, list)):
        if len(tuple1) != len(tuple2):
            return False
        return all(are_tuples_close(a, b, tolerance) for a, b in zip(tuple1, tuple2))

    return abs(tuple1 - tuple2) <= tolerance
