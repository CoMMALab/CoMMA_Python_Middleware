import timeit

import numpy as np
import torch

from python_robotics_middleware.transforms.math import quaternion_close
from python_robotics_middleware.transforms.rotation_conversions import axis_and_angle_to_matrix_33, axis_angle_to_matrix, \
    pos_rot_to_matrix, matrix_to_pos_rot, random_rotations, quaternion_from_euler


def test_axis_angle_to_matrix_perf():
    """
    Test and compare performance of two implementations of axis-angle to matrix conversion.
    Uses batch and non-batched inputs to evaluate speed and correctness.
    
    Author: UM-ARM Lab
    """
    print("Test two implementations of axis_angle_to_matrix from UM-ARM lab.")
    number = 100
    N = 1_000

    device = 'cuda' if torch.cuda.is_available() else 'cpu'
    axis_angle = torch.randn([N, 3], device=device, dtype=torch.float64)
    axis_1d = torch.tensor([1., 0, 0], device=device, dtype=torch.float64)  # in the FK code this is NOT batched!
    theta = axis_angle.norm(dim=1, keepdim=True)

    dt1 = timeit.timeit(lambda: axis_angle_to_matrix(axis_angle), number=number)
    print(f'Old method: {dt1:.5f}ms')

    dt2 = timeit.timeit(lambda: axis_and_angle_to_matrix_33(axis=axis_1d, theta=theta), number=number)
    print(f'New method: {dt2:.5f}ms')


def test_quaternion_not_close():
    """
    Validate that the quaternion comparison function correctly identifies
    quaternions that are far apart in space.

    Author: UM-ARM Lab
    """
    # ensure it returns false for quaternions that are far apart
    q1 = torch.tensor([1., 0, 0, 0])
    q2 = torch.tensor([0., 1, 0, 0])
    assert not quaternion_close(q1, q2)


def test_quaternion_from_euler():
    """
    Test the conversion of Euler angles to quaternions.
    Verifies correctness for both single and batched transformations.

    Author: UM-ARM Lab
    """
    q = quaternion_from_euler(torch.tensor([0., 0, 0]))
    assert quaternion_close(q, torch.tensor([1., 0, 0, 0]))
    root2_over_2 = np.sqrt(2) / 2

    q = quaternion_from_euler(torch.tensor([0, 0, np.pi / 2]))
    assert quaternion_close(q, torch.tensor([root2_over_2, 0, 0, root2_over_2], dtype=q.dtype))

    q = quaternion_from_euler(torch.tensor([-np.pi / 2, 0, 0]))
    assert quaternion_close(q, torch.tensor([root2_over_2, -root2_over_2, 0, 0], dtype=q.dtype))

    q = quaternion_from_euler(torch.tensor([0, np.pi / 2, 0]))
    assert quaternion_close(q, torch.tensor([root2_over_2, 0, root2_over_2, 0], dtype=q.dtype))

    # Test batched
    b = 32
    rpy = torch.tensor([0, np.pi / 2, 0])
    rpy_batch = torch.tile(rpy[None], (b, 1))
    q_batch = quaternion_from_euler(rpy_batch)
    q_expected = torch.tensor([root2_over_2, 0, root2_over_2, 0], dtype=q.dtype)
    q_expected_batch = torch.tile(q_expected[None], (b, 1))
    assert quaternion_close(q_batch, q_expected_batch)


def test_pos_rot_conversion():
    """
    Test conversion between transformation matrices and their position and rotation components.
    Ensures accuracy of forward and reverse transformations.

    Author: UM-ARM Lab
    """
    N = 1000
    R = random_rotations(N)
    t = torch.randn((N, 3), dtype=R.dtype, device=R.device)
    T = torch.eye(4, dtype=R.dtype, device=R.device).repeat(N, 1, 1)
    T[:, :3, 3] = t
    T[:, :3, :3] = R
    pos, rot = matrix_to_pos_rot(T)
    TT = pos_rot_to_matrix(pos, rot)
    assert torch.allclose(T, TT, atol=1e-6)


if __name__ == '__main__':
    test_axis_angle_to_matrix_perf()
    test_pos_rot_conversion()
