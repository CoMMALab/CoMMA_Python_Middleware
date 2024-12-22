# import numpy as np
# import genesis as gs
# import time
# import os 
# from python_robotics_middleware import Obstacle, Sphere, Cube, are_tuples_close


# def visualize_collision_with_sphere(data, urdf, visualize=True):
#     """
#     Visualize collision checking with a sphere in PyBullet.
    
#     Args:
#         data: List of dictionaries containing pre-computed joint angles for robot poses.
#         urdf: Path to the robot URDF file.
#         visualize: Whether to visualize the robot in PyBullet.
#     """
#     if visualize:
#         gs.init(backend=gs.gpu)
#         scene = gs.Scene(
#         viewer_options = gs.options.ViewerOptions(
#             camera_pos    = (3, -1, 1.5),
#             camera_lookat = (0.0, 0.0, 0.5),
#             camera_fov    = 30,
#             max_FPS       = 60,
#         ),
#         sim_options = gs.options.SimOptions(
#             dt = 0.01,
#             substeps = 20, # for more stable grasping contact
#         ),
#         show_viewer = True,
#         vis_options = gs.options.VisOptions(
#             show_world_frame = True, # visualize the coordinate frame of `world` at its origin
#             world_frame_size = 1.0, # length of the world frame in meter
#             show_link_frame  = False, # do not visualize coordinate frames of entity links
#             show_cameras     = False, # do not visualize mesh and frustum of the cameras added
#             plane_reflection = True, # turn on plane reflection
#             ambient_light    = (0.1, 0.1, 0.1), # ambient light setting
#         ),
#         renderer = gs.renderers.Rasterizer(),
#         )

#         plane = scene.add_entity(
#             gs.morphs.Plane(),
#         )
#         cube = scene.add_entity(
#             gs.morphs.Box(
#                 size = (0.08, 0.08, 0.08),
#                 pos  = (0.65, 0.0, 0.04),
#             )
#         )
#         franka = scene.add_entity(
#             gs.morphs.MJCF(file='xml/franka_emika_panda/panda.xml'),
#         )

#         scene.build()

#         motors_dof = np.arange(7)
#         fingers_dof = np.arange(7, 9)


#         franka.set_dofs_kp(
#         np.array([4500, 4500, 3500, 3500, 2000, 2000, 2000, 100, 100]),
#         )
#         franka.set_dofs_kv(
#             np.array([450, 450, 350, 350, 200, 200, 200, 10, 10]),
#         )
#         franka.set_dofs_force_range(
#             np.array([-87, -87, -87, -87, -12, -12, -12, -100, -100]),
#             np.array([ 87,  87,  87,  87,  12,  12,  12,  100,  100]),
#         )
#         # get the end-effector link
#         end_effector = franka.get_link('hand')

# TODO: Continue when collision mechanics are available in genesis