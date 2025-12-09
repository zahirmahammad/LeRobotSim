import genesis as gs

gs.init(backend=gs.gpu)

scene = gs.Scene(
    show_viewer = True,
    viewer_options = gs.options.ViewerOptions(
        res           = (1280, 960),
        camera_pos    = (3.5, 0.0, 2.5),
        camera_lookat = (0.0, 0.0, 0.5),
        camera_fov    = 40,
        max_FPS       = 60,
    ),
    vis_options = gs.options.VisOptions(
        show_world_frame = True,
        world_frame_size = 1.0,
        show_link_frame  = False,
        show_cameras     = False,
        plane_reflection = True,
        ambient_light    = (0.1, 0.1, 0.1),
    ),
    renderer=gs.renderers.Rasterizer(),
)

plane = scene.add_entity(
    gs.morphs.Plane(),
)

lerobot = scene.add_entity(
    gs.morphs.URDF(file='SimulationRobot/SO101/so101_new_calib.urdf', fixed=True),)

cam = scene.add_camera(
    res    = (640, 480),
    pos    = (3.5, 0.0, 2.5),
    lookat = (0, 0, 0.5),
    fov    = 30,
    GUI    = False,
)

scene.build()

# render rgb, depth, segmentation, and normal
# rgb, depth, segmentation, normal = cam.render(rgb=True, depth=True, segmentation=True, normal=True)

# cam.start_recording()
import numpy as np

joint_names = ['shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll', 'gripper']

dofs_idx = [lerobot.get_joint(name).dof_idx_local for name in joint_names]

print(f"DOF indices: {dofs_idx}")

# # set positional gains
# lerobot.set_dofs_kp(
#     kp             = np.array([4500, 4500, 3500, 3500, 2000, 2000]),
#     dofs_idx_local = dofs_idx,
# )
# # set velocity gains
# lerobot.set_dofs_kv(
#     kv             = np.array([450, 450, 350, 350, 200, 200]),
#     dofs_idx_local = dofs_idx,
# )

def move_to_position(position):
    error = 1.0
    while error > 0.01:
        curr_pos = lerobot.get_dofs_position(dofs_idx).cpu().numpy()
        error =np.max(np.abs(position - curr_pos))
        print(f"Moving... Current Pos : {curr_pos}, Target Pos : {position}, Error: {error}")
        lerobot.control_dofs_position(
            position,
            dofs_idx,
        )
        scene.step()
    # curr_pos = lerobot.get_dofs_position(dofs_idx)
    # print(f"Current Pos : {curr_pos}")
    # print(f"Target  Pos : {position}")

trajectory = [
    np.array([0.787, 0.0, 0.0, 0.0, 0.0, 0.0]),
    np.array([1.57, 0.0, 0.0, 0.0, 0.0, 0.0]),
    np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
    # np.array([1.57, 0.0, 0.0, 0.0, 0.0, 0.0]),
]


while True:
    for position in trajectory:
        move_to_position(position)
    scene.step()