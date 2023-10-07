import numpy as np
from scipy.spatial.transform import Rotation as R
from bvh_loader import BVHMotion
from physics_warpper import PhysicsInfo


def part1_cal_torque(pose, physics_info: PhysicsInfo, **kargs):
    kp = kargs.get('kp', 500)
    kd = kargs.get('kd', 20) 
    parent_index = physics_info.parent_index
    joint_name = physics_info.joint_name
    joint_orientation = physics_info.get_joint_orientation()
    joint_avel = physics_info.get_body_angular_velocity()

    global_orientation = np.zeros((20, 3))
    global_orientation[0] = (R.from_quat(pose[0]) * R.from_quat(joint_orientation[0])).as_euler("XYZ",degrees=True)
    for i in range(1, len(joint_orientation)):
        global_orientation[i] = (R.from_quat(pose[i]) * R.from_quat(joint_orientation[parent_index[i]]) * R.from_quat(joint_orientation[i]).inv()).as_euler("XYZ",degrees=True)

    global_torque = kp * global_orientation - kd * joint_avel
    
    return global_torque

def part2_cal_float_base_torque(target_position, pose, physics_info, **kargs):
    global_torque = part1_cal_torque(pose, physics_info)
    kp = kargs.get('root_kp', 500) # 需要自行调整root的kp和kd！
    kd = kargs.get('root_kd', 5)
    root_position, root_velocity = physics_info.get_root_pos_and_vel()
    global_root_force = kp * (target_position - root_position) - kd * root_velocity
    return global_root_force, global_torque

def part3_cal_static_standing_torque(bvh: BVHMotion, physics_info):
    tar_pos = bvh.joint_position[0][0]
    joint_positions = physics_info.get_joint_translation()
    tar_pos = tar_pos * 0.5 + joint_positions[9] * 0.25 + joint_positions[10] * 0.25

    pose = bvh.joint_rotation[0]
    joint_name = physics_info.joint_name
    global_torque = part1_cal_torque(pose, physics_info)
    joint_velocity = physics_info.get_body_velocity()
    joint_mass = physics_info.get_body_mass()

    com = np.zeros(3)
    com_velocity = np.zeros(3)
    mass = 0
    for i in range(len(joint_mass)):
        com += joint_mass[i] * joint_positions[i]
        com_velocity = joint_mass[i] * joint_velocity[i]
        mass += joint_mass[i]
    com /= mass
    com_velocity /= mass
    desired_com = tar_pos

    Kp = 4000
    Kd = 20
    virtual_force = Kp * (desired_com - com) - Kd * com_velocity

    torque = global_torque
    for i in range(0, len(torque)):
        torque[i] -= np.cross(com - joint_positions[i], virtual_force)

    return torque

