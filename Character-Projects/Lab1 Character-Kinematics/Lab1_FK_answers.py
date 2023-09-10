import numpy as np
from scipy.spatial.transform import Rotation as R

def load_motion_data(bvh_file_path):
    with open(bvh_file_path, 'r') as f:
        lines = f.readlines()
        for i in range(len(lines)):
            if lines[i].startswith('Frame Time'):
                break
        motion_data = []
        for line in lines[i+1:]:
            data = [float(x) for x in line.split()]
            if len(data) == 0:
                break
            motion_data.append(np.array(data).reshape(1,-1))
        motion_data = np.concatenate(motion_data, axis=0)
    return motion_data

def part1_calculate_T_pose(bvh_file_path):
    joint_name = []; joint_parent = []; joint_offset = []; stack = []
    with open(bvh_file_path, 'r') as f:
        lines = f.readlines()
        for i in range(len(lines)):
            line = lines[i].split()
            if line[0] == '{':
                stack.append(len(joint_name)-1)
            elif line[0] == '}':
                stack.pop()
            elif line[0] == 'ROOT':
                joint_name.append(line[1])
                joint_parent.append(-1)
            elif line[0] == 'JOINT':
                joint_name.append(line[1])
                joint_parent.append(stack[-1])
            elif line[0] == 'End':
                joint_name.append(joint_name[stack[-1]]+'_end')
                joint_parent.append(stack[-1])
            elif line[0] == 'OFFSET':
                joint_offset.append([float(line[1]),float(line[2]),float(line[3])])
            elif line[0] == 'MOTION':
                return joint_name, joint_parent, joint_offset

def part2_forward_kinematics(joint_name, joint_parent, joint_offset, motion_data, frame_id):
    cnt = 1; frame_data = motion_data[frame_id]; joint_positions = [frame_data[0:3]]
    joint_orientations = [R.from_euler('XYZ',frame_data[3:6],True).as_quat()]
    for i in range(1,len(joint_name)):
        if '_end' in joint_name[i]:
            joint_positions.append(joint_positions[joint_parent[i]] + R.from_quat(joint_orientations[joint_parent[i]]).as_matrix() @ joint_offset[i])
            joint_orientations.append(joint_orientations[joint_parent[i]])
        else:
            joint_positions.append(joint_positions[joint_parent[i]] + R.from_quat(joint_orientations[joint_parent[i]]).as_matrix() @ joint_offset[i])
            joint_orientations.append((R.from_quat(joint_orientations[joint_parent[i]]) * R.from_euler('XYZ',frame_data[3*cnt+3:3*cnt+6],True)).as_quat())
            cnt+=1
    return np.asarray(joint_positions), np.asarray(joint_orientations)


def part3_retarget_func(T_pose_bvh_path, A_pose_bvh_path):
    motion_data = []
    amotion_data = load_motion_data(A_pose_bvh_path)
    t_name,t_parent,t_offset = part1_calculate_T_pose(T_pose_bvh_path)
    a_name,a_parent,a_offset = part1_calculate_T_pose(A_pose_bvh_path)

    cnt = 0;a_map = {}
    for i in range(len(a_name)):
        if '_end' not in a_name[i]:
            a_map[a_name[i]] = cnt; cnt += 1

    for i in range(amotion_data.shape[0]):
        line_data = [amotion_data[i][0:3]]
        for j in range(len(t_name)):
            if t_name[j] == 'lShoulder':
                cnt = a_map[t_name[j]]
                line_data.append((R.from_euler('XYZ',list(amotion_data[i][cnt*3+3:cnt*3+6]),True)*R.from_euler('XYZ',[0,0,-45],True)).as_euler('XYZ',True))
            elif t_name[j] == 'rShoulder':
                cnt = a_map[t_name[j]]
                line_data.append((R.from_euler('XYZ',list(amotion_data[i][cnt*3+3:cnt*3+6]),True)*R.from_euler('XYZ',[0,0,45],True)).as_euler('XYZ',True))
            elif '_end' not in t_name[j]:
                cnt = a_map[t_name[j]]
                line_data.append(amotion_data[i][cnt*3+3:cnt*3+6])
        motion_data.append(np.concatenate(line_data))
    return np.asarray(motion_data)
'''
    T_joint_name, _, _ = part1_calculate_T_pose(T_pose_bvh_path)
    A_joint_name, _, _ = part1_calculate_T_pose(A_pose_bvh_path)
    A_motion_data = load_motion_data(A_pose_bvh_path)

    A_joint_map = {}
    count = 0
    for i in range(len(A_joint_name)):
        if '_end' in A_joint_name[i]:
            count += 1
        A_joint_map[A_joint_name[i]] = i - count

    motion_data = []
    # for i in range(1): debug init pose. lShoulder add 0,0,-45, rShoulder add 0,0,45
    for i in range(A_motion_data.shape[0]):
        data = []
        for joint in T_joint_name:
            index = A_joint_map[joint]

            if joint == 'RootJoint':
                data += list(A_motion_data[i][0:6])
            elif joint == 'lShoulder':
                Rot = (R.from_euler('XYZ', list(A_motion_data[i][index * 3 + 3: index*3 + 6]), degrees=True) * R.from_euler('XYZ', [0., 0., -45.], degrees=True)).as_euler('XYZ',True)
                data += list(Rot)
            elif joint == 'rShoulder':
                Rot = (R.from_euler('XYZ', list(A_motion_data[i][index * 3 + 3: index*3 + 6]), degrees=True) * R.from_euler('XYZ', [0., 0., 45.], degrees=True)).as_euler('XYZ',True)
                data += list(Rot)
            elif '_end' in joint:
                continue
            else:
                data += list(A_motion_data[i][index * 3 + 3: index * 3 + 6])
        motion_data.append(np.array(data).reshape(1, -1))

    motion_data = np.concatenate(motion_data, axis=0)
    return motion_data
    '''