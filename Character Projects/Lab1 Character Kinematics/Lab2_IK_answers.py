import numpy as np
from scipy.spatial.transform import Rotation as R
import torch


def part1_inverse_kinematics(meta_data, joint_positions, joint_orientations, target_pose):
    joint_parent = meta_data.joint_parent
    joint_offset = [meta_data.joint_initial_position[i] - meta_data.joint_initial_position[joint_parent[i]] for i in
                    range(len(joint_positions))]
    joint_offset[0] = np.array([0., 0., 0.])
    joint_ik_path, _, _, _ = meta_data.get_path_from_root_to_end()

    local_rotation = [R.from_quat(joint_orientations[joint_parent[i]]).inv() * R.from_quat(joint_orientations[i]) for i
                      in range(len(joint_orientations))]
    local_rotation[0] = R.from_quat(joint_orientations[0])

    joint_offset_t = [torch.tensor(data) for data in joint_offset]
    joint_positions_t = [torch.tensor(data) for data in joint_positions]
    joint_orientations_t = [torch.tensor(R.from_quat(data).as_matrix(), requires_grad=True) for data in joint_orientations]
    local_rotation_t = [torch.tensor(data.as_matrix(),requires_grad=True) for data in local_rotation]
    target_pose_t = torch.tensor(target_pose)

    epoch = 300
    alpha = 0.001
    for _ in range(epoch):
        for j in range(len(joint_ik_path)):
            a = chain_current = joint_ik_path[j]
            b = chain_parent = joint_ik_path[j - 1]
            if j == 0:
                local_rotation_t[a] = local_rotation_t[a]
                joint_positions_t[a] = joint_positions_t[a]
            elif b == joint_parent[a]:
                joint_orientations_t[a] = joint_orientations_t[b] @ local_rotation_t[a]
                joint_positions_t[a] = joint_positions_t[b] + joint_offset_t[a] @ torch.transpose(joint_orientations_t[b],0,1)
            else:
                joint_orientations_t[a] = joint_orientations_t[b] @ torch.transpose(local_rotation_t[b],0,1)
                joint_positions_t[a] = joint_positions_t[b] + (-joint_offset_t[a]) @ torch.transpose(joint_orientations_t[a],0,1)


        optimize_target = torch.norm(joint_positions_t[joint_ik_path[-1]] - target_pose_t)
        optimize_target.backward()
        for num in joint_ik_path:
            if local_rotation_t[num].grad is not None:
                tmp = local_rotation_t[num] - alpha * local_rotation_t[num].grad
                local_rotation_t[num] = torch.tensor(tmp, requires_grad=True)

    for j in range(len(joint_ik_path)):
        a = chain_current = joint_ik_path[j]
        b = chain_parent = joint_ik_path[j - 1]
        if j == 0:
             local_rotation[a] = R.from_matrix(local_rotation_t[a].detach().numpy())
             joint_positions[a] = joint_positions[a]
        elif b == joint_parent[a]:
             joint_orientations[a] = (R.from_quat(joint_orientations[b]) * R.from_matrix(local_rotation_t[a].detach().numpy())).as_quat()
             joint_positions[a] = joint_positions[b] + joint_offset[a] * np.asmatrix(R.from_quat(joint_orientations[b]).as_matrix()).transpose()
        else:
             joint_orientations[a] = (R.from_quat(joint_orientations[b]) * R.from_matrix(local_rotation_t[b].detach().numpy()).inv()).as_quat()
             joint_positions[a] = joint_positions[b] + (-joint_offset[b]) * np.asmatrix(R.from_quat(joint_orientations[a]).as_matrix()).transpose()

    ik_path_set = set(joint_ik_path)
    for i in range(len(joint_positions)):
        if i in ik_path_set:
            joint_orientations[i] = R.from_matrix(joint_orientations_t[i].detach().numpy()).as_quat()
        else:
            joint_orientations[i] = (R.from_quat(joint_orientations[joint_parent[i]]) * local_rotation[i]).as_quat()
            joint_positions[i] = joint_positions[joint_parent[i]] + joint_offset[i] * np.asmatrix(
                R.from_quat(joint_orientations[joint_parent[i]]).as_matrix()).transpose()

    return joint_positions, joint_orientations


def part2_inverse_kinematics(meta_data, joint_positions, joint_orientations, relative_x, relative_z, target_height):
    joint_parent = meta_data.joint_parent
    joint_offset = [meta_data.joint_initial_position[i] - meta_data.joint_initial_position[joint_parent[i]] for i in
                    range(len(joint_positions))]
    joint_offset[0] = np.array([0., 0., 0.])
    joint_ik_path, _, _, _ = meta_data.get_path_from_root_to_end()

    local_rotation = [R.from_quat(joint_orientations[joint_parent[i]]).inv() * R.from_quat(joint_orientations[i]) for i
                      in range(len(joint_orientations))]
    local_rotation[0] = R.from_quat(joint_orientations[0])

    joint_offset_t = [torch.tensor(data) for data in joint_offset]
    joint_positions_t = [torch.tensor(data) for data in joint_positions]
    joint_orientations_t = [torch.tensor(R.from_quat(data).as_matrix(), requires_grad=True) for data in
                            joint_orientations]
    local_rotation_t = [torch.tensor(data.as_matrix(), requires_grad=True) for data in local_rotation]
    target_pose_t = torch.tensor([relative_x + joint_positions[0][0], target_height, relative_z + joint_positions[0][2]])

    for j in joint_ik_path:
        local_rotation_t[j] = torch.tensor(R.from_quat([1.,0.,0.,0.]).as_matrix(), requires_grad=True)

    epoch = 200
    alpha = 10

    for time in range(epoch):
        for j in joint_ik_path:
            if j == joint_ik_path[0]:
                 joint_orientations_t[j] = local_rotation_t[j]
                 joint_positions_t[j] = joint_positions_t[j]
            else:
                joint_orientations_t[j] = joint_orientations_t[joint_parent[j]] @ local_rotation_t[j]
                joint_positions_t[j] = joint_positions_t[joint_parent[j]] + joint_offset_t[j] @ torch.transpose(joint_orientations_t[joint_parent[j]],0, 1)

        optimize_target = torch.norm(joint_positions_t[joint_ik_path[-1]] - target_pose_t)
        optimize_target.backward()
        for num in joint_ik_path:
            if local_rotation_t[num].grad is not None:
                tmp = local_rotation_t[num] - alpha * torch.exp(-torch.tensor(time/5)) * local_rotation_t[num].grad
                local_rotation_t[num] = torch.tensor(tmp, requires_grad=True)


    for j in joint_ik_path:
        local_rotation[j] = R.from_matrix(local_rotation_t[j].detach().numpy())

    for i in range(len(joint_positions)):
        if i == 0:
            joint_orientations[i] = local_rotation[i].as_quat()
            joint_positions[i] = joint_positions[i]
        else:
            if i == joint_ik_path[0]:
                joint_orientations[i] = local_rotation[i].as_quat()
            else:
                joint_orientations[i] = (R.from_quat(joint_orientations[joint_parent[i]]) * local_rotation[i]).as_quat()
            joint_positions[i] = joint_positions[joint_parent[i]] + joint_offset[i] * np.asmatrix(R.from_quat(joint_orientations[joint_parent[i]]).as_matrix()).transpose()

    return joint_positions, joint_orientations


def bonus_inverse_kinematics(meta_data, joint_positions, joint_orientations, left_target_pose, right_target_pose):
    joint_ik_path2 = [meta_data.joint_name.index('rWrist_end')]
    while meta_data.joint_parent[joint_ik_path2[-1]] != -1:
        joint_ik_path2.append(meta_data.joint_parent[joint_ik_path2[-1]])


    joint_parent = meta_data.joint_parent
    joint_offset = [meta_data.joint_initial_position[i] - meta_data.joint_initial_position[joint_parent[i]] for i in
                    range(len(joint_positions))]
    joint_offset[0] = np.array([0., 0., 0.])
    joint_ik_path, _, _, _ = meta_data.get_path_from_root_to_end()

    local_rotation = [R.from_quat(joint_orientations[joint_parent[i]]).inv() * R.from_quat(joint_orientations[i]) for i
                      in range(len(joint_orientations))]
    local_rotation[0] = R.from_quat(joint_orientations[0])

    joint_offset_t = [torch.tensor(data) for data in joint_offset]
    joint_positions_t = [torch.tensor(data) for data in joint_positions]
    joint_orientations_t = [torch.tensor(R.from_quat(data).as_matrix(), requires_grad=True) for data in joint_orientations]
    local_rotation_t = [torch.tensor(data.as_matrix(),requires_grad=True) for data in local_rotation]
    target_pose_t_1 = torch.tensor(left_target_pose)
    target_pose_t_2 = torch.tensor(right_target_pose)

    epoch = 300
    alpha = 0.001
    for _ in range(epoch):
        for j in range(len(joint_ik_path)):
            a = chain_current = joint_ik_path[j]
            b = chain_parent = joint_ik_path[j - 1]
            if j == 0:
                local_rotation_t[a] = local_rotation_t[a]
                joint_positions_t[a] = joint_positions_t[a]
            elif b == joint_parent[a]:  
                joint_orientations_t[a] = joint_orientations_t[b] @ local_rotation_t[a]
                joint_positions_t[a] = joint_positions_t[b] + joint_offset_t[a] @ torch.transpose(joint_orientations_t[b],0,1)
            else:  
                joint_orientations_t[a] = joint_orientations_t[b] @ torch.transpose(local_rotation_t[b],0,1)
                joint_positions_t[a] = joint_positions_t[b] + (-joint_offset_t[a]) @ torch.transpose(joint_orientations_t[a],0,1)

        optimize_target = torch.norm(joint_positions_t[joint_ik_path[-1]] - target_pose_t_1)
        optimize_target.backward()
        for num in joint_ik_path:
            if local_rotation_t[num].grad is not None:
                tmp = local_rotation_t[num] - alpha * local_rotation_t[num].grad
                local_rotation_t[num] = torch.tensor(tmp, requires_grad=True)

    for j in range(len(joint_ik_path)):
        a = chain_current = joint_ik_path[j]
        b = chain_parent = joint_ik_path[j - 1]
        if j == 0:
            local_rotation[a] = R.from_matrix(local_rotation_t[a].detach().numpy())
            joint_positions[a] = joint_positions[a]
        elif b == joint_parent[a]:
            joint_orientations[a] = (R.from_quat(joint_orientations[b]) * R.from_matrix(
                local_rotation_t[a].detach().numpy())).as_quat()
            joint_positions[a] = joint_positions[b] + joint_offset[a] * np.asmatrix(
                R.from_quat(joint_orientations[b]).as_matrix()).transpose()
        else:
            joint_orientations[a] = (R.from_quat(joint_orientations[b]) * R.from_matrix(
                local_rotation_t[b].detach().numpy()).inv()).as_quat()
            joint_positions[a] = joint_positions[b] + (-joint_offset[b]) * np.asmatrix(
                R.from_quat(joint_orientations[a]).as_matrix()).transpose()

    joint_offset_t = [torch.tensor(data) for data in joint_offset]
    joint_positions_t = [torch.tensor(data) for data in joint_positions]
    joint_orientations_t = [torch.tensor(R.from_quat(data).as_matrix(), requires_grad=True) for data in joint_orientations]
    local_rotation_t = [torch.tensor(data.as_matrix(),requires_grad=True) for data in local_rotation]
    epoch = 300
    alpha = 0.001
    for _ in range(epoch):
        for j in reversed(joint_ik_path2):
            if j == 0:
                continue
            else:
                joint_orientations_t[j] = joint_orientations_t[joint_parent[j]] @ local_rotation_t[j]
                joint_positions_t[j] = joint_positions_t[joint_parent[j]] + (joint_offset_t[j]) @ torch.transpose(
                    joint_orientations_t[joint_parent[j]], 0, 1)

        optimize_target2 = torch.norm(joint_positions_t[joint_ik_path2[0]] - target_pose_t_2)
        optimize_target2.backward()
        for num in joint_ik_path2:
            if local_rotation_t[num].grad is not None:
                tmp = local_rotation_t[num] - alpha * local_rotation_t[num].grad
                local_rotation_t[num] = torch.tensor(tmp, requires_grad=True)

    for j in reversed(joint_ik_path2):
        if j != 0:
            joint_orientations[j] = (R.from_quat(joint_orientations[joint_parent[j]]) * R.from_matrix(local_rotation_t[j].detach().numpy())).as_quat()
            joint_positions[j] = joint_positions[joint_parent[j]] + (joint_offset[j]) * np.asmatrix(R.from_quat(joint_orientations[joint_parent[j]]).as_matrix()).transpose()

    ik_path_set = set( joint_ik_path + joint_ik_path2)
    for i in range(len(joint_positions)):
        if i in ik_path_set:
            joint_orientations[i] = R.from_matrix(joint_orientations_t[i].detach().numpy()).as_quat()
        else:
            joint_orientations[i] = (R.from_quat(joint_orientations[joint_parent[i]]) * local_rotation[i]).as_quat()
            joint_positions[i] = joint_positions[joint_parent[i]] + joint_offset[i] * np.asmatrix(
                R.from_quat(joint_orientations[joint_parent[i]]).as_matrix()).transpose()
    
    return joint_positions, joint_orientations