import numpy as np
import copy
from scipy.spatial.transform import Rotation as R
# ------------- lab1里的代码 -------------#
def load_meta_data(bvh_path):
    with open(bvh_path, 'r') as f:
        channels = []
        joints = []
        joint_parents = []
        joint_offsets = []
        end_sites = []

        parent_stack = [None]
        for line in f:
            if 'ROOT' in line or 'JOINT' in line:
                joints.append(line.split()[-1])
                joint_parents.append(parent_stack[-1])
                channels.append('')
                joint_offsets.append([0, 0, 0])

            elif 'End Site' in line:
                end_sites.append(len(joints))
                joints.append(parent_stack[-1] + '_end')
                joint_parents.append(parent_stack[-1])
                channels.append('')
                joint_offsets.append([0, 0, 0])

            elif '{' in line:
                parent_stack.append(joints[-1])

            elif '}' in line:
                parent_stack.pop()

            elif 'OFFSET' in line:
                joint_offsets[-1] = np.array([float(x) for x in line.split()[-3:]]).reshape(1,3)

            elif 'CHANNELS' in line:
                trans_order = []
                rot_order = []
                for token in line.split():
                    if 'position' in token:
                        trans_order.append(token[0])

                    if 'rotation' in token:
                        rot_order.append(token[0])

                channels[-1] = ''.join(trans_order)+ ''.join(rot_order)

            elif 'Frame Time:' in line:
                break
        
    joint_parents = [-1]+ [joints.index(i) for i in joint_parents[1:]]
    channels = [len(i) for i in channels]
    return joints, joint_parents, channels, joint_offsets

def load_motion_data(bvh_path):
    with open(bvh_path, 'r') as f:
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

# ------------- 实现一个简易的BVH对象，进行数据处理 -------------#

'''
注释里统一N表示帧数，M表示关节数
position, rotation表示局部平移和旋转
translation, orientation表示全局平移和旋转
'''

class BVHMotion():
    def __init__(self, bvh_file_name = None) -> None:
        
        # 一些 meta data
        self.joint_name = []
        self.joint_channel = []
        self.joint_parent = []
        
        # 一些local数据, 对应bvh里的channel, XYZposition和 XYZrotation
        #! 这里我们把没有XYZ position的joint的position设置为offset, 从而进行统一
        self.joint_position = None # (N,M,3) 的ndarray, 局部平移
        self.joint_rotation = None # (N,M,4)的ndarray, 用四元数表示的局部旋转
        
        if bvh_file_name is not None:
            self.load_motion(bvh_file_name)
        pass
    
    #------------------- 一些辅助函数 ------------------- #
    def load_motion(self, bvh_file_path):
        '''
            读取bvh文件，初始化元数据和局部数据
        '''
        self.joint_name, self.joint_parent, self.joint_channel, joint_offset = \
            load_meta_data(bvh_file_path)
        
        motion_data = load_motion_data(bvh_file_path)

        # 把motion_data里的数据分配到joint_position和joint_rotation里
        self.joint_position = np.zeros((motion_data.shape[0], len(self.joint_name), 3))
        self.joint_rotation = np.zeros((motion_data.shape[0], len(self.joint_name), 4))
        self.joint_rotation[:,:,3] = 1.0 # 四元数的w分量默认为1
        
        cur_channel = 0
        for i in range(len(self.joint_name)):
            if self.joint_channel[i] == 0:
                self.joint_position[:,i,:] = joint_offset[i].reshape(1,3)
                continue   
            elif self.joint_channel[i] == 3:
                self.joint_position[:,i,:] = joint_offset[i].reshape(1,3)
                rotation = motion_data[:, cur_channel:cur_channel+3]
            elif self.joint_channel[i] == 6:
                self.joint_position[:, i, :] = motion_data[:, cur_channel:cur_channel+3]
                rotation = motion_data[:, cur_channel+3:cur_channel+6]
            self.joint_rotation[:, i, :] = R.from_euler('XYZ', rotation,degrees=True).as_quat()
            cur_channel += self.joint_channel[i]
        
        return

    def batch_forward_kinematics(self, joint_position = None, joint_rotation = None):
        '''
        利用自身的metadata进行批量前向运动学
        joint_position: (N,M,3)的ndarray, 局部平移
        joint_rotation: (N,M,4)的ndarray, 用四元数表示的局部旋转
        '''
        if joint_position is None:
            joint_position = self.joint_position
        if joint_rotation is None:
            joint_rotation = self.joint_rotation
        
        joint_translation = np.zeros_like(joint_position)
        joint_orientation = np.zeros_like(joint_rotation)
        joint_orientation[:,:,3] = 1.0 # 四元数的w分量默认为1
        
        # 一个小hack是root joint的parent是-1, 对应最后一个关节
        # 计算根节点时最后一个关节还未被计算，刚好是0偏移和单位朝向
        
        for i in range(len(self.joint_name)):
            pi = self.joint_parent[i]
            parent_orientation = R.from_quat(joint_orientation[:,pi,:]) 
            joint_translation[:, i, :] = joint_translation[:, pi, :] + \
                parent_orientation.apply(joint_position[:, i, :])
            joint_orientation[:, i, :] = (parent_orientation * R.from_quat(joint_rotation[:, i, :])).as_quat()
        return joint_translation, joint_orientation
    
    
    def adjust_joint_name(self, target_joint_name):
        '''
        调整关节顺序为target_joint_name
        '''
        idx = [self.joint_name.index(joint_name) for joint_name in target_joint_name]
        idx_inv = [target_joint_name.index(joint_name) for joint_name in self.joint_name]
        self.joint_name = [self.joint_name[i] for i in idx]
        self.joint_parent = [idx_inv[self.joint_parent[i]] for i in idx]
        self.joint_parent[0] = -1
        self.joint_channel = [self.joint_channel[i] for i in idx]
        self.joint_position = self.joint_position[:,idx,:]
        self.joint_rotation = self.joint_rotation[:,idx,:]
        pass
    
    def raw_copy(self):
        '''
        返回一个拷贝
        '''
        return copy.deepcopy(self)
    
    @property
    def motion_length(self):
        return self.joint_position.shape[0]
    
    
    def sub_sequence(self, start, end):
        '''
        返回一个子序列
        start: 开始帧
        end: 结束帧
        '''
        res = self.raw_copy()
        res.joint_position = res.joint_position[start:end,:,:]
        res.joint_rotation = res.joint_rotation[start:end,:,:]
        return res
    
    def append(self, other):
        '''
        在末尾添加另一个动作
        '''
        other = other.raw_copy()
        other.adjust_joint_name(self.joint_name)
        self.joint_position = np.concatenate((self.joint_position, other.joint_position), axis=0)
        self.joint_rotation = np.concatenate((self.joint_rotation, other.joint_rotation), axis=0)
        pass
    
    #--------------------- 你的任务 -------------------- #
    
    def decompose_rotation_with_yaxis(self, rotation):
        '''
        输入: rotation 形状为(4,)的ndarray, 四元数旋转
        输出: Ry, Rxz，分别为绕y轴的旋转和转轴在xz平面的旋转，并满足R = Ry * Rxz
        '''
        Rxyz = R.from_quat(rotation).as_euler("XYZ", degrees = True)
        Ry = R.from_euler("XYZ", [0, Rxyz[1], 0], degrees = True)

        return Ry
    
    # part 1
    def translation_and_rotation(self, frame_num, target_translation_xz, target_facing_direction_xz):
        
        res = self.raw_copy() # 拷贝一份，不要修改原始数据
        
        # 比如说，你可以这样调整第frame_num帧的根节点平移
        offset = target_translation_xz - res.joint_position[frame_num, 0, [0,2]]
        res.joint_position[:, 0, [0,2]] += offset
        # TODO: 你的代码
        sin_theta_xz = np.cross(target_facing_direction_xz, np.array([0, 1])) / np.linalg.norm(target_facing_direction_xz)
        cos_theta_xz = np.dot(target_facing_direction_xz, np.array([0, 1])) / np.linalg.norm(target_facing_direction_xz)
        theta = np.arctan(sin_theta_xz / cos_theta_xz)

        new_root_Ry = R.from_euler("Y", theta, degrees=False)
        R_y = self.decompose_rotation_with_yaxis(res.joint_rotation[frame_num, 0, :])
        
        delta_rot = new_root_Ry * R_y.inv()
        res.joint_rotation[:, 0, :] = (delta_rot * R.from_quat(res.joint_rotation[:, 0, :])).as_quat()

        delta_rot_mat = (new_root_Ry * R_y.inv()).as_matrix()
        for i in range(len(res.joint_position)):
             res.joint_position[i, 0,:] = delta_rot_mat  @ (res.joint_position[i, 0, :] - res.joint_position[frame_num, 0, :]) + res.joint_position[frame_num,0,:]
        return res

# part2
def Slerp(r1, r2, t):
    theta = 2 * np.arccos(np.dot(r1, r2))
    return r1
    if theta < 0.01:
        return r1 * t + r2 * (1 - t)
    else:
        return r1 * np.sin(theta - t * theta) / np.sin(theta) + r2 * np.sin(t * theta) / np.sin(theta)

def blend_two_motions(bvh_motion1, bvh_motion2, alpha):
    res = bvh_motion1.raw_copy()
    res.joint_position = np.zeros((len(alpha), res.joint_position.shape[1], res.joint_position.shape[2]))
    res.joint_rotation = np.zeros((len(alpha), res.joint_rotation.shape[1], res.joint_rotation.shape[2]))
    res.joint_rotation[...,3] = 1.0

    n1 = len(bvh_motion1.joint_position)
    n2 = len(bvh_motion2.joint_position)
    n3 = len(alpha)

    for i in range(0, n3):
        j = i * n1 // n3
        k = i * n2 // n3
        res.joint_position[i, :, :] = (1 - alpha[i]) * bvh_motion1.joint_position[j, :, :] + alpha[i] * bvh_motion2.joint_position[k, :, :]

        for l in range(0, len(res.joint_rotation[0])):
            res.joint_rotation[i, l, :] = Slerp(bvh_motion1.joint_rotation[j, l, :], bvh_motion2.joint_rotation[k, l, :], alpha[i])
    return res

# part3
def build_loop_motion(bvh_motion):
    res = bvh_motion.raw_copy()
    
    from smooth_utils import build_loop_motion
    return build_loop_motion(res)

# part4
def concatenate_two_motions(bvh_motion1, bvh_motion2, mix_frame1, mix_time):
    res = bvh_motion1.raw_copy()
    rot = bvh_motion1.joint_rotation[mix_frame1, 0]
    facing_axis = R.from_quat(rot).apply(np.array([0, 0, 1])).flatten()[[0, 2]]

    new_bvh_motion2 = bvh_motion2.translation_and_rotation(0, bvh_motion1.joint_position[mix_frame1, 0, [0, 2]], facing_axis)

    blending_joint_position = np.zeros((mix_time, new_bvh_motion2.joint_position.shape[1], new_bvh_motion2.joint_position.shape[2]))
    blending_joint_rotation = np.zeros((mix_time, new_bvh_motion2.joint_rotation.shape[1], new_bvh_motion2.joint_rotation.shape[2]))

    for i in range(mix_time):
        t = i / mix_time
        blending_joint_position[i] = (1 - t) * res.joint_position[mix_frame1 + i] + t * new_bvh_motion2.joint_position[i]
        for j in range(len(res.joint_rotation[mix_frame1])):
            blending_joint_rotation[i, j] = Slerp(res.joint_rotation[mix_frame1 + i, j], new_bvh_motion2.joint_rotation[i, j], t)

    new_bvh_motion2.joint_position = np.concatenate([blending_joint_position, new_bvh_motion2.joint_position[mix_time:]], axis=0)
    new_bvh_motion2.joint_rotation = np.concatenate([blending_joint_rotation, new_bvh_motion2.joint_rotation[mix_time:]], axis=0)


    res.joint_position = np.concatenate([res.joint_position[:mix_frame1],  new_bvh_motion2.joint_position], axis=0)
    res.joint_rotation = np.concatenate([res.joint_rotation[:mix_frame1],  new_bvh_motion2.joint_rotation], axis=0)
    
    return res

