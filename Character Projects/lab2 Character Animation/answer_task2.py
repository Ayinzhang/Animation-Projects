from answer_task1 import *
from smooth_utils import quat_to_avel, decay_spring_implicit_damping_pos,decay_spring_implicit_damping_rot

class CharacterController():
    def __init__(self, controller) -> None:
        self.motions = []
        self.motions.append(BVHMotion('motion_material/walk_forward.bvh'))
        self.motions.append(BVHMotion('motion_material/idle.bvh'))
        self.motion_id = 0
        self.controller = controller
        self.cur_root_pos = None
        self.cur_root_rot = None
        self.cur_frame = 0
        self.blending_motion = build_loop_motion(self.motions[0])
        self.idle_motion = build_loop_motion(self.motions[1])
        self.idle2move_motion = concatenate_two_motions(self.motions[1], self.motions[0], 60, 30)
        self.move2idle_motion = concatenate_two_motions(self.motions[0], self.motions[1], 60, 30)
        self.motion_state = "idle"
        pass
    
    def update_state(self, desired_pos_list, desired_rot_list, desired_vel_list, desired_avel_list, current_gait):

        joint_name = self.blending_motion.joint_name
        last_motion_state = self.motion_state
        self.motion_state = "idle" if abs(desired_vel_list[0,0])+abs(desired_vel_list[0,1]) < 1e-2 else "move"

        if self.motion_state == "move":
            motion_id = self.motion_id
            current_motion = self.blending_motion.raw_copy()
            if self.motion_state != last_motion_state:
                facing_axis = R.from_quat(self.idle_motion.joint_rotation[self.cur_frame, 0, :]).apply(np.array([0, 0, 1])).flatten()[[0, 2]]
                current_motion = current_motion.translation_and_rotation(0, self.idle_motion.joint_position[self.cur_frame, 0, [0, 2]], facing_axis)
                self.cur_frame = 0
            key_frame = [(self.cur_frame + 20 * i) % self.motions[motion_id].motion_length for i in range(6)]
            current_motion_key_frame_vel = current_motion.joint_position[key_frame, 0, :] - current_motion.joint_position[[(frame - 1) for frame in key_frame], 0, :]
            current_motion_avel = quat_to_avel(current_motion.joint_rotation[:, 0, :], 1 / 60)

            diff_root_pos = desired_pos_list - current_motion.joint_position[ key_frame, 0, :]
            diff_root_pos[:, 1] = 0
            diff_root_rot = (R.from_quat(desired_rot_list[0:6]) * R.from_quat(current_motion.joint_rotation[ key_frame, 0, :]).inv()).as_rotvec()
            diff_root_vel = (desired_vel_list - current_motion_key_frame_vel)/60
            diff_root_avel = desired_avel_list[0:6] - current_motion_avel[[(frame-1) for frame in key_frame]]

            for i in range(self.cur_frame, self.cur_frame+self.motions[motion_id].motion_length//2):
                half_time = 0.2
                index = (i - self.cur_frame) // 20
                dt = (i-self.cur_frame) % 20

                off_pos, _ = decay_spring_implicit_damping_pos(diff_root_pos[index], diff_root_vel[index], half_time, dt/60)
                off_rot, _ = decay_spring_implicit_damping_rot(diff_root_rot[index], diff_root_avel[index], half_time, dt/60)

                current_motion.joint_position[ i % self.motions[motion_id].motion_length, 0, :] += off_pos
                current_motion.joint_rotation[ i % self.motions[motion_id].motion_length, 0, :] = (R.from_rotvec(off_rot) * R.from_quat(current_motion.joint_rotation[ i % self.motions[motion_id].motion_length, 0, :])).as_quat()

            joint_translation, joint_orientation = current_motion.batch_forward_kinematics()
            joint_translation = joint_translation[self.cur_frame]
            joint_orientation = joint_orientation[self.cur_frame]
            self.cur_root_pos = joint_translation[0]
            self.cur_root_rot = joint_orientation[0]

            self.blending_motion = current_motion
            self.cur_frame = (self.cur_frame + 1) % self.motions[motion_id].motion_length

        elif self.motion_state == "idle":
            motion_id = self.motion_id
            current_motion = self.idle_motion
            if self.motion_state != last_motion_state:
                facing_axis = R.from_quat(self.blending_motion.joint_rotation[self.cur_frame, 0, :]).apply(np.array([0, 0, 1])).flatten()[[0, 2]]
                current_motion = current_motion.translation_and_rotation(0, self.blending_motion.joint_position[self.cur_frame, 0, [0, 2]], facing_axis)
                self.cur_frame = 0

            joint_translation, joint_orientation = current_motion.batch_forward_kinematics()
            joint_translation = joint_translation[self.cur_frame]
            joint_orientation = joint_orientation[self.cur_frame]
            self.cur_root_pos = joint_translation[0]
            self.cur_root_rot = joint_orientation[0]
            self.cur_frame = 0
            self.idle_motion = current_motion
            self.cur_frame = (self.cur_frame + 1) % self.motions[motion_id].motion_length

        return joint_name, joint_translation, joint_orientation
    
    
    def sync_controller_and_character(self, controller, character_state):

        controller.set_pos(self.cur_root_pos)
        controller.set_rot(self.cur_root_rot)
        return character_state