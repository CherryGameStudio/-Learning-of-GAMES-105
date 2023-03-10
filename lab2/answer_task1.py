import numpy as np
import copy
from scipy.spatial.transform import Rotation as R
from scipy.spatial.transform import Slerp
# ------------- lab1้็ไปฃ็  -------------#
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

# ------------- ๅฎ็ฐไธไธช็ฎๆ็BVHๅฏน่ฑก๏ผ่ฟ่กๆฐๆฎๅค็ -------------#

'''
ๆณจ้้็ปไธN่กจ็คบๅธงๆฐ๏ผM่กจ็คบๅณ่ๆฐ
position, rotation่กจ็คบๅฑ้จๅนณ็งปๅๆ่ฝฌ
translation, orientation่กจ็คบๅจๅฑๅนณ็งปๅๆ่ฝฌ
'''

class BVHMotion():
    def __init__(self, bvh_file_name = None) -> None:
        
        # ไธไบ meta data
        self.joint_name = []
        self.joint_channel = []
        self.joint_parent = []
        
        # ไธไบlocalๆฐๆฎ, ๅฏนๅบbvh้็channel, XYZpositionๅ XYZrotation
        #! ่ฟ้ๆไปฌๆๆฒกๆXYZ position็joint็position่ฎพ็ฝฎไธบoffset, ไป่่ฟ่ก็ปไธ
        self.joint_position = None # (N,M,3) ็ndarray, ๅฑ้จๅนณ็งป
        self.joint_rotation = None # (N,M,4)็ndarray, ็จๅๅๆฐ่กจ็คบ็ๅฑ้จๆ่ฝฌ
        
        if bvh_file_name is not None:
            self.load_motion(bvh_file_name)
        pass
    
    #------------------- ไธไบ่พๅฉๅฝๆฐ ------------------- #
    def load_motion(self, bvh_file_path):
        '''
            ่ฏปๅbvhๆไปถ๏ผๅๅงๅๅๆฐๆฎๅๅฑ้จๆฐๆฎ
        '''
        self.joint_name, self.joint_parent, self.joint_channel, joint_offset = \
            load_meta_data(bvh_file_path)
        
        motion_data = load_motion_data(bvh_file_path)

        # ๆmotion_data้็ๆฐๆฎๅ้ๅฐjoint_positionๅjoint_rotation้
        self.joint_position = np.zeros((motion_data.shape[0], len(self.joint_name), 3))
        self.joint_rotation = np.zeros((motion_data.shape[0], len(self.joint_name), 4))
        self.joint_rotation[:,:,3] = 1.0 # ๅๅๆฐ็wๅ้้ป่ฎคไธบ1
        
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
        ๅฉ็จ่ช่บซ็metadata่ฟ่กๆน้ๅๅ่ฟๅจๅญฆ
        joint_position: (N,M,3)็ndarray, ๅฑ้จๅนณ็งป
        joint_rotation: (N,M,4)็ndarray, ็จๅๅๆฐ่กจ็คบ็ๅฑ้จๆ่ฝฌ
        '''
        if joint_position is None:
            joint_position = self.joint_position
        if joint_rotation is None:
            joint_rotation = self.joint_rotation
        
        joint_translation = np.zeros_like(joint_position)
        joint_orientation = np.zeros_like(joint_rotation)
        joint_orientation[:,:,3] = 1.0 # ๅๅๆฐ็wๅ้้ป่ฎคไธบ1
        
        # ไธไธชๅฐhackๆฏroot joint็parentๆฏ-1, ๅฏนๅบๆๅไธไธชๅณ่
        # ่ฎก็ฎๆ น่็นๆถๆๅไธไธชๅณ่่ฟๆช่ขซ่ฎก็ฎ๏ผๅๅฅฝๆฏ0ๅ็งปๅๅไฝๆๅ
        
        for i in range(len(self.joint_name)):
            pi = self.joint_parent[i]
            parent_orientation = R.from_quat(joint_orientation[:,pi,:]) 
            joint_translation[:, i, :] = joint_translation[:, pi, :] + \
                parent_orientation.apply(joint_position[:, i, :])
            joint_orientation[:, i, :] = (parent_orientation * R.from_quat(joint_rotation[:, i, :])).as_quat()
        return joint_translation, joint_orientation
    
    
    def adjust_joint_name(self, target_joint_name):
        '''
        ่ฐๆดๅณ่้กบๅบไธบtarget_joint_name
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
        ่ฟๅไธไธชๆท่ด
        '''
        return copy.deepcopy(self)
    
    @property
    def motion_length(self):
        return self.joint_position.shape[0]
    
    
    def sub_sequence(self, start, end):
        '''
        ่ฟๅไธไธชๅญๅบๅ
        start: ๅผๅงๅธง
        end: ็ปๆๅธง
        '''
        res = self.raw_copy()
        res.joint_position = res.joint_position[start:end,:,:]
        res.joint_rotation = res.joint_rotation[start:end,:,:]
        return res
    
    def append(self, other):
        '''
        ๅจๆซๅฐพๆทปๅ ๅฆไธไธชๅจไฝ
        '''
        other = other.raw_copy()
        other.adjust_joint_name(self.joint_name)
        self.joint_position = np.concatenate((self.joint_position, other.joint_position), axis=0)
        self.joint_rotation = np.concatenate((self.joint_rotation, other.joint_rotation), axis=0)
        pass
    
    #--------------------- ไฝ ็ไปปๅก -------------------- #
    
    def decompose_rotation_with_yaxis(self, rotation):
        '''
        ่พๅฅ: rotation ๅฝข็ถไธบ(4,)็ndarray, ๅๅๆฐๆ่ฝฌ
        ่พๅบ: Ry, Rxz๏ผๅๅซไธบ็ปy่ฝด็ๆ่ฝฌๅ่ฝฌ่ฝดๅจxzๅนณ้ข็ๆ่ฝฌ๏ผๅนถๆปก่ถณR = Ry * Rxz
        '''
        Ry = np.zeros_like(rotation)
        Rxz = np.zeros_like(rotation)
        # TODO: ไฝ ็ไปฃ็ 
        
        return Ry, Rxz
    
    # part 1
    def translation_and_rotation(self, frame_num, target_translation_xz, target_facing_direction_xz):
        '''
        ่ฎก็ฎๅบๆฐ็joint_positionๅjoint_rotation
        ไฝฟ็ฌฌframe_numๅธง็ๆ น่็นๅนณ็งปไธบtarget_translation_xz, ๆฐดๅนณ้ขๆๅไธบtarget_facing_direction_xz
        frame_num: int
        target_translation_xz: (2,)็ndarray
        target_faceing_direction_xz: (2,)็ndarray๏ผ่กจ็คบๆฐดๅนณๆๅใไฝ ๅฏไปฅ็่งฃไธบๅๆฌ็z่ฝด่ขซๆ่ฝฌๅฐ่ฟไธชๆนๅใ
        Tips:
            ไธป่ฆๆฏ่ฐๆดroot่็น็joint_positionๅjoint_rotation
            frame_numๅฏ่ฝๆฏ่ดๆฐ๏ผ้ตๅพชpython็็ดขๅผ่งๅ
            ไฝ ้่ฆๅฎๆๅนถไฝฟ็จdecompose_rotation_with_yaxis
            ่พๅฅ็target_facing_direction_xz็normไธไธๅฎๆฏ1
        '''
        
        res = self.raw_copy() # ๆท่ดไธไปฝ๏ผไธ่ฆไฟฎๆนๅๅงๆฐๆฎ
        
        # ๆฏๅฆ่ฏด๏ผไฝ ๅฏไปฅ่ฟๆ ท่ฐๆด็ฌฌframe_numๅธง็ๆ น่็นๅนณ็งป
        offset = target_translation_xz - res.joint_position[frame_num, 0, [0,2]]
        res.joint_position[:, 0, [0,2]] += offset
        # TODO: ไฝ ็ไปฃ็ 
        return res

# part2
def blend_two_motions(bvh_motion1, bvh_motion2, alpha):
    '''
    blendไธคไธชbvhๅจไฝ
    ๅ่ฎพไธคไธชๅจไฝ็ๅธงๆฐๅๅซไธบn1, n2
    alpha: 0~1ไน้ด็ๆตฎ็นๆฐ็ป๏ผๅฝข็ถไธบ(n3,)
    ่ฟๅ็ๅจไฝๅบ่ฏฅๆn3ๅธง๏ผ็ฌฌiๅธง็ฑ(1-alpha[i]) * bvh_motion1[j] + alpha[i] * bvh_motion2[k]ๅพๅฐ
    iๅๅๅฐ้ๅ0~n3-1็ๅๆถ๏ผjๅkๅบ่ฏฅๅๅๅฐ้ๅ0~n1-1ๅ0~n2-1
    '''
    
    res = bvh_motion1.raw_copy()
    res.joint_position = np.zeros((len(alpha), res.joint_position.shape[1], res.joint_position.shape[2]))
    res.joint_rotation = np.zeros((len(alpha), res.joint_rotation.shape[1], res.joint_rotation.shape[2]))
    res.joint_rotation[...,3] = 1.0

    # TODO: ไฝ ็ไปฃ็ 
    
    return res

# part3
def build_loop_motion(bvh_motion):
    '''
    ๅฐbvhๅจไฝๅไธบๅพช็ฏๅจไฝ
    ็ฑไบๆฏ่พๅคๆ,ไฝไธบ็ฆๅฉ,ไธ็จ่ชๅทฑๅฎ็ฐ
    (ๅฝ็ถไฝ ไนๅฏไปฅ่ชๅทฑๅฎ็ฐ่ฏไธไธ)
    ๆจ่้่ฏป https://theorangeduck.com/
    Creating Looping Animations from Motion Capture
    '''
    res = bvh_motion.raw_copy()
    
    from smooth_utils import build_loop_motion
    return build_loop_motion(res)

# part4
def concatenate_two_motions(bvh_motion1, bvh_motion2, mix_frame1, mix_time):
    '''
    ๅฐไธคไธชbvhๅจไฝๅนณๆปๅฐ่ฟๆฅ่ตทๆฅ๏ผmix_time่กจ็คบ็จไบๆททๅ็ๅธงๆฐ
    ๆททๅๅผๅงๆถ้ดๆฏ็ฌฌไธไธชๅจไฝ็็ฌฌmix_frame1ๅธง
    ่ฝ็ถๆไบๆททๅๆนๆณๅฏ่ฝไธ้่ฆmix_time๏ผไฝๆฏไธบไบไฟ่ฏๆฅๅฃไธ่ด๏ผๆไปฌ่ฟๆฏไฟ็่ฟไธชๅๆฐ
    Tips:
        ไฝ ๅฏ่ฝ้่ฆ็จๅฐBVHMotion.sub_sequence ๅ BVHMotion.append
    '''
    res = bvh_motion1.raw_copy()
    
    # TODO: ไฝ ็ไปฃ็ 
    # ไธ้ข่ฟ็ง็ดๆฅๆผ่ฏๅฎๆฏไธ่ก็(
    res.joint_position = np.concatenate([res.joint_position[:mix_frame1], bvh_motion2.joint_position], axis=0)
    res.joint_rotation = np.concatenate([res.joint_rotation[:mix_frame1], bvh_motion2.joint_rotation], axis=0)
    
    return res

