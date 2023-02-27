import numpy as np
from scipy.spatial.transform import Rotation as R

def load_motion_data(bvh_file_path):
    """part2 辅助函数，读取bvh文件"""
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
    """请填写以下内容
    输入： bvh 文件路径
    输出:
        joint_name: List[str]，字符串列表，包含着所有关节的名字
        joint_parent: List[int]，整数列表，包含着所有关节的父关节的索引,根节点的父关节索引为-1
        joint_offset: np.ndarray，形状为(M, 3)的numpy数组，包含着所有关节的偏移量

    Tips:
        joint_name顺序应该和bvh一致
    """

    # 1.按行读取BVH文件,读到ROOT代表开始,读到MOTION代表结束
    # 2.读到ROOT,JOINT,End Site存入名字,每次存入需要保留上一次的名字（For End Site Name）
    # 3.记录last_joint_name为End Site的名字准备,记录current_index,递增关节索引
    # 4.记录deep_parent,current_deep,分别为当前的深度父类和当前深度,保证能获取到正确的父关节索引。深度计算通过"{","}"这两种括号记录
    # 5.依据上述逻辑填充joint_name, joint_parent, joint_offset

    joint_name = []
    joint_parent = []
    joint_offset = []
    bvh_file = open(bvh_file_path, mode='r')

    last_joint_name = ""
    deep_parent = []
    current_deep = 0
    current_index = 0
    strs = []

    while(True) :
        line_str = bvh_file.readline()
        if(line_str.find("HIERARCHY") != -1):
            #print("HIERARCHY")
            pass

        elif(line_str.find("ROOT") != -1):
            strs = line_str.split()
            result_str = strs[1]
            last_joint_name = result_str
            
            #print(result_str, -1)
            joint_name.append(result_str)
            joint_parent.append(-1)
            if (len(deep_parent) > current_deep):
                deep_parent[current_deep] = current_index
            else:
                deep_parent.append(current_index)
            current_index += 1

        elif(line_str.find("JOINT") != -1):
            strs = line_str.split()
            result_str = strs[1]
            last_joint_name = result_str
            joint_name.append(result_str)
            if (len(deep_parent) > current_deep):
                deep_parent[current_deep] = current_index
            else:
                deep_parent.append(current_index)
            joint_parent.append(deep_parent[current_deep - 1])
            #print(result_str, joint_name[deep_parent[current_deep - 1]])
            current_index += 1

        elif(line_str.find("End Site") != -1):
            result_str = last_joint_name +'_end'
            joint_name.append(result_str)
            if (len(deep_parent) > current_deep):
                deep_parent[current_deep] = current_index
            else:
                deep_parent.append(current_index)
            joint_parent.append(deep_parent[current_deep - 1])
            #print(result_str, joint_name[deep_parent[current_deep - 1]])
            current_index += 1

        elif(line_str.find("OFFSET") != -1):
            strs = line_str.split()
            #print(strs)
            joint_offset.append(np.array([[float(strs[1]),  float(strs[2]), float(strs[3])]]))
        elif(line_str.find("{") != -1):
            current_deep = current_deep + 1

        elif(line_str.find("}") != -1):
            current_deep = current_deep - 1

        elif(line_str.find("MOTION") != -1):
            #print("MOTION")
            bvh_file.close()
            break

        else:
            continue

    


    return joint_name, joint_parent, joint_offset


def part2_forward_kinematics(joint_name, joint_parent, joint_offset, motion_data, frame_id):
    """请填写以下内容
    输入: part1 获得的关节名字，父节点列表，偏移量列表
        motion_data: np.ndarray，形状为(N,X)的numpy数组，其中N为帧数，X为Channel数
        frame_id: int，需要返回的帧的索引
    输出:
        joint_positions: np.ndarray，形状为(M, 3)的numpy数组，包含着所有关节的全局位置
        joint_orientations: np.ndarray，形状为(M, 4)的numpy数组，包含着所有关节的全局旋转(四元数)
    Tips:
        1. joint_orientations的四元数顺序为(x, y, z, w)
        2. from_euler时注意使用大写的XYZ
    """

    count = len(joint_name)
    joint_positions = np.zeros(shape=(count,3))
    joint_orientations = np.zeros(shape=(count,4))

    # FK算法
    # 假定Root JOINT为Global旋转Q0，Local旋转为R0，Local关节偏移为V0
    # 其子JOINT为Q1，R1，V1依次类推
    # Oritentation
    # Q0 = R0
    # Q1 = R0 * R1 = Q0 * R1
    # Q2 = R0 * R1 * R2 = Q1 * R2   乘法顺序为从右到左(右结合)
    # Offset
    # V0 = V0
    # V1 = V0 + Q1 * V1 = V0 + R0 * R1 * V1
    # V2 = V1 + Q2 * V2 = V1 + R0 * R1 * R2 * V2


    one_frame_motion_data = motion_data[frame_id]
    
    one_frame_rotation_data = []
    one_frame_orientation_data = []     # 临时存储所有关节旋转
    one_frame_offset_data = []          # 临时存储所有关节偏移
    joint_index = 0
    channel_index = 0
    joint_offset_index = 0

    # Channel三种情况 RootJoint Joint Joint_end
    while(channel_index < one_frame_motion_data.size) :
        one_joint_name = joint_name[joint_index]
        rotation = None     # Global Rotation
        oritenation = None  # Global Oritenation
        offset = None       # Global Offset
        if one_joint_name.find("RootJoint") != -1 :
            # ROOT JOINT: Channel-6
            current_offset = joint_offset[joint_offset_index]
            offset = np.array(
                [
                    [
                        float(one_frame_motion_data[channel_index + 0]),
                        float(one_frame_motion_data[channel_index + 1]),
                        float(one_frame_motion_data[channel_index + 2])
                    ]
                ]
            ) + current_offset

            rotation = R.from_euler("XYZ",
                        [
                            [   
                                float(one_frame_motion_data[channel_index + 3]),
                                float(one_frame_motion_data[channel_index + 4]),
                                float(one_frame_motion_data[channel_index + 5])
                            ]
                        ] , True)

            oritenation = rotation.as_quat()

            channel_index += 6
            joint_offset_index += 1
        elif one_joint_name.find("_end") != -1 :
            # End: Only Offset
            parent_index = joint_parent[joint_index]
            parent_rotation = one_frame_rotation_data[parent_index]
            parent_offset = one_frame_offset_data[parent_index]
            current_offset = joint_offset[joint_offset_index]


            rotation = parent_rotation
            oritenation = rotation.as_quat()


            delta_offset = parent_rotation.apply(current_offset)
            offset = parent_offset + delta_offset

            joint_offset_index += 1
        else :
            # JOINT: Channel-3
            parent_index = joint_parent[joint_index]
            parent_rotation = one_frame_rotation_data[parent_index]
            current_rotation = R.from_euler("XYZ",
                [                        
                    [
                        float(one_frame_motion_data[channel_index + 0]),
                        float(one_frame_motion_data[channel_index + 1]),
                        float(one_frame_motion_data[channel_index + 2])
                    ]
                ] , True)

            parent_offset = one_frame_offset_data[parent_index]
            current_offset = joint_offset[joint_offset_index]
            delta_offset = parent_rotation.apply(current_offset) #.apply -> .RotatorVector

            rotation = parent_rotation * current_rotation
            oritenation = rotation.as_quat()
            offset = parent_offset + delta_offset

            channel_index += 3
            joint_offset_index += 1

        one_frame_offset_data.append(offset)
        one_frame_rotation_data.append(rotation)
        one_frame_orientation_data.append(oritenation)


        joint_positions[joint_index] = offset
        joint_orientations[joint_index] = oritenation
        joint_index += 1
    


    # print(joint_positions.shape)
    # print(joint_orientations.shape)
    # print(joint_positions)
    # print(joint_orientations)

    return joint_positions, joint_orientations


def part3_retarget_func(T_pose_bvh_path, A_pose_bvh_path):
    """
    将 A-pose的bvh重定向到T-pose上
    输入: 两个bvh文件的路径
    输出: 
        motion_data: np.ndarray，形状为(N,X)的numpy数组，其中N为帧数，X为Channel数。retarget后的运动数据
    Tips:
        两个bvh的joint name顺序可能不一致哦(
        as_euler时也需要大写的XYZ
    """

    # 需要建立t_joint_names和a_joint_names的数据映射关系
    t_joint_names, t_joint_parents, t_joint_offsets = part1_calculate_T_pose(T_pose_bvh_path)
    a_joint_names, a_joint_parents, a_joint_offsets = part1_calculate_T_pose(A_pose_bvh_path)

    a_motion_data = load_motion_data(A_pose_bvh_path)

    # 将A_pose bvh文件Channels顺序映射到T_pose骨架做准备
    # a_joint_name_to_t_channel_index即A_pose关节名称映射到T_pose bvh Channels的索引
    a_joint_name_to_t_channel_index = []
    a_joint_index = 0
    for a_joint_name in a_joint_names :
        
        channel_index = 0
        for t_joint_name in t_joint_names :
            if t_joint_name == a_joint_name :
                a_joint_name_to_t_channel_index.append(channel_index)
                break

            if t_joint_name == "RootJoint" :
                channel_index += 6
            elif t_joint_name.find("_end") != -1 :
                pass
            else :
                channel_index += 3
        a_joint_index += 1

    a_to_t_lshoulder_matrix = R.from_matrix(R.from_euler("XYZ", [0, 0, -45], True).as_matrix())
    a_to_t_rshoulder_matrix = R.from_matrix(R.from_euler("XYZ", [0, 0, 45], True).as_matrix())
    new_motion_data = np.empty_like(a_motion_data)
    progress = 0
    for a_one_motion_data in a_motion_data :
        new_one_motion_data = np.copy(a_one_motion_data)
        
        print(progress / a_motion_data.shape[0])    #加载进度
        a_joint_index = 0
        a_channel_index = 0
        for a_joint_name in a_joint_names :
            t_channel_index = a_joint_name_to_t_channel_index[a_joint_index]
            if a_joint_name == "RootJoint" :
                for range_index in range(6) :
                    new_one_motion_data[t_channel_index + range_index] = float(a_one_motion_data[a_channel_index + range_index])
                    
                a_channel_index += 6
            elif a_joint_name.find("_end") != -1 :
                pass
            else :
                for range_index in range(3) :
                    new_one_motion_data[t_channel_index + range_index] = float(a_one_motion_data[a_channel_index + range_index])
                a_channel_index += 3
            a_joint_index += 1

        # 此时拿着T_pose的骨骼结构，Channel数据也映射到T_pose上，但使用A_pose的动作数据
        # 即使用T_pose骨骼播放A_pose动画，那么将T_pose动作重定向到A_pose动作上即可
        # (A-Pose是在Local-Rotation上将lshoulder用欧拉角XYZ的格式旋转[0, 0, -45]度，
        # ——将rshoulder用欧拉角XYZ的格式旋转[0, 0, 45]度)
        # 那么只需要将T_pose上的shoulder旋转对应角度来达到A_pose的表现即可
        channel_index = 0
        for joint_name in t_joint_names :
            if joint_name == "lShoulder" :
                cur_matrix = R.from_matrix(
                    R.from_euler("XYZ", [
                        float(new_one_motion_data[channel_index + 0]),
                        float(new_one_motion_data[channel_index + 1]),
                        float(new_one_motion_data[channel_index + 2]),
                    ], True).as_matrix()
                )
                
                new_matrix = a_to_t_lshoulder_matrix * cur_matrix
                new_euler = new_matrix.as_euler("XYZ", True)

                new_one_motion_data[channel_index + 0] = new_euler[0]
                new_one_motion_data[channel_index + 1] = new_euler[1]
                new_one_motion_data[channel_index + 2] = new_euler[2]
                pass
            elif joint_name == "rShoulder" :
                cur_matrix = R.from_matrix(
                    R.from_euler("XYZ", [
                        float(new_one_motion_data[channel_index + 0]),
                        float(new_one_motion_data[channel_index + 1]),
                        float(new_one_motion_data[channel_index + 2]),
                    ], True).as_matrix()
                )

                new_matrix = a_to_t_rshoulder_matrix * cur_matrix
                new_euler = new_matrix.as_euler("XYZ", True)

                new_one_motion_data[channel_index + 0] = new_euler[0]
                new_one_motion_data[channel_index + 1] = new_euler[1]
                new_one_motion_data[channel_index + 2] = new_euler[2]
                pass
            else :
                pass

            if joint_name == "RootJoint" :
                channel_index += 6
            elif joint_name.find("_end") != -1 :
                pass
            else :
                channel_index += 3

        new_motion_data[progress] = new_one_motion_data
        progress += 1


    motion_data = new_motion_data
    # native_motion_data = load_motion_data(A_pose_bvh_path)
    return motion_data
