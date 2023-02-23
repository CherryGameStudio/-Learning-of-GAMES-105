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

    print("--------------------part1_calculate_T_pose start--------------------")
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
            print("HIERARCHY")

        elif(line_str.find("ROOT") != -1):
            strs = line_str.split()
            result_str = strs[1]
            last_joint_name = result_str
            
            print(result_str, -1)
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
            print(result_str, joint_name[deep_parent[current_deep - 1]])
            current_index += 1

        elif(line_str.find("End Site") != -1):
            result_str = last_joint_name +'_end'
            joint_name.append(result_str)
            if (len(deep_parent) > current_deep):
                deep_parent[current_deep] = current_index
            else:
                deep_parent.append(current_index)
            joint_parent.append(deep_parent[current_deep - 1])
            print(result_str, joint_name[deep_parent[current_deep - 1]])
            current_index += 1

        elif(line_str.find("OFFSET") != -1):
            strs = line_str.split()
            print(strs)
            # euler_angle = R.from_euler('XYZ', [float(strs[1]),  float(strs[2]), float(strs[3])], degrees=True)
            # print(euler_angle)
            joint_offset.append(np.array([[float(strs[1]),  float(strs[2]), float(strs[3])]]))
        elif(line_str.find("{") != -1):
            current_deep = current_deep + 1

        elif(line_str.find("}") != -1):
            current_deep = current_deep - 1

        elif(line_str.find("MOTION") != -1):
            print("MOTION")
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
    joint_positions = None
    joint_orientations = None
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
    motion_data = None
    return motion_data
