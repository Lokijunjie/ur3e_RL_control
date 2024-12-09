#！/home/wangjunjie/.conda/envs/DMP/bin/python
# -*- coding: utf-8 -*-
# 导入基本ros和moveit库
import rospy, sys
import moveit_commander
from moveit_commander import MoveGroupCommander, PlanningSceneInterface, RobotCommander
from moveit_msgs.msg import  PlanningScene, ObjectColor,CollisionObject, AttachedCollisionObject,Constraints,OrientationConstraint
from geometry_msgs.msg import PoseStamped, Pose
from tf.transformations import quaternion_from_euler
from copy import deepcopy
import math
import csv

import os

# 添加 PyTorch 的安装路径
sys.path.append('/home/wangjunjie/.conda/envs/DMP/lib/python3.8/site-packages')
import numpy as np
print(np.__file__)

import pandas as pd

sys.path.insert(0, "/home/wangjunjie/catkin_ws/src/dmp_control/scripts")
from dmp_discrete import dmp_discrete



class MoveIt_Control:
    # 初始化程序
    def __init__(self, is_use_gripper=False):
        # Init ros config
        moveit_commander.roscpp_initialize(sys.argv)

        # 初始化ROS节点
        
        self.arm = moveit_commander.MoveGroupCommander('manipulator')
        self.arm.set_goal_joint_tolerance(0.001)
        self.arm.set_goal_position_tolerance(0.001)
        self.arm.set_goal_orientation_tolerance(0.01)

        self.end_effector_link = self.arm.get_end_effector_link()
        # 设置机械臂基座的参考系
        self.reference_frame = 'base'
        self.arm.set_pose_reference_frame(self.reference_frame)

        # 设置最大规划时间和是否允许重新规划
        self.arm.set_planning_time(5)
        self.arm.allow_replanning(True)
        self.arm.set_planner_id("RRTConnect")

        # 设置允许的最大速度和加速度（范围：0~1）
        self.arm.set_max_acceleration_scaling_factor(1)
        self.arm.set_max_velocity_scaling_factor(1)

        # 机械臂初始姿态
        self.go_home()
       
        # 发布场景
        self.set_scene()  # set table
        #self.arm.set_workspace([-2,-2,0,2,2,2])  #[minx miny minz maxx maxy maxz]
        # 初始化写字姿态
        self.initial_pose = [45*math.pi/180, -60*math.pi/180, 30*math.pi/180, -85*math.pi/180, -90*math.pi/180, 0*math.pi/180]
        self.move_j(self.initial_pose,a=0.5,v=0.5)

    def go_home(self,a=1,v=1):
        self.arm.set_max_acceleration_scaling_factor(a)
        self.arm.set_max_velocity_scaling_factor(v)
        # “up”为自定义姿态，你可以使用“home”或者其他姿态
        self.arm.set_named_target('up')
        self.arm.go()
        rospy.sleep(1)

    def setColor(self, name, r, g, b, a=0.9):
        # 初始化moveit颜色对象
        color = ObjectColor()
        # 设置颜色值
        color.id = name
        color.color.r = r
        color.color.g = g
        color.color.b = b
        color.color.a = a
        # 更新颜色字典
        self.colors[name] = color

    # 将颜色设置发送并应用到moveit场景当中
    def sendColors(self):
        # 初始化规划场景对象
        p = PlanningScene()
        # 需要设置规划场景是否有差异
        p.is_diff = True
        # 从颜色字典中取出颜色设置
        for color in self.colors.values():
            p.object_colors.append(color)
        # 发布场景物体颜色设置
        self.scene_pub.publish(p)

    # 测试程序用
    def testRobot(self):
        try:
            print("Test for robot...")
            # self.go_home()
            self.move_j([0.3, -1.5, 1.2, 0.0, -1, 0.454125],a=0.5,v=0.5)
            rospy.sleep(2)
            self.move_p([0.4, 0, 0.4, -np.pi, 0, 0])
            rospy.sleep(5)
            # self.set_constraints()
            #self.move_l([0.4, 0.1, 0.4, -np.pi, 0, 0] )
            #rospy.sleep(2)
            self.move_l([0.4, 0.1, 0.4, -np.pi, 0, 0,
                         0.3, 0.1, 0.3, -np.pi, 0, 0,],waypoints_number=2)
            rospy.sleep(2)
            # self.close_gripper()
            # self.open_gripper()
            # self.grasp([0.4,0.2,0 ],[-np.pi, 0, 0])
            # self.move_p([-0.3, 0, 0.3, 0, -np.pi / 2, 0])
            # self.go_home()
            # waypoints=[]
            # start_pose = self.arm.get_current_pose(self.end_effector_link).pose
            # pose1=deepcopy(start_pose)
            # pose1.position.z +=0.1
            # waypoints.append(deepcopy(pose1))
            # self.move_l(waypoints)
            # self.go_home()
            #self.some_useful_function_you_may_use()
        except:
            print("Test fail! ")

    # 在机械臂下方添加一个table，使得机械臂只能够在上半空间进行规划和运动
    # 避免碰撞到下方的桌子等其他物体
    def set_scene(self):
        ## set table
        self.scene = PlanningSceneInterface()
        self.scene_pub = rospy.Publisher('planning_scene', PlanningScene, queue_size=5)
        self.colors = dict()
        rospy.sleep(1)
        table_id = 'table'
        self.scene.remove_world_object(table_id)
        rospy.sleep(1)
        table_size = [2, 2, 0.01]
        table_pose = PoseStamped()
        table_pose.header.frame_id = self.reference_frame
        table_pose.pose.position.x = 0.0
        table_pose.pose.position.y = 0.0
        table_pose.pose.position.z = -table_size[2]/2 -0.02
        table_pose.pose.orientation.w = 1.0
        self.scene.add_box(table_id, table_pose, table_size)
        self.setColor(table_id, 0.5, 0.5, 0.5, 1.0)
        self.sendColors()

    # 关节规划，输入6个关节角度（单位：弧度）
    def move_j(self, joint_configuration=None,a=1,v=1):
        # 设置机械臂的目标位置，使用六轴的位置数据进行描述（单位：弧度）
        if joint_configuration==None:
            joint_configuration = [0, -1.5707, 0, -1.5707, 0, 0]
        self.arm.set_max_acceleration_scaling_factor(a)
        self.arm.set_max_velocity_scaling_factor(v)
        self.arm.set_joint_value_target(joint_configuration)
        rospy.loginfo("move_j:"+str(joint_configuration))
        self.arm.go()
        rospy.sleep(0.5)

    # 空间规划，输入xyzRPY
    def move_p(self, tool_configuration=None, a=1, v=1):
        if tool_configuration is None:
            tool_configuration = [0.3, 0, 0.3, 0, 0, 0, 1]  # x, y, z, quaternion

        self.arm.set_max_acceleration_scaling_factor(a)
        self.arm.set_max_velocity_scaling_factor(v)

        # Define the target pose
        target_pose = PoseStamped()
        target_pose.header.frame_id = self.reference_frame
        target_pose.header.stamp = rospy.Time.now()
        target_pose.pose.position.x = tool_configuration[0]
        target_pose.pose.position.y = tool_configuration[1]
        target_pose.pose.position.z = tool_configuration[2]
        target_pose.pose.orientation.x = tool_configuration[3]
        target_pose.pose.orientation.y = tool_configuration[4]
        target_pose.pose.orientation.z = tool_configuration[5]
        target_pose.pose.orientation.w = tool_configuration[6]

        # Set the target pose for the end-effector
        self.arm.set_start_state_to_current_state()
        self.arm.set_pose_target(target_pose, self.end_effector_link)

        rospy.loginfo("move_p: " + str(tool_configuration))

        # Plan the trajectory
        success, traj, _, _ = self.arm.plan()  # Extract the trajectory from the plan result

        if success:
            self.arm.execute(traj)  # Execute the planned trajectory
            rospy.sleep(0.5)
        else:
            rospy.logerr("Failed to plan trajectory to the given pose.")


def test():
    ## 1.DMP learning
    read_path = '/home/wangjunjie/catkin_ws/src/dmp_control/demo_trajectory/data/raw/ur3e_tip_positions.csv'
    df = pd.read_csv(read_path, header=None,skiprows=1)
    reference_trajectory = np.array(df).T              
    data_dim = reference_trajectory.shape[0]
    data_len = reference_trajectory.shape[1]
    print("reference_trajectory shape: ", reference_trajectory.shape)
    dmp = dmp_discrete(n_dmps=data_dim, n_bfs=2000, dt=1.0/data_len)
    dmp.learning(reference_trajectory)

    reproduced_trajectory, _, _ = dmp.reproduce()

    goal_pose = reproduced_trajectory[-1,:]
    print("goal_pose: ", goal_pose)
    print("reproduced_trajectory.shape: ", reproduced_trajectory.shape)

    # 初始化ros节点
    rospy.init_node('dmp_control')

    ## 2.将dmp生成的轨迹通过moveit控制机械臂运动
    moveit_server = MoveIt_Control(is_use_gripper=False)

    print("Main loop is begining ...")
    max_loop = 1
    reproduced_trajectory_record_x = np.zeros((data_len, max_loop))
    reproduced_trajectory_record_y = np.zeros((data_len, max_loop))
    reproduced_trajectory_record_z = np.zeros((data_len, max_loop))
    # 2.1获取机械臂tip姿态信息
    eef_pose = moveit_server.arm.get_current_pose().pose
    initial_pose = [eef_pose.position.x, eef_pose.position.y, eef_pose.position.z]
    initial_quat = [eef_pose.orientation.x, eef_pose.orientation.y, eef_pose.orientation.z, eef_pose.orientation.w]
    # DMP produce with initial pose
    reproduced_trajectory, _, _ = dmp.reproduce(initial=reference_trajectory[:,0],goal=goal_pose)

    # record the reproduced trajectory into csv file
    write_path = '/home/wangjunjie/catkin_ws/src/dmp_control/demo_trajectory/data/reproduced_positions.csv'
    with open(write_path, 'w', newline='') as file:
        writer = csv.writer(file)
        for i in range(data_len):
            writer.writerow(reproduced_trajectory[i, :])
        print("Reproduced trajectory has been saved to: ", write_path)



    for i in range(data_len) :
        position = reproduced_trajectory[i,:]
        combined = np.concatenate((position, initial_quat))
        info=combined.tolist()
        moveit_server.move_p(info,a=1,v=1)

    


if __name__ == '__main__':
    test()
