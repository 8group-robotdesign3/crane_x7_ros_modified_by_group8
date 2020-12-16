#! /usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import time
import actionlib
import moveit_commander
import geometry_msgs.msg
from std_msgs.msg import Float64
import rosnode
from tf.transformations import quaternion_from_euler
from std_msgs.msg import Int32

# Arm communication
from control_msgs.msg import (
    FollowJointTrajectoryAction,
    FollowJointTrajectoryGoal
)
# Gripper communication
from control_msgs.msg import (      
    GripperCommandAction,
    GripperCommandGoal
 )

from trajectory_msgs.msg import JointTrajectoryPoint
import math
import sys
import numpy
import random
import copy

finish = True
global Once_flag_uemae

class ArmJointTrajectoryExample(object):
    def __init__(self):

        self._client = actionlib.SimpleActionClient(
            "/crane_x7/arm_controller/follow_joint_trajectory", FollowJointTrajectoryAction)
        rospy.sleep(0.1)
        if not self._client.wait_for_server(rospy.Duration(secs=5)):
            rospy.logerr("Action Server Not Found")
            rospy.signal_shutdown("Action Server Not Found")
            sys.exit(1)


        self.gripper_client = actionlib.SimpleActionClient("/crane_x7/gripper_controller/gripper_cmd",GripperCommandAction)
        self.gripper_goal = GripperCommandGoal()
        self.gripper_client.wait_for_server(rospy.Duration(5.0))
        if not self.gripper_client.wait_for_server(rospy.Duration(5.0)):
            rospy.logerr("Exiting - Gripper Action Server Not Found.")
            rospy.signal_shutdown("Action Server not found.")
            sys.exit(1)

    def go(self):
        #とりあえず直立
        point = JointTrajectoryPoint()
        goal = FollowJointTrajectoryGoal()
        goal.trajectory.joint_names = ["crane_x7_shoulder_fixed_part_pan_joint","crane_x7_shoulder_revolute_part_tilt_joint","crane_x7_upper_arm_revolute_part_twist_joint","crane_x7_upper_arm_revolute_part_rotate_joint","crane_x7_lower_arm_fixed_part_joint","crane_x7_lower_arm_revolute_part_joint","crane_x7_wrist_joint"]
        
        joint_values = [0.0, 0.0, 0.0, 0.0, 0, 0.0, 0.0] #角度指定部
                
        effort  = 1.0
        self.gripper_goal.command.position = math.radians(15.0)
        self.gripper_goal.command.max_effort = effort

        for i, p in enumerate(joint_values):
            point.positions.append(p)
        
        point.time_from_start = rospy.Duration(secs=2.0)
        goal.trajectory.points.append(point)
        self._client.send_goal(goal)
        self.gripper_client.send_goal(self.gripper_goal,feedback_cb=self.feedback)
        self._client.wait_for_result(timeout=rospy.Duration(100.0))

        rospy.sleep(1)

#ここから投げの動作
        #上投げ準備
        point = JointTrajectoryPoint()
        goal = FollowJointTrajectoryGoal()
        goal.trajectory.joint_names = ["crane_x7_shoulder_fixed_part_pan_joint","crane_x7_shoulder_revolute_part_tilt_joint","crane_x7_upper_arm_revolute_part_twist_joint","crane_x7_upper_arm_revolute_part_rotate_joint","crane_x7_lower_arm_fixed_part_joint","crane_x7_lower_arm_revolute_part_joint","crane_x7_wrist_joint"]
        
        joint_values = [0.0, math.radians(45), 0.0, math.radians(-35), 0.0, math.radians(-15), 0.0] #角度指定部
                
        for i, p in enumerate(joint_values):
            point.positions.append(p)
        
        point.time_from_start = rospy.Duration(secs=2.0)
        goal.trajectory.points.append(point)
        self._client.send_goal(goal)
        self._client.wait_for_result(timeout=rospy.Duration(100.0))
        #rospy.sleep(1)

        #投げる(頂点)
        point = JointTrajectoryPoint()
        goal = FollowJointTrajectoryGoal()
        goal.trajectory.joint_names = ["crane_x7_shoulder_fixed_part_pan_joint","crane_x7_shoulder_revolute_part_tilt_joint","crane_x7_upper_arm_revolute_part_twist_joint","crane_x7_upper_arm_revolute_part_rotate_joint","crane_x7_lower_arm_fixed_part_joint","crane_x7_lower_arm_revolute_part_joint","crane_x7_wrist_joint"]
        
        joint_values = [0.0, 0.0, 0.0, math.radians(-20), 0.0, math.radians(-15), 0.0] #角度指定部
                
        for i, p in enumerate(joint_values):
            point.positions.append(p)
        
        point.time_from_start = rospy.Duration(secs=2.0)
        goal.trajectory.points.append(point)
        self._client.send_goal(goal)
        self._client.wait_for_result(timeout=rospy.Duration(100.0))

        #上投げ準備
        point = JointTrajectoryPoint()
        goal = FollowJointTrajectoryGoal()
        goal.trajectory.joint_names = ["crane_x7_shoulder_fixed_part_pan_joint","crane_x7_shoulder_revolute_part_tilt_joint","crane_x7_upper_arm_revolute_part_twist_joint","crane_x7_upper_arm_revolute_part_rotate_joint","crane_x7_lower_arm_fixed_part_joint","crane_x7_lower_arm_revolute_part_joint","crane_x7_wrist_joint"]
        
        joint_values = [0.0, math.radians(-30), 0.0, math.radians(-35), 0.0, math.radians(-15), 0.0] #角度指定部
        position = math.radians(50.0)
        effort  = 1.0
        self.gripper_goal.command.position = position
        self.gripper_goal.command.max_effort = effort

        for i, p in enumerate(joint_values):
            point.positions.append(p)
        
        # ハンドを開く
        point.time_from_start = rospy.Duration(secs=0.2)
        goal.trajectory.points.append(point)
        self._client.send_goal(goal)
        rospy.sleep(0.05)
        self.gripper_client.send_goal(self.gripper_goal,feedback_cb=self.feedback)
        self._client.wait_for_result(timeout=rospy.Duration(100.0))
        rospy.sleep(1)

        #とりあえず直立
        point = JointTrajectoryPoint()
        goal = FollowJointTrajectoryGoal()
        goal.trajectory.joint_names = ["crane_x7_shoulder_fixed_part_pan_joint","crane_x7_shoulder_revolute_part_tilt_joint","crane_x7_upper_arm_revolute_part_twist_joint","crane_x7_upper_arm_revolute_part_rotate_joint","crane_x7_lower_arm_fixed_part_joint","crane_x7_lower_arm_revolute_part_joint","crane_x7_wrist_joint"]
        
        joint_values = [0.0, 0.0, 0.0, 0.0, 0, 0.0, 0.0] #角度指定部
                
        for i, p in enumerate(joint_values):
            point.positions.append(p)
        
        point.time_from_start = rospy.Duration(secs=2.0)
        goal.trajectory.points.append(point)
        self._client.send_goal(goal)
        self._client.wait_for_result(timeout=rospy.Duration(100.0))

        rospy.sleep(1)

        return self._client.get_result()

    def feedback(self,msg):
        print("feedback callback")

def sub(data):
    global Once_flag_uemae
    if data.data == 3 and Once_flag_uemae:
        Once_flag_uemae = False
        arm_joint_trajectory_example = ArmJointTrajectoryExample()
        arm_joint_trajectory_example.go()

if __name__ == "__main__":
    Once_flag_uemae = True 
    rospy.init_node("uemae_throw")
    rospy.Subscriber("activate_node",Int32,sub,queue_size = 1);
    rospy.spin()

