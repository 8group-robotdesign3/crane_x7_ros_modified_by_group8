#! /usr/bin/env python
# -*- coding: utf-8 -*-

import rospy, sys, time, actionlib
import moveit_commander
import geometry_msgs.msg
import rosnode
import math
from std_msgs.msg import Float64
from control_msgs.msg import GripperCommandAction, GripperCommandGoal, FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from tf.transformations import quaternion_from_euler
from trajectory_msgs.msg import JointTrajectoryPoint

from std_msgs.msg import Int32

global Once_flag

class ArmJointTrajectoryExample(object):
    def __init__(self):
        self._client = actionlib.SimpleActionClient(
            "/crane_x7/arm_controller/follow_joint_trajectory", FollowJointTrajectoryAction)
        rospy.sleep(0.1)
        if not self._client.wait_for_server(rospy.Duration(secs=5)):
            rospy.logerr("Action Server Not Found")
            rospy.signal_shutdown("Action Server Not Found")
            sys.exit(1)

    def move_arm(self, joint_values, secs):
        
        goal = FollowJointTrajectoryGoal()
        goal.trajectory.joint_names = ["crane_x7_shoulder_fixed_part_pan_joint", "crane_x7_shoulder_revolute_part_tilt_joint",
                                       "crane_x7_upper_arm_revolute_part_twist_joint", "crane_x7_upper_arm_revolute_part_rotate_joint",
                                       "crane_x7_lower_arm_fixed_part_joint", "crane_x7_lower_arm_revolute_part_joint", "crane_x7_wrist_joint"]
        point = JointTrajectoryPoint()
        for p in joint_values:            
            point.positions.append(math.radians(p))           
        point.time_from_start = rospy.Duration(secs)
        goal.trajectory.points.append(point)
        self._client.send_goal(goal)
        
    def wait(self, timeout=0.1):   
        self._client.wait_for_result(timeout=rospy.Duration(timeout))
        return self._client.get_result()
        

class GripperClient(object):
    def __init__(self):
        self._client = actionlib.SimpleActionClient("/crane_x7/gripper_controller/gripper_cmd",GripperCommandAction)
        self.clear()

        if not self._client.wait_for_server(rospy.Duration(10.0)):
            rospy.logerr("Exiting - Gripper Action Server Not Found.")
            rospy.signal_shutdown("Action Server not found.")
            sys.exit(1)

    def command(self, position, effort):
        self._goal.command.position = position
        self._goal.command.max_effort = effort
        self._client.send_goal(self._goal,feedback_cb=self.feedback)
        
    def feedback(self,msg):
        print("feedback callback")
        print(msg)

    def stop(self):
        self._client.cancel_goal()

    def wait(self, timeout=0.1):
        self._client.wait_for_result(timeout=rospy.Duration(timeout))
        return self._client.get_result()

    def clear(self):
        self._goal = GripperCommandGoal()

   
       
def main():
    jt = ArmJointTrajectoryExample()
    gc = GripperClient()
       
    
    joint_values = [0.0, -50.0, 0.0, -60.0, 0.0, -30.0, 0.0]
    secs=3.0
    jt.move_arm(joint_values, secs)
    jt.wait(2.0)
        
    time.sleep(2)
        
    joint_values = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    secs=0.7
    jt.move_arm(joint_values, secs)
    jt.wait(2.0)
        
    joint_values = [0.0, 45.0, 0.0, 0.0, 0.0, 0.0, 0.0]       
    secs=0.4
    jt.move_arm(joint_values, secs)
                
    gripper = 80.0
    effort = 1.0
    gc.command(math.radians(gripper), effort)
        
    gc.wait(2.0)
    jt.wait(2.0)
        
    print("done")

def sub(data):
    global Once_flag
    if data.data == 0 and Once_flag:
        Once_flag = False
        main()

if __name__ == '__main__':
    Once_flag = True
    rospy.init_node("throw_pt1")
    rospy.Subscriber("activate_node",Int32,sub,queue_size = 1);
    rospy.spin()
