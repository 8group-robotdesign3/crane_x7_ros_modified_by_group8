#! /usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import moveit_commander
import geometry_msgs.msg
from std_msgs.msg import Int32
import rosnode
from tf.transformations import quaternion_from_euler

robot = moveit_commander.RobotCommander()
arm = moveit_commander.MoveGroupCommander("arm")

finish = True

def call(data):
    #rospy.loginfo('call callback function')
    global finish
    if data.data == 2 and finish:
        global robot
        global arm
        arm.set_max_velocity_scaling_factor(0.1)
        gripper = moveit_commander.MoveGroupCommander("gripper")

        while len([s for s in rosnode.get_node_names() if 'rviz' in s]) == 0:
            rospy.sleep(1.0)
        rospy.sleep(1.0)

        print("Group names:")
        print(robot.get_group_names())

        print("Current state:")
        print(robot.get_current_state())

        target_pose = geometry_msgs.msg.Pose()
        target_pose.position.x = 0.3
        target_pose.position.y = 0.1
        target_pose.position.z = 0.31
        q = quaternion_from_euler(-3.14, 0.0, -3.14/2.0)  # 上方から掴みに行く場合
        target_pose.orientation.x = q[0]
        target_pose.orientation.y = q[1]
        target_pose.orientation.z = q[2]
        target_pose.orientation.w = q[3]
        arm.set_pose_target(target_pose)  # 目標ポーズ設定
        arm.go()  # 実行

        target_pose = geometry_msgs.msg.Pose()
        target_pose.position.x = 0.35
        target_pose.position.y = 0.0
        target_pose.position.z = 0.31
        q = quaternion_from_euler(-3.14, 0.0, -3.14/2.0)  # 上方から掴みに行く場合
        target_pose.orientation.x = q[0]
        target_pose.orientation.y = q[1]
        target_pose.orientation.z = q[2]
        target_pose.orientation.w = q[3]
        arm.set_pose_target(target_pose)  # 目標ポーズ設定
        arm.go()  # 実行

        target_pose = geometry_msgs.msg.Pose()
        target_pose.position.x = 0.3
        target_pose.position.y = -0.1
        target_pose.position.z = 0.31
        q = quaternion_from_euler(-3.14, 0.0, -3.14/2.0)  # 上方から掴みに行く場合
        target_pose.orientation.x = q[0]
        target_pose.orientation.y = q[1]
        target_pose.orientation.z = q[2]
        target_pose.orientation.w = q[3]
        arm.set_pose_target(target_pose)  # 目標ポーズ設定
        arm.go()  # 実行

        target_pose = geometry_msgs.msg.Pose()
        target_pose.position.x = 0.35
        target_pose.position.y = 0.0
        target_pose.position.z = 0.31
        q = quaternion_from_euler(-3.14, 0.0, -3.14/2.0)  # 上方から掴みに行く場合
        target_pose.orientation.x = q[0]
        target_pose.orientation.y = q[1]
        target_pose.orientation.z = q[2]
        target_pose.orientation.w = q[3]
        arm.set_pose_target(target_pose)  # 目標ポーズ設定
        arm.go()  # 実行

        pub = rospy.Publisher("main_node/report_progress",Int32,queue_size = 1)
        rospy.loginfo("finish")
        finish = False
        progress = data.data + 1
        for i in range(100):
            pub.publish(progress)
            rospy.loginfo("publish progress2")
        #rospy.signal_shutdown("shutdown node1")
        rospy.loginfo("finish nagemae")
    #else:
    #    if finish:
    #        rospy.loginfo("not receive start sign")

def main_func():
    sub = rospy.Subscriber("main_node/activate_node",Int32,call)
    rospy.spin()


if __name__ == '__main__':
    rospy.init_node("node2_nagemae",anonymous=True)
    rospy.loginfo("success init node2")
    try:
        if not rospy.is_shutdown():
            main_func()
    except rospy.ROSInterruptException:
        pass
