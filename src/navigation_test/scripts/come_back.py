#!/usr/bin/python3
# coding=UTF-8

import rospy
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
import os

def main():
    rospy.init_node("back",anonymous=True)
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server()
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = 0
    goal.target_pose.pose.position.y = 0
    goal.target_pose.pose.position.z = 0
    goal.target_pose.pose.orientation.x = 0
    goal.target_pose.pose.orientation.y = 0
    goal.target_pose.pose.orientation.z = 0
    goal.target_pose.pose.orientation.w = 1
    client.send_goal(goal)
    rospy.loginfo("Back command sent")
    client.wait_for_result()
    rospy.loginfo("done")

if __name__ == "__main__":
    main()
