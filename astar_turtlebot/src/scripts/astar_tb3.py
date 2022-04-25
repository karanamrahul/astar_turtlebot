#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Twist
import csv
import tf
import roslib
import numpy as np
from astardd import *
from Node import *
from helper import *
import sys


wheel_radius = 0.038
wheel_distance = 0.354


def turtlebot_pub():

    rospy.sleep(3.)
    path_pub = rospy.Publisher('path', Path, queue_size=10)
    vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    rospy.init_node('turtlebot_pub', anonymous=True)

    listener = tf.TransformListener()

    start_point = [int(sys.argv[1]), int(sys.argv[2]), int(sys.argv[3])]
    goal_state = [int(sys.argv[4]), int(sys.argv[5])]
    clearance = int(sys.argv[6])
    rpm1, rpm2 = int(sys.argv[7]), int(sys.argv[8])

    AstarPath = astar(start_point, goal_state, rpm1, rpm2, clearance)
    moves, node_points, path_points = AstarPath.astarPath()
    rospy.sleep(3.)


    # moves = read_file(move_file_name)
    # node_points = read_file(node_file_name)
    # path_points = read_file(path_file_name)

    path = update_path_msg(path_points)

    rate = rospy.Rate(1)

    while not rospy.is_shutdown():

        try:
            (trans,rot) = listener.lookupTransform('/odom', '/base_footprint', rospy.Time(0))
            current_pose = getCurrectPosition(trans, rot)
            dx, dy, min_dist = getNearestPoint(current_pose, path_points)
  

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            print("No tf available between map and baselink......................")
            rospy.sleep(1.)
            continue

        msg_str = "Publishing vel and path %s" % rospy.get_time()


        if len(moves) == 0:
            move = [0,0]
        else:
            move = moves.pop(0)



        vel = get_twist_msg(move)

        if (abs(min_dist) > 0.1 and len(moves) > 0):
           vel =  updateOmega(vel, min_dist)

        path_pub.publish(path)
        vel_pub.publish(vel)

        rate.sleep()

if __name__ == '__main__':
    try:
        turtlebot_pub()
    except rospy.ROSInterruptException:
        pass