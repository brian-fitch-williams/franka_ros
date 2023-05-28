#!/usr/bin/env python3
import sys

import numpy as np
import roboticstoolbox as rtb
import rospy
import spatialmath

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


def perform_trajectory():
    """
    """
    
    rospy.init_node('panda_trajectory_publisher')
    contoller_topic='/panda_course_jtc/command'
    trajectory_publihser = rospy.Publisher(contoller_topic, JointTrajectory, queue_size=10)

    ## Printing its DH table
    panda = rtb.models.Panda()
    print(panda)

    point = spatialmath.SE3(0.484, 0, 0.416)
    point_sol = panda.ikine_LM(point) 
    print(point_sol)

    goal_positions = np.append(point_sol.q, [0.3, 0.3])
    print(goal_positions)
    
    argv = sys.argv[1:]                         
    panda_joints = [
        'panda_joint1',
        'panda_joint2',
        'panda_joint3',
        'panda_joint4',
        'panda_joint5',
        'panda_joint6',
        'panda_joint7',
        'panda_finger_joint1',
        'panda_finger_joint2',
        ]
 
    rospy.loginfo("Goal Position set lets go ! ")
    rospy.sleep(1)

    trajectory_msg = JointTrajectory()
    trajectory_msg.joint_names = panda_joints
    trajectory_msg.points.append(JointTrajectoryPoint())
    trajectory_msg.points[0].positions = goal_positions
    trajectory_msg.points[0].velocities = [0.0 for i in panda_joints]
    trajectory_msg.points[0].accelerations = [0.0 for i in panda_joints]
    trajectory_msg.points[0].time_from_start = rospy.Duration(3)
    rospy.sleep(1)
    trajectory_publihser.publish(trajectory_msg)


if __name__ == '__main__':
    perform_trajectory()