#!/usr/bin/env python3
import rospy
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal, FollowJointTrajectoryActionResult
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import roboticstoolbox as rtb
from spatialmath import SE3
import numpy as np

class RobotDraw:

    def __init__(self):
        """Class to move robot arm lineraly to draw shapes
        """
        self._seed = 100
        self._move_complete = False
        self._panda_joints = ['panda_joint1','panda_joint2','panda_joint3',
                              'panda_joint4','panda_joint5','panda_joint6',
                              'panda_joint7','panda_finger_joint1','panda_finger_joint2']

        rospy.init_node('dual_dual_arm_trajectorymsg_actionLib')
        rospy.Subscriber("/panda_course_jtc/follow_joint_trajectory/result",
                         FollowJointTrajectoryActionResult, self.trajectory_result_cb)

        self._panda_rtb = rtb.models.URDF.Panda()
        self._panda_client = actionlib.SimpleActionClient(
            'panda_course_jtc/follow_joint_trajectory',
            FollowJointTrajectoryAction)
        if not self._panda_client.wait_for_server(timeout=rospy.Duration(5)):
            rospy.logerr("Could not connect to actionlib, exit")
            exit()

    def trajectory_result_cb(self, msg):
        """Receive this when each move completes from actionlib
        """
        rospy.loginfo("Result cb code %s: %s", msg.result.error_code, msg.result.error_string)
        self._move_complete = True

    def draw_square(self):
        """Move arm linerally as if drawing a square
        """          
        waypoints_sqaure=[
                        [0.5, 0, 0.6],
                        [0.5, 0, 0.9],
                        [0.5, 0.5, 0.9],
                        [0.5, 0.5, 0.6],
                        [0.5, 0, 0.6]
                        ]

        pt_count = len(waypoints_sqaure)
        joints_trajectory_points=[]

        for i in range(pt_count):
            point = SE3(waypoints_sqaure[i])
            tpoint = self._panda_rtb.ikine_LM(point, seed=self._seed)
            rospy.logdebug("waypoint %s to traject point %s", point, tpoint)
            joints_trajectory_points.append(np.append(tpoint.q ,[0.01, 0.01]))

        for i in range(pt_count):
            trajectory_message = JointTrajectory()
            trajectory_message.joint_names = self._panda_joints
            trajectory_message.points.append(JointTrajectoryPoint())
            trajectory_message.points[0].positions = joints_trajectory_points[i]
            trajectory_message.points[0].velocities = [0.0 for i in self._panda_joints]
            trajectory_message.points[0].accelerations = [0.0 for i in self._panda_joints]
            trajectory_message.points[0].time_from_start = rospy.Duration(3)
            goal_positions = FollowJointTrajectoryGoal()
            goal_positions.trajectory = trajectory_message
            goal_positions.goal_time_tolerance = rospy.Duration(0)
            self._move_complete = False
            self._panda_client.send_goal(goal_positions)

            while not self._move_complete:
                rospy.sleep(0.1)

if __name__ == '__main__':
    robot = RobotDraw()
    robot.draw_square()