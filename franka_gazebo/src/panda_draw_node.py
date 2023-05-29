#!/usr/bin/env python3
import rospy
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal, FollowJointTrajectoryActionResult

from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import roboticstoolbox as rtb
from spatialmath import SE3
import numpy as np

class RobotDraw:
    def __init__(self):
        """ Ros setup, robot model
        """
        self._ikine_seed = 100
        self._actionlib_timeout_s = 5
        self._num_moves = 0
        self._current_move = 0
        self._joints_trajectory_points=[]
        self._panda_joints = ['panda_joint1','panda_joint2','panda_joint3','panda_joint4',
                        'panda_joint5','panda_joint6','panda_joint7',
                        'panda_finger_joint1','panda_finger_joint2']

        self._panda_rtb = rtb.models.URDF.Panda()

        rospy.init_node('robot_draw')
        self._status_sub = rospy.Subscriber("/joint_states", JointState, self.joint_state_cb)
        self._result_sub = rospy.Subscriber(
            "/panda_course_jtc/follow_joint_trajectory/result",
            FollowJointTrajectoryActionResult,
            self.result_cb)

        self._panda_client = actionlib.SimpleActionClient('panda_course_jtc/follow_joint_trajectory',
                                                    FollowJointTrajectoryAction)
        if not self._panda_client.wait_for_server(timeout=rospy.Duration(self._actionlib_timeout_s)):
            rospy.logerror("Error, could not init actionlib.  Exit")
            exit()
        
        rospy.loginfo('Connected to Actionlib')

        # keeps python from exiting until this node is stopped
        rospy.spin()

    def joint_state_cb(self, msg):
        """
        """
        #rospy.loginfo("Got joint state: %s", str(msg))
        pass

    def result_cb(self, msg):
        """
        """
        print("cd, send next")
        rospy.loginfo("Result error code: %s: %s", msg.result.error_code, msg.result.error_string)
        #self._send_next()
        pass

    def draw_square(self):
        """
        """
        waypoints_sqaure=[
                        [0.5, 0, 0.6],
                        [0.5, 0, 0.9],
                        [0.5, 0.5, 0.9],
                        [0.5, 0.5, 0.6],
                        [0.5, 0, 0.6]
                        ]
        self._num_moves = len(waypoints_sqaure)
        print("Num moves: %d" % self._num_moves)
        for i in range(self._num_moves):
            point = SE3(waypoints_sqaure[i])
            tpoint = self._panda_rtb.ikine_LM(point, seed=self._ikine_seed)
            rospy.logdebug("waypoint %s to traject point %s", point, tpoint)

            # Add gripper finger points
            self._joints_trajectory_points.append(np.append(tpoint.q ,[0.01, 0.01]))

        self._send_next()

    def _send_next(self):
        """ Send next trajectory command
        """
        #if self._current_move < self._num_moves:
        while self._current_move < self._num_moves:
            rospy.loginfo("Make move %d out of %d", self._current_move, self._num_moves)
            trajectory_message = JointTrajectory()
            trajectory_message.joint_names = self._panda_joints
            trajectory_message.points.append(JointTrajectoryPoint)
            trajectory_message.points[0].positions = self._joints_trajectory_points[self._current_move]
            trajectory_message.points[0].velocities = [0.0 for i in self._panda_joints]
            trajectory_message.points[0].accelerations = [0.0 for i in self._panda_joints]
            trajectory_message.points[0].time_from_start = rospy.Duration(3)
            goal_positions = FollowJointTrajectoryGoal()
            goal_positions.trajectory = trajectory_message
            goal_positions.goal_time_tolerance = rospy.Duration(0)
            print("send goal %s" %(str(goal_positions.trajectory.points[0].positions)))
            self._panda_client.send_goal(goal_positions)
            print("after send goal")
            rospy.sleep(5)
   

if __name__ == '__main__':
    pandabot = RobotDraw()
    pandabot.draw_square()