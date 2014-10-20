# Copyright (c) 2013-2014, Rethink Robotics
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice,
#    this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
# 3. Neither the name of the Rethink Robotics nor the names of its
#    contributors may be used to endorse or promote products derived from
#    this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

"""
Baxter RSDK Joint Trajectory Action Server
"""
import bisect
import threading
from copy import deepcopy
import math
import operator
import numpy as np
import matplotlib as mpl
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import bezier

import rospy

import actionlib

from control_msgs.msg import (
    FollowJointTrajectoryAction,
    FollowJointTrajectoryFeedback,
    FollowJointTrajectoryResult,
)
from std_msgs.msg import (
    UInt16,
)
from trajectory_msgs.msg import (
    JointTrajectoryPoint,
)

import baxter_control
import baxter_dataflow
import baxter_interface


class JointTrajectoryActionServer(object):
    def __init__(self, limb, reconfig_server, rate=100.0,
                 mode='position_w_id'):
        self._mutex = threading.Lock()
        self._dyn = reconfig_server
        self._ns = 'robot/limb/' + limb
        self._fjt_ns = self._ns + '/follow_joint_trajectory'
        self._server = actionlib.SimpleActionServer(
            self._fjt_ns,
            FollowJointTrajectoryAction,
            execute_cb=self._on_trajectory_action,
            auto_start=False)
        self._action_name = rospy.get_name()
        self._server.start()
        self._limb = baxter_interface.Limb(limb)
        self._name = limb
        self._cuff = baxter_interface.DigitalIO('%s_lower_cuff' % (limb,))
        self._cuff.state_changed.connect(self._cuff_cb)
        # Verify joint control mode
        self._mode = mode
        if (self._mode != 'position' and self._mode != 'position_w_id'
            and self._mode != 'velocity'):
            rospy.logerr("%s: Action Server Creation Failed - "
                         "Provided Invalid Joint Control Mode '%s' (Options: "
                         "'position_w_id', 'position', 'velocity')" %
                    (self._action_name, self._mode,))
            return
        self._server.start()
        self._alive = True
        self._cuff_state = False
        # Action Feedback/Result
        self._fdbk = FollowJointTrajectoryFeedback()
        self._result = FollowJointTrajectoryResult()

        # Controller parameters from arguments, messages, and dynamic
        # reconfigure
        self._control_rate = rate  # Hz
        self._control_joints = []
        self._pid_gains = {'kp': dict(), 'ki': dict(), 'kd': dict()}
        self._goal_time = 0.0
        self._stopped_velocity = 0.0
        self._goal_error = dict()
        self._path_thresh = dict()

        # Create our PID controllers
        self._pid = dict()
        for joint in self._limb.joint_names():
            self._pid[joint] = baxter_control.PID()

        # Create our spline coefficients
        self._coeff = [None] * len(self._limb.joint_names())

        # Set joint state publishing to specified control rate
        self._pub_rate = rospy.Publisher(
            '/robot/joint_state_publish_rate', UInt16)
        self._pub_rate.publish(self._control_rate)

        self._pub_ff_cmd = rospy.Publisher(
            self._ns + '/inverse_dynamics_command',
            JointTrajectoryPoint,
            tcp_nodelay=True)

    def clean_shutdown(self):
        self._alive = False
        self._limb.exit_control_mode()

    def _cuff_cb(self, value):
        self._cuff_state = value

    def _get_trajectory_parameters(self, joint_names, goal):
        # For each input trajectory, if path, goal, or goal_time tolerances
        # provided, we will use these as opposed to reading from the
        # parameter server/dynamic reconfigure

        # Goal time tolerance - time buffer allowing goal constraints to be met
        if goal.goal_time_tolerance:
            self._goal_time = goal.goal_time_tolerance.to_sec()
        else:
            self._goal_time = self._dyn.config['goal_time']

        # Stopped velocity tolerance - max velocity at end of execution
        self._stopped_velocity = self._dyn.config['stopped_velocity_tolerance']

        # Path execution and goal tolerances per joint
        for jnt in joint_names:
            if jnt not in self._limb.joint_names():
                rospy.logerr(
                    "%s: Trajectory Aborted - Provided Invalid Joint Name %s" %
                    (self._action_name, jnt,))
                self._result.error_code = self._result.INVALID_JOINTS
                self._server.set_aborted(self._result)
                return
            # Path execution tolerance
            path_error = self._dyn.config[jnt + '_trajectory']
            if goal.path_tolerance:
                for tolerance in goal.path_tolerance:
                    if jnt == tolerance.name:
                        if tolerance.position != 0.0:
                            self._path_thresh[jnt] = tolerance.position
                        else:
                            self._path_thresh[jnt] = path_error
            else:
                self._path_thresh[jnt] = path_error
            # Goal error tolerance
            goal_error = self._dyn.config[jnt + '_goal']
            if goal.goal_tolerance:
                for tolerance in goal.goal_tolerance:
                    if jnt == tolerance.name:
                        if tolerance.position != 0.0:
                            self._goal_error[jnt] = tolerance.position
                        else:
                            self._goal_error[jnt] = goal_error
            else:
                self._goal_error[jnt] = goal_error

            # PID gains if executing using the velocity (integral) controller
            if self._mode == 'velocity':
                self._pid[jnt].set_kp(self._dyn.config[jnt + '_kp'])
                self._pid[jnt].set_ki(self._dyn.config[jnt + '_ki'])
                self._pid[jnt].set_kd(self._dyn.config[jnt + '_kd'])
                self._pid[jnt].initialize()

    def _get_current_position(self, joint_names):
        return [self._limb.joint_angle(joint) for joint in joint_names]

    def _get_current_velocities(self, joint_names):
        return [self._limb.joint_velocity(joint) for joint in joint_names]

    def _get_current_error(self, joint_names, set_point):
        current = self._get_current_position(joint_names)
        error = map(operator.sub, set_point, current)
        return zip(joint_names, error)

    def _update_feedback(self, cmd_point, jnt_names, cur_time):
        self._fdbk.header.stamp = rospy.Duration.from_sec(rospy.get_time())
        self._fdbk.joint_names = jnt_names
        self._fdbk.desired = cmd_point
        self._fdbk.desired.time_from_start = rospy.Duration.from_sec(cur_time)
        self._fdbk.actual.positions = self._get_current_position(jnt_names)
        self._fdbk.actual.time_from_start = rospy.Duration.from_sec(cur_time)
        self._fdbk.error.positions = map(operator.sub,
                                         self._fdbk.desired.positions,
                                         self._fdbk.actual.positions
                                        )
        self._fdbk.error.time_from_start = rospy.Duration.from_sec(cur_time)
        self._server.publish_feedback(self._fdbk)

    def _command_stop(self, joint_names, joint_angles):
        if self._mode == 'velocity':
            velocities = [0.0] * len(joint_names)
            cmd = dict(zip(joint_names, velocities))
            while (not self._server.is_new_goal_available() and self._alive):
                self._limb.set_joint_velocities(cmd)
                if self._cuff_state:
                    self._limb.exit_control_mode()
                    break
                rospy.sleep(1.0 / self._control_rate)
        elif self._mode == 'position' or self._mode == 'position_w_id':
            if self._mode == 'position_w_id':
                pnt = JointTrajectoryPoint()
                pnt.time_from_start = rospy.Time.now()
                pnt.positions = [0.0] * len(joint_names)
                pnt.velocities = [0.0] * len(joint_names)
                pnt.accelerations = [0.0] * len(joint_names)
            while (not self._server.is_new_goal_available() and self._alive):
                self._limb.set_joint_positions(joint_angles, raw=True)
                # zero inverse dynamics feedforward command
                if self._mode == 'position_w_id':
                    self._pub_ff_cmd.publish(pnt)
                if self._cuff_state:
                    self._limb.exit_control_mode()
                    break
                rospy.sleep(1.0 / self._control_rate)

    def _command_joints(self, joint_names, point):
        if self._server.is_preempt_requested():
            rospy.loginfo("%s: Trajectory Preempted" % (self._action_name,))
            self._server.set_preempted()
            self._command_stop(joint_names, self._limb.joint_angles())
            return False
        velocities = []
        deltas = self._get_current_error(joint_names, point.positions)
        for delta in deltas:
            if (math.fabs(delta[1]) >= self._path_thresh[delta[0]]
                and self._path_thresh[delta[0]] >= 0.0):
                rospy.logerr("%s: Exceeded Error Threshold on %s: %s" %
                             (self._action_name, delta[0], str(delta[1]),))
                self._result.error_code = self._result.PATH_TOLERANCE_VIOLATED
                self._server.set_aborted(self._result)
                self._command_stop(joint_names, self._limb.joint_angles())
                return False
            if self._mode == 'velocity':
                velocities.append(self._pid[delta[0]].compute_output(delta[1]))
        if ((self._mode == 'position' or self._mode == 'position_w_id')
            and self._alive):
            cmd = dict(zip(joint_names, point.positions))
            self._limb.set_joint_positions(cmd, raw=True)
            if self._mode == 'position_w_id':
                self._pub_ff_cmd.publish(point)
        elif self._alive:
            cmd = dict(zip(joint_names, velocities))
            self._limb.set_joint_velocities(cmd)
        return True

    def _get_bezier_point(self, joint_names, b_matrix, idx, t, cmd_time):
        pnt = JointTrajectoryPoint()
        pnt.time_from_start = rospy.Duration(cmd_time)
        pnt.positions = [0.0] * len(joint_names)
        pnt.velocities = [0.0] * len(joint_names)
        pnt.accelerations = [0.0] * len(joint_names)

        for jnt in xrange(len(joint_names)):
            b_point = bezier.compute_point(b_matrix[jnt, :, :, :], idx, t)
            # Positions at specified time
            pnt.positions[jnt] = b_point[0]
            # Velocities at specified time
            pnt.velocities[jnt] = b_point[1]
            # Accelerations at specified time
            pnt.accelerations[jnt] = b_point[2]

        return pnt

    def _compute_bezier_coeff(self, joint_names, trajectory_points):
        # Compute Full Bezier Curve
        num_joints = len(joint_names)
        num_traj_pts = len(trajectory_points)
        num_traj_dim = len(['position', 'velocity', 'acceleration'])
        num_b_values = len(['b0', 'b1', 'b2', 'b3'])
        b_matrix = np.zeros(shape=(num_joints, num_traj_dim, num_traj_pts-1, num_b_values))
        for jnt in xrange(num_joints):
            traj_array = np.zeros(shape=(len(trajectory_points), num_traj_dim))
            for idx, point in enumerate(trajectory_points):
                traj_array[idx, :] = [point.positions[jnt], 
                                            point.velocities[jnt], 
                                            point.accelerations[jnt]] 
            d_pts = bezier.compute_de_boor_control_pts(traj_array)
            b_matrix[jnt, :, :, :] = bezier.compute_bvals(traj_array, d_pts) 
        return b_matrix

    def _on_trajectory_action(self, goal):
        joint_names = goal.trajectory.joint_names
        trajectory_points = goal.trajectory.points
        rospy.loginfo("%s: Executing requested joint trajectory" %
                      (self._action_name,))
        # Load parameters for trajectory
        self._get_trajectory_parameters(joint_names, goal)
        # Create a new discretized joint trajectory
        num_points = len(trajectory_points)
        if num_points == 0:
            rospy.logerr("%s: Empty Trajectory" % (self._action_name,))
            self._server.set_aborted()
            return
        control_rate = rospy.Rate(self._control_rate)
        # Compute Full Bezier Curve Coefficients for all 7 joints
        bezier_start = rospy.get_time()
        pnt_times = [pnt.time_from_start.to_sec() for pnt in trajectory_points]
        b_matrix = self._compute_bezier_coeff(joint_names, trajectory_points)
        bezier_end = rospy.get_time()
        print 'Bezier Time Elapsed:' + str(bezier_start - bezier_end)
            
        # Reset feedback/result
        start_point = JointTrajectoryPoint()
        start_point.positions = self._get_current_position(joint_names)
        start_point.velocities = self._get_current_velocities(joint_names)
        start_point.accelerations = [0.0] * len(joint_names)
        self._update_feedback(deepcopy(start_point), joint_names,
                              rospy.get_time())

        # Wait for the specified execution time, if not provided use now
        start_time = goal.trajectory.header.stamp.to_sec()
        if start_time == 0.0:
            start_time = rospy.get_time()
        baxter_dataflow.wait_for(
            lambda: rospy.get_time() >= start_time,
            timeout=float('inf')
        )
        # Loop until end of trajectory time.  Provide a single time step
        # of the control rate past the end to ensure we get to the end.
        # Keep track of current indices for spline segment generation
        plt_pnts = list()
        point = start_point 
        now_from_start = rospy.get_time() - start_time
        end_time = trajectory_points[-1].time_from_start.to_sec()
        while (now_from_start < end_time and not rospy.is_shutdown()):
            #Acquire Mutex
            self._mutex.acquire()
            now = rospy.get_time()
            now_from_start = now - start_time
            idx = bisect.bisect(pnt_times, now_from_start)
            #Calculate percentage of time passed in this interval
            if idx == num_points:
                cmd_time = now_from_start - pnt_times[-1]
                t = 1.0
            elif idx > 0:
                cmd_time = (now_from_start - pnt_times[idx-1])
                t = cmd_time / (pnt_times[idx] - pnt_times[idx-1])
            else:
                cmd_time = 0
                t = 0

            # If the current time is after the last trajectory point,
            # just hold that position.
            if idx < num_points:
                point = self._get_bezier_point(joint_names, b_matrix, idx, t, cmd_time)
            else:
                point = self._get_bezier_point(joint_names, b_matrix, num_points, t, cmd_time)

            # Command Joint Position, Velocity, Acceleration
            command_success = self._command_joints(joint_names, point)
            point.time_from_start = now_from_start
            #plt_pnts.append(point)
            self._update_feedback(deepcopy(point), joint_names, now_from_start)
            # Release the Mutex
            self._mutex.release()
            if not command_success:
                return
            control_rate.sleep()

        """
        if self._name == 'left':
            joint_no = 1
            print 'Number points = ' + str(len(plt_pnts))
            traj_array = np.zeros((len(trajectory_points),3))
            for idx, point in enumerate(trajectory_points):
                plt.subplot(3, 1, 1)
                x_out = point.time_from_start.to_sec()
                y_pos_out = point.positions[joint_no]
                print "timestamps: {0}".format(x_out)
                print "pos: {0}".format(y_pos_out)
                plt.plot(x_out, y_pos_out, 'ro')
                plt.title('Position Commanded right_s0 In')
 
                plt.subplot(3, 1, 2)
                x_out = point.time_from_start.to_sec()
                y_vel_out = point.velocities[joint_no]
                print "timestamps: {0}".format(x_out)
                print "vel: {0}".format(y_vel_out)
                plt.plot(x_out, y_vel_out, 'go')
                plt.title('Velocity Commanded left_s0 In')
 
                plt.subplot(3, 1, 3)
                x_out = point.time_from_start.to_sec()
                y_accel_out = point.accelerations[joint_no]
                print "timestamps: {0}".format(x_out)
                print "accel: {0}".format(y_accel_out)
                plt.plot(x_out, y_accel_out, 'bo')
                plt.title('Acceleration Commanded left_s0 In')
                traj_array[idx, range(0,3)] = [y_pos_out, y_vel_out, y_accel_out] 
            pnt_array = np.zeros((len(plt_pnts),3))
            for idx, point in enumerate(plt_pnts):
                plt.subplot(3, 1, 1)
                x_out = point.time_from_start
                y_pos_out = point.positions[joint_no]
                plt.plot(x_out, y_pos_out, 'bo')
                plt.title('Position Commanded right_s0 Out')
 
                plt.subplot(3, 1, 2)
                x_out = point.time_from_start
                y_vel_out = point.velocities[joint_no]
                plt.plot(x_out, y_vel_out, 'ro')
                plt.title('Velocity Commanded left_s0 Out')
 
                plt.subplot(3, 1, 3)
                x_out = point.time_from_start
                y_accel_out = point.accelerations[joint_no]
                plt.plot(x_out, y_accel_out, 'go')
                plt.title('Acceleration Commanded left_s0 Out')
                pnt_array[idx, range(0,3)] = [y_pos_out, y_vel_out, y_accel_out] 
            fig = plt.figure()
            ax = fig.gca(projection='3d')
            ax.plot(pnt_array[:,0], pnt_array[:,1], pnt_array[:,2])
            ax.plot(traj_array[:,0], traj_array[:,1], traj_array[:,2], 'g*')
            ax.set_xlabel("Position")
            ax.set_ylabel("Velocity")
            ax.set_zlabel("Acceleration")
            ax.set_title("PR2 Quintic")
            fig = plt.figure()
            d_pts = bezier.compute_de_boor_control_pts(traj_array)
            bvals = bezier.compute_bvals(traj_array, d_pts)
            b_curve = bezier.compute_curve(bvals, 50)
            ab = fig.gca(projection='3d')
            ab.plot(b_curve[:,0], b_curve[:,1], b_curve[:,2])
            ab.plot(traj_array[:,0], traj_array[:,1], traj_array[:,2], 'g*')
            ab.set_xlabel("Position")
            ab.set_ylabel("Velocity")
            ab.set_zlabel("Acceleration")
            ax.set_title("Bezier Cubic")
            plt.show()
        """
        # Keep trying to meet goal until goal_time constraint expired
        last = trajectory_points[-1]
        last_time = trajectory_points[-1].time_from_start.to_sec()
        end_angles = dict(zip(joint_names, last.positions))

        def check_goal_state():
            for error in self._get_current_error(joint_names, last.positions):
                if (self._goal_error[error[0]] > 0
                        and self._goal_error[error[0]] < math.fabs(error[1])):
                    return error[0]
            if (self._stopped_velocity > 0 and
                max(self._get_current_velocities(joint_names)) >
                    self._stopped_velocity):
                return False
            else:
                return True

        while (now_from_start < (last_time + self._goal_time)
               and check_goal_state() and not rospy.is_shutdown()):
            if not self._command_joints(joint_names, last):
                return
            now_from_start = rospy.get_time() - start_time
            self._update_feedback(deepcopy(last), joint_names,
                                  now_from_start)
            control_rate.sleep()

        now_from_start = rospy.get_time() - start_time
        self._update_feedback(deepcopy(last), joint_names,
                                  now_from_start)

        # Verify goal constraint
        result = check_goal_state()
        if result is True:
            rospy.loginfo("%s: Joint Trajectory Action Succeeded" %
                          self._action_name)
            self._result.error_code = self._result.SUCCESSFUL
            self._server.set_succeeded(self._result)
        elif result is False:
            rospy.logerr("%s: Exceeded Max Goal Velocity Threshold" %
                         (self._action_name,))
            self._result.error_code = self._result.GOAL_TOLERANCE_VIOLATED
            self._server.set_aborted(self._result)
        else:
            rospy.logerr("%s: Exceeded Goal Threshold Error %s" %
                         (self._action_name, result,))
            self._result.error_code = self._result.GOAL_TOLERANCE_VIOLATED
            self._server.set_aborted(self._result)
        self._command_stop(goal.trajectory.joint_names, end_angles)
