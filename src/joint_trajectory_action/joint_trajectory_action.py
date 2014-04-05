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
from copy import deepcopy
import math
import operator

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
    def __init__(self, limb, reconfig_server, rate=100.0, mode='position'):
        self._dyn = reconfig_server
        self._ns = 'robot/limb/' + limb + '/follow_joint_trajectory'
        self._server = actionlib.SimpleActionServer(
            self._ns,
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
        if self._mode != 'position' and self._mode != 'velocity':
            rospy.logerr("%s: Action Server Creation Failed - "
                         "Provided Invalid Joint Control Mode '%s' (Options: "
                         "'position', 'velocity')" %
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
            if not jnt in self._limb.joint_names():
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
        elif self._mode == 'position':
            while (not self._server.is_new_goal_available() and self._alive):
                self._limb.set_joint_positions(joint_angles, raw=True)
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
        if self._mode == 'position' and self._alive:
            cmd = dict(zip(joint_names, point.positions))
            self._limb.set_joint_positions(cmd, raw=True)
        elif self._alive:
            cmd = dict(zip(joint_names, velocities))
            self._limb.set_joint_velocities(cmd)
        return True

    def _compute_spline_coefficients(self, start, end):
        duration = (end.time_from_start.to_sec() -
                    start.time_from_start.to_sec())
        duration2 = math.pow(duration, 2)
        duration3 = math.pow(duration, 3)
        duration4 = math.pow(duration, 4)
        duration5 = math.pow(duration, 5)

        for jnt in xrange(len(start.positions)):
            # Check if Quintic Spline is appropriate
            # (position, velocity, acceleration provided)
            if len(start.accelerations) != 0 and len(end.accelerations) != 0:
                # Quintic Spline Coefficients
                self._coeff[jnt][0] = start.positions[jnt]
                self._coeff[jnt][1] = start.velocities[jnt]
                self._coeff[jnt][2] = 0.5 * start.accelerations[jnt]
                if duration == 0.0:
                    self._coeff[jnt][3] = 0.0
                    self._coeff[jnt][4] = 0.0
                    self._coeff[jnt][5] = 0.0
                else:
                    self._coeff[jnt][3] = ((-20.0 * start.positions[jnt]
                                            + 20.0 * end.positions[jnt]
                                            - (3.0 * start.accelerations[jnt]
                                               * duration2)
                                            + (end.accelerations[jnt]
                                               * duration2)
                                            - (12.0 * start.velocities[jnt]
                                               * duration)
                                            - (8.0 * end.velocities[jnt]
                                               * duration))
                                           / (2.0 * duration3))
                    self._coeff[jnt][4] = ((30.0 * start.positions[jnt]
                                            - 30.0 * end.positions[jnt]
                                            + (3.0 * start.accelerations[jnt]
                                               * duration2)
                                            - (2.0 * end.accelerations[jnt]
                                               * duration2)
                                            + (16.0 * start.velocities[jnt]
                                               * duration)
                                            + (14.0 * end.velocities[jnt]
                                               * duration))
                                           / (2.0 * duration4))
                    self._coeff[jnt][5] = ((-12.0 * start.positions[jnt]
                                            + 12.0 * end.positions[jnt]
                                            - (start.accelerations[jnt]
                                               * duration2)
                                            + (end.accelerations[jnt]
                                               * duration2)
                                            - (6.0 * start.velocities[jnt]
                                               * duration)
                                            - (6.0 * end.velocities[jnt]
                                               * duration))
                                           / (2.0 * duration5))

            # Check if Cubic Spline is appropriate
            # (position, velocity provided)
            elif (len(start.velocities) != 0 and len(end.velocities) != 0):
                # Cubic Spline Coefficients
                self._coeff[jnt][0] = start.positions[jnt]
                self._coeff[jnt][1] = start.velocities[jnt]
                if duration == 0.0:
                    self._coeff[jnt][2] = 0.0
                    self._coeff[jnt][3] = 0.0
                else:
                    self._coeff[jnt][2] = ((-3.0 * start.positions[jnt]
                                            + 3.0 * end.positions[jnt]
                                            - (2.0 * start.velocities[jnt]
                                               * duration)
                                            - end.velocities[jnt] * duration)
                                           / duration2)
                    self._coeff[jnt][3] = ((2.0 * start.positions[jnt]
                                            - 2.0 * end.positions[jnt]
                                            + start.velocities[jnt] * duration
                                            + end.velocities[jnt] * duration)
                                           / duration3)
                self._coeff[jnt][4] = 0.0
                self._coeff[jnt][5] = 0.0

            # Linear Spline
            # (position only)
            else:
                # Linear Spline
                self._coeff[jnt][0] = start.positions[jnt]
                if duration == 0.0:
                    self._coeff[jnt][1] = 0.0
                else:
                    self._coeff[jnt][1] = ((end.positions[jnt]
                                            - start.positions[jnt])
                                           / duration)
                self._coeff[jnt][2] = 0.0
                self._coeff[jnt][3] = 0.0
                self._coeff[jnt][4] = 0.0
                self._coeff[jnt][5] = 0.0

    def _get_point(self, joint_names, time):
        pnt = JointTrajectoryPoint()
        pnt.time_from_start = rospy.Duration(time)
        pnt.positions = [0.0] * len(joint_names)
        pnt.velocities = [0.0] * len(joint_names)
        pnt.accelerations = [0.0] * len(joint_names)
        time2 = math.pow(time, 2)
        time3 = math.pow(time, 3)
        time4 = math.pow(time, 4)
        time5 = math.pow(time, 5)

        for jnt in xrange(len(joint_names)):
            # Positions at specified time
            pnt.positions[jnt] = (self._coeff[jnt][0]
                                  + time * self._coeff[jnt][1]
                                  + time2 * self._coeff[jnt][2]
                                  + time3 * self._coeff[jnt][3]
                                  + time4 * self._coeff[jnt][4]
                                  + time5 * self._coeff[jnt][5]
                                  )

            # Velocities at specified time
            pnt.velocities[jnt] = (self._coeff[jnt][1]
                                   + 2.0 * time * self._coeff[jnt][2]
                                   + (3.0 * time2
                                      * self._coeff[jnt][3])
                                   + (4.0 * time3
                                      * self._coeff[jnt][4])
                                   + (5.0 * time4
                                      * self._coeff[jnt][5])
                                   )

            # Accelerations at specified time
            pnt.accelerations[jnt] = (2.0 * self._coeff[jnt][2]
                                      + 6.0 * time * self._coeff[jnt][3]
                                      + (12.0 * time2
                                         * self._coeff[jnt][4])
                                      + (20.0 * time3
                                         * self._coeff[jnt][5])
                                      )
        return pnt

    def _on_trajectory_action(self, goal):
        joint_names = goal.trajectory.joint_names
        trajectory_points = goal.trajectory.points

        rospy.loginfo("%s: Executing requested joint trajectory" %
                      (self._action_name,))

        # Clear spline coefficients
        for jnt in xrange(len(joint_names)):
            self._coeff[jnt] = [0.0] * 6

        # Load parameters for trajectory
        self._get_trajectory_parameters(joint_names, goal)

        # Create a new discretized joint trajectory
        num_points = len(trajectory_points)

        if num_points == 0:
            rospy.logerr("%s: Empty Trajectory" % (self._action_name,))
            self._server.set_aborted()
            return

        def interp(a, b, pct):
            return a + (b - a) * pct

        def interp_positions(p1, p2, pct):
            return map(interp, p1.positions, p2.positions, [pct] *
                       len(p1.positions))

        end_time = trajectory_points[-1].time_from_start.to_sec()
        control_rate = rospy.Rate(self._control_rate)

        pnt_times = [pnt.time_from_start.to_sec() for pnt in trajectory_points]

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
        now_from_start = rospy.get_time() - start_time
        # Keep track of current indices for spline segment generation
        last_idx = -1
        while (now_from_start < end_time and not rospy.is_shutdown()):
            idx = bisect.bisect(pnt_times, now_from_start)

            if idx == 0:
                # If our current time is before the first specified point
                # in the trajectory, then we should interpolate between
                # our start position and that point.
                p1 = deepcopy(start_point)
            else:
                p1 = deepcopy(trajectory_points[idx - 1])

            if idx != num_points:
                p2 = trajectory_points[idx]
                # If entering a new trajectory segment, calculate coefficients
                if last_idx != idx:
                    self._compute_spline_coefficients(p1, p2)
                # Find goal command point at commanded time
                cmd_time = now_from_start - p1.time_from_start.to_sec()
                point = self._get_point(joint_names, cmd_time)
            else:
                # If the current time is after the last trajectory point,
                # just hold that position.
                point = p1

            if not self._command_joints(joint_names, point):
                return

            control_rate.sleep()
            now_from_start = rospy.get_time() - start_time
            self._update_feedback(deepcopy(point), joint_names, now_from_start)
            last_idx = idx

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
