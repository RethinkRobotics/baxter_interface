#!/usr/bin/env python

# Copyright (c) 2018, Hamburg University
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
# 3. Neither the name of Hamburg University nor the names of its
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
Baxter Gripper Trajectory Controller
    This provides a high-level interface to open/close
    the grippers from MoveIt.

    It wraps the RSDK Gripper Controller and reacts to opened/closed
    gripper state targets by forwarding them as open/close controller requests.
"""

import argparse

import rospy

import actionlib

from control_msgs.msg import (
    FollowJointTrajectoryAction,
    FollowJointTrajectoryResult
)

from baxter_interface import Gripper

class GripperTrajectoryAction:
    # @name in ["left", "right"]
    def __init__(self, gripper_name):
        self._gripper= Gripper(gripper_name)
        self._as= actionlib.SimpleActionServer("robot/end_effector/"+gripper_name+"_gripper/follow_joint_trajectory", FollowJointTrajectoryAction, execute_cb=self.execute, auto_start= False)
        self._as.start()

    def execute(self, goal):
        rospy.loginfo("received goal")

        result= FollowJointTrajectoryResult(
            error_code= FollowJointTrajectoryResult.SUCCESSFUL
        )

        if len(goal.trajectory.points) == 0:
            result.error_code= FollowJointTrajectoryResult.INVALID_GOAL
            result.error_string= "empty trajectory"
            self._as.set_aborted(result)

        def test(j):
            return "l_finger_joint" in j
        try:
            control_idx= [test(j) for j in goal.trajectory.joint_names].index(True)
        except ValueError:
            result.error_code= FollowJointTrajectoryResult.INVALID_GOAL
            result.error_string= "does not contain an l_finger_joint"
            self._as.set_aborted(result)

        if len(goal.trajectory.points[-1].positions) <= control_idx:
            result.error_code= FollowJointTrajectoryResult.INVALID_GOAL
            result.error_string= "missing joint positions in waypoint"
            self._as.set_aborted(result)

        target= goal.trajectory.points[-1].positions[control_idx]

        rospy.loginfo("target is " + str(target))
        if target > 1e-4:
            rospy.loginfo("open gripper")
            # 0.0207349748752
            ret= self._gripper.open()
        else:
            rospy.loginfo("close gripper")
            ret= self._gripper.close()

        if ret:
            self._as.set_succeeded(result)
        else:
            result.error_code= FollowJointTrajectoryResult.GOAL_TOLERANCE_VIOLATED
            result.error_string= "gripper action failed"

            self._as.set_aborted(result)

def start_server(limb):
    print("Initializing node... ")
    rospy.init_node("sdk_gripper_trajectory_action_server%s" %
                    ("" if limb == 'both' else "_" + limb))
    print("Initializing gripper trajectory action server...")

    tas = []
    if limb == 'both':
        tas.append(GripperTrajectoryAction('left'))
        tas.append(GripperTrajectoryAction('right'))
    else:
        tas.append(GripperTrajectoryAction(limb))

    #def cleanup():
    #    pass
    #rospy.on_shutdown(cleanup)

    print("Running. Ctrl-c to quit")
    rospy.spin()


def main():
    arg_fmt = argparse.ArgumentDefaultsHelpFormatter
    parser = argparse.ArgumentParser(formatter_class=arg_fmt)
    parser.add_argument(
        "-l", "--limb", dest="limb", default="both",
        choices=['both', 'left', 'right'],
        help="joint trajectory action server limb"
    )
    args = parser.parse_args(rospy.myargv()[1:])
    start_server(args.limb)


if __name__ == "__main__":
    main()
