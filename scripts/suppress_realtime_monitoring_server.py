#!/usr/bin/env python

# Copyright (c) 2016, Kentaro Wada
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
# 3. Neither the name of the Kentaro Wada nor the names of its
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

import os

from dynamic_reconfigure.server import Server
import rospy
from std_msgs.msg import Empty

from baxter_interface.cfg import SuppressRealtimeMonitoringServerConfig


class SuppressRealtimeMonitoringServer(object):

    arms = ['left', 'right']

    params = [
        'suppress_body_avoidance',
        'suppress_collision_avoidance',
        'suppress_contact_safety',
        'suppress_cuff_interaction',
        'suppress_gravity_compensation',
        'suppress_hand_overwrench_safety',
    ]

    def __init__(self):
        # Initialize config and set dynamic parameter server
        self.config = {}
        self.dynparam_server = Server(
            SuppressRealtimeMonitoringServerConfig, self._dynparam_cb)
        # Initialize config and publishers
        self.publishers = {}
        for arm in self.arms:
            for param in self.params:
                key = arm + '_' + param
                self.publishers[key] = rospy.Publisher(
                    os.path.join('/robot/limb', arm, param), Empty,
                    queue_size=1)
        # Set timer with 10Hz
        # and it's enough because 5Hz is requested for supression
        self.timer = rospy.Timer(
            rospy.Duration(1.0 / 10), self.suppress_timer_cb)

    def _dynparam_cb(self, config, level):
        for arm in self.arms:
            for param in self.params:
                key = arm + '_' + param
                self.config[key] = config[key]
        return config

    def suppress_timer_cb(self, event):
        for key, pub in self.publishers.items():
            if self.config[key]:
                pub.publish(Empty())


if __name__ == '__main__':
    rospy.init_node('suppress_realtime_monitoring_server')
    server = SuppressRealtimeMonitoringServer()
    rospy.spin()
