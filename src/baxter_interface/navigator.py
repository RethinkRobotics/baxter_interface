# Copyright (c) 2013-2015, Rethink Robotics
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

import rospy

import baxter_dataflow

from baxter_core_msgs.msg import (
    NavigatorState,
)

from baxter_interface import (
    digital_io,
)

class Navigator(object):
    """
    Interface class for a Navigator on the Baxter robot.

    Inputs:
        Button 0     - press wheel
        Button 1     - above wheel
        Button 2     - below wheel
        Scroll wheel - 0-255

    Outputs:
        Inner LED
        Outer LED

    Signals:
        button0_changed - True/False
        button1_changed - True/False
        button2_changed - True/False
        wheel_changed   - New wheel value

    Valid identifiers:
        left, right, torso_left, torso_right
    """

    __LOCATIONS = ('left', 'right', 'torso_left', 'torso_right')

    def __init__(self, location):
        """
        Constructor.

        @type location: str
        @param location: body location (prefix) of Navigator to control.

        Valid locations are in L{Navigator.__LOCATIONS}::
          left, right, torso_left, torso_right
        """
        if not location in self.__LOCATIONS:
            raise AttributeError("Invalid Navigator name '%s'" % (location,))
        self._id = location
        self._state = None
        self.button0_changed = baxter_dataflow.Signal()
        self.button1_changed = baxter_dataflow.Signal()
        self.button2_changed = baxter_dataflow.Signal()
        self.wheel_changed = baxter_dataflow.Signal()

        nav_state_topic = 'robot/navigators/{0}_navigator/state'.format(self._id)
        self._state_sub = rospy.Subscriber(
            nav_state_topic,
            NavigatorState,
            self._on_state)

        self._inner_led = digital_io.DigitalIO(
            '%s_inner_light' % (self._id,))
        self._inner_led_idx = 0

        self._outer_led = digital_io.DigitalIO(
            '%s_outer_light' % (self._id,))
        self._outer_led_idx = 1

        init_err_msg = ("Navigator init failed to get current state from %s" %
                        (nav_state_topic,))
        baxter_dataflow.wait_for(lambda: self._state != None,
                                 timeout_msg=init_err_msg)

    @property
    def wheel(self):
        """
        Current state of the wheel
        """
        return self._state.wheel

    @property
    def button0(self):
        """
        Current state of button 0
        """
        return self._state.buttons[0]

    @property
    def button1(self):
        """
        Current state of button 1
        """
        return self._state.buttons[1]

    @property
    def button2(self):
        """
        Current state of button 2
        """
        return self._state.buttons[2]

    @property
    def inner_led(self):
        """
        Current state of the inner LED
        """
        return self._state.lights[self._inner_led_idx]

    @inner_led.setter
    def inner_led(self, enable):
        """
        Control the inner LED.

        @type enable: bool
        @param enable: True to enable the light, False otherwise
        """
        self._inner_led.set_output(enable)

    @property
    def outer_led(self):
        """
        Current state of the outer LED.
        """
        return self._state.lights[self._outer_led_idx]

    @outer_led.setter
    def outer_led(self, enable):
        """
        Control the outer LED.

        @type enable: bool
        @param enable: True to enable the light, False otherwise
        """
        self._outer_led.set_output(enable)

    def _on_state(self, msg):
        if not self._state:
            self._state = msg
            try:
                self._inner_led_idx = self._state.light_names.index("inner")
            except:
                pass
            try:
                self._outer_led_idx = self._state.light_names.index("outer")
            except:
                pass
        if self._state == msg:
            return

        old_state = self._state
        self._state = msg

        buttons = [self.button0_changed,
                   self.button1_changed,
                   self.button2_changed
                   ]
        for i, signal in enumerate(buttons):
            if old_state.buttons[i] != msg.buttons[i]:
                signal(msg.buttons[i],self._id,"button",i)

        if old_state.wheel != msg.wheel:
            diff = msg.wheel - old_state.wheel
            if abs(diff % 256) < 127:
                self.wheel_changed(diff % 256,self._id,"wheel",msg.wheel)
            else:
                self.wheel_changed(diff % (-256),self._id,"wheel",msg.wheel)
