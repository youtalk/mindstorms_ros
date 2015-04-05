#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import absolute_import, division, print_function
import math
import thread

import roslib; roslib.load_manifest('ev3_ros')
import rospy
from sensor_msgs.msg import JointState, Imu
from std_msgs.msg import Bool
from PyKDL import Rotation
import rpyc
import nxt.error

from nxt_msgs.msg import Range, Contact, JointCommand, Color, Gyro, Accelerometer


class Device(object):
    def __init__(self, params):
        self.desired_period = 1.0 / params['desired_frequency']
        self.period = self.desired_period
        self.initialized = False
        self.name = params['name']

    def needs_trigger(self):
        # initialize
        if not self.initialized:
            self.initialized = True
            self.last_run = rospy.Time.now()
            rospy.logdebug('Initializing %s', self.name)
            return False
        # compute frequency
        now = rospy.Time.now()
        period = 0.9 * self.period + 0.1 * (now - self.last_run).to_sec()

        # check period
        if period > self.desired_period * 1.2:
            rospy.logwarn('%s not reaching desired frequency: actual %f, desired %f',
                          self.name, 1.0 / period, 1.0 / self.desired_period)
        elif period > self.desired_period * 1.5:
            rospy.logerr('%s not reaching desired frequency: actual %f, desired %f',
                         self.name, 1.0 / period, 1.0 / self.desired_period)

        return period > self.desired_period

    def trigger(self):
        raise NotImplementedError()

    def do_trigger(self):
        try:
            rospy.logdebug('Trigger %s with current frequency %f',
                           self.name, 1.0/self.period)
            now = rospy.Time.now()
            self.period = 0.9 * self.period + 0.1 * (now - self.last_run).to_sec()
            self.last_run = now
            self.trigger()
            rospy.logdebug('Trigger %s took %f mili-seconds',
                           self.name, (rospy.Time.now() - now).to_sec() * 1000)
        except nxt.error.I2CError:
            rospy.logwarn('caught an exception nxt.error.I2CError')
        except nxt.error.DirProtError:
            rospy.logwarn('caught an exception nxt.error.DirProtError')


class Motor(Device):
    POWER_TO_NM = 0.01
    POWER_MAX = 125

    def __init__(self, params, lego):
        super(Motor, self).__init__(params)

        # create motor
        self.motor = lego.Motor(params['port'])
        self.cmd = 0  # default command

        # create publisher
        self.pub = rospy.Publisher('joint_state', JointState)
        self.last_js = None

        # create subscriber
        self.sub = rospy.Subscriber('joint_command', JointCommand, self.cmd_cb, None, 2)

    def cmd_cb(self, msg):
        if msg.name == self.name:

            cmd = msg.effort / self.POWER_TO_NM
            if cmd > self.POWER_MAX:
                cmd = self.POWER_MAX
            elif cmd < -self.POWER_MAX:
                cmd = -self.POWER_MAX
            self.cmd = cmd  # save command

    def trigger(self):
        js = JointState()
        js.header.stamp = rospy.Time.now()
        state = self.motor.get_output_state()
        js.name.append(self.name)
        js.position.append(state[9] * math.pi / 180.0)
        js.effort.append(state[1] * self.POWER_TO_NM)
        if self.last_js:
            vel = (js.position[0] - self.last_js.position[0]) / \
                  (js.header.stamp - self.last_js.header.stamp).to_sec()
            js.velocity.append(vel)
        else:
            vel = 0
            js.velocity.append(vel)
        self.pub.publish(js)
        self.last_js = js

        # send command
        self.motor.run_forever(int(self.cmd), regulation_mode=False)


def main():
    rospy.init_node('ev3_ros')
    ns = 'ev3_robot'
    host = rospy.get_param('~host', None)
    conn = rpyc.classic.connect(host)
    lego = conn.modules.ev3.lego

    config = rospy.get_param('~' + ns)
    components = []
    for c in config:
        rospy.loginfo('Creating %s with name %s on %s',
                      c['type'], c['name'], c['port'])
        if c['type'] == 'motor':
            components.append(Motor(c, lego))
        else:
            rospy.logerr('Invalid sensor/actuator type %s', c['type'])

    callback_handle_frequency = 10.0
    last_callback_handle = rospy.Time.now()
    lock = thread.allocate_lock()

    while not rospy.is_shutdown():
        lock.acquire()
        triggered = False
        for c in components:
            if c.needs_trigger() and not triggered:
                c.do_trigger()
                triggered = True
        lock.release()
        now = rospy.Time.now()
        if (now - last_callback_handle).to_sec() > 1.0 / callback_handle_frequency:
            last_callback_handle = now
            rospy.sleep(0.01)


if __name__ == '__main__':
    main()
