#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import absolute_import, division, print_function
import math
import thread

import roslib; roslib.load_manifest('ev3_ros')
import rospy
from sensor_msgs.msg import JointState, Imu
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
    POWER_TO_NM = 50
    POWER_MAX = 50

    def __init__(self, params, lego):
        super(Motor, self).__init__(params)

        # create motor
        self.motor = lego.Motor(params['port'])
        self.cmd = 0  # default command

        # create publisher
        self.pub = rospy.Publisher('joint_state', JointState, queue_size=5)

        # create subscriber
        self.sub = rospy.Subscriber('joint_command', JointCommand, self.cmd_cb,
                                    None, queue_size=2)

    def cmd_cb(self, msg):
        if msg.name == self.name:
            cmd = msg.effort * self.POWER_TO_NM
            if cmd > self.POWER_MAX:
                cmd = self.POWER_MAX
            elif cmd < -self.POWER_MAX:
                cmd = -self.POWER_MAX
            self.cmd = cmd  # save command

    def trigger(self):
        js = JointState()
        js.header.stamp = rospy.Time.now()
        js.name.append(self.name)
        js.position.append(math.radians(self.motor.position))
        js.velocity.append(math.radians(self.motor.pulses_per_second))
        js.effort.append(0)
        self.pub.publish(js)

        # send command
        self.motor.run_forever(int(self.cmd), regulation_mode=False)


class UltraSonicSensor(Device):
    def __init__(self, params, lego):
        super(UltraSonicSensor, self).__init__(params)
        # create ultrasonic sensor
        self.ultrasonic = lego.UltrasonicSensor(params['port'])
        self.frame_id = params['frame_id']
        self.spread = params['spread_angle']
        self.min_range = params['min_range']
        self.max_range = params['max_range']

        # create publisher
        self.pub = rospy.Publisher(params['name'], Range, queue_size=5)

    def trigger(self):
        ds = Range()
        ds.header.frame_id = self.frame_id
        ds.header.stamp = rospy.Time.now()
        ds.range = self.ultrasonic.dist_cm / 100.0
        ds.spread_angle = self.spread
        ds.range_min = self.min_range
        ds.range_max = self.max_range
        self.pub.publish(ds)


class GyroSensor(Device):
    def __init__(self, params, lego):
        super(GyroSensor, self).__init__(params)
        # create gyro sensor
        self.gyro = lego.GyroSensor(params['port'])
        self.frame_id = params['frame_id']
        self.orientation = 0.0
        self.offset = 0.0
        self.prev_time = rospy.Time.now()

        # calibrate
        rospy.loginfo('Calibrating Gyro. Don\'t move the robot now')
        start_time = rospy.Time.now()
        cal_duration = rospy.Duration(2)
        offset = 0
        tmp_time = rospy.Time.now()
        while rospy.Time.now() < start_time + cal_duration:
            rospy.sleep(0.01)
            sample = self.gyro.ang
            now = rospy.Time.now()
            offset += (sample * (now - tmp_time).to_sec())
            tmp_time = now
        self.offset = offset / (tmp_time - start_time).to_sec()
        rospy.loginfo('Gyro calibrated with offset %f', self.offset)

        # create publisher
        self.pub = rospy.Publisher(params['name'], Gyro, queue_size=5)

        # create publisher
        self.pub2 = rospy.Publisher(params['name'] + '_imu', Imu, queue_size=5)

    def trigger(self):
        sample = self.gyro.ang
        gs = Gyro()
        gs.header.frame_id = self.frame_id
        gs.header.stamp = rospy.Time.now()
        gs.calibration_offset.x = 0.0
        gs.calibration_offset.y = 0.0
        gs.calibration_offset.z = self.offset
        gs.angular_velocity.x = 0.0
        gs.angular_velocity.y = 0.0
        gs.angular_velocity.z = (sample - self.offset) * math.pi / 180.0
        gs.angular_velocity_covariance = [0, 0, 0, 0, 0, 0, 0, 0, 1]
        self.pub.publish(gs)

        imu = Imu()
        imu.header.frame_id = self.frame_id
        imu.header.stamp = rospy.Time.now()
        imu.angular_velocity.x = 0.0
        imu.angular_velocity.y = 0.0
        imu.angular_velocity.z = (sample-self.offset) * math.pi / 180.0
        imu.angular_velocity_covariance = [0, 0, 0, 0, 0, 0, 0, 0, 1]
        imu.orientation_covariance = [0.001, 0, 0, 0, 0.001, 0, 0, 0, 0.1]
        self.orientation += imu.angular_velocity.z * (imu.header.stamp - self.prev_time).to_sec()
        self.prev_time = imu.header.stamp
        imu.orientation.x, imu.orientation.y, imu.orientation.z, imu.orientation.w = \
            Rotation.RotZ(self.orientation).GetQuaternion()
        self.pub2.publish(imu)


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
        elif c['type'] == 'ultrasonic':
            components.append(UltraSonicSensor(c, lego))
        elif c['type'] == 'gyro':
            components.append(GyroSensor(c, lego))
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
