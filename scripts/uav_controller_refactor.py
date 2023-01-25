#!/usr/bin/env python3

"""
 * File: offb_node.py
 * Stack and tested in Gazebo 9 SITL
"""

import math

import rospy
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest


class PIController:
    def __init__(self, Kp: float, Ki: float,
                 sat_i_min: float, sat_i_max: float,
                 out_min: float, out_max: float,
                 setpoint: float):
        self.Kp = Kp
        self.Ki = Ki
        self.sat_i_min = sat_i_min
        self.sat_i_max = sat_i_max
        self.out_min = out_min
        self.out_max = out_max

        self.setpoint = setpoint

        self.integral_v = 0.0
        self.integral_v_previous = 0.0

    def compute(self, x: float, delta_t: float):
        # X error calculation
        x_error = self.setpoint - x

        # Integral Calculation
        self.integral_v = self.integral_v_previous + (self.Ki * x_error * delta_t)

        # Integral should be sat_i_min and sat_i_max
        if self.integral_v < self.sat_i_min:
            self.integral_v = self.sat_i_min
        elif self.integral_v > self.sat_i_max:
            self.integral_v = self.sat_i_max

        self.integral_v_previous = self.integral_v

        # V final value calculation
        v = self.integral_v + self.Kp * x_error

        # V interval between out_min and out_max
        if v < self.out_min:
            v = self.out_min
        elif v > self.out_max:
            v = self.out_max

        return v


class ZController:
    def __init__(self, Kp: float, out_min: float, out_max: float, setpoint_x: float, setpoint_y: float, setpoint_z: float):
        self.Kp = Kp
        self.out_min = out_min
        self.out_max = out_max

        self.setpoint_x = setpoint_x
        self.setpoint_y = setpoint_y
        self.setpoint_z = setpoint_z

    # Condition to init the particle downward movement; if not, the particle stops its movement
    def compute(self, x: float, y: float, z: float, delta_t: float):
        if math.sqrt(pow(self.setpoint_x - x, 2) + pow(self.setpoint_y - y, 2)) < 0.5 and self.setpoint_z > 0.2:
            v_ref = self.out_min
        else:
            v_ref = 0

        # Virtual particle position update
        self.setpoint_z = self.setpoint_z + v_ref * delta_t

        # Z error calculation
        z_error = self.setpoint_z - z

        # V final value calculation
        v = v_ref +  math.tanh(self.Kp * z_error)

        # V interval between out_min and out_max
        if v < self.out_min:
            v = self.out_min
        elif v > self.out_max:
            v = self.out_max

        return v


class UAVController:

    def __init__(self):
        rospy.init_node("uav_controller")

        # Subscribers
        self.state_sub = rospy.Subscriber("mavros/state", State, callback=self.state_cb)
        self.local_pose_sub = rospy.Subscriber("mavros/local_position/pose", PoseStamped, callback=self.local_pose_cb)
        self.setpoint_xy_sub = rospy.Subscriber("setpoint_xy", PoseStamped, callback=self.setpoint_cb)

        # Publishers
        # Creates publisher to send position command msg
        self.local_pos_pub = rospy.Publisher("mavros/setpoint_position/local", PoseStamped, queue_size=10)
        # Creates publisher to send velocity command msg
        self.vel_pub = rospy.Publisher("/mavros/setpoint_velocity/cmd_vel", TwistStamped, queue_size=10)

        rospy.wait_for_service("/mavros/cmd/arming")
        self.arming_client = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)

        rospy.wait_for_service("/mavros/set_mode")
        self.set_mode_client = rospy.ServiceProxy("mavros/set_mode", SetMode)

        self.setpoint = None  # None when no value has been set yet, and PoseStamped for real values
        self.local_pose = None  # None when no value has been received yet, and PoseStamped for real values
        self.current_state = None  # None when no value has been received yet, and State for real values; has UAV mode, e.g. offboard
        self.previous_time = 0  # Time since last controller update

    def run_landing(self):
        # Confirmar se está em offboard e, caso não esteja, comutar para esse modo?

        # Init 2D controllers
        self.vx_controller = PIController(Kp=0.2, Ki=0.05, sat_i_min=-0.3, sat_i_max=0.3, out_min=-1, out_max=1,
                                     setpoint=0)
        self.vy_controller = PIController(Kp=0.2, Ki=0.05, sat_i_min=-0.3, sat_i_max=0.3, out_min=-1, out_max=1,
                                     setpoint=0)
        # Init 3D controller
        self.vz_controller = ZController(Kp=0.2, out_min= -0.2, out_max=1, setpoint_x=0, setpoint_y=0, setpoint_z=0)


        vel_x = self.vx_controller.compute(self.local_pose.pose.position.x, delta_t)
        vel_y = self.vy_controller.compute(self.local_pose.pose.position.y, delta_t)
        vel_z = self.vz_controller.compute(self.local_pose.pose.position.z, delta_t)

        vel = TwistStamped()

        vel.twist.linear.x = vel_x
        vel.twist.linear.y = vel_y
        vel.twist.linear.z = vel_z

        self.vel_pub.publish(vel)

        rate.sleep()


    def run_position(self):
        # Comutar para o modo que não é offboard

         = PoseStamped()

        .pose.position.x =
        .pose.position.y =
        .pose.position.z =


    def state_cb(self, msg):
        self.current_state = msg

    def local_pose_cb(self, msg):
        self.local_pose = msg

    def setpoint_cb(self, msg):
        self.setpoint = msg
        self.vx_controller.setpoint = msg.pose.position.x
        self.vy_controller.setpoint = msg.pose.position.y
        self.vz_controller.setpoint = msg.pose.position.z


    def run(self):
        previous_time = rospy.get_time()
        delta_t = 0

        # Setpoint publishing MUST be faster than 2Hz
        rate = rospy.Rate(20)

        # wait for first state to arrive
        rospy.loginfo("wait for first state to arrive")
        while (not rospy.is_shutdown() and self.current_state is None):
            rate.sleep()

        # Wait for Flight Controller connection
        rospy.loginfo("waiting for Flight Controller connection")
        while (not rospy.is_shutdown() and not self.current_state.connected):
            rate.sleep()

        offb_set_mode = SetModeRequest()
        offb_set_mode.custom_mode = 'OFFBOARD'

        arm_cmd = CommandBoolRequest()
        arm_cmd.value = True

        last_req = rospy.Time.now()

        rospy.loginfo("waiting for OFFBOARD mode")
        while (not rospy.is_shutdown()):
            if (self.current_state.mode != "OFFBOARD" and (rospy.Time.now() - last_req) > rospy.Duration(5.0)):
                if (self.set_mode_client.call(offb_set_mode).mode_sent == True):
                    rospy.loginfo("OFFBOARD enabled")

                last_req = rospy.Time.now()
            else:
                if (not self.current_state.armed and (rospy.Time.now() - last_req) > rospy.Duration(5.0)):
                    if (self.arming_client.call(arm_cmd).success == True):
                        rospy.loginfo("Vehicle armed")

                    last_req = rospy.Time.now()

            # Wait for first setpoint
            rospy.loginfo("waiting for first setpoint")
            if self.setpoint is None:
                rate.sleep()
                continue

            delta_t = rospy.get_time() - previous_time
            previous_time = rospy.get_time()

            # print('Time is', delta_t)
            # print('Position_X is', self.local_pose.pose.position.x)
            # print('Position_Y is', self.local_pose.pose.position.y)
            # print('Position_Z is', self.local_pose.pose.position.z)


            vel_x = self.vx_controller.compute(self.local_pose.pose.position.x, delta_t)
            vel_y = self.vy_controller.compute(self.local_pose.pose.position.y, delta_t)
            vel_z = self.vz_controller.compute(self.local_pose.pose.position.z, delta_t)

            # print('setpoint', self.setpoint)
            # print('vx_commanded', vel_x)
            # print('vy_commanded', vel_y)
            # print('vz_commanded', vel_z)
            # print('-----------------------------------')

            vel = TwistStamped()

            vel.twist.linear.x = vel_x
            vel.twist.linear.y = vel_y
            vel.twist.linear.z = vel_z

            self.vel_pub.publish(vel)

            rate.sleep()


def main_new():
    uav_ctrl = UAVController()
    uav_ctrl.run()


if __name__ == "__main__":
    # main1()
    main_new()