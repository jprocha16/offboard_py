#!/usr/bin/env python3

"""
 * File: offb_node.py
 * Stack and tested in Gazebo 9 SITL
"""

import math

import rospy
from std_msgs.msg import UInt8
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest
from offboard_py.msg import ControllerErrors
from datetime import datetime


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

    def compute(self, measure: float, delta_t: float):
        # Error calculation
        error = self.setpoint - measure
        print('Error is', error)

        # Integral Calculation
        self.integral_v = self.integral_v_previous + (self.Ki * error * delta_t)

        # Integral should be sat_i_min and sat_i_max
        if self.integral_v < self.sat_i_min:
            self.integral_v = self.sat_i_min
        elif self.integral_v > self.sat_i_max:
            self.integral_v = self.sat_i_max

        self.integral_v_previous = self.integral_v

        # V final value calculation
        v = self.integral_v + self.Kp * error

        # V interval between out_min and out_max
        if v < self.out_min:
            v = self.out_min
        elif v > self.out_max:
            v = self.out_max

        return v, error, measure


class VzController:
    def __init__(self, Kp: float, out_min: float, out_max: float, setpoint_x: float, setpoint_y: float,
                 setpoint_z: float, v_particle: float, E: float, alt_min: float):
        self.Kp = Kp
        self.out_min = out_min
        self.out_max = out_max
        self.setpoint_x = setpoint_x
        self.setpoint_y = setpoint_y
        self.setpoint_z = setpoint_z
        self.v_particle = v_particle
        self.E = E
        self.alt_min = alt_min

    # Condition to init the particle downward movement; if not achieved, the particle stops its movement
    def compute(self, x: float, y: float, z: float, delta_t: float):
        if math.sqrt(pow(self.setpoint_x - x, 2) + pow(self.setpoint_y - y, 2)) < self.E and \
                self.setpoint_z > self.alt_min:
            v_ref = 0 * self.v_particle
        else:
            v_ref = 0

        # Virtual particle position update
        self.setpoint_z = self.setpoint_z + v_ref * delta_t

        # Z error calculation
        z_error = self.setpoint_z - z
        print('Error_Z is', z_error)

        # V final value calculation
        v = v_ref + math.tanh(self.Kp * z_error)

        # V interval between out_min and out_max
        if v < self.out_min:
            v = self.out_min
        elif v > self.out_max:
            v = self.out_max

        return v, z_error, z


class UAVController:
    valid_modes = ["POSITION", "LANDING"]

    def __init__(self):
        rospy.init_node("uav_controller")

        self.mode = None
        self.change_mode("POSITION")

        # Subscribers
        self.mode_sub = rospy.Subscriber("uav/mode", UInt8, callback=self.mode_cb)
        self.state_sub = rospy.Subscriber("mavros/state", State, callback=self.state_cb)
        self.local_pose_sub = rospy.Subscriber("mavros/local_position/pose", PoseStamped, callback=self.local_pose_cb)
        self.setpoint_xy_sub = rospy.Subscriber("setpoint_xy", PoseStamped, callback=self.setpoint_cb)

        # Publishers
        self.local_pos_pub = rospy.Publisher("mavros/setpoint_position/local", PoseStamped, queue_size=10)
        # Creates publisher to send velocity command msg
        self.vel_pub = rospy.Publisher("/mavros/setpoint_velocity/cmd_vel", TwistStamped, queue_size=10)
        # Pub controller errors
        self.error_pub = rospy.Publisher("uav/controller/errors", ControllerErrors, queue_size=10)

        rospy.wait_for_service("/mavros/cmd/arming")
        self.arming_client = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)

        rospy.wait_for_service("/mavros/set_mode")
        self.set_mode_client = rospy.ServiceProxy("mavros/set_mode", SetMode)

        self.setpoint = None  # None when no value has been set yet, and PoseStamped for real values -> landing coord
        self.local_pose = None  # None when no value has been received yet, and PoseStamped for real values
        self.current_state = None  # None when no value has been received yet, and State for real values; has UAV mode, e.g. offboard
        self.previous_time = 0  # Time since last controller update

        # Init 2D controllers
        self.vx_controller = PIController(Kp=1, Ki=0, sat_i_min=-0.05, sat_i_max=0.05, out_min=-0.5, out_max=0.5,
                                          setpoint=0)
        self.vy_controller = PIController(Kp=1, Ki=0, sat_i_min=-0.05, sat_i_max=0.05, out_min=-0.5, out_max=0.5,
                                          setpoint=0)

        # Init 3D controller
        self.vz_controller = VzController(Kp=2, out_min=-0.3, out_max=0.3, setpoint_x=0, setpoint_y=0, setpoint_z=0,
                                          v_particle=-0.2, E=0.5, alt_min=0.2)

        # Setpoint publishing MUST be faster than 2Hz
        self.rate = rospy.Rate(20)

        self.previous_time = rospy.get_time()

    def mode_cb(self, msg):
        # print(msg, type(msg), dir(msg), msg.data, type(msg.data))
        mode_int = msg.data
        if mode_int < 0 or mode_int > len(UAVController.valid_modes) - 1:
            rospy.logerr(
                f"mode integer received {msg} is not valid. Must be in interval [0,{len(UAVController.valid_modes) - 1}]")
        mode_name = UAVController.valid_modes[mode_int]
        self.change_mode(mode_name)

    def change_mode(self, new_mode):
        if new_mode not in UAVController.valid_modes:
            rospy.logerr(f"Invalid mode {new_mode}")
        else:
            rospy.loginfo(f"new mode: {new_mode}")
            self.mode = new_mode

    def state_cb(self, msg):
        self.current_state = msg

    def local_pose_cb(self, msg):
        self.local_pose = msg

    def setpoint_cb(self, msg):
        self.setpoint = msg
        self.vx_controller.setpoint = msg.pose.position.x
        self.vy_controller.setpoint = msg.pose.position.y

        self.vz_controller.setpoint_x = msg.pose.position.x
        self.vz_controller.setpoint_y = msg.pose.position.y
        self.vz_controller.setpoint_z = msg.pose.position.z

    def setup(self):
        # change to offboard mode
        # wait for first state to arrive
        rospy.loginfo("wait for first state to arrive")
        while not rospy.is_shutdown() and self.current_state is None:
            self.rate.sleep()

        # Wait for Flight Controller connection
        rospy.loginfo("waiting for Flight Controller connection")
        while not rospy.is_shutdown() and not self.current_state.connected:
            self.rate.sleep()

    def _ensure_offboard(self):
        if not self.current_state.armed:
            arm_cmd = CommandBoolRequest()
            arm_cmd.value = True

            last_req = rospy.Time.now()
            while not rospy.is_shutdown() and not self.current_state.armed:
                rospy.loginfo("waiting for armed")
                if rospy.Time.now() - last_req > rospy.Duration(5.0):
                    self.arming_client.call(arm_cmd)
                    last_req = rospy.Time.now()
                self.rate.sleep()

            rospy.loginfo("vehicle armed")

        if self.current_state.mode != "OFFBOARD":

            pose = self.local_pose

            for i in range(100):
                if rospy.is_shutdown():
                    break

            self.local_pos_pub.publish(pose)
            self.rate.sleep()

            rospy.loginfo("waiting for OFFBOARD")
            offb_set_mode = SetModeRequest()
            offb_set_mode.custom_mode = 'OFFBOARD'
            last_req = rospy.Time.now()
            while not rospy.is_shutdown() and self.current_state.mode != "OFFBOARD":
                if rospy.Time.now() - last_req > rospy.Duration(5.0):
                    if self.set_mode_client.call(offb_set_mode).mode_sent == True:
                        rospy.loginfo("sending OFFBOARD commands")
                    last_req = rospy.Time.now()
                self.rate.sleep()
            rospy.loginfo("OFFBOARD enabled")

    def run(self):
        rospy.loginfo(f"starting operation | current mode: {self.mode}")
        while not rospy.is_shutdown():

            if self.mode == 'POSITION':
                self.run_position()
            elif self.mode == 'LANDING':
                self.run_landing()

            self.rate.sleep()

    def run_position(self):
        return None

    def run_landing(self):
        self._ensure_offboard()

        delta_t = 0

        # Wait for first setpoint
        rospy.loginfo("waiting for first setpoint")
        if self.setpoint is None:
            rospy.logerr("no landing coord has been received yet")
            return None

        delta_t = rospy.get_time() - self.previous_time
        self.previous_time = rospy.get_time()

        print('Time is', delta_t)
        # UAV Position
        print('Position_X is', self.local_pose.pose.position.x)
        print('Position_Y is', self.local_pose.pose.position.y)
        print('Position_Z is', self.local_pose.pose.position.z)

        vel_x, e_x, pos_x = self.vx_controller.compute(self.local_pose.pose.position.x, delta_t)
        vel_y, e_y, pos_y = self.vy_controller.compute(self.local_pose.pose.position.y, delta_t)
        vel_z, e_z, pos_z = self.vz_controller.compute(self.local_pose.pose.position.x, self.local_pose.pose.position.y,
                                                       self.local_pose.pose.position.z, delta_t)

        # print('setpoint', self.setpoint)
        print('vx_commanded', vel_x)
        print('vy_commanded', vel_y)
        print('vz_commanded', vel_z)
        print('-----------------------------------')

        vel = TwistStamped()
        # vel_header_stamp = rospy.get_time()
        # timestamp = datetime.fromtimestamp(vel_header_stamp).strftime('%Y-%m-%d %H:%M:%S')
        # vel_header_stamp = "Twiststamp"  " | " + timestamp

        vel.twist.linear.x = vel_x
        vel.twist.linear.y = vel_y
        vel.twist.linear.z = vel_z

        self.vel_pub.publish(vel)

        controller_variables = ControllerErrors()
        controller_variables.e_x = e_x
        controller_variables.e_y = e_y
        controller_variables.e_z = e_z

        controller_variables.v_x_cmd = vel_x
        controller_variables.v_y_cmd = vel_y
        controller_variables.v_z_cmd = vel_z

        controller_variables.pos_x = pos_x
        controller_variables.pos_y = pos_y
        controller_variables.pos_z = pos_z

        self.error_pub.publish(controller_variables)


def main_new():
    uav_ctrl = UAVController()
    uav_ctrl.setup()
    uav_ctrl.run()


if __name__ == "__main__":
    main_new()
