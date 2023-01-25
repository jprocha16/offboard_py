#!/usr/bin/env python3

"""
 * File: offb_node.py
 * Stack and tested in Gazebo 9 SITL
"""


import rospy
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest


# import vx_control
# import vy_control


current_state = State()


def state_cb(msg):
    global current_state
    current_state = msg


def local_pose_cb(msg):
    global local_pose
    local_pose = msg
    #print(local_pose.pose.position.x)


def process_new_setpoint_cb(msg):
    global setpoint_xy
    setpoint_xy = msg


def compute_time():
    global previous_time, delta_t
    delta_t = rospy.get_time() - previous_time
    previous_time = rospy.get_time()


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
        # Defines the coefficient for the integral and proportional terms
        ki = 0.05
        kp = 0.3

        # X error calculation
        x_error = self.setpoint - x

        # Integral Calculation
        self.integral_v = self.integral_v_previous + (self.Ki * x_error * delta_t)

        # Integral should be between -0.3 and 0.3
        if self.integral_v  < self.sat_i_min:  # saturação do integral
            self.integral_v  = self.sat_i_min
        elif self.integral_v  > self.sat_i_max:
            self.integral_v  = self.sat_i_max

        self.integral_v_previous = self.integral_v

        # V final value calculation
        v = self.integral_v + kp * x_error

        # V interval between vx_min = -1 m/s and vx_max = 1 m/s
        if v < self.out_min:
            v = self.out_min
        elif v > self.out_max:
            v = self.out_max

        return v

def compute_vx(x, delta_t, setpoint):

    global integral_vx_previous

    # Defines the coefficient for the integral and proportional terms
    ki = 0.05
    kp = 0.3

    x_setpoint = setpoint.pose.position.x

    # X error calculation
    x_error = x_setpoint - x

    # Integral Calculation
    integral_vx = integral_vx_previous + (ki * x_error * delta_t)

    # Integral should be between -0.3 and 0.3
    if integral_vx < -0.3:      # saturação do integral
        integral_vx = -0.3
    elif integral_vx > 0.3:
        integral_vx = 0.3
    else:
        integral_vx

    integral_vx_previous = integral_vx

    # Vx final value calculation
    vx = integral_vx + kp * x_error

    # Vx interval between vx_min = -1 m/s and vx_max = 1 m/s
    if vx < -1:
        vx = -1
    elif vx > 1:
        vx = 1
    else:
        vx

    return vx


def compute_vy(y, delta_t, setpoint):

    global integral_vy_previous

    # Defines the coefficient for the integral and proportional terms
    ki = 0.05
    kp = 0.2

    y_setpoint = setpoint.pose.position.y

    # Y error calculation
    y_error = y_setpoint - y

    # Integral Calculation
    integral_vy = integral_vy_previous + (ki * y_error * delta_t)

    # Integral should be between -0.3 and 0.3
    if integral_vy < -0.3:
        integral_vy = -0.3
    elif integral_vy > 0.3:
        integral_vy = 0.3
    else:
        integral_vy

    integral_vy_previous = integral_vy

    # Vy final value calculation
    vy = integral_vy + kp * y_error

    # Vy interval between vy_min = -1 m/s and vy_max = 1 m/s
    if vy < -1:
        vy = -1
    elif vy > 1:
        vy = 1
    else:
        vy

    return vy


def compute_vz(z, delta_t, setpoint):

    global integral_vz_previous

    # Defines the coefficient for the integral and proportional terms
    ki = 0.05
    kp = 0.2

    z_setpoint = setpoint.pose.position.z

    # Altitude error calculation
    z_error = z_setpoint - z

    # Integral Calculation
    integral_vz = integral_vz_previous + (ki * z_error * delta_t)

    # Integral should be between -0.3 and 0.3
    if integral_vz < -0.3:
        integral_vz = -0.3
    elif integral_vz > 0.3:
        integral_vz = 0.3
    else:
        integral_vz

    integral_vz_previous = integral_vz

    # Vz final value calculation
    vz = integral_vz + kp * z_error

    # Vz interval between vz_min = -1 m/s and vz_max = 1 m/s
    if vz < -1:
        vz = -1
    elif vz > 1:
        vz = 1
    else:
        vz

    return vz


if __name__ == "__main__":
    rospy.init_node("offb_node_py")

    state_sub = rospy.Subscriber("mavros/state", State, callback=state_cb)

    local_pose_sub = rospy.Subscriber("mavros/local_position/pose", PoseStamped, callback=local_pose_cb)

    setpoint_xy_sub = rospy.Subscriber("setpoint_xy", PoseStamped, callback=process_new_setpoint_cb)

    #local_pos_pub = rospy.Publisher("mavros/setpoint_position/local", PoseStamped, queue_size=10)
    
    # Creates publisher to send velocity command msg
    vel_pub = rospy.Publisher("/mavros/setpoint_velocity/cmd_vel", TwistStamped, queue_size=10) 

    rospy.wait_for_service("/mavros/cmd/arming")
    arming_client = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)
    
    rospy.wait_for_service("/mavros/set_mode")
    set_mode_client = rospy.ServiceProxy("mavros/set_mode", SetMode)


    #global integral_vx_previous
    integral_vx_previous = 0.0

    #global integral_vy_previous
    integral_vy_previous = 0.0

    #global integral_vz_previous
    integral_vz_previous = 0.0


    previous_time = rospy.get_time()
    delta_t=0


    # Setpoint publishing MUST be faster than 2Hz
    rate = rospy.Rate(20)
    
    # Wait for Flight Controller connection
    while (not rospy.is_shutdown() and not current_state.connected):
        rate.sleep()


    local_pose = PoseStamped()

    setpoint_xy = PoseStamped()

    setpoint_xy.pose.position.x = 0
    setpoint_xy.pose.position.y = 0
    setpoint_xy.pose.position.z = 2
    

    offb_set_mode = SetModeRequest()
    offb_set_mode.custom_mode = 'OFFBOARD'

    arm_cmd = CommandBoolRequest()
    arm_cmd.value = True

    last_req = rospy.Time.now()

    while (not rospy.is_shutdown()):
        if (current_state.mode != "OFFBOARD" and (rospy.Time.now() - last_req) > rospy.Duration(5.0)):
            if (set_mode_client.call(offb_set_mode).mode_sent == True):
                rospy.loginfo("OFFBOARD enabled")

            last_req = rospy.Time.now()
        else:
            if (not current_state.armed and (rospy.Time.now() - last_req) > rospy.Duration(5.0)):
                if (arming_client.call(arm_cmd).success == True):
                    rospy.loginfo("Vehicle armed")

                last_req = rospy.Time.now()

        compute_time()
        print('Time is', delta_t)
        print('INTEGRAL_X is', integral_vx_previous)
        print('INTEGRAL_Y is', integral_vy_previous)
        print('INTEGRAL_Z is', integral_vz_previous)

        vel_x = compute_vx(local_pose.pose.position.x, delta_t, setpoint_xy)
        vel_y = compute_vy(local_pose.pose.position.y, delta_t, setpoint_xy)
        vel_z = compute_vz(local_pose.pose.position.z, delta_t, setpoint_xy)

        print('vx_commanded', vel_x)
        print('vy_commanded', vel_y)
        print('vz_commanded', vel_z)
        print('------------------------------------')

        vel = TwistStamped()

        vel.twist.linear.x = vel_x
        vel.twist.linear.y = vel_y
        vel.twist.linear.z = vel_z


        vel_pub.publish(vel)

        rate.sleep()
