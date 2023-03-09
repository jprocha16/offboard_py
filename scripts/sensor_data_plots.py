import subprocess as sp
import time
import datetime
import os
import shutil
import signal

# imports for plots
from bagpy import bagreader
import pandas as pd
import seaborn as sea
import matplotlib.pyplot as plt
import numpy as np

TIME_WAIT_OFFBOARD = 15
TIME_WAIT_GO_AWAY = 10
TIME_WAIT_GO_LAND_COORD = 15
TIME_WAIT_LANDING = 180

TOPICS_TO_SUBSCRIBE = [
    "/mavros/local_position/pose",
    "/uav/marker/position",
    "/uav/marker/arucos",
    "/uav/controller/errors",
    "/mavros/setpoint_velocity/cmd_vel",
    "/uav/fused_pose",
    "/uav/ground_truth",
    "/uav/state",
    ]


def create_plots_new(bag_fn_basename):
    # Give filename of rosbag
    b = bagreader(bag_fn_basename)
    dataframes = {}
    topics = ['/mavros/local_position/pose', '/uav/marker/position', '/uav/controller/errors', '/uav/fused_pose',
              '/uav/ground_truth']
    for topic in topics:
        data = b.message_by_topic(topic)
        print("File saved: {}".format(data))
        print(type(data))

        df = pd.read_csv(data)
        dataframes[topic] = df

    plt.figure(figsize=(18, 8))

    # PLOT 1 - X axis
    plt.subplot(2, 2, 1)

    # gps position
    plt.plot(dataframes['/mavros/local_position/pose'].Time,
             dataframes['/mavros/local_position/pose']['pose.position.x'], label="PX4 position")
    # vision position
    plt.plot(dataframes['/uav/marker/position'].Time, dataframes['/uav/marker/position'].x, label="Vision position")
    # fusion position
    plt.plot(dataframes['/uav/controller/errors'].Time, dataframes['/uav/controller/errors'].pos_x,
             label="Fused position")
    # # ground truth position
    plt.plot(dataframes['/uav/ground_truth'].Time, dataframes['/uav/ground_truth'].x, label="Ground truth")

    plt.xlabel('Time')
    plt.ylabel('X_Axis Position')
    plt.legend()
    plt.grid()

    # PLOT 2 - Y axis
    plt.subplot(2, 2, 2)

    # gps position
    plt.plot(dataframes['/mavros/local_position/pose'].Time,
             dataframes['/mavros/local_position/pose']['pose.position.y'],
             label="PX4 position")
    # vision position
    plt.plot(dataframes['/uav/marker/position'].Time, dataframes['/uav/marker/position'].y, label="Vision position")
    # fusion position
    plt.plot(dataframes['/uav/controller/errors'].Time, dataframes['/uav/controller/errors'].pos_y,
             label="Fused position")
    # # ground truth position
    plt.plot(dataframes['/uav/ground_truth'].Time, dataframes['/uav/ground_truth'].y, label="Ground truth")

    plt.xlabel('Time')
    plt.ylabel('Y_Axis Position')
    plt.legend()
    plt.grid()

    # PLOT 3 - Z axis
    plt.subplot(2, 2, 3)

    # gps position
    plt.plot(dataframes['/mavros/local_position/pose'].Time,
             dataframes['/mavros/local_position/pose']['pose.position.z'],
             label="PX4 position")
    # vision position
    plt.plot(dataframes['/uav/marker/position'].Time, dataframes['/uav/marker/position'].z, label="Vision position")
    # fusion position
    plt.plot(dataframes['/uav/controller/errors'].Time, dataframes['/uav/controller/errors'].pos_z,
             label="Fused position")
    # # ground truth position
    plt.plot(dataframes['/uav/ground_truth'].Time, dataframes['/uav/ground_truth'].z, label="Ground truth")

    plt.xlabel('Time')
    plt.ylabel('Z_Axis Position')
    plt.legend()
    plt.grid()

    # PLOT 4 - controller telemetry
    plt.subplot(2, 2, 4)

    plt.plot(dataframes['/uav/controller/errors'].Time, dataframes['/uav/controller/errors'].v_x_cmd, label="Vx cmd")
    plt.plot(dataframes['/uav/controller/errors'].Time, dataframes['/uav/controller/errors'].v_y_cmd, label="Vy cmd")
    plt.plot(dataframes['/uav/controller/errors'].Time, dataframes['/uav/controller/errors'].v_z_cmd, label="Vz cmd")

    plt.xlabel('Time')
    plt.ylabel('Cmd velocities')
    plt.legend()
    plt.grid()

    plt.savefig(bag_fn_basename + ".png")
    shutil.rmtree(bag_fn_basename)

    plt.show()


def create_plots_ziegler(bag_fn_basename):
    # Give filename of rosbag
    b = bagreader(bag_fn_basename + ".bag")

    print(b.topic_table)

    for t in b.topics:
        data = b.message_by_topic(t)

    data = b.message_by_topic('/uav/controller/errors')
    print("File saved: {}".format(data))
    print(type(data))

    df = pd.read_csv(data)

    # Plot 1 - Position
    plt.figure(figsize=(18, 8))
    plt.subplot(2, 2, 1)

    plt.plot(df.Time, df.pos_x, label="Position_X")
    plt.plot(df.Time, df.pos_y, label="Position_Y")
    plt.plot(df.Time, df.pos_z, label="Position_Z")

    plt.xlabel('Time')
    plt.ylabel('Position')

    plt.legend()
    plt.grid()

    # Plot 2 - Error
    plt.subplot(2, 2, 2)

    plt.plot(df.Time, df.e_x, label="Error_X")
    plt.plot(df.Time, df.e_y, label="Error_Y")
    plt.plot(df.Time, df.e_z, label="Error_Z")

    plt.xlabel('Time')
    plt.ylabel('Error')

    plt.legend()
    plt.grid()

    # Plot 3 - Velocity
    plt.subplot(2, 2, 3)

    plt.plot(df.Time, df.v_x_cmd, label="Vx")
    plt.plot(df.Time, df.v_y_cmd, label="Vy")
    plt.plot(df.Time, df.v_z_cmd, label="Vz")

    plt.xlabel('Time')
    plt.ylabel('Velocity')

    plt.legend()
    plt.grid()

    plt.savefig(bag_fn_basename + ".png")
    shutil.rmtree(bag_fn_basename)

    plt.show()


def main():
    # Init data_fusion_node
    print("Starting data fusion node")
    # data_fusion_cmd = """rosrun data_fusion_py kalman_filter.py"""
    # sp.call(data_fusion_cmd, shell=True)
    p_fusion = sp.Popen("rosrun data_fusion_py kalman_filter.py".split())

    # Init controller node
    print("Starting UAV controller node")
    # uav_ctrl_cmd = """rosrun offboard_py uav_controller.py"""
    # sp.call(uav_ctrl_cmd, shell=True)
    p_ctrl = sp.Popen("rosrun offboard_py uav_controller.py".split())

    print("Waiting for arming and offboard complete")
    time.sleep(TIME_WAIT_OFFBOARD)

    # Move UAV for goal pose
    print("Sending command go away")
    cmd_go_away = """rostopic pub -1 uav/goal_pose geometry_msgs/PoseStamped '{header: {stamp: now, frame_id: 
    "base_footprint"}, pose: {position: {x: 10, y: 10, z: 10}, orientation: {w: 1.0}}}'"""
    for i in range(2):
        sp.call(cmd_go_away, shell=True)

    # Wait until UAV reaches goal pose
    print("Waiting for destination")
    #   method 1: wait X s
    time.sleep(TIME_WAIT_GO_AWAY)
    #   abordagem 2: monitorizar telemetria e esperar até erro ser inferior a E

    # Send landing coord
    print("Sending landing coord")
    landing_pos_cmd = """rostopic pub -1 setpoint_xy geometry_msgs/PoseStamped '{header: {stamp: now, frame_id: 
    "base_footprint"}, pose: {position: {x: 0, y: 0, z: 10}, orientation: {w: 1.0}}}'"""
    sp.call(landing_pos_cmd, shell=True)

    # Move UAV to landing coord
    print("Sending UAV to landing coord")
    cmd_go_to_land_coord = """rostopic pub -1 uav/goal_pose geometry_msgs/PoseStamped '{header: {stamp: now, frame_id: 
        "base_footprint"}, pose: {position: {x: 0, y: 0, z: 10}, orientation: {w: 1.0}}}'"""
    sp.call(cmd_go_to_land_coord, shell=True)

    # Wait until UAV reaches land coord
    print("Waiting for reaching landing coord")
    #   method 1: wait X s
    time.sleep(TIME_WAIT_GO_LAND_COORD)
    #   abordagem 2: monitorizar telemetria e esperar até erro ser inferior a E

    # Init vision node
    print("Starting vision sensor")
    # landing_pos_cmd = """roslaunch landing_sensor_py landing_sensor.launch"""
    # sp.call(landing_pos_cmd, shell=True)
    p_vision = sp.Popen("roslaunch landing_sensor_py landing_sensor.launch".split())

    # Start rosbag record
    now = datetime.datetime.now()  # current date and time
    bag_fn_basename = now.strftime("%Y_%m_%d__%H_%M_%S")
    bag_fn = bag_fn_basename + '.bag'
    print(f"Starting rosbag record, path:{bag_fn_basename}")
    topic_lst = ' '.join(TOPICS_TO_SUBSCRIBE)
    p_record = sp.Popen(f"rosbag record {topic_lst} -O {bag_fn_basename}".split())
    # p_record = sp.Popen(f"rosbag record -a -O {bag_fn_basename}".split())

    # Change to landing mode
    print("Changing to landing mode")
    change_mode_cmd = """rostopic pub -1 uav/mode std_msgs/UInt8 '1'"""
    sp.call(change_mode_cmd, shell=True)

    # Wait until the UAV reaches the landing coord and starts landing
    print("Waiting for landing completion")
    #   Method 1: wait X s
    time.sleep(TIME_WAIT_LANDING)
    #   abordagem 2: monitorizar telemetria e esperar até erro ser inferior a E

    # ---STOP ALL PROCESSES---
    # Stop rosbag record
    print("Stopping rosbag record")
    # p_record.kill()
    # Get the process id
    pid = p_record.pid
    os.kill(pid, signal.SIGINT)
    time.sleep(1)

    # Stop UAV controller
    print("Stopping UAV controller")
    # Get the process id
    ctrl_pid = p_ctrl.pid
    os.kill(ctrl_pid, signal.SIGINT)
    time.sleep(1)

    # Stop vision node
    print("Stopping vision node")
    # Get the process id
    vision_pid = p_vision.pid
    os.kill(vision_pid, signal.SIGINT)
    time.sleep(1)

    # Stop data fusion node
    print("Stopping data fusion node")
    # Get the process id
    fusion_pid = p_fusion.pid
    os.kill(fusion_pid, signal.SIGINT)
    time.sleep(1)

    # Create plots (uncomment if plots are wanted)
    # print("create plots from rosbag")
    # create_plots_ziegler(bag_fn_basename)
    # create_plots_new(bag_fn_basename)


def main_test_rosbag():
    '''main apenas para testar a componente de iniciar e parar gravação de rosbag, e criar e mostrat plots'''
    # start rosbag record
    now = datetime.datetime.now()  # current date and time
    bag_fn_basename = now.strftime("%Y_%m_%d__%H_%M_%S")
    bag_fn = bag_fn_basename + '.bag'
    print(f"starting rosbag record, path:{bag_fn_basename}")
    p_record = sp.Popen(f"rosbag record /uav/controller/errors -O {bag_fn_basename}".split())

    time.sleep(5)

    # parar rosbag record
    print("stopping rosbag record")
    #p_record.kill()
    # Get the process id
    pid = p_record.pid
    os.kill(pid, signal.SIGINT)
    time.sleep(1)

    # criar plots
    print("create plots from rosbag")
    create_plots(bag_fn_basename)


if __name__ == '__main__':
    main()
    # main_test_rosbag()
