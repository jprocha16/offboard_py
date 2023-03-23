import subprocess as sp
import time
import datetime
import os
import shutil
import signal

# imports for plots
import bagpy
from bagpy import bagreader
import pandas as pd
import seaborn as sea
import matplotlib.pyplot as plt
import numpy as np

TIME_WAIT_OFFBOARD = 12
TIME_WAIT_GO_AWAY = 15
TIME_WAIT_LANDING = 120


def create_plots(bag_fn_basename):
    # Give filename of rosbag
    b = bagreader(bag_fn_basename + ".bag")

    print(b.topic_table)

    for t in b.topics:
        data = b.message_by_topic(t)

    data = b.message_by_topic('/uav/controller/errors')
    print("File saved: {}".format(data))
    print(type(data))

    df = pd.read_csv(data)

    # # Plot 1 - Position
    # plt.figure(figsize=(18,8))
    # plt.subplot(2, 2, 1)
    #
    # plt.plot(df.Time, df.pos_x, label="Position_X")
    # plt.plot(df.Time, df.pos_y, label="Position_Y")
    # plt.plot(df.Time, df.pos_z, label="Position_Z")
    #
    # plt.xlabel('Time')
    # plt.ylabel('Position')
    #
    # plt.legend()
    # plt.grid()

    # Plot 2 - Error
    plt.subplot(1, 2, 1)

    plt.plot(df.Time, df.e_x, label="X_axis Error")
    plt.plot(df.Time, df.e_y, label="Y_axis Error")

    plt.xlabel('Time')
    plt.ylabel('Error')

    plt.legend()
    plt.grid()

    # Plot 2 - Error
    plt.subplot(1, 2, 2)

    plt.plot(df.Time, df.e_z, label="Z_axis Error")

    plt.xlabel('Time')
    plt.ylabel('Error')

    plt.legend()
    plt.grid()

    # # Plot 3 - Velocity cmds
    # plt.subplot(2, 2, 3)
    #
    # plt.plot(df.Time, df.v_x_cmd, label="Vx")
    # plt.plot(df.Time, df.v_y_cmd, label="Vy")
    # plt.plot(df.Time, df.v_z_cmd, label="Vz")
    #
    # plt.xlabel('Time')
    # plt.ylabel('Velocity')
    #
    # plt.legend()
    # plt.grid()

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
        "base_footprint"}, pose: {position: {x: 0, y: 0, z: 15}, orientation: {w: 1.0}}}'"""
    for i in range(2):
        sp.call(cmd_go_away, shell=True)

    # Wait until UAV reaches goal pose
    print("Waiting for destination")
    #   method 1: wait X s
    time.sleep(TIME_WAIT_GO_AWAY)
    #   abordagem 2: monitorizar telemetria e esperar até erro ser inferior a E

    # start rosbag record
    now = datetime.datetime.now()  # current date and time
    bag_fn_basename = now.strftime("%Y_%m_%d__%H_%M_%S")
    bag_fn = bag_fn_basename + '.bag'
    print(f"starting rosbag record, path:{bag_fn_basename}")
    p_record = sp.Popen(f"rosbag record /uav/controller/errors -O {bag_fn_basename}".split())

    # Send landing coord
    print("Sending landing coord")
    landing_pos_cmd = """rostopic pub -1 setpoint_xy geometry_msgs/PoseStamped '{header: {stamp: now, frame_id: 
        "base_footprint"}, pose: {position: {x: 0, y: 0, z: 5}, orientation: {w: 1.0}}}'"""
    sp.call(landing_pos_cmd, shell=True)

    # ativar modo landing
    print("changing to landing mode")
    change_mode_cmd = """rostopic pub -1 uav/mode std_msgs/UInt8 '1'"""
    sp.call(change_mode_cmd, shell=True)

    # esperar até chegar ao ponto de landing
    print("waiting for landing completion")
    #   abordagem 1: esperar X s
    time.sleep(TIME_WAIT_LANDING)

    # parar rosbag record
    print("stopping rosbag record")
    # p_record.kill()
    # Get the process id
    pid = p_record.pid
    os.kill(pid, signal.SIGINT)
    time.sleep(1)

    # # criar plots
    # print("creating plots from rosbag")
    # create_plots(bag_fn_basename)




    # # ativar modo landing
    # print("changing to landing(offboard mode)")
    # change_mode_cmd = """rostopic pub -1 uav/mode std_msgs/UInt8 '1'"""
    # sp.call(change_mode_cmd, shell=True)
    #
    # print("waiting for offboard complete")
    # time.sleep(TIME_WAIT_OFFBOARD)
    #
    # # enviar comando de deslocação para ponto longe de landing 2x
    # print("sending command go away")
    # cmd_go_away = """rostopic pub -1 mavros/setpoint_position/local geometry_msgs/PoseStamped '{header: {stamp: now, frame_id: "base_footprint"}, pose: {position: {x: 0, y: 0, z: 15}, orientation: {w: 1.0}}}'"""
    # for i in range(2):
    #     sp.call(cmd_go_away, shell=True)
    #
    # # esperar até chegar ao destino
    # print("waiting for destination")
    # #   abordagem 1: esperar X s
    # time.sleep(TIME_WAIT_GO_AWAY)  # segundos
    # #   abordagem 2: monitorizar telemetria e esperar até erro ser inferior a E
    #
    # # start rosbag record
    # now = datetime.datetime.now()  # current date and time
    # bag_fn_basename = now.strftime("%Y_%m_%d__%H_%M_%S")
    # bag_fn = bag_fn_basename + '.bag'
    # print(f"starting rosbag record, path:{bag_fn_basename}")
    # p_record = sp.Popen(f"rosbag record /uav/controller/errors -O {bag_fn_basename}".split())
    #
    # # enviar posição para landing
    # print("sending landing coord")
    # landing_pos_cmd = """rostopic pub -1 setpoint_xy geometry_msgs/PoseStamped '{header: {stamp: now, frame_id: "base_footprint"}, pose: {position: {x: 0, y: 0, z: 10}, orientation: {w: 1.0}}}'"""
    # sp.call(landing_pos_cmd, shell=True)
    #
    # # ativar modo landing
    # print("changing to landing mode")
    # change_mode_cmd = """rostopic pub -1 uav/mode std_msgs/UInt8 '1'"""
    # sp.call(change_mode_cmd, shell=True)
    #
    # # esperar até chegar ao ponto de landing
    # print("waiting for landing completion")
    # #   abordagem 1: esperar X s
    # time.sleep(TIME_WAIT_LANDING)
    # #   abordagem 2: monitorizar telemetria e esperar até erro ser inferior a E
    #
    # # parar rosbag record
    # print("stopping rosbag record")
    # #p_record.kill()
    # # Get the process id
    # pid = p_record.pid
    # os.kill(pid, signal.SIGINT)
    # time.sleep(1)




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
    #main_test_rosbag()
