import bagpy
from bagpy import bagreader
import pandas as pd
import seaborn as sea
import matplotlib.pyplot as plt
import numpy as np

# # Give filename of rosbag
# b = bagreader('/home/ciafa/2023_02_06__15_01_49.bag')
#
# print(b.topic_table)
#
# csvfiles = []
# for t in b.topics:
#     data = b.message_by_topic(t)
#     csvfiles.append(data)
#
# data = b.message_by_topic('/uav/controller/errors')
# print("File saved: {}".format(data))
#
# df = pd.read_csv(data)
#
# # Plot 1 - Position
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
#
# # Plot 2 - Error
# plt.subplot(2, 2, 2)
#
# plt.plot(df.Time, df.e_x, label="Error_X")
# plt.plot(df.Time, df.e_y, label="Error_Y")
# plt.plot(df.Time, df.e_z, label="Error_Z")
#
# plt.xlabel('Time')
# plt.ylabel('Error')
#
# plt.legend()
# plt.grid()
#
# # Plot 2 - Velocity cmds
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
#
# plt.show()

b = bagreader('/home/ciafa/2023_02_11__16_32_25.bag')
dataframes = {}
topics = ['/mavros/local_position/pose', '/uav/marker/position', '/uav/controller/errors', '/uav/ground_truth']
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
plt.plot(df['/mavros/local_position/pose'].Time, df['/mavros/local_position/pose'].pose.position.x,
         label="PX4 position")
# vision position
plt.plot(df['/uav/marker/position'].Time, df['/uav/marker/position'].x, label="Vision position")
# fusion position
plt.plot(df['/uav/controller/errors'].Time, df['/uav/controller/errors'].pos_x, label="Fused position")
# ground truth position
plt.plot(df['/uav/ground_truth'].Time, df['/uav/ground_truth'].x, label="Ground truth")

plt.xlabel('Time')
plt.ylabel('X_Axis Position')
plt.legend()
plt.grid()

# PLOT 2 - Y axis
plt.subplot(2, 2, 2)

# gps position
plt.plot(df['/mavros/local_position/pose'].Time, df['/mavros/local_position/pose'].pose.position.y,
         label="PX4 position")
# vision position
plt.plot(df['/uav/marker/position'].Time, df['/uav/marker/position'].y, label="Vision position")
# fusion position
plt.plot(df['/uav/controller/errors'].Time, df['/uav/controller/errors'].pos_y, label="Fused position")
# ground truth position
plt.plot(df['/uav/ground_truth'].Time, df['/uav/ground_truth'].y, label="Ground truth")

plt.xlabel('Time')
plt.ylabel('Y_Axis Position')
plt.legend()
plt.grid()

# PLOT 3 - Z axis
plt.subplot(2, 2, 3)
# gps position
plt.plot(df['/mavros/local_position/pose'].Time, df['/mavros/local_position/pose'].pose.position.z,
         label="PX4 position")
# vision position
plt.plot(df['/uav/marker/position'].Time, df['/uav/marker/position'].z, label="Vision position")
# fusion position
plt.plot(df['/uav/controller/errors'].Time, df['/uav/controller/errors'].pos_z, label="Fused position")
# ground truth position
plt.plot(df['/uav/ground_truth'].Time, df['/uav/ground_truth'].z, label="Ground truth")

plt.xlabel('Time')
plt.ylabel('Z_Axis Position')
plt.legend()
plt.grid()

# plt.savefig(bag_fn_basename + ".png")
# shutil.rmtree(bag_fn_basename)

plt.show()