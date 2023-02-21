import math
from bagpy import bagreader
import pandas as pd
import seaborn as sea
import matplotlib.pyplot as plt
import numpy as np

# Give filename of rosbag
b = bagreader('/home/ciafa/2023_02_16__09_56_13.bag')
dataframes = {}
topics = ['/mavros/local_position/pose', '/uav/marker/position', '/uav/marker/arucos', '/uav/controller/errors',
          '/uav/fused_pose', '/uav/ground_truth']
for topic in topics:
    data = b.message_by_topic(topic)
    print("File saved: {}".format(data))
    print(type(data))

    df = pd.read_csv(data)
    dataframes[topic] = df

dataframes['/mavros/local_position/pose'].rename(columns={
    'pose.position.x': 'x',
    'pose.position.y': 'y',
    'pose.position.z': 'z'
}, inplace=True)

dataframes['/uav/fused_pose'].rename(columns={
    'pose.position.x': 'x',
    'pose.position.y': 'y',
    'pose.position.z': 'z'
}, inplace=True)

dataframes['/uav/marker/position'].rename(columns={
    'x': 'x',
    'y': 'y',
    'z': 'z'
}, inplace=True)

dataframes['/uav/ground_truth'].rename(columns={
    'x': 'x',
    'y': 'y',
    'z': 'z'
}, inplace=True)

# # convert all Time to timedeltas
# sample_period = "100ms"
# for topic in topics:
#     dataframes[topic]["Time"] = pd.to_timedelta(dataframes[topic]["Time"], "sec")
#     dataframes[topic].set_index("Time", inplace=True)
#     dataframes[topic] = dataframes[topic].resample(sample_period).max()
#
# #     if topic == '/mavros/local_position/pose':
# #         dataframes[topic]["Time"] = pd.to_timedelta(dataframes[topic]["Time"], "sec")
# #         dataframes[topic].set_index("Time", inplace=True)
# #         dataframes[topic] = dataframes[topic].resample(sample_period, offset="-2ms").max()
# #
# #     if topic == '/uav/marker/position':
# #         dataframes[topic]["Time"] = pd.to_timedelta(dataframes[topic]["Time"], "sec")
# #         dataframes[topic].set_index("Time", inplace=True)
# #         dataframes[topic] = dataframes[topic].resample(sample_period, offset="+4ms").max()
# #
# #     if topic == '/uav/fused_pose':
# #         dataframes[topic]["Time"] = pd.to_timedelta(dataframes[topic]["Time"], "sec")
# #         dataframes[topic].set_index("Time", inplace=True)
# #         dataframes[topic] = dataframes[topic].resample(sample_period).max()
# #
# #     if topic == '/uav/ground_truth':
# #         dataframes[topic]["Time"] = pd.to_timedelta(dataframes[topic]["Time"], "sec")
# #         dataframes[topic].set_index("Time", inplace=True)
# #         dataframes[topic] = dataframes[topic].resample(sample_period, offset="+2ms").max()
# #
# df_combined = pd.concat([dataframes['/mavros/local_position/pose'].add_prefix("px4_"),
#                          dataframes['/uav/marker/position'].add_prefix("marker_"),
#                          dataframes['/uav/fused_pose'].add_prefix("fused_"),
#                          dataframes['/uav/ground_truth'].add_prefix("truth_")],
#                         axis=1).ffill().bfill()
# print(df_combined)

# plot_errors_cols = ['px4_', 'marker_', 'fused_']
# plt.figure(figsize=(18, 8))

# for i, col in enumerate(plot_errors_cols, start=1):
#     plt.subplot(2, 2, i)
#     err = ((df_combined[col + 'x'] - df_combined.truth_x) ** 2 + (df_combined[col + 'y'] - df_combined.truth_y)**2 +
#            (df_combined[col + 'z'] - df_combined.truth_z)**2) ** 0.5
#     plt.plot(err.index, err.values)

# err_px4 = ((df_combined.truth_x - df_combined.px4_x) ** 2 + (df_combined.truth_y - df_combined.px4_y) ** 2 +
#            (df_combined.truth_z - df_combined.px4_z) ** 2) ** 0.5
# err_vision = ((df_combined.truth_x - df_combined.marker_x) ** 2 + (df_combined.truth_y - df_combined.marker_y) ** 2 +
#               (df_combined.truth_z - df_combined.marker_z) ** 2) ** 0.5
# err_fused = ((df_combined.truth_x - df_combined.fused_x) ** 2 + (df_combined.truth_y - df_combined.fused_y) ** 2 +
#              (df_combined.truth_z - df_combined.fused_z) ** 2) ** 0.5

# plt.plot(err_px4.index, err_px4.values, label="PX4 error")
# plt.plot(err_vision.index, err_vision.values, label="Vision error")
# plt.plot(err_fused.index, err_fused.values, label="Fusion error")

# plt.xlabel('Time')
# plt.ylabel('Error')
# plt.legend()
# plt.grid()
# plt.show()


plt.figure(figsize=(18, 8))

# PLOT 1 - X axis
plt.subplot(2, 3, 1)

# gps position
plt.plot(dataframes['/mavros/local_position/pose'].Time, dataframes['/mavros/local_position/pose'].x,
         label="PX4 position")
# vision position
plt.plot(dataframes['/uav/marker/position'].Time, dataframes['/uav/marker/position'].x, 'b.', label="Vision position")
# fusion position
plt.plot(dataframes['/uav/fused_pose'].Time, dataframes['/uav/fused_pose'].x, 'r.', label="Fused position")
# # ground truth position
plt.plot(dataframes['/uav/ground_truth'].Time, dataframes['/uav/ground_truth'].x, label="Ground truth")

plt.xlabel('Time')
plt.ylabel('X_Axis Position')
plt.legend()
plt.grid()

# PLOT 2 - Y axis
plt.subplot(2, 3, 2)

# gps position
plt.plot(dataframes['/mavros/local_position/pose'].Time, dataframes['/mavros/local_position/pose'].y,
         label="PX4 position")
# vision position
plt.plot(dataframes['/uav/marker/position'].Time, dataframes['/uav/marker/position'].y, 'b.', label="Vision position")
# fusion position
plt.plot(dataframes['/uav/fused_pose'].Time, dataframes['/uav/fused_pose'].y, 'r.', label="Fused position")
# # ground truth position
plt.plot(dataframes['/uav/ground_truth'].Time, dataframes['/uav/ground_truth'].y, label="Ground truth")

plt.xlabel('Time')
plt.ylabel('Y_Axis Position')
plt.legend()
plt.grid()

# PLOT 3 - Z axis
plt.subplot(2, 3, 3)

# gps position
plt.plot(dataframes['/mavros/local_position/pose'].Time, dataframes['/mavros/local_position/pose'].z,
         label="PX4 position")
# vision position
plt.plot(dataframes['/uav/marker/position'].Time, dataframes['/uav/marker/position'].z, 'b.', label="Vision position")
# fusion position
plt.plot(dataframes['/uav/fused_pose'].Time, dataframes['/uav/fused_pose'].z, 'r.', label="Fused position")
# # ground truth position
plt.plot(dataframes['/uav/ground_truth'].Time, dataframes['/uav/ground_truth'].z, label="Ground truth")

plt.xlabel('Time')
plt.ylabel('Z_Axis Position')
plt.legend()
plt.grid()


# # PLOT 4 - Controller telemetry
# plt.subplot(2, 3, 4)
#
# plt.plot(dataframes['/uav/controller/errors'].Time, dataframes['/uav/controller/errors'].v_x_cmd, label="Vx cmd")
# plt.plot(dataframes['/uav/controller/errors'].Time, dataframes['/uav/controller/errors'].v_y_cmd, label="Vy cmd")
# plt.plot(dataframes['/uav/controller/errors'].Time, dataframes['/uav/controller/errors'].v_z_cmd, label="Vz cmd")
#
# plt.xlabel('Time')
# plt.ylabel('Commanded Velocities')
# plt.legend()
# plt.grid()

# PLOT 5 - Position errors
# plt.subplot(2, 3, 5)

# px_x = np.power(dataframes['/uav/ground_truth'].x - dataframes['/mavros/local_position/pose']['pose.position.x'], 2)
# px_y = pow(dataframes['/uav/ground_truth'].y - dataframes['/mavros/local_position/pose']['pose.position.y'], 2)
# px_z = pow(dataframes['/uav/ground_truth'].z - dataframes['/mavros/local_position/pose']['pose.position.z'], 2)

# vis_x = pow(dataframes['/uav/ground_truth'].x - dataframes['/uav/marker/position'].x, 2)
# vis_y = pow(dataframes['/uav/ground_truth'].y - dataframes['/uav/marker/position'].y, 2)
# vis_z = pow(dataframes['/uav/ground_truth'].z - dataframes['/uav/marker/position'].z, 2)

# fus_x = pow(dataframes['/uav/ground_truth'].x - dataframes['/uav/controller/errors'].pos_x, 2)
# fus_y = pow(dataframes['/uav/ground_truth'].y - dataframes['/uav/controller/errors'].pos_y, 2)
# fus_z = pow(dataframes['/uav/ground_truth'].z - dataframes['/uav/controller/errors'].pos_z, 2)

# plt.plot(dataframes['/uav/ground_truth'].Time, np.sqrt(px_x + px_y + px_z), label="PX4 error")
# plt.plot(dataframes['/uav/ground_truth'].Time, int(math.sqrt(vis_x + vis_y + vis_z)), label="Vision error")
# plt.plot(dataframes['/uav/ground_truth'].Time, int(math.sqrt(fus_x + fus_y + fus_z)), label="Fusion error")


# plt.xlabel('Time')
# plt.ylabel('Position Errors')
# plt.legend()
# plt.grid()

plt.show()
