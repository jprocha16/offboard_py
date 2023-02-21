import math
from bagpy import bagreader
import pandas as pd
import seaborn as sea
import matplotlib.pyplot as plt
import numpy as np
from functools import reduce


bag_fn = r'/home/ciafa/2023_02_19__17_57_53.bag'
topics = [
    '/mavros/local_position/pose',
    '/uav/marker/position',
    '/uav/fused_pose',
    '/uav/controller/errors',
    '/uav/ground_truth',
    '/uav/marker/arucos',
    ]

# Give filename of rosbag
b = bagreader(bag_fn)

# load messages to dataframes
dataframes = {}

for topic in topics:
    data = b.message_by_topic(topic)
    print("File saved: {}".format(data))
    print(type(data))
    df = pd.read_csv(data)
    dataframes[topic] = df

# rename all position cols
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

# init fig
plt.figure(figsize=(18, 8))

# PLOT 1 - X axis
plt.subplot(2, 3, 1)
# gps position
plt.plot(dataframes['/mavros/local_position/pose'].Time, dataframes['/mavros/local_position/pose'].x,
         label="PX4 position")
# vision position
plt.plot(dataframes['/uav/marker/position'].Time, dataframes['/uav/marker/position'].x, label="Vision position")
# fusion position
plt.plot(dataframes['/uav/fused_pose'].Time, dataframes['/uav/fused_pose'].x, label="Fused position")
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
plt.plot(dataframes['/uav/marker/position'].Time, dataframes['/uav/marker/position'].y, label="Vision position")
# fusion position
plt.plot(dataframes['/uav/fused_pose'].Time, dataframes['/uav/fused_pose'].y, label="Fused position")
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
plt.plot(dataframes['/uav/marker/position'].Time, dataframes['/uav/marker/position'].z, label="Vision position")
# fusion position
plt.plot(dataframes['/uav/fused_pose'].Time, dataframes['/uav/fused_pose'].z, label="Fused position")
# # ground truth position
plt.plot(dataframes['/uav/ground_truth'].Time, dataframes['/uav/ground_truth'].z, label="Ground truth")
plt.xlabel('Time')
plt.ylabel('Z_Axis Position')
plt.legend()
plt.grid()

# PLOT 4 - Controller telemetry
plt.subplot(2, 3, 4)
plt.plot(dataframes['/uav/controller/errors'].Time, dataframes['/uav/controller/errors'].v_x_cmd, label="Vx cmd")
plt.plot(dataframes['/uav/controller/errors'].Time, dataframes['/uav/controller/errors'].v_y_cmd, label="Vy cmd")
plt.plot(dataframes['/uav/controller/errors'].Time, dataframes['/uav/controller/errors'].v_z_cmd, label="Vz cmd")

plt.xlabel('Time')
plt.ylabel('Commanded Velocities')
plt.legend()
plt.grid()

# PLOT 5 - Controller telemetry
plt.subplot(2, 3, 5)
plt.plot(dataframes['/uav/marker/arucos'].Time, dataframes['/uav/marker/arucos'].id, "r.")

plt.xlabel('Time')
plt.ylabel('Ids')
# plt.legend()
plt.grid()

# PLOT 6
# copy dataframes
topics2 = [
    '/mavros/local_position/pose',
    '/uav/marker/position',
    '/uav/fused_pose',
    '/uav/ground_truth',
    ]
dfs_for_errors = {}
for topic in topics2:
    dfs_for_errors[topic] = dataframes[topic].copy()


prefixes = {
    '/mavros/local_position/pose': 'px4_',
    '/uav/marker/position': 'marker_',
    '/uav/fused_pose': 'fused_',
}

# filter all time steps before the first aruco detection
t_start = dfs_for_errors['/uav/marker/position'].Time.values[0]
for t in topics2:
    dfs_for_errors[t] = dfs_for_errors[t][dfs_for_errors[t].Time > t_start]
    dfs_for_errors[t]["Time"] = pd.to_timedelta(dfs_for_errors[t]["Time"], "sec")
    dfs_for_errors[t] = dfs_for_errors[t][["Time", "x", "y", "z"]]
    dfs_for_errors[t].set_index("Time", inplace=True)
    dfs_for_errors[t] = dfs_for_errors[t].add_prefix(prefixes.get(t, ""))

# merge all dataframes
df_merged = reduce(lambda  left,right: left.join(right, how='outer'), dfs_for_errors.values())
df_merged = df_merged.interpolate(axis=0).dropna()

# compute errors
for p in prefixes.values():
    df_merged[p+"error"] = ((df_merged[p+"x"] - df_merged["x"]) ** 2 + (df_merged[p+"y"] - df_merged["y"]) ** 2 +
                            (df_merged[p+"z"] - df_merged["z"]) ** 2) ** 0.5
print(df_merged)

px4_mse = df_merged["px4_error"].mean()
vision_mse = df_merged["marker_error"].mean()
fusion_mse = df_merged["fused_error"].mean()

print('PX4 MSE = ', px4_mse)
print('Vision MSE = ', vision_mse)
print('Fusion MSE = ', fusion_mse)

plt.subplot(2, 3, 6)
# df_merged[[p+"error" for p in prefixes.values()]].plot()
plt.plot(df_merged[[p+"error" for p in prefixes.values()]])
# plt.title("errors")
plt.xlabel('Time')
plt.ylabel('Errors')
# plt.legend()
plt.grid()

plt.show()
