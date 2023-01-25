import bagpy
from bagpy import bagreader
import pandas as pd
import seaborn as sea
import matplotlib.pyplot as plt
import numpy as np

# Give filename of rosbag
b = bagreader('/home/ciafa/figure_5.bag')

print(b.topic_table)

csvfiles = []
for t in b.topics:
    data = b.message_by_topic(t)
    csvfiles.append(data)

data = b.message_by_topic('/uav/controller/errors')
print("File saved: {}".format(data))

df = pd.read_csv(data)

# Plot 1 - Position
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

# Plot 2 - Velocity cmds
plt.subplot(2, 2, 3)

plt.plot(df.Time, df.v_x_cmd, label="Vx")
plt.plot(df.Time, df.v_y_cmd, label="Vy")
plt.plot(df.Time, df.v_z_cmd, label="Vz")

plt.xlabel('Time')
plt.ylabel('Velocity')

plt.legend()
plt.grid()

plt.show()
