import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

# Import data using Pandas (install with: pip install pandas)
data = pd.read_csv("Code\data.csv")
print(data)

# Retrieve data columns
D = data.to_numpy();
t = D[:,0]
pan = D[:,1]
r_tof = D[:,2] + 7

# Create a basic plot
plt.plot(t,r_tof)
plt.legend(["tof"])
plt.ylabel("Range (mm)")
plt.xlabel("Time (s)")
plt.xlim([min(t),max(t)])
plt.savefig('basic.png') # Save plot to a file
plt.show()

# Process data -- convert to position
# Compute angle around z-axis (polar angle)
theta = pan*np.pi/180

# Convert to cartesian
x_tof = r_tof*np.cos(theta)
z_tof = r_tof

# Scatter plot of 3D points
fig = plt.figure()
fig.tight_layout()
ax = fig.add_subplot(111, projection='2d')
ax.scatter(x_tof,z_tof,s=2)
plt.legend(["tof"])
ax.set_ylim2d(-2000, 2000)
ax.set_xlim2d(-2000, 2000)
ax.set_zlim2d(0, 2000)
plt.show()