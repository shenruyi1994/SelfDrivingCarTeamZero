import matplotlib as mpl
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.cbook as cbook

def read_datafile(file_name):
    data = np.loadtxt(file_name, delimiter=',', skiprows=10)
    return data

targetPoints = read_datafile('/Users/awr/Desktop/SelfDrivingCarTeamZero/targetPoints.csv')
locationPoints = read_datafile('/Users/awr/Desktop/SelfDrivingCarTeamZero/locationPoints.csv')
##roadPoints = read_datafile('/Users/awr/Desktop/SelfDrivingCarTeamZero/roadPoints.csv')


x= targetPoints[:,0]
y= targetPoints[:,1]

fig = plt.figure()

ax1 = fig.add_subplot(111)

ax1.set_title("Gazebo target points path")
ax1.set_xlabel('Gazebo x coords')
ax1.set_ylabel('Gazebo y coords')

ax1.plot(x,y, c='r', label='target Points')

ax2 = fig.add_subplot(111)

ax2.set_title("Gazebo car path")
ax2.set_xlabel('Gazebo x coords')
ax2.set_ylabel('Gazebo y coords')

x1 = locationPoints[:,0]
y1 = locationPoints[:,1]

ax2.plot(x1,y1, c='b', label='car Points')

##ax3 = fig.add_subplot(111)

##ax3.set_title("Gazebo road path")
##ax3.set_xlabel('Gazebo x coords')
##ax3.set_ylabel('Gazebo y coords')

##x2 = roadPoints[:,0]
##y2 = roadsPoints[:,1]

##ax3.plot(x2,y2, c='g', label='road Points')

leg = ax1.legend()

plt.show()
