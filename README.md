Rosnode for attitude estimator on SO(3).  Tested on Ubuntu 16.04

## Installation:

- Install ROS. Instructions [here](http://wiki.ros.org/kinetic/Installation).
- Setup ROS workspace. Instructions [here](http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment).
- Install eigen3
```
sudo apt-get install libeigen3-dev
```
- Clone node into src folder of ROS workspace.
```
git clone https://github.com/andrewspielvogel/att_so3.git
```
- Make you workspace. `cd` into the top level of your ROS workspace and run:
```
catkin_make
```
- Create a location for the log file storage or to use default make a KVH log directory in `/log` and give proper permissions
```
sudo mkdir /log   
sudo chown USER.USER /log
chmod +rwx /log           
mkdir /log/kvh
mkdir /log/phins
mkdir /log/microstrain
```

## Launch File Params

lat : Latitude (degrees). Default: 39.32

r0 : Initial vehicle estimate of vehicle attitude (In RPY Euler Angles). Default: "[0,0,0]"

k_g: Diagonal of the gravity (local level) gain matrix for the attitude estimator. 

k_north: Diagonal of the north gain matrix of the attitude estimator. 

frame_id: Frame ID of the IMU (dscl_msgs::Imu9DOF) topic subscribed to.

## Using the Node

- Start Node via the launch file:
```
roslaunch att_so3/att.launch
```

## Topic

Topic subscribed to:
- IMU Data topic (dscl_msgs::Imu9DOF)
```
/frame_id_bias/imu_corrected
```

Topics published:

- Quaternion topic (geometry_msgs::QuaternionStamped)
```
/frame_id_att/att
```

- RPY topic (geometry_msgs::Vector3Stamped)
```
/frame_id_att/rpy
```

## Generate Documentation

To generate documentation, use doxygen.

- Install doxygen from [here](http://www.stack.nl/~dimitri/doxygen/download.html). Note: you need to install the packages flex and bison before making doxygen with:
```
sudo apt install flex
sudo apt install bison
```

- Then `cd` into the truenorth directory and run:
```
doxygen Doxyfile
```

- To view the documentation, open `index.html` located in the `html/` directory.
