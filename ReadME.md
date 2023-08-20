# How To 


```
roslaunch LidarSLAM GUI.launch 
```

for camera

roslaunch realsense2_camera rs_rgbd.launch 

rosrun tf static_transform_publisher 0 0 0.1 0 0 0 /laser /camera_link 10



# OLD 

source the package where the workspace is: (is done in the .bashrc not needed anymore)

```
source devel/setup.bash 

```

Usually I run in a seperate terminal the 

```
roscore
```
Is not mandatory if you run a ros package using roslaunch. But i prefer if we want to run different packages or using a bagfile

for real setup lunch: 

```
roslaunch LidarSLAM simple_lidar_slam_hector.launch 

```

if we want to use the Hokuyo laser, launch

```

roslaunch LidarSLAM hokuyo_hector_slam_hector.launch 

```

in a second terminal 

```
rosrun LidarSLAM ludlum2221.py 
```
for now the ludlum is not inside the all-in-one launch file. since we need to check ports. And for now the port parameter isn't a ros parameter.
So we cannot set using the xml syntax.



## with bag files
using a bag file, that you can find in the folder /bag :

```
rosbag play laser_path_234.bag --clock
rosparam set use_sim_time true
roslaunch LidarSLAM bag_lidar_slam_hector.launch 

```














# TEST DONE and deps repos

sudo apt-get install libqt5datavisualization5*
sudo apt-get install libqt5charts5*




export CMAKE_PREFIX_PATH=${CMAKE_PREFIX_PATH}:'/home/andrea/Qt/5.15.2/gcc_64/lib/cmake'

sudo apt-get install qt5-default qtdeclarative5-dev*


### hokuyo
https://tuw-cpsg.github.io/tutorials/hokuyo-urg-04lx/

cd ~/catkin_ws/src/
git clone https://github.com/ros-drivers/driver_common.git
catkin_make -C ..
git clone https://github.com/ros-drivers/hokuyo_node.git
catkin_make -C ..




 sudo apt-get install ros-melodic-scan-tools 
http://wiki.ros.org/laser_scan_matcher
roslaunch laser_scan_matcher demo.launch


sudo apt-get install qt4-dev-tools 
git clone https://github.com/tu-darmstadt-ros-pkg/hector_slam.git




git clone https://github.com/ros-perception/slam_gmapping.git
git clone https://github.com/ros-perception/openslam_gmapping.git

https://github.com/LightWare-Optoelectronics/lightwarelidar


git clone https://bitbucket.org/gtborg/gtsam.git
https://github.com/jtpils/mim_slam.git

https://github.com/nkuwenjian/karto_scan_matcher




# tested - not working IMU

sudo apt-get install ros-melodic-rosserial-arduino 
git clone https://github.com/soarbear/mpu9250_imu_ros.git


https://github.com/bjajoh/ros-mpu9250-ahrs.git

https://github.com/StefanKrupop/ros-mpu9250-imu.git

https://github.com/pcdangio/ros-sensor_msgs_ext.git

https://github.com/pcdangio/ros-driver_mpu9250

