How to run the algorithms:
step1  Start the ORB-SLAM3 algorithm
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run ros2_orb_slam3 rgbd_node_cpp

step2 Feed data to the algorithm
ros2 bag play ~/Downloads/freiburg1_ros2/

step3 Monitor (optional)
ros2 topic hz /camera/rgb/image_color

when Run the last two nodes use the following command as the node name can't match:
ros2 run ros2_orb_slam3 mono_inertial_node_cpp \
  --ros-args \
  -r /camera/image_raw:=/cam0/image_raw \
  -r /imu:=/imu0



data format we need(after preprocessing):
✅ ROS2 bag with synchronized topics
✅ Camera calibration YAML file
✅ ORB vocabulary file 


dataset we use:
freiburg3/long_office_household - Loop closure
freiburg2/desk_with_person - Dynamic objects
freiburg3/walking_xyz - Fast motion
freiburg1/desk - Baseline (simple, good tracking)
euroc_mh01_ros2

ORB-SLAM3 modes:
mono_node_cpp             RGB                  Single camera, no scale
rgbd_node_cpp             RGB + Depth          Indoor with depth sensor(TUM dataset, all of freiburg dataset)
mono_inertial_node_cpp    RGB + IMU            Single camera with IMU(EuRoC MH_01_easy)
stereo_inertial_node_cpp  2×RGB + IMU          Stereo camera with IMU(EuRoC MH_01_easy)

Chcek the node info:
ros2 node info /mono_inertial_node_cpp

Check whether the node publish and subscribe successfully:
ros2 topic info /cam0/image_raw

example success output:
Type: sensor_msgs/msg/Image
Publisher count: 1
Subscription count: 1
