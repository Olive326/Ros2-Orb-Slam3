How to run the ORB_SLAM3: 
step1  Start the ORB-SLAM3 algorithm
source ~/Final-project/install/setup.bash
ros2 run ros2_orb_slam3 rgbd_node_cpp

step2 Feed data to the algorithm
ros2 bag play ~/Downloads/freiburg1_ros2/

step3 Monitor (optional)
ros2 topic hz /camera/rgb/image_color


data format we need:
✅ ROS2 bag with synchronized topics
✅ Camera calibration YAML file
✅ ORB vocabulary file (already have: ORBvoc.txt.bin)

dataset we use:
freiburg3/long_office_household - Loop closure
freiburg2/desk_with_person - Dynamic objects
freiburg3/walking_xyz - Fast motion
freiburg1/desk - Baseline (simple, good tracking)
(Optional) freiburg1/rpy - Pure rotation challenge

check node:
ls ~/Final-project/install/ros2_orb_slam3/lib/ros2_orb_slam3/

mono_node_cpp             RGB                  Single camera, no scale
rgbd_node_cpp             RGB + Depth          Indoor with depth sensor
rgbd_inertial_node_cpp    RGB + Depth + IMU    Indoor with IMU support
mono_inertial_node_cpp    RGB + IMU            Single camera with IMU
stereo_inertial_node_cpp  2×RGB + IMU          Stereo camera with IMU
