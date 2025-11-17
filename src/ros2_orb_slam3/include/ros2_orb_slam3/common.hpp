// Include file 
#ifndef COMMON_HPP  // Header guard to prevent multiple inclusions
#define COMMON_HPP

// C++ includes
#include <iostream> // The iostream library is an object-oriented library that provides input and output functionality using streams
#include <algorithm> // The header <algorithm> defines a collection of functions especially designed to be used on ranges of elements.
#include <fstream> // Input/output stream class to operate on files.
#include <chrono> // c++ timekeeper library
#include <vector> // vectors are sequence containers representing arrays that can change in size.
#include <queue>
#include <thread> // class to represent individual threads of execution.
#include <mutex> // A mutex is a lockable object that is designed to signal when critical sections of code need exclusive access, preventing other threads with the same protection from executing concurrently and access the same memory locations.
#include <cstdlib> // to find home directory

#include <cstring>
#include <sstream> // String stream processing functionalities

//* ROS2 includes
//* std_msgs in ROS 2 https://docs.ros2.org/foxy/api/std_msgs/index-msg.html
#include "rclcpp/rclcpp.hpp"

// #include "your_custom_msg_interface/msg/custom_msg_field.hpp" // Example of adding in a custom message
#include <std_msgs/msg/header.hpp>
#include "std_msgs/msg/float64.hpp"
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>
#include "sensor_msgs/msg/image.hpp"
#include <sensor_msgs/msg/imu.hpp>
#include "ImuTypes.h"  // ORB-SLAM3 IMU types
using std::placeholders::_1; //* TODO why this is suggested in official tutorial

// Include Eigen
// Quick reference: https://eigen.tuxfamily.org/dox/group__QuickRefPage.html
#include <Eigen/Dense> // Includes Core, Geometry, LU, Cholesky, SVD, QR, and Eigenvalues header file

// Include cv-bridge
#include <cv_bridge/cv_bridge.hpp>

// Include OpenCV computer vision library
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp> // Image processing tools
#include <opencv2/highgui/highgui.hpp> // GUI tools
#include <opencv2/core/eigen.hpp>
#include <image_transport/image_transport.hpp>

//* ORB SLAM 3 includes
#include "System.h" //* Also imports the ORB_SLAM3 namespace

//* Gobal defs
#define pass (void)0 // Python's equivalent of "pass" i.e. no operation


//* Node specific definitions
class MonocularMode : public rclcpp::Node
{   
    //* This slam node inherits from both rclcpp and ORB_SLAM3::System classes
    //* public keyword needs to come before the class constructor and anything else
    public:
    std::string experimentConfig = ""; // String to receive settings sent by the python driver
    double timeStep; // Timestep data received from the python node
    std::string receivedConfig = "";

    //* Class constructor
    MonocularMode(); // Constructor 

    ~MonocularMode(); // Destructor
        
    private:
        
        // Class internal variables
        std::string homeDir = "";
        std::string packagePath = "ros2_test/src/ros2_orb_slam3/"; //! Change to match path to your workspace
        std::string OPENCV_WINDOW = ""; // Set during initialization
        std::string nodeName = ""; // Name of this node
        std::string vocFilePath = ""; // Path to ORB vocabulary provided by DBoW2 package
        std::string settingsFilePath = ""; // Path to settings file provided by ORB_SLAM3 package
        bool bSettingsFromPython = false; // Flag set once when experiment setting from python node is received
        
        std::string subexperimentconfigName = ""; // Subscription topic name
        std::string pubconfigackName = ""; // Publisher topic name
        std::string subImgMsgName = ""; // Topic to subscribe to receive RGB images from a python node
        std::string subTimestepMsgName = ""; // Topic to subscribe to receive the timestep related to the 

        //* Definitions of publisher and subscribers
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr expConfig_subscription_;
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr configAck_publisher_;
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subImgMsg_subscription_;
        rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr subTimestepMsg_subscription_;

        //* ORB_SLAM3 related variables
        ORB_SLAM3::System* pAgent; // pointer to a ORB SLAM3 object
        ORB_SLAM3::System::eSensor sensorType;
        bool enablePangolinWindow = false; // Shows Pangolin window output
        bool enableOpenCVWindow = false; // Shows OpenCV window output

        //* ROS callbacks
        void experimentSetting_callback(const std_msgs::msg::String& msg); // Callback to process settings sent over by Python node
        void Timestep_callback(const std_msgs::msg::Float64& time_msg); // Callback to process the timestep for this image
        void Img_callback(const sensor_msgs::msg::Image& msg); // Callback to process RGB image and semantic matrix sent by Python node
        
        //* Helper functions
        // ORB_SLAM3::eigenMatXf convertToEigenMat(const std_msgs::msg::Float32MultiArray& msg); // Helper method, converts semantic matrix eigenMatXf, a Eigen 4x4 float matrix
        void initializeVSLAM(std::string& configString); //* Method to bind an initialized VSLAM framework to this node


};

//* RGB-D Mode Node Definition
class RGBDMode : public rclcpp::Node
{
    public:
        std::string experimentConfig = "";
        double timeStep;
        std::string receivedConfig = "";
        
        RGBDMode();
        ~RGBDMode();
        
    private:
        std::string homeDir = "";
        std::string packagePath = "Final-project/src/ros2_orb_slam3/";
        std::string nodeName = "";
        std::string vocFilePath = "";
        std::string settingsFilePath = "";
        bool bSettingsFromPython = false;
        
        std::string subRgbImgName = "/camera/rgb/image_color";
        std::string subDepthImgName = "/camera/depth/image";
        
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr rgb_subscription_;
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_subscription_;
        
        sensor_msgs::msg::Image::SharedPtr latest_rgb_;
        sensor_msgs::msg::Image::SharedPtr latest_depth_;
        std::mutex sync_mutex_;
        
        ORB_SLAM3::System* pAgent;
        ORB_SLAM3::System::eSensor sensorType;
        bool enablePangolinWindow = true;
        
        void rgb_callback(const sensor_msgs::msg::Image::SharedPtr msg);
        void depth_callback(const sensor_msgs::msg::Image::SharedPtr msg);
        void process_rgbd();
        void initializeVSLAM();
};


//* RGB-D-Inertial Mode Node Definition
class RGBDInertialMode : public rclcpp::Node
{
    public:
        RGBDInertialMode();
        ~RGBDInertialMode();
        
    private:
        std::string homeDir = "";
        std::string packagePath = "Final-project/src/ros2_orb_slam3/";
        std::string vocFilePath = "";
        std::string settingsFilePath = "";
        
        // Topic names
        std::string subRgbImgName = "/camera/rgb/image_color";
        std::string subDepthImgName = "/camera/depth/image";
        std::string subImuName = "/imu";
        
        // Subscribers
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr rgb_subscription_;
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_subscription_;
        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
        
        // Image storage for synchronization
        sensor_msgs::msg::Image::SharedPtr latest_rgb_;
        sensor_msgs::msg::Image::SharedPtr latest_depth_;
        std::mutex sync_mutex_;
        
        // IMU buffer
        std::vector<ORB_SLAM3::IMU::Point> imu_buffer_;
        std::mutex imu_mutex_;
        double last_image_time_;
        
        // ORB_SLAM3 related
        ORB_SLAM3::System* pAgent;
        ORB_SLAM3::System::eSensor sensorType;
        bool enablePangolinWindow = true;
        bool system_initialized_ = false;
        
        // Callbacks
        void rgb_callback(const sensor_msgs::msg::Image::SharedPtr msg);
        void depth_callback(const sensor_msgs::msg::Image::SharedPtr msg);
        void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg);
        void process_rgbd_imu();
        
        // Helper
        void initializeVSLAM();
        std::vector<ORB_SLAM3::IMU::Point> get_imu_measurements(double t0, double t1);
};

//* Monocular-Inertial Mode Node Definition
class MonocularInertialMode : public rclcpp::Node
{
    public:
        MonocularInertialMode();
        ~MonocularInertialMode();
        
    private:
        std::string homeDir = "";
        std::string packagePath = "Final-project/src/ros2_orb_slam3/";
        std::string vocFilePath = "";
        std::string settingsFilePath = "";
        
        // Topic names
        std::string subImgName = "/camera/image_raw";
        std::string subImuName = "/imu";
        
        // Subscribers
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_subscription_;
        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
        
        // Image storage
        sensor_msgs::msg::Image::SharedPtr latest_img_;
        std::mutex sync_mutex_;
        
        // IMU buffer
        std::vector<ORB_SLAM3::IMU::Point> imu_buffer_;
        std::mutex imu_mutex_;
        double last_image_time_;
        
        // ORB_SLAM3 related
        ORB_SLAM3::System* pAgent;
        ORB_SLAM3::System::eSensor sensorType;
        bool enablePangolinWindow = true;
        bool system_initialized_ = false;
        
        // Callbacks
        void img_callback(const sensor_msgs::msg::Image::SharedPtr msg);
        void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg);
        void process_mono_imu();
        
        // Helper
        void initializeVSLAM();
        std::vector<ORB_SLAM3::IMU::Point> get_imu_measurements(double t0, double t1);
};

//* Stereo-Inertial Mode Node Definition
class StereoInertialMode : public rclcpp::Node
{
    public:
        StereoInertialMode();
        ~StereoInertialMode();
        
    private:
        std::string homeDir = "";
        std::string packagePath = "Final-project/src/ros2_orb_slam3/";
        std::string vocFilePath = "";
        std::string settingsFilePath = "";
        
        // Topic names
        std::string subLeftImgName = "/camera/left/image_raw";
        std::string subRightImgName = "/camera/right/image_raw";
        std::string subImuName = "/imu";
        
        // Subscribers
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr left_subscription_;
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr right_subscription_;
        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
        
        // Image storage for synchronization
        sensor_msgs::msg::Image::SharedPtr latest_left_;
        sensor_msgs::msg::Image::SharedPtr latest_right_;
        std::mutex sync_mutex_;
        
        // IMU buffer
        std::vector<ORB_SLAM3::IMU::Point> imu_buffer_;
        std::mutex imu_mutex_;
        double last_image_time_;
        
        // ORB_SLAM3 related
        ORB_SLAM3::System* pAgent;
        ORB_SLAM3::System::eSensor sensorType;
        bool enablePangolinWindow = true;
        bool system_initialized_ = false;
        
        // Callbacks
        void left_callback(const sensor_msgs::msg::Image::SharedPtr msg);
        void right_callback(const sensor_msgs::msg::Image::SharedPtr msg);
        void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg);
        void process_stereo_imu();
        
        // Helper
        void initializeVSLAM();
        std::vector<ORB_SLAM3::IMU::Point> get_imu_measurements(double t0, double t1);
};

#endif
