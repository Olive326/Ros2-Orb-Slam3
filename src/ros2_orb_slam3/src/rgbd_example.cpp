/*
RGB-D mode example node for ORB-SLAM3
Adapted from mono_example.cpp
Date: 2025
*/

#include "ros2_orb_slam3/common.hpp"

// Constructor
RGBDMode::RGBDMode() : Node("rgbd_node_cpp")
{
    RCLCPP_INFO(this->get_logger(), "\nORB-SLAM3 RGB-D NODE STARTED");
    
    // Get home directory
    homeDir = getenv("HOME");
    
    // Set default paths
    vocFilePath = homeDir + "/" + packagePath + "orb_slam3/Vocabulary/ORBvoc.txt.bin";
    settingsFilePath = homeDir + "/" + packagePath + "orb_slam3/config/RGB-D/TUM1.yaml";
    
    RCLCPP_INFO(this->get_logger(), "Vocabulary file: %s", vocFilePath.c_str());
    RCLCPP_INFO(this->get_logger(), "Settings file: %s", settingsFilePath.c_str());
    
    // Initialize ORB-SLAM3
    initializeVSLAM();
    
    // Create subscribers for RGB and Depth
    rgb_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
        subRgbImgName, 10, 
        std::bind(&RGBDMode::rgb_callback, this, _1));
    
    depth_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
        subDepthImgName, 10, 
        std::bind(&RGBDMode::depth_callback, this, _1));
    
    RCLCPP_INFO(this->get_logger(), "Subscribed to:");
    RCLCPP_INFO(this->get_logger(), "  RGB: %s", subRgbImgName.c_str());
    RCLCPP_INFO(this->get_logger(), "  Depth: %s", subDepthImgName.c_str());
    RCLCPP_INFO(this->get_logger(), "RGB-D node ready!");
}

// Destructor
RGBDMode::~RGBDMode()
{
    pAgent->Shutdown();
    RCLCPP_INFO(this->get_logger(), "RGB-D node shutting down");
}

// Initialize ORB-SLAM3 system
void RGBDMode::initializeVSLAM()
{
    sensorType = ORB_SLAM3::System::RGBD;
    enablePangolinWindow = true;
    
    pAgent = new ORB_SLAM3::System(vocFilePath, settingsFilePath, sensorType, enablePangolinWindow);
    RCLCPP_INFO(this->get_logger(), "ORB-SLAM3 RGB-D system initialized");
}

// RGB image callback
void RGBDMode::rgb_callback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(sync_mutex_);
    latest_rgb_ = msg;
    
    // If we have both RGB and Depth, process them
    if (latest_rgb_ && latest_depth_)
    {
        // Check if timestamps are close (within 50ms)
        double rgb_time = latest_rgb_->header.stamp.sec + latest_rgb_->header.stamp.nanosec * 1e-9;
        double depth_time = latest_depth_->header.stamp.sec + latest_depth_->header.stamp.nanosec * 1e-9;
        
        if (std::abs(rgb_time - depth_time) < 0.05) // 50ms threshold
        {
            process_rgbd();
            // Reset after processing
            latest_rgb_.reset();
            latest_depth_.reset();
        }
    }
}

// Depth image callback
void RGBDMode::depth_callback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(sync_mutex_);
    latest_depth_ = msg;
    
    // If we have both RGB and Depth, process them
    if (latest_rgb_ && latest_depth_)
    {
        // Check if timestamps are close
        double rgb_time = latest_rgb_->header.stamp.sec + latest_rgb_->header.stamp.nanosec * 1e-9;
        double depth_time = latest_depth_->header.stamp.sec + latest_depth_->header.stamp.nanosec * 1e-9;
        
        if (std::abs(rgb_time - depth_time) < 0.05)
        {
            process_rgbd();
            // Reset after processing
            latest_rgb_.reset();
            latest_depth_.reset();
        }
    }
}

// Process synchronized RGB-D pair
void RGBDMode::process_rgbd()
{
    cv_bridge::CvImagePtr cv_rgb, cv_depth;
    
    try
    {
        // Convert RGB
        cv_rgb = cv_bridge::toCvCopy(latest_rgb_, sensor_msgs::image_encodings::BGR8);
        
        // Convert Depth (usually in millimeters as 16-bit)
        cv_depth = cv_bridge::toCvCopy(latest_depth_, sensor_msgs::image_encodings::TYPE_16UC1);
        
        // Get timestamp (use RGB timestamp)
        double timestamp = latest_rgb_->header.stamp.sec + latest_rgb_->header.stamp.nanosec * 1e-9;
        
        // Track with ORB-SLAM3
        Sophus::SE3f Tcw = pAgent->TrackRGBD(cv_rgb->image, cv_depth->image, timestamp);
        
    }
    catch (cv_bridge::Exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "CV Bridge error: %s", e.what());
    }
}

// Main function
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RGBDMode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
