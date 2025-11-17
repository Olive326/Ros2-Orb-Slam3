/*
RGB-D-Inertial mode for ORB-SLAM3
Handles RGB + Depth + IMU data
Date: 2025
*/

#include "ros2_orb_slam3/common.hpp"

// Constructor
RGBDInertialMode::RGBDInertialMode() : Node("rgbd_inertial_node_cpp")
{
    RCLCPP_INFO(this->get_logger(), "\nORB-SLAM3 RGB-D-INERTIAL NODE STARTED");
    
    // Get home directory
    homeDir = getenv("HOME");
    
    // Set default paths - NOTE: Change TUM1.yaml to TUM2.yaml or TUM3.yaml as needed
    vocFilePath = homeDir + "/" + packagePath + "orb_slam3/Vocabulary/ORBvoc.txt.bin";
    settingsFilePath = homeDir + "/" + packagePath + "orb_slam3/config/RGB-D-Inertial/TUM_512.yaml";
    
    RCLCPP_INFO(this->get_logger(), "Vocabulary file: %s", vocFilePath.c_str());
    RCLCPP_INFO(this->get_logger(), "Settings file: %s", settingsFilePath.c_str());
    
    // Initialize ORB-SLAM3
    initializeVSLAM();
    
    // Initialize time tracking
    last_image_time_ = 0.0;
    
    // Create subscribers for RGB, Depth, and IMU
    rgb_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
        subRgbImgName, 10, 
        std::bind(&RGBDInertialMode::rgb_callback, this, std::placeholders::_1));
    
    depth_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
        subDepthImgName, 10, 
        std::bind(&RGBDInertialMode::depth_callback, this, std::placeholders::_1));
    
    imu_subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
        subImuName, 1000,  // Large queue for high-frequency IMU data
        std::bind(&RGBDInertialMode::imu_callback, this, std::placeholders::_1));
    
    RCLCPP_INFO(this->get_logger(), "Subscribed to:");
    RCLCPP_INFO(this->get_logger(), "  RGB: %s", subRgbImgName.c_str());
    RCLCPP_INFO(this->get_logger(), "  Depth: %s", subDepthImgName.c_str());
    RCLCPP_INFO(this->get_logger(), "  IMU: %s", subImuName.c_str());
    RCLCPP_INFO(this->get_logger(), "RGB-D-Inertial node ready!");
}

// Destructor
RGBDInertialMode::~RGBDInertialMode()
{
    // Save trajectories before shutting down
    RCLCPP_INFO(this->get_logger(), "Saving trajectories...");
    
    std::string home = getenv("HOME");
    std::string traj_path = home + "/orb_slam3_results/";
    
    // Create directory if it doesn't exist
    system(("mkdir -p " + traj_path).c_str());
    
    // Save trajectories
    pAgent->SaveTrajectoryTUM(traj_path + "CameraTrajectory.txt");
    pAgent->SaveKeyFrameTrajectoryTUM(traj_path + "KeyFrameTrajectory.txt");
    
    RCLCPP_INFO(this->get_logger(), "Trajectories saved to: %s", traj_path.c_str());
    
    pAgent->Shutdown();
    RCLCPP_INFO(this->get_logger(), "RGB-D-Inertial node shutting down");
}

// Initialize ORB-SLAM3 system
void RGBDInertialMode::initializeVSLAM()
{
    sensorType = ORB_SLAM3::System::IMU_RGBD;  // RGB-D + IMU mode
    enablePangolinWindow = true;
    
    pAgent = new ORB_SLAM3::System(vocFilePath, settingsFilePath, sensorType, enablePangolinWindow);
    RCLCPP_INFO(this->get_logger(), "ORB-SLAM3 RGB-D-Inertial system initialized");
}

// IMU callback - stores IMU measurements
void RGBDInertialMode::imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(imu_mutex_);
    
    // Convert ROS IMU message to ORB-SLAM3 IMU::Point
    double t = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
    
    ORB_SLAM3::IMU::Point imu_point(
        msg->linear_acceleration.x,
        msg->linear_acceleration.y,
        msg->linear_acceleration.z,
        msg->angular_velocity.x,
        msg->angular_velocity.y,
        msg->angular_velocity.z,
        t
    );
    
    imu_buffer_.push_back(imu_point);
    
    // Keep buffer reasonable size (last 1000 measurements ~ 10 seconds at 100Hz)
    if (imu_buffer_.size() > 1000) {
        imu_buffer_.erase(imu_buffer_.begin());
    }
}

// RGB image callback
void RGBDInertialMode::rgb_callback(const sensor_msgs::msg::Image::SharedPtr msg)
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
            process_rgbd_imu();
            // Reset after processing
            latest_rgb_.reset();
            latest_depth_.reset();
        }
    }
}

// Depth image callback
void RGBDInertialMode::depth_callback(const sensor_msgs::msg::Image::SharedPtr msg)
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
            process_rgbd_imu();
            // Reset after processing
            latest_rgb_.reset();
            latest_depth_.reset();
        }
    }
}

// Get IMU measurements between two timestamps
std::vector<ORB_SLAM3::IMU::Point> RGBDInertialMode::get_imu_measurements(double t0, double t1)
{
    std::lock_guard<std::mutex> lock(imu_mutex_);
    std::vector<ORB_SLAM3::IMU::Point> imu_meas;
    
    for (const auto& imu : imu_buffer_)
    {
        if (imu.t > t0 && imu.t <= t1) {
            imu_meas.push_back(imu);
        }
    }
    
    return imu_meas;
}

// Process synchronized RGB-D-IMU data
void RGBDInertialMode::process_rgbd_imu()
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
        
        // Get IMU measurements between last image and current image
        std::vector<ORB_SLAM3::IMU::Point> vImuMeas;
        
        if (system_initialized_ && last_image_time_ > 0) {
            vImuMeas = get_imu_measurements(last_image_time_, timestamp);
        }
        
        // Track with ORB-SLAM3 (with IMU)
        if (!vImuMeas.empty() || !system_initialized_) {
            Sophus::SE3f Tcw = pAgent->TrackRGBD(cv_rgb->image, cv_depth->image, timestamp, vImuMeas);
            
            system_initialized_ = true;
            last_image_time_ = timestamp;
            
            // Log occasionally
            static int frame_count = 0;
            frame_count++;
            if (frame_count % 50 == 0) {
                RCLCPP_INFO(this->get_logger(), 
                    "Processed frame %d | IMU measurements: %zu | Time: %.3f", 
                    frame_count, vImuMeas.size(), timestamp);
            }
        } else {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "Waiting for IMU data... (IMU buffer size: %zu)", imu_buffer_.size());
        }
        
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
    auto node = std::make_shared<RGBDInertialMode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
