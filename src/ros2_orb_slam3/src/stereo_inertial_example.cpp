/*
Stereo-Inertial mode for ORB-SLAM3
Handles Left + Right cameras + IMU data
Date: 2025
*/

#include "ros2_orb_slam3/common.hpp"

// Constructor
StereoInertialMode::StereoInertialMode() : Node("stereo_inertial_node_cpp")
{
    RCLCPP_INFO(this->get_logger(), "\nORB-SLAM3 STEREO-INERTIAL NODE STARTED");
    
    // Get home directory
    homeDir = getenv("HOME");
    
    // Set default paths
    vocFilePath = homeDir + "/" + packagePath + "orb_slam3/Vocabulary/ORBvoc.txt.bin";
    settingsFilePath = homeDir + "/" + packagePath + "orb_slam3/config/Stereo-Inertial/EuRoC.yaml";
    
    RCLCPP_INFO(this->get_logger(), "Vocabulary file: %s", vocFilePath.c_str());
    RCLCPP_INFO(this->get_logger(), "Settings file: %s", settingsFilePath.c_str());
    
    // Initialize ORB-SLAM3
    initializeVSLAM();
    
    // Initialize time tracking
    last_image_time_ = 0.0;
    
    // Create subscribers for Left, Right cameras and IMU
    left_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
        subLeftImgName, 10, 
        std::bind(&StereoInertialMode::left_callback, this, std::placeholders::_1));
    
    right_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
        subRightImgName, 10, 
        std::bind(&StereoInertialMode::right_callback, this, std::placeholders::_1));
    
    imu_subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
        subImuName, 1000,  // Large queue for high-frequency IMU data
        std::bind(&StereoInertialMode::imu_callback, this, std::placeholders::_1));
    
    RCLCPP_INFO(this->get_logger(), "Subscribed to:");
    RCLCPP_INFO(this->get_logger(), "  Left: %s", subLeftImgName.c_str());
    RCLCPP_INFO(this->get_logger(), "  Right: %s", subRightImgName.c_str());
    RCLCPP_INFO(this->get_logger(), "  IMU: %s", subImuName.c_str());
    RCLCPP_INFO(this->get_logger(), "Stereo-Inertial node ready!");
}

// Destructor
StereoInertialMode::~StereoInertialMode()
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
    RCLCPP_INFO(this->get_logger(), "Stereo-Inertial node shutting down");
}

// Initialize ORB-SLAM3 system
void StereoInertialMode::initializeVSLAM()
{
    sensorType = ORB_SLAM3::System::IMU_STEREO;  // Stereo + IMU mode
    enablePangolinWindow = true;
    
    pAgent = new ORB_SLAM3::System(vocFilePath, settingsFilePath, sensorType, enablePangolinWindow);
    RCLCPP_INFO(this->get_logger(), "ORB-SLAM3 Stereo-Inertial system initialized");
}

// IMU callback - stores IMU measurements
void StereoInertialMode::imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
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
    
    // Keep buffer reasonable size
    if (imu_buffer_.size() > 1000) {
        imu_buffer_.erase(imu_buffer_.begin());
    }
}

// Left image callback
void StereoInertialMode::left_callback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(sync_mutex_);
    latest_left_ = msg;
    
    // If we have both left and right, process them
    if (latest_left_ && latest_right_)
    {
        // Check if timestamps are close (within 10ms for stereo)
        double left_time = latest_left_->header.stamp.sec + latest_left_->header.stamp.nanosec * 1e-9;
        double right_time = latest_right_->header.stamp.sec + latest_right_->header.stamp.nanosec * 1e-9;
        
        if (std::abs(left_time - right_time) < 0.01) // 10ms threshold
        {
            process_stereo_imu();
            // Reset after processing
            latest_left_.reset();
            latest_right_.reset();
        }
    }
}

// Right image callback
void StereoInertialMode::right_callback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(sync_mutex_);
    latest_right_ = msg;
    
    // If we have both left and right, process them
    if (latest_left_ && latest_right_)
    {
        // Check if timestamps are close
        double left_time = latest_left_->header.stamp.sec + latest_left_->header.stamp.nanosec * 1e-9;
        double right_time = latest_right_->header.stamp.sec + latest_right_->header.stamp.nanosec * 1e-9;
        
        if (std::abs(left_time - right_time) < 0.01)
        {
            process_stereo_imu();
            // Reset after processing
            latest_left_.reset();
            latest_right_.reset();
        }
    }
}

// Get IMU measurements between two timestamps
std::vector<ORB_SLAM3::IMU::Point> StereoInertialMode::get_imu_measurements(double t0, double t1)
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

// Process synchronized stereo-IMU data
void StereoInertialMode::process_stereo_imu()
{
    cv_bridge::CvImagePtr cv_left, cv_right;
    
    try
    {
        // Convert left and right images
        cv_left = cv_bridge::toCvCopy(latest_left_, sensor_msgs::image_encodings::MONO8);
        cv_right = cv_bridge::toCvCopy(latest_right_, sensor_msgs::image_encodings::MONO8);
        
        // Get timestamp (use left timestamp)
        double timestamp = latest_left_->header.stamp.sec + latest_left_->header.stamp.nanosec * 1e-9;
        
        // Get IMU measurements between last image and current image
        std::vector<ORB_SLAM3::IMU::Point> vImuMeas;
        
        if (system_initialized_ && last_image_time_ > 0) {
            vImuMeas = get_imu_measurements(last_image_time_, timestamp);
        }
        
        // Track with ORB-SLAM3 (with IMU)
        if (!vImuMeas.empty() || !system_initialized_) {
            Sophus::SE3f Tcw = pAgent->TrackStereo(cv_left->image, cv_right->image, timestamp, vImuMeas);
            
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
    auto node = std::make_shared<StereoInertialMode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
