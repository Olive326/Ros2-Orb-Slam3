/*
Monocular-Inertial mode for ORB-SLAM3
Handles RGB + IMU data
Date: 2025
*/

#include "ros2_orb_slam3/common.hpp"

// Constructor
MonocularInertialMode::MonocularInertialMode() : Node("mono_inertial_node_cpp")
{
    RCLCPP_INFO(this->get_logger(), "\nORB-SLAM3 MONOCULAR-INERTIAL NODE STARTED");
    
    // Get home directory
    homeDir = getenv("HOME");
    
    // Set default paths
    vocFilePath = homeDir + "/" + packagePath + "orb_slam3/Vocabulary/ORBvoc.txt.bin";
    settingsFilePath = homeDir + "/" + packagePath + "orb_slam3/config/Monocular-Inertial/TUM-VI.yaml";
    
    RCLCPP_INFO(this->get_logger(), "Vocabulary file: %s", vocFilePath.c_str());
    RCLCPP_INFO(this->get_logger(), "Settings file: %s", settingsFilePath.c_str());
    
    // Initialize ORB-SLAM3
    initializeVSLAM();
    
    // Initialize time tracking
    last_image_time_ = 0.0;
    
    // Create subscribers for Image and IMU
    img_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
        subImgName, 10, 
        std::bind(&MonocularInertialMode::img_callback, this, std::placeholders::_1));
    
    imu_subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
        subImuName, 1000,  // Large queue for high-frequency IMU data
        std::bind(&MonocularInertialMode::imu_callback, this, std::placeholders::_1));
    
    RCLCPP_INFO(this->get_logger(), "Subscribed to:");
    RCLCPP_INFO(this->get_logger(), "  Image: %s", subImgName.c_str());
    RCLCPP_INFO(this->get_logger(), "  IMU: %s", subImuName.c_str());
    RCLCPP_INFO(this->get_logger(), "Monocular-Inertial node ready!");
}

// Destructor
MonocularInertialMode::~MonocularInertialMode()
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
    RCLCPP_INFO(this->get_logger(), "Monocular-Inertial node shutting down");
}

// Initialize ORB-SLAM3 system
void MonocularInertialMode::initializeVSLAM()
{
    sensorType = ORB_SLAM3::System::IMU_MONOCULAR;  // Monocular + IMU mode
    enablePangolinWindow = true;
    
    pAgent = new ORB_SLAM3::System(vocFilePath, settingsFilePath, sensorType, enablePangolinWindow);
    RCLCPP_INFO(this->get_logger(), "ORB-SLAM3 Monocular-Inertial system initialized");
}

// IMU callback - stores IMU measurements
void MonocularInertialMode::imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
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

// Image callback
void MonocularInertialMode::img_callback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(sync_mutex_);
    latest_img_ = msg;
    process_mono_imu();
}

// Get IMU measurements between two timestamps
std::vector<ORB_SLAM3::IMU::Point> MonocularInertialMode::get_imu_measurements(double t0, double t1)
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

// Process monocular image with IMU
void MonocularInertialMode::process_mono_imu()
{
    if (!latest_img_) return;
    
    cv_bridge::CvImagePtr cv_img;
    
    try
    {
        // Convert image
        cv_img = cv_bridge::toCvCopy(latest_img_, sensor_msgs::image_encodings::MONO8);
        
        // Get timestamp
        double timestamp = latest_img_->header.stamp.sec + latest_img_->header.stamp.nanosec * 1e-9;
        
        // Get IMU measurements between last image and current image
        std::vector<ORB_SLAM3::IMU::Point> vImuMeas;
        
        if (system_initialized_ && last_image_time_ > 0) {
            vImuMeas = get_imu_measurements(last_image_time_, timestamp);
        }
        
        // Track with ORB-SLAM3 (with IMU)
        if (!vImuMeas.empty() || !system_initialized_) {
            Sophus::SE3f Tcw = pAgent->TrackMonocular(cv_img->image, timestamp, vImuMeas);
            
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
        
        latest_img_.reset();
        
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
    auto node = std::make_shared<MonocularInertialMode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
