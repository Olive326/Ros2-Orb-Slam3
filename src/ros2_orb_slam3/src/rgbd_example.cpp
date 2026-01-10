/*
RGB-D mode example node for ORB-SLAM3
Adapted from mono_example.cpp
Date: 2025
*/

#include "ros2_orb_slam3/common.hpp"
#include <fstream>

// Constructor
RGBDMode::RGBDMode() : Node("rgbd_node_cpp")
{
    RCLCPP_INFO(this->get_logger(), "\nORB-SLAM3 RGB-D NODE STARTED");
    
    // Get home directory
    homeDir = getenv("HOME");
    
    // Set default paths
    vocFilePath = homeDir + "/" + packagePath + "orb_slam3/Vocabulary/ORBvoc.txt.bin";
    settingsFilePath = homeDir + "/" + packagePath + "orb_slam3/config/RGB-D/TUM1.yaml";
    
    // Declare parameters for trajectory saving
    this->declare_parameter<std::string>("trajectory_file", homeDir + "/orb_slam3_trajectory.txt");
    this->declare_parameter<bool>("save_trajectory", true);
    this->declare_parameter<int>("save_interval", 1); // Save every N frames
    
    this->get_parameter("trajectory_file", trajectoryFilePath);
    this->get_parameter("save_trajectory", saveTrajectory);
    this->get_parameter("save_interval", saveInterval);
    
    RCLCPP_INFO(this->get_logger(), "Vocabulary file: %s", vocFilePath.c_str());
    RCLCPP_INFO(this->get_logger(), "Settings file: %s", settingsFilePath.c_str());
    RCLCPP_INFO(this->get_logger(), "Trajectory file: %s", trajectoryFilePath.c_str());
    
    // Initialize trajectory file
    if (saveTrajectory)
    {
        trajectoryFile.open(trajectoryFilePath);
        if (trajectoryFile.is_open())
        {
            // Write TUM format header (commented)
            trajectoryFile << "# timestamp tx ty tz qx qy qz qw" << std::endl;
            RCLCPP_INFO(this->get_logger(), "Trajectory file opened successfully");
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to open trajectory file!");
        }
    }
    
    frameCount = 0;
    trackedFrames = 0;
    lostFrames = 0;
    
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
    
    // Create timer to print statistics every 5 seconds
    stats_timer_ = this->create_wall_timer(
        std::chrono::seconds(5),
        std::bind(&RGBDMode::print_statistics, this));
}

// Destructor
RGBDMode::~RGBDMode()
{
    // Print final statistics
    print_final_statistics();
    
    // Close trajectory file
    if (trajectoryFile.is_open())
    {
        trajectoryFile.close();
        RCLCPP_INFO(this->get_logger(), "Trajectory file closed");
    }
    
    // Shutdown ORB-SLAM3 and save trajectories
    if (pAgent)
    {
        RCLCPP_INFO(this->get_logger(), "Saving KeyFrame trajectory...");
        pAgent->SaveKeyFrameTrajectoryTUM(homeDir + "/KeyFrameTrajectory.txt");
        
        RCLCPP_INFO(this->get_logger(), "Saving all frame trajectory...");
        pAgent->SaveTrajectoryTUM(homeDir + "/CameraTrajectory.txt");
        
        pAgent->Shutdown();
        delete pAgent;
    }
    
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
        
        frameCount++;
        
        // Track with ORB-SLAM3
        Sophus::SE3f Tcw = pAgent->TrackRGBD(cv_rgb->image, cv_depth->image, timestamp);
        
        // Check tracking state
        int state = pAgent->GetTrackingState();
        
        if (state == ORB_SLAM3::Tracking::OK)
        {
            trackedFrames++;
            
            // Save trajectory at specified interval
            if (saveTrajectory && trajectoryFile.is_open() && (frameCount % saveInterval == 0))
            {
                save_pose_to_file(Tcw, timestamp);
            }
        }
        else
        {
            lostFrames++;
            RCLCPP_WARN(this->get_logger(), "Tracking LOST at frame %d", frameCount);
        }
        
    }
    catch (cv_bridge::Exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "CV Bridge error: %s", e.what());
    }
}

// Save pose to trajectory file in TUM format
void RGBDMode::save_pose_to_file(const Sophus::SE3f& Tcw, double timestamp)
{
    // Get camera pose (inverse of Tcw)
    Sophus::SE3f Twc = Tcw.inverse();
    
    // Extract translation
    Eigen::Vector3f t = Twc.translation();
    
    // Extract quaternion (x, y, z, w)
    Eigen::Quaternionf q = Twc.unit_quaternion();
    
    // Write in TUM format: timestamp tx ty tz qx qy qz qw
    trajectoryFile << std::fixed << std::setprecision(6) << timestamp << " "
                   << std::setprecision(7) 
                   << t.x() << " " << t.y() << " " << t.z() << " "
                   << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << std::endl;
}

//* Print statistics periodically
void RGBDMode::print_statistics()
{
    if (frameCount > 0 && pAgent)
    {
        float tracking_rate = (float)trackedFrames / frameCount * 100.0f;
        RCLCPP_INFO(this->get_logger(), 
                    "\n=== SLAM Statistics ===\n"
                    "Total frames: %d\n"
                    "Tracked frames: %d\n"
                    "Lost frames: %d\n"
                    "Tracking rate: %.2f%%\n"
                    "=======================",
                    frameCount, trackedFrames, lostFrames, tracking_rate);
    }
}

//* Print final statistics
void RGBDMode::print_final_statistics()
{
    if (frameCount > 0 && pAgent)
    {
        RCLCPP_INFO(this->get_logger(), 
                    "\n========== FINAL STATISTICS ==========\n"
                    "Total frames processed: %d\n"
                    "Successfully tracked: %d\n"
                    "Tracking lost: %d\n"
                    "Final tracking rate: %.2f%%\n"
                    "======================================",
                    frameCount, 
                    trackedFrames, 
                    lostFrames,
                    (float)trackedFrames / frameCount * 100.0f);
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