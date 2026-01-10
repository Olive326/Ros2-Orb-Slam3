/*

A bare-bones example node demonstrating the use of the Monocular mode in ORB-SLAM3

Author: Azmyin Md. Kamal
Date: 01/01/24

REQUIREMENTS
* Make sure to set path to your workspace in common.hpp file

*/

//* Includes
#include "ros2_orb_slam3/common.hpp"
#include <fstream>
#include <iomanip>

//* Constructor
MonocularMode::MonocularMode() :Node("mono_node_cpp")
{
    // Declare parameters to be passsed from command line
    // https://roboticsbackend.com/rclcpp-params-tutorial-get-set-ros2-params-with-cpp/
    
    //* Find path to home directory
    homeDir = getenv("HOME");
    // std::cout<<"Home: "<<homeDir<<std::endl;
    
    // std::cout<<"VLSAM NODE STARTED\n\n";
    RCLCPP_INFO(this->get_logger(), "\nORB-SLAM3-V1 NODE STARTED");

    this->declare_parameter("node_name_arg", "not_given"); // Name of this agent 
    this->declare_parameter("voc_file_arg", "file_not_set"); // Needs to be overriden with appropriate name  
    this->declare_parameter("settings_file_path_arg", "file_path_not_set"); // path to settings file  
    
    //* Watchdog, populate default values
    nodeName = "not_set";
    vocFilePath = "file_not_set";
    settingsFilePath = "file_not_set";

    //* Populate parameter values
    rclcpp::Parameter param1 = this->get_parameter("node_name_arg");
    nodeName = param1.as_string();
    
    rclcpp::Parameter param2 = this->get_parameter("voc_file_arg");
    vocFilePath = param2.as_string();

    rclcpp::Parameter param3 = this->get_parameter("settings_file_path_arg");
    settingsFilePath = param3.as_string();

    // rclcpp::Parameter param4 = this->get_parameter("settings_file_name_arg");
    
  
    //* HARDCODED, set paths
    if (vocFilePath == "file_not_set" || settingsFilePath == "file_not_set")
    {
        pass;
        vocFilePath = homeDir + "/" + packagePath + "orb_slam3/Vocabulary/ORBvoc.txt.bin";
        settingsFilePath = homeDir + "/" + packagePath + "orb_slam3/config/Monocular/";
    }

    // std::cout<<"vocFilePath: "<<vocFilePath<<std::endl;
    // std::cout<<"settingsFilePath: "<<settingsFilePath<<std::endl;
    
    
    //* DEBUG print
    RCLCPP_INFO(this->get_logger(), "nodeName %s", nodeName.c_str());
    RCLCPP_INFO(this->get_logger(), "voc_file %s", vocFilePath.c_str());
    // RCLCPP_INFO(this->get_logger(), "settings_file_path %s", settingsFilePath.c_str());
    
    subexperimentconfigName = "/mono_py_driver/experiment_settings"; // topic that sends out some configuration parameters to the cpp ndoe
    pubconfigackName = "/mono_py_driver/exp_settings_ack"; // send an acknowledgement to the python node
    subImgMsgName = "/mono_py_driver/img_msg"; // topic to receive RGB image messages
    subTimestepMsgName = "/mono_py_driver/timestep_msg"; // topic to receive RGB image messages

    //* subscribe to python node to receive settings
    expConfig_subscription_ = this->create_subscription<std_msgs::msg::String>(subexperimentconfigName, 1, std::bind(&MonocularMode::experimentSetting_callback, this, _1));

    //* publisher to send out acknowledgement
    configAck_publisher_ = this->create_publisher<std_msgs::msg::String>(pubconfigackName, 10);

    //* subscrbite to the image messages coming from the Python driver node
    subImgMsg_subscription_= this->create_subscription<sensor_msgs::msg::Image>(subImgMsgName, 1, std::bind(&MonocularMode::Img_callback, this, _1));

    //* subscribe to receive the timestep
    subTimestepMsg_subscription_= this->create_subscription<std_msgs::msg::Float64>(subTimestepMsgName, 1, std::bind(&MonocularMode::Timestep_callback, this, _1));

    // Initialize trajectory saving parameters
    this->declare_parameter<std::string>("trajectory_file", homeDir + "/orb_slam3_trajectory.txt");
    this->declare_parameter<bool>("save_trajectory", true);
    this->declare_parameter<int>("save_interval", 1);
    
    this->get_parameter("trajectory_file", trajectoryFilePath);
    this->get_parameter("save_trajectory", saveTrajectory);
    this->get_parameter("save_interval", saveInterval);
    
    // Initialize counters
    frameCount = 0;
    trackedFrames = 0;
    lostFrames = 0;
    
    RCLCPP_INFO(this->get_logger(), "Trajectory will be saved to: %s", trajectoryFilePath.c_str());
    
    RCLCPP_INFO(this->get_logger(), "Waiting to finish handshake ......");
    
}

//* Destructor
MonocularMode::~MonocularMode()
{   
    // Print final statistics
    print_final_statistics();
    
    // Close trajectory file
    if (trajectoryFile.is_open())
    {
        trajectoryFile.close();
        RCLCPP_INFO(this->get_logger(), "Trajectory file closed");
    }
    
    // Save ORB-SLAM3 trajectories
    if (pAgent)
    {
        RCLCPP_INFO(this->get_logger(), "Saving KeyFrame trajectory...");
        pAgent->SaveKeyFrameTrajectoryTUM(homeDir + "/KeyFrameTrajectory.txt");
        
        RCLCPP_INFO(this->get_logger(), "Saving all frame trajectory...");
        pAgent->SaveTrajectoryTUM(homeDir + "/CameraTrajectory.txt");
        
        // Stop all threads and shutdown
        pAgent->Shutdown();
        delete pAgent;
    }
}

//* Callback which accepts experiment parameters from the Python node
void MonocularMode::experimentSetting_callback(const std_msgs::msg::String& msg){
    
    // std::cout<<"experimentSetting_callback"<<std::endl;
    bSettingsFromPython = true;
    experimentConfig = msg.data.c_str();
    // receivedConfig = experimentConfig; // Redundant
    
    RCLCPP_INFO(this->get_logger(), "Configuration YAML file name: %s", this->receivedConfig.c_str());

    //* Publish acknowledgement
    auto message = std_msgs::msg::String();
    message.data = "ACK";
    
    std::cout<<"Sent response: "<<message.data.c_str()<<std::endl;
    configAck_publisher_->publish(message);

    //* Wait to complete VSLAM initialization
    initializeVSLAM(experimentConfig);

}

//* Method to bind an initialized VSLAM framework to this node
void MonocularMode::initializeVSLAM(std::string& configString){
    
    // Watchdog, if the paths to vocabular and settings files are still not set
    if (vocFilePath == "file_not_set" || settingsFilePath == "file_not_set")
    {
        RCLCPP_ERROR(get_logger(), "Please provide valid voc_file and settings_file paths");       
        rclcpp::shutdown();
    } 
    
    //* Build .yaml`s file path
    
    settingsFilePath = settingsFilePath.append(configString);
    settingsFilePath = settingsFilePath.append(".yaml"); // Example ros2_ws/src/orb_slam3_ros2/orb_slam3/config/Monocular/TUM2.yaml

    RCLCPP_INFO(this->get_logger(), "Path to settings file: %s", settingsFilePath.c_str());
    
    // NOTE if you plan on passing other configuration parameters to ORB SLAM3 Systems class, do it here
    // NOTE you may also use a .yaml file here to set these values
    sensorType = ORB_SLAM3::System::MONOCULAR; 
    enablePangolinWindow = true; // Shows Pangolin window output
    enableOpenCVWindow = true; // Shows OpenCV window output
    
    pAgent = new ORB_SLAM3::System(vocFilePath, settingsFilePath, sensorType, enablePangolinWindow);
    std::cout << "MonocularMode node initialized" << std::endl;
    
    // Open trajectory file after SLAM system is initialized
    if (saveTrajectory)
    {
        trajectoryFile.open(trajectoryFilePath);
        if (trajectoryFile.is_open())
        {
            trajectoryFile << "# timestamp tx ty tz qx qy qz qw" << std::endl;
            RCLCPP_INFO(this->get_logger(), "Trajectory file opened successfully");
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to open trajectory file!");
        }
    }
    
    // Create statistics timer
    stats_timer_ = this->create_wall_timer(
        std::chrono::seconds(5),
        std::bind(&MonocularMode::print_statistics, this));
}

//* Callback that processes timestep sent over ROS
void MonocularMode::Timestep_callback(const std_msgs::msg::Float64& time_msg){
    // timeStep = 0; // Initialize
    timeStep = time_msg.data;
}

//* Callback to process image message and run SLAM node
void MonocularMode::Img_callback(const sensor_msgs::msg::Image& msg)
{
    // Initialize
    cv_bridge::CvImagePtr cv_ptr; //* Does not create a copy, memory efficient
    
    //* Convert ROS image to openCV image
    try
    {
        //cv::Mat im =  cv_bridge::toCvShare(msg.img, msg)->image;
        cv_ptr = cv_bridge::toCvCopy(msg); // Local scope
        
        // DEBUGGING, Show image
        // Update GUI Window
        // cv::imshow("test_window", cv_ptr->image);
        // cv::waitKey(3);
    }
    catch (cv_bridge::Exception& e)
    {
        RCLCPP_ERROR(this->get_logger(),"Error reading image");
        return;
    }
    
    frameCount++;
    
    //* Perform all ORB-SLAM3 operations in Monocular mode
    //! Pose with respect to the camera coordinate frame not the world coordinate frame
    Sophus::SE3f Tcw = pAgent->TrackMonocular(cv_ptr->image, timeStep); 
    
    // Check tracking state and save trajectory
    int state = pAgent->GetTrackingState();
    
    if (state == ORB_SLAM3::Tracking::OK)
    {
        trackedFrames++;
        
        // Save trajectory at specified interval
        if (saveTrajectory && trajectoryFile.is_open() && (frameCount % saveInterval == 0))
        {
            save_pose_to_file(Tcw, timeStep);
        }
    }
    else
    {
        lostFrames++;
        if (lostFrames % 10 == 0) // Only warn every 10 lost frames to avoid spam
        {
            RCLCPP_WARN(this->get_logger(), "Tracking LOST at frame %d", frameCount);
        }
    }
}

//* Save pose to trajectory file in TUM format
void MonocularMode::save_pose_to_file(const Sophus::SE3f& Tcw, double timestamp)
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
void MonocularMode::print_statistics()
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
void MonocularMode::print_final_statistics()
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