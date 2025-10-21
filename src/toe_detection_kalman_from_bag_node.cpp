#include <ros/ros.h>
#include <std_srvs/Trigger.h>
#include <std_msgs/String.h>
#include <vector>
#include <string>
#include <sstream>
#include <boost/filesystem.hpp> // For directory and path manipulation
#include "../include/kalman_processing_from_bag.h"

class ToeDetectionKalmanNode {
public:
    ToeDetectionKalmanNode() : nh_("~") {
        // Subscribe to the topic that provides the name of the bag file
        study_status_sub_ = nh_.subscribe("/robotrainer_user_study_manager/study_status", 10, &ToeDetectionKalmanNode::studyStatusCallback, this);

        // Advertise the service to trigger bag processing
        process_service_ = nh_.advertiseService("/toe_detection_kalman_from_bag_node/process", &ToeDetectionKalmanNode::processCallback, this);

        // Load processing parameters from the parameter server
        nh_.param<std::string>("input_bag_folder", input_bag_folder_, "/default/input/folder/raw");
        nh_.param<std::string>("output_bag_folder", output_bag_folder_, "/default/output/folder/toe");
        nh_.param<std::string>("camera_depth_frame_id", params_.camera_depth_frame_id, "camera_depth_frame");
        nh_.param<double>("min_z", params_.min_z, 0.01);
        nh_.param<double>("max_z", params_.max_z, 0.3);
        nh_.param<double>("min_y", params_.min_y, -0.4);
        nh_.param<double>("max_y", params_.max_y, 0.4);
        nh_.param<double>("cluster_tolerance", params_.cluster_tolerance, 0.03);
        nh_.param<double>("downsample_point_size", params_.downsample_point_size, 0.01);
        nh_.param<int>("min_cluster_size", params_.min_cluster_size, 150);
        nh_.param<double>("target_frequency", params_.target_frequency, 20.0);
        nh_.param<double>("max_prediction_time", params_.max_prediction_time, 0.25);
        nh_.param<double>("likelihood_threshold", params_.likelihood_threshold, 0.6);
        nh_.param<double>("swap_distance_threshold_ratio", params_.swap_distance_threshold_ratio, 0.5);
        
        params_.tf_topics = {"/tf_static", "/tf"};
        params_.pc_topics = {"/lower_legs_camera/depth_registered/points"};

        // Create the output folder if it doesn't exist
        try {
            if (!output_bag_folder_.empty() && !boost::filesystem::exists(output_bag_folder_)) {
                if (boost::filesystem::create_directories(output_bag_folder_)) {
                    ROS_INFO("Created output directory: %s", output_bag_folder_.c_str());
                }
            }
        } catch (const boost::filesystem::filesystem_error& e) {
            ROS_FATAL("Failed to create output directory %s: %s", output_bag_folder_.c_str(), e.what());
            ros::shutdown();
        }

        ROS_INFO("ToeDetectionKalmanNode service ready.");
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber study_status_sub_;
    ros::ServiceServer process_service_;
    std::string current_study_status_;
    std::string input_bag_folder_;
    std::string output_bag_folder_;
    ProcessingParameters params_;

    void studyStatusCallback(const std_msgs::String::ConstPtr& msg) {
        if (current_study_status_ != msg->data) {
            ROS_INFO("Study status updated to: %s", msg->data.c_str());
            current_study_status_ = msg->data;
            // KATE_AA_U010_16_yellow_line_force_right_60-1
        }
    }

    bool processCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) {
        if (current_study_status_.empty()) {
            res.success = false;
            res.message = "Failed: No bag file path received yet from /robotrainer_user_study_manager/study_status topic.";
            ROS_ERROR("%s", res.message.c_str());
            return true;
        }

        // Find the unique bag file that starts with the current_study_status_ string
        std::vector<std::string> matches;
        try {
            if (boost::filesystem::exists(input_bag_folder_) && boost::filesystem::is_directory(input_bag_folder_)) {
                for (const auto& entry : boost::filesystem::directory_iterator(input_bag_folder_)) {
                    const std::string filename = entry.path().filename().string();
                    // Check if filename starts with the study status and ends with .bag
                    if (filename.rfind(current_study_status_, 0) == 0 && 
                        filename.size() >= 4 && filename.substr(filename.size() - 4) == ".bag") {
                        matches.push_back(entry.path().string());
                    }
                }
            }
        } catch (const boost::filesystem::filesystem_error& e) {
            res.success = false;
            res.message = "Filesystem error while searching for bag file: " + std::string(e.what());
            ROS_ERROR("%s", res.message.c_str());
            return true;
        }

        if (matches.size() != 1) {
            std::stringstream ss;
            ss << "Found " << matches.size() << " bag files starting with '" << current_study_status_ 
               << "' in " << input_bag_folder_ << ". Expected 1.";
            res.success = false;
            res.message = ss.str();
            ROS_ERROR("%s", res.message.c_str());
            return true;
        }

        std::string bag_file_path = matches[0];
        ROS_INFO("Trigger received. Starting bag processing for: %s", bag_file_path.c_str());
        
        // Define output path based on the found bag file
        boost::filesystem::path input_path(bag_file_path);
        std::string base_name = input_path.filename().string();
        size_t pos = base_name.rfind(".bag");
        if (pos != std::string::npos) {
            base_name.replace(pos, 4, "_toe_output.bag");
        }
        boost::filesystem::path output_path = boost::filesystem::path(output_bag_folder_) / base_name;

        params_.input_bag_path = bag_file_path;
        params_.output_bag_path = output_path.string();
        
        ros::Time startup_time = ros::Time::now();
        
        // Run the processing
        ProcessingMetrics metrics = processBag(params_, startup_time);

        if (metrics.message_count == 0) {
             res.success = false;
             res.message = "Processing finished, but no pointcloud messages were found or processed.";
             ROS_WARN("%s", res.message.c_str());
        } else {
            res.success = true;
            res.message = "Bag processing completed successfully.";
            ROS_INFO("%s", res.message.c_str());
        }

        // Print the performance metrics
        printMetrics(metrics);

        return true;
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "toe_detection_kalman_from_bag_node");
    ToeDetectionKalmanNode node;
    ros::spin();
    return 0;
}