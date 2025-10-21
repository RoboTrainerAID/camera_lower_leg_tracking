#include <ros/ros.h>
#include "../include/kalman_processing_from_bag.h"


int main(int argc, char **argv) {
    ros::init(argc, argv, "toe_detection_kalman_from_bag");
    ros::NodeHandle nh;
    ros::Time startup_time = ros::Time::now();

    ProcessingParameters params;
    ros::param::param<std::string>("~/input_bag_path", params.input_bag_path, "test.bag");
    ros::param::param<std::string>("~/output_bag_path", params.output_bag_path, "toe_positions_output.bag");
    ros::param::param<std::string>("~/camera_depth_frame_id", params.camera_depth_frame_id, "camera_depth_frame");
    ros::param::param<double>("~/min_z", params.min_z, 0.01);
    ros::param::param<double>("~/max_z", params.max_z, 0.3);
    ros::param::param<double>("~/min_y", params.min_y, -0.4);
    ros::param::param<double>("~/max_y", params.max_y, 0.4);
    ros::param::param<double>("~/cluster_tolerance", params.cluster_tolerance, 0.03);
    ros::param::param<double>("~/downsample_point_size", params.downsample_point_size, 0.01);
    ros::param::param<int>("~/min_cluster_size", params.min_cluster_size, 150);
    ros::param::param<double>("~/target_frequency", params.target_frequency, 20.0);
    ros::param::param<double>("~/max_prediction_time", params.max_prediction_time, 0.25); // Max time to predict into the future
    ros::param::param<double>("~/likelihood_threshold", params.likelihood_threshold, 0.6); // Min likelihood, outliers below will be rejected
    ros::param::param<double>("~/swap_distance_threshold_ratio", params.swap_distance_threshold_ratio, 0.5); // Ratio for distance-based swapping, e.g. 0.5 means swapped distance must be less than half of non-swapped to trigger swap

    // Define topics to read.
    params.tf_topics = {"/tf_static", "/tf"};
    params.pc_topics = {"/lower_legs_camera/depth_registered/points"};

    // Process the bag and write toe positions to new bag file.
    ProcessingMetrics metrics = processBag(params, startup_time);

    printMetrics(metrics);

    ros::shutdown();
    return 0;
}