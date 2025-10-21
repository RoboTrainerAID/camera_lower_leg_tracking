#ifndef KALMAN_PROCESSING_FROM_BAG_H
#define KALMAN_PROCESSING_FROM_BAG_H

#include <ros/ros.h>
#include <string>
#include <vector>

// PCL + Kalman + BAG parameters.
struct ProcessingParameters {
    std::string input_bag_path, output_bag_path, camera_depth_frame_id;
    double min_z, max_z, min_y, max_y, cluster_tolerance, downsample_point_size;
    double target_frequency, max_prediction_time, likelihood_threshold, swap_distance_threshold_ratio;
    int min_cluster_size;
    std::vector<std::string> tf_topics, pc_topics;
};

// Metrics collected during bag processing.
struct ProcessingMetrics {
    ros::Duration total_loop_duration{0.0}, pure_processing_duration{0.0}, pure_kalman_duration{0.0};
    ros::Duration tf_duration{0.0}, startup_duration{0.0}, bag_duration{0.0};
    int message_count = 0, written_message_count = 0, predicted_message_count = 0;
};

// Main function to process a rosbag file.
ProcessingMetrics processBag(const ProcessingParameters& params, const ros::Time& startup_time);

// Prints the final metrics to the console.
void printMetrics(const ProcessingMetrics& metrics);

#endif // KALMAN_PROCESSING_FROM_BAG_H