#include <ros/ros.h>
#include "../include/kalman_processing_from_bag.h"
#include "../include/toe_detection.h"
#include "../include/frequency_locked_kalman_filter.h"
#include <std_msgs/Float64.h>
#include <deque>
#include <numeric>

// The following block is a workaround for the fact that rosbag includes lz4.h which defines symbols that conflict with pcl's use of lz4.
// See https://github.com/ethz-asl/lidar_align/issues/16#issuecomment-504348488
#define LZ4_stream_t LZ4_stream_t_deprecated
#define LZ4_resetStream LZ4_resetStream_deprecated
#define LZ4_createStream LZ4_createStream_deprecated
#define LZ4_freeStream LZ4_freeStream_deprecated
#define LZ4_loadDict LZ4_loadDict_deprecated
#define LZ4_compress_fast_continue LZ4_compress_fast_continue_deprecated
#define LZ4_saveDict LZ4_saveDict_deprecated
#define LZ4_streamDecode_t LZ4_streamDecode_t_deprecated
#define LZ4_compress_continue LZ4_compress_continue_deprecated
#define LZ4_compress_limitedOutput_continue LZ4_compress_limitedOutput_continue_deprecated
#define LZ4_createStreamDecode LZ4_createStreamDecode_deprecated
#define LZ4_freeStreamDecode LZ4_freeStreamDecode_deprecated
#define LZ4_setStreamDecode LZ4_setStreamDecode_deprecated
#define LZ4_decompress_safe_continue LZ4_decompress_safe_continue_deprecated
#define LZ4_decompress_fast_continue LZ4_decompress_fast_continue_deprecated
#include <rosbag/bag.h>
#include <rosbag/view.h>
#undef LZ4_stream_t
#undef LZ4_resetStream
#undef LZ4_createStream
#undef LZ4_freeStream
#undef LZ4_loadDict
#undef LZ4_compress_fast_continue
#undef LZ4_saveDict
#undef LZ4_streamDecode_t
#undef LZ4_compress_continue
#undef LZ4_compress_limitedOutput_continue
#undef LZ4_createStreamDecode
#undef LZ4_freeStreamDecode
#undef LZ4_setStreamDecode
#undef LZ4_decompress_safe_continue
#undef LZ4_decompress_fast_continue

// Helper function to calculate 2D Euclidean distance between two points
double pointDistance2D(const geometry_msgs::Point& p1, const geometry_msgs::Point& p2) {
    return std::sqrt(std::pow(p1.x - p2.x, 2) + std::pow(p1.y - p2.y, 2));
}

// Helper function to calculate the average of a deque of points
geometry_msgs::Point getAveragePoint(const std::deque<geometry_msgs::Point>& points) {
    geometry_msgs::Point avg;
    avg.x = 0; avg.y = 0; avg.z = 0;
    if (points.empty()) {
        return avg;
    }
    for (const auto& p : points) {
        avg.x += p.x;
        avg.y += p.y;
        avg.z += p.z;
    }
    avg.x /= points.size();
    avg.y /= points.size();
    avg.z /= points.size();
    return avg;
}

void print_state(const std::string& label, const std::vector<double>& state) {
    std::cout << label << ": [";
    for (size_t i = 0; i < state.size(); ++i) {
        std::cout << state[i] << (i == state.size() - 1 ? "" : ", ");
    }
    std::cout << "]" << std::endl;
}

ProcessingMetrics processBag(const ProcessingParameters& params, const ros::Time& startup_time) {
    ProcessingMetrics metrics;
    rosbag::Bag bag, outBag;

    try {
        bag.open(params.input_bag_path, rosbag::bagmode::Read);
        ROS_INFO("Reading from bag file: %s", params.input_bag_path.c_str());
    } catch(rosbag::BagIOException &ex){
        ROS_ERROR("Error opening bag file: %s", ex.what());
        return metrics;
    }

    try {
        outBag.open(params.output_bag_path, rosbag::bagmode::Write);
        ROS_INFO("Writing to output bag file: %s", params.output_bag_path.c_str());
    } catch(rosbag::BagIOException &ex){
        ROS_ERROR("Error opening output bag file: %s", ex.what());
        bag.close();
        return metrics;
    }

    // Instantiate two Kalman Filter wrappers, one for each toe
    FrequencyLockedKalmanFilter kf_left_wrapper("KalmanFilter", params.target_frequency, params.max_prediction_time, params.likelihood_threshold);
    FrequencyLockedKalmanFilter kf_right_wrapper("KalmanFilter", params.target_frequency, params.max_prediction_time, params.likelihood_threshold);

    // Create a view to iterate over tf messages and populate the buffer.
    ros::Time tf_processing_start_time = ros::Time::now();
    tf2_ros::Buffer tfBuffer;
    rosbag::View tf_view(bag, rosbag::TopicQuery(params.tf_topics));
    for (const rosbag::MessageInstance& m : tf_view) {
        if (m.getDataType() == "tf2_msgs/TFMessage") {
            tf2_msgs::TFMessage::ConstPtr tf_msg = m.instantiate<tf2_msgs::TFMessage>();
            if (tf_msg) {
                for (const auto &transform : tf_msg->transforms) {
                    try {
                        tfBuffer.setTransform(transform, "bag", true);
                    } catch (tf2::TransformException &ex) {
                        ROS_WARN("Failed to set transform: %s", ex.what());
                    }
                }
                outBag.write(m.getTopic(), m.getTime(), m);
            }
        }
    }

    geometry_msgs::TransformStamped transformStamped;
    try {
        transformStamped = tfBuffer.lookupTransform("base_link", params.camera_depth_frame_id, ros::Time(0));
        ROS_INFO("Transform from base_link to %s obtained.", params.camera_depth_frame_id.c_str());
    } catch (tf2::TransformException &ex) {
        ROS_ERROR("Error looking up transform: %s", ex.what());
        bag.close();
        outBag.close();
        return metrics;
    }

    ros::Time total_loop_start_time = ros::Time::now();
    metrics.tf_duration = total_loop_start_time - tf_processing_start_time;
    metrics.startup_duration = total_loop_start_time - startup_time;

    ros::Time first_msg_stamp, last_real_stamp;
    bool is_first_message = true;
    std::deque<geometry_msgs::Point> left_history, right_history;
    const size_t history_size = 3;

    rosbag::View pc_view(bag, rosbag::TopicQuery(params.pc_topics));
    for (const rosbag::MessageInstance& m : pc_view) {
        if (!ros::ok()) {
            ROS_WARN("Shutdown signal received, stopping bag processing.");
            break;
        }

        sensor_msgs::PointCloud2::ConstPtr pc_msg = m.instantiate<sensor_msgs::PointCloud2>();
        if (!pc_msg) continue;

        metrics.message_count++;
        ros::Time current_msg_stamp = pc_msg->header.stamp;

        if (is_first_message) {
            first_msg_stamp = current_msg_stamp;
            last_real_stamp = current_msg_stamp;
            is_first_message = false;
        }

        // --- Main Processing ---
        ros::Time processing_start_time = ros::Time::now();
        Cloud_ptr input_pcl = boost::make_shared<Cloud>();
        pcl_df::fromROSMsg(*pc_msg, *input_pcl);
        Cloud_ptr removedGround = removeGround(input_pcl, params.downsample_point_size, params.min_z, params.max_z, params.min_y, params.max_y, transformStamped);
        std::vector<Cloud_ptr> legs = splitLegs(removedGround, params.cluster_tolerance, params.min_cluster_size);
        
        geometry_msgs::Point left_toe, right_toe, detected_left_toe, detected_right_toe;
        if (legs.size() != 2) {
            ROS_WARN("Vector length not as expected, size is: %zu. Skipping this message.", legs.size());
            continue;
        }
        if (!legs[0]->empty()) detected_left_toe = left_toe = findToe(legs[0]);
        if (!legs[1]->empty()) detected_right_toe = right_toe = findToe(legs[1]);
        metrics.pure_processing_duration += (ros::Time::now() - processing_start_time);
        
        // --- Swapping logic based on distance to rolling average or Likelihood ---
        bool grace_period_is_over = (current_msg_stamp - first_msg_stamp) > ros::Duration(10 * params.max_prediction_time);
        if (grace_period_is_over && (left_toe.x != 0.0 || left_toe.y != 0.0) && (right_toe.x != 0.0 || right_toe.y != 0.0) && !left_history.empty() && !right_history.empty()) {
            // Only perform swapping after an initial grace period to allow filters to stabilize

            // kf_left_wrapper.isInitialized() && kf_right_wrapper.isInitialized()) {
            // std::vector<double> left_meas = {left_toe.x, left_toe.y};
            // std::vector<double> right_meas = {right_toe.x, right_toe.y};

            // double ll = kf_left_wrapper.getLikelihood(left_meas);
            // double rr = kf_right_wrapper.getLikelihood(right_meas);
            // double lr = kf_left_wrapper.getLikelihood(right_meas);
            // double rl = kf_right_wrapper.getLikelihood(left_meas);

            // // Write likelihoods to bag
            // std_msgs::Float64 ll_msg, rr_msg, lr_msg, rl_msg;
            // ll_msg.data = ll;
            // rr_msg.data = rr;
            // lr_msg.data = lr;
            // rl_msg.data = rl;
            // outBag.write("/toe_position/likelihood/left_kf_left_toe", current_msg_stamp, ll_msg);
            // outBag.write("/toe_position/likelihood/right_kf_right_toe", current_msg_stamp, rr_msg);
            // outBag.write("/toe_position/likelihood/left_kf_right_toe", current_msg_stamp, lr_msg);
            // outBag.write("/toe_position/likelihood/right_kf_left_toe", current_msg_stamp, rl_msg);

            // // If the swapped configuration has a higher combined likelihood, swap the toes.
            // bool swap_condition = (lr + rl) > (ll + rr);

            // --- Swapping based on distance to previous measurements ---
            geometry_msgs::Point avg_left = getAveragePoint(left_history);
            geometry_msgs::Point avg_right = getAveragePoint(right_history);

            // Calculate sum of distances for non-swapped and swapped scenarios
            double dist_not_swapped = pointDistance2D(left_toe, avg_left) + pointDistance2D(right_toe, avg_right);
            double dist_swapped = pointDistance2D(left_toe, avg_right) + pointDistance2D(right_toe, avg_left);

            std_msgs::Float64 dist_swapped_msg, dist_not_swapped_msg;
            dist_swapped_msg.data = dist_swapped;
            dist_not_swapped_msg.data = dist_not_swapped;
            outBag.write("/toe_position/distance/swapped", current_msg_stamp, dist_swapped_msg);
            outBag.write("/toe_position/distance/not_swapped", current_msg_stamp, dist_not_swapped_msg);

            // Swap if the swapped distance is significantly smaller (e.g., less than ratio * non-swapped)
            if (dist_swapped < params.swap_distance_threshold_ratio * dist_not_swapped) {
                ROS_INFO("Swapping detected legs based on average distance.");
                std::swap(left_toe, right_toe);
            }
        }

        // Update history with current (potentially swapped) measurements
        if (left_toe.x != 0.0 || left_toe.y != 0.0) {
            left_history.push_back(left_toe);
            if (left_history.size() > history_size) left_history.pop_front();
        }
        if (right_toe.x != 0.0 || right_toe.y != 0.0) {
            right_history.push_back(right_toe);
            if (right_history.size() > history_size) right_history.pop_front();
        }

        bool any_toe_detected = false;
        ros::Time kalman_start_time = ros::Time::now();

        // --- Process Left Toe ---
        if (left_toe.x != 0.0 || left_toe.y != 0.0) {
            any_toe_detected = true;

            // Call the wrapper to get corrected and predicted states
            std::vector<KalmanState> states = kf_left_wrapper.update_and_predict_frequency_gap({left_toe.x, left_toe.y}, current_msg_stamp);
            
            // Iterate through all returned states and write them to the bag
            for (const auto& state_pair : states) {
                geometry_msgs::PointStamped toes_msg;
                toes_msg.header.stamp = state_pair.first;
                toes_msg.header.frame_id = "base_link";
                toes_msg.point.x = state_pair.second[0];
                toes_msg.point.y = state_pair.second[1];
                outBag.write("/toe_position/left/kalman", state_pair.first, toes_msg);
                metrics.predicted_message_count++;
            }
            // Also write the raw measurement for comparison
            geometry_msgs::PointStamped measured_toes;
            measured_toes.header.stamp = current_msg_stamp;
            measured_toes.header.frame_id = "base_link";
            measured_toes.point = detected_left_toe;
            outBag.write("/toe_position/left/original", current_msg_stamp, measured_toes);
            measured_toes.point = left_toe;
            outBag.write("/toe_position/left/swaped", current_msg_stamp, measured_toes);
        }

        // --- Process Right Toe ---
        if (right_toe.x != 0.0 || right_toe.y != 0.0) {
            any_toe_detected = true;

            // Call the wrapper to get corrected and predicted states
            std::vector<KalmanState> states = kf_right_wrapper.update_and_predict_frequency_gap({right_toe.x, right_toe.y}, current_msg_stamp);
            
            // Iterate through all returned states and write them to the bag
            for (const auto& state_pair : states) {
                geometry_msgs::PointStamped toes_msg;
                toes_msg.header.stamp = state_pair.first;
                toes_msg.header.frame_id = "base_link";
                toes_msg.point.x = state_pair.second[0];
                toes_msg.point.y = state_pair.second[1];
                outBag.write("/toe_position/right/kalman", state_pair.first, toes_msg);
                metrics.predicted_message_count++;
            }
            // Also write the raw measurement for comparison
            geometry_msgs::PointStamped measured_toes;
            measured_toes.header.stamp = current_msg_stamp;
            measured_toes.header.frame_id = "base_link";
            measured_toes.point = detected_right_toe;
            outBag.write("/toe_position/right/original", current_msg_stamp, measured_toes);
            measured_toes.point = right_toe;
            outBag.write("/toe_position/right/swaped", current_msg_stamp, measured_toes);
        }
        
        metrics.pure_kalman_duration += (ros::Time::now() - kalman_start_time);

        if (any_toe_detected) {
            metrics.written_message_count++;
            last_real_stamp = current_msg_stamp;
        }
    }

    metrics.total_loop_duration = ros::Time::now() - total_loop_start_time;
    if (!is_first_message) {
        metrics.bag_duration = last_real_stamp - first_msg_stamp;
    }

    outBag.close();
    bag.close();
    return metrics;
}

void printMetrics(const ProcessingMetrics& metrics) {
    double message_loss_percentage = 0.0;
    if (metrics.message_count > 0) {
        message_loss_percentage = (1.0 - static_cast<double>(metrics.written_message_count) / metrics.message_count) * 100.0;
    }

    ROS_INFO("================================================");
    ROS_INFO("Bag Processing Performance Metrics:");
    ROS_INFO("Total PCL messages read: %d", metrics.message_count);
    ROS_INFO("Toe position messages written: %d", metrics.written_message_count);
    ROS_INFO("Message loss: %.2f%%", message_loss_percentage);
    ROS_INFO("Kalman predicted messages written: %d", metrics.predicted_message_count);
    ROS_INFO("Total loop time (read + process + write): %.4f s", metrics.total_loop_duration.toSec());
    ROS_INFO("Startup time (until first bag read): %.4f s", metrics.startup_duration.toSec());
    ROS_INFO("TF processing time: %.4f s", metrics.tf_duration.toSec());
    ROS_INFO("Pure PCL processing time: %.4f s", metrics.pure_processing_duration.toSec());
    ROS_INFO("Pure Kalman filter time: %.4f s", metrics.pure_kalman_duration.toSec());
    ROS_INFO("Bag duration (time between first/last msg): %.4f s", metrics.bag_duration.toSec());
    if (metrics.bag_duration.toSec() > 0) {
        double msgs_per_sec = metrics.message_count / metrics.bag_duration.toSec();
        ROS_INFO("Messages per second (bag time): %.2f Hz", msgs_per_sec);
    }
    if (metrics.total_loop_duration.toSec() > 0) {
        double real_time_factor = metrics.bag_duration.toSec() / metrics.total_loop_duration.toSec();
        ROS_INFO("Real-time factor: %.2fx", real_time_factor);
    }
    ROS_INFO("================================================");
}