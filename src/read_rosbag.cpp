#include <ros/ros.h>
#include "../include/toe_detection.h"

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

#include <iirob_filters/kalman_filter.h>
typedef iirob_filters::MultiChannelKalmanFilter<double> KalmanFilter;
KalmanFilter* kf;

void print_state(const std::string& label, const std::vector<double>& state) {
    std::cout << label << ": [";
    for (size_t i = 0; i < state.size(); ++i) {
        std::cout << state[i] << (i == state.size() - 1 ? "" : ", ");
    }
    std::cout << "]" << std::endl;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "read_rosbag");
    ros::NodeHandle nh;

    kf = new KalmanFilter();

    // Get parameters including filtering/clustering values.
    std::string bag_file_path;
    std::string camera_depth_frame_id = "camera_depth_frame";
    double min_z, max_z, min_y, max_y, cluster_tolerance, downsample_point_size, target_frequency, max_prediction_time;
    int min_cluster_size;

    ros::param::param<std::string>("~/bag_file_path", bag_file_path, "test.bag");
    ros::param::param<std::string>("~/camera_depth_frame_id", camera_depth_frame_id, "camera_depth_frame");
    ros::param::param<double>("~/min_z", min_z, 0.01);
    ros::param::param<double>("~/max_z", max_z, 0.3);
    ros::param::param<double>("~/min_y", min_y, -0.4);
    ros::param::param<double>("~/max_y", max_y, 0.4);
    ros::param::param<double>("~/cluster_tolerance", cluster_tolerance, 0.03);
    ros::param::param<double>("~/downsample_point_size", downsample_point_size, 0.01);
    ros::param::param<int>("~/min_cluster_size", min_cluster_size, 150);
    ros::param::param<double>("~/target_frequency", target_frequency, 20.0); // Added target frequency parameter
    ros::param::param<double>("~/max_prediction_time", max_prediction_time, 0.25); // Max time to predict into the future

    rosbag::Bag bag;
    try {
        bag.open(bag_file_path, rosbag::bagmode::Read);
    } catch(rosbag::BagIOException &ex){
        ROS_ERROR("Error opening bag file: %s", ex.what());
        return 1;
    }
    
    // Define topics to read.
    std::vector<std::string> tf_topics = {"/tf_static", "/tf"};
    std::vector<std::string> pc_topics = {"/lower_legs_camera/depth_registered/points"};

    // Create a view to iterate over tf messages and populate the buffer.
    rosbag::View tf_view(bag, rosbag::TopicQuery(tf_topics));
    tf2_ros::Buffer tfBuffer;
    for (const rosbag::MessageInstance& m : tf_view) {
        if (m.getDataType() == "tf2_msgs/TFMessage") {
            tf2_msgs::TFMessage::ConstPtr tf_msg = m.instantiate<tf2_msgs::TFMessage>();
            if (tf_msg != nullptr) {
                for (const auto &transform : tf_msg->transforms) {
                    try {
                        tfBuffer.setTransform(transform, "bag", true);
                    } catch (tf2::TransformException &ex) {
                        ROS_WARN("Failed to set transform: %s", ex.what());
                    }
                }
            }
        }
    }

    geometry_msgs::TransformStamped transformStamped;
    try {
        transformStamped = tfBuffer.lookupTransform("base_link", camera_depth_frame_id, ros::Time(0));
        ROS_INFO("Transform from base_link to %s obtained.", camera_depth_frame_id.c_str());
    } catch (tf2::TransformException &ex) {
        ROS_ERROR("Error looking up transform: %s", ex.what());
        return 1;
    }

    // Process the bag and write toe positions.
    rosbag::View pc_view(bag, rosbag::TopicQuery(pc_topics));
    rosbag::Bag outBag;
    outBag.open("/home/docker/ros_ws/data/toe_positions.bag", rosbag::bagmode::Write);

    ros::Time total_loop_start_time = ros::Time::now();
    ros::Duration pure_processing_duration(0.0), pure_kalman_duration(0.0);
    ros::Time first_msg_stamp, last_real_stamp;
    bool is_first_message = true;
    int message_count = 0;
    int written_message_count = 0;
    const ros::Duration target_period(1.0 / target_frequency);

    for (const rosbag::MessageInstance& m : pc_view) {
        sensor_msgs::PointCloud2::ConstPtr pc_msg = m.instantiate<sensor_msgs::PointCloud2>();
        if (pc_msg == nullptr) continue;

        message_count++;
        ros::Time current_msg_stamp = pc_msg->header.stamp;

        if (is_first_message) {
            first_msg_stamp = current_msg_stamp;
            
            is_first_message = false;
        }

        // --- Main Processing ---
        ros::Time processing_start_time = ros::Time::now();
        Cloud_ptr input_pcl = boost::make_shared<Cloud>();
        pcl_df::fromROSMsg(*pc_msg, *input_pcl);
        Cloud_ptr removedGround = removeGround(input_pcl, downsample_point_size, min_z, max_z, min_y, max_y, transformStamped);
        std::vector<Cloud_ptr> legs = splitLegs(removedGround, cluster_tolerance, min_cluster_size);
        geometry_msgs::Point left_toe, right_toe;
        
        // Left leg is always [0], right leg [1]
        if (legs.size() != 2) {
            ROS_WARN("Vector length not as expected, size is: %zu. Skipping this message.", legs.size());
            continue;
        }
        if (!legs[0]->empty()) {
            left_toe = findToe(legs[0]);
        }
        if (!legs[1]->empty()) {
            right_toe = findToe(legs[1]);
        }

        pure_processing_duration += (ros::Time::now() - processing_start_time);
        ros::Time kalman_start_time = ros::Time::now();
        
        if(left_toe.x != 0.0 && left_toe.y != 0.0 && left_toe.z != 0.0) {
            // Valid left toe detected
            // --- Prediction and Update Logic ---
            if (kf->isInitializated()) {
                // 1. Predict to fill the gap from previous msg if necessary
                ros::Duration time_gap = current_msg_stamp - last_real_stamp;
                if ((time_gap > target_period)) {

                    ros::Duration prediction_gap = time_gap;
                    // Limit the prediction time to the configured maximum
                    if (time_gap.toSec() > max_prediction_time) {
                        ROS_WARN("Large time gap detected (%.2f s), limiting prediction time to %.2f s.", time_gap.toSec(), max_prediction_time);
                        prediction_gap = ros::Duration(max_prediction_time);
                    }

                    int num_predictions_needed = static_cast<int>(prediction_gap.toSec() / target_period.toSec());

                    for (int i = 0; i < num_predictions_needed; ++i) {
                        std::vector<double> predicted_state;
                        
                        // This does NOT advance the filter's internal state
                        // It just computes what the state would be after the given time interval
                        kf->computePrediction(predicted_state, target_period.toSec() * (i + 1));

                        pure_kalman_duration += (ros::Time::now() - kalman_start_time);

                        geometry_msgs::PoseArray predicted_toes;
                        ros::Time predicted_stamp = last_real_stamp + target_period * (i + 1);
                        predicted_toes.header.stamp = predicted_stamp;
                        predicted_toes.header.frame_id = "base_link";
                        predicted_toes.poses.resize(1); // Only one predicted pose
                        predicted_toes.poses[0].position.x = predicted_state[0];
                        predicted_toes.poses[0].position.y = predicted_state[1];
                        predicted_toes.poses[0].position.z = predicted_state[2];

                        // geometry_msgs::Point vel;
                        // vel.x = predicted_state[3];
                        // vel.y = predicted_state[4];
                        // vel.z = predicted_state[5];
                        // geometry_msgs::Point acc;
                        // acc.x = predicted_state[6];
                        // acc.y = predicted_state[7];
                        // acc.z = predicted_state[8];

                        // outBag.write("toe_velocities", predicted_stamp, vel);
                        // outBag.write("toe_accelerations", predicted_stamp, acc);

                        ROS_INFO("Predicted toe position at t+%.2f s: [%.3f, %.3f, %.3f]", (predicted_stamp - last_real_stamp).toSec(), predicted_state[0], predicted_state[1], predicted_state[2]);
                        
                        outBag.write("toe_positions_kalman", predicted_stamp, predicted_toes);
                    }
                }
            }

            // 2. Update the filter with the new real measurement
            std::vector<double> measurement = {left_toe.x, left_toe.y, left_toe.z};
            std::vector<double> corrected_state;

            if (!kf->isInitializated()) {
                // Initialize the filter state with the first measurement
                std::vector<double> initial_state = {left_toe.x, left_toe.y, left_toe.z, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
                if (kf->configure(initial_state, "KalmanFilter")) {
                    ROS_INFO("Kalman Filter initialized with first measurement.");
                    pure_kalman_duration += (ros::Time::now() - kalman_start_time);
                } else {
                    ROS_ERROR("Failed to initialize Kalman Filter with first measurement.");
                    return -1;
                }
            }
            else {
            
                double sensor_dt = (current_msg_stamp - last_real_stamp).toSec();

                if (sensor_dt <= 0.0) {
                    ROS_WARN("Sensor dt is non-positive (%.3f s). Setting to 0.1 s to avoid issues.", sensor_dt);
                    sensor_dt = 0.1;
                }

                if (kf->update(measurement, corrected_state, sensor_dt, true)) {
                    pure_kalman_duration += (ros::Time::now() - kalman_start_time);
                    geometry_msgs::PoseArray corrected_toes;
                    corrected_toes.header.stamp = current_msg_stamp;
                    corrected_toes.header.frame_id = "base_link";
                    corrected_toes.poses.resize(1);
                    corrected_toes.poses[0].position.x = corrected_state[0];
                    corrected_toes.poses[0].position.y = corrected_state[1];
                    corrected_toes.poses[0].position.z = corrected_state[2];

                    // geometry_msgs::Point vel;
                    // vel.x = corrected_state[3];
                    // vel.y = corrected_state[4];
                    // vel.z = corrected_state[5];
                    // geometry_msgs::Point acc;
                    // acc.x = corrected_state[6];
                    // acc.y = corrected_state[7];
                    // acc.z = corrected_state[8];

                    // outBag.write("toe_velocities", current_msg_stamp, vel);
                    // outBag.write("toe_accelerations", current_msg_stamp, acc);

                    outBag.write("toe_positions_kalman", current_msg_stamp, corrected_toes);
                } else {
                    ROS_WARN("Kalman Filter update failed");
                }


                geometry_msgs::PoseArray measured_toes;
                measured_toes.header.stamp = current_msg_stamp;
                measured_toes.header.frame_id = "base_link";
                measured_toes.poses.resize(1);
                measured_toes.poses[0].position = left_toe;
                outBag.write("toe_positions", current_msg_stamp, measured_toes);

                written_message_count++;
            }
            last_real_stamp = current_msg_stamp;
        }
    }

    ros::Time total_loop_end_time = ros::Time::now();
    ros::Duration total_loop_duration = total_loop_end_time - total_loop_start_time;
    ros::Duration bag_duration = last_real_stamp - first_msg_stamp;

    double message_loss_percentage = 0.0;
    if (message_count > 0) {
        message_loss_percentage = (1.0 - static_cast<double>(written_message_count) / message_count) * 100.0;
    }

    ROS_INFO("================================================");
    ROS_INFO("Bag Processing Performance Metrics:");
    ROS_INFO("Total PCL messages read: %d", message_count);
    ROS_INFO("Toe position messages written: %d", written_message_count);
    ROS_INFO("Message loss: %.2f%%", message_loss_percentage);
    ROS_INFO("Total loop time (read + process + write): %.4f s", total_loop_duration.toSec());
    ROS_INFO("Pure PCL processing time: %.4f s", pure_processing_duration.toSec());
    ROS_INFO("Pure Kalman filter time: %.4f s", pure_kalman_duration.toSec());
    ROS_INFO("Bag duration (time between first/last msg): %.4f s", bag_duration.toSec());
    if (bag_duration.toSec() > 0) {
        double msgs_per_sec = message_count / bag_duration.toSec();
        ROS_INFO("Messages per second (bag time): %.2f Hz", msgs_per_sec);
    }
    if (total_loop_duration.toSec() > 0) {
        double real_time_factor = bag_duration.toSec() / total_loop_duration.toSec();
        ROS_INFO("Real-time factor: %.2fx", real_time_factor);
    }
    ROS_INFO("================================================");

    outBag.close();
    bag.close();
    ros::shutdown();
    return 0;
}