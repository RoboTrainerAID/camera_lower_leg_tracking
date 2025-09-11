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


int main(int argc, char **argv) {
    ros::init(argc, argv, "read_rosbag");
    ros::NodeHandle nh;

    // Get parameters including filtering/clustering values.
    std::string bag_file_path;
    std::string camera_depth_frame_id = "camera_depth_frame";
    double min_z, max_z, min_y, max_y, cluster_tolerance, downsample_point_size;
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
    ros::Duration pure_processing_duration(0.0);
    ros::Time first_msg_stamp, last_msg_stamp;
    bool is_first_message = true;
    int message_count = 0;
    int written_message_count = 0;

    for (const rosbag::MessageInstance& m : pc_view) {
        sensor_msgs::PointCloud2::ConstPtr pc_msg = m.instantiate<sensor_msgs::PointCloud2>();
        if (pc_msg != nullptr) {
            message_count++;
            if (is_first_message) {
                first_msg_stamp = pc_msg->header.stamp;
                is_first_message = false;
            }
            last_msg_stamp = pc_msg->header.stamp;

            ros::Time processing_start_time = ros::Time::now();

            Cloud_ptr input_pcl = boost::make_shared<Cloud>();
            pcl_df::fromROSMsg(*pc_msg, *input_pcl);
            // Call the updated functions with parameters.
            Cloud_ptr removedGround = removeGround(input_pcl, downsample_point_size, min_z, max_z, min_y, max_y, transformStamped);
            std::vector<Cloud_ptr> legs = splitLegs(removedGround, cluster_tolerance, min_cluster_size);
            
            if (legs.size() == 2 && !legs[0]->empty() && !legs[1]->empty()) {
                geometry_msgs::PoseArray toe_positions;
                toe_positions.header.stamp = pc_msg->header.stamp;
                toe_positions.header.frame_id = "base_link";
                toe_positions.poses.resize(2);
                toe_positions.poses[0].position = findToe(legs[0]);
                toe_positions.poses[1].position = findToe(legs[1]);
                
                pure_processing_duration += (ros::Time::now() - processing_start_time);
                
                outBag.write("toe_positions", pc_msg->header.stamp, toe_positions);
                written_message_count++;
                ROS_INFO("Wrote toe positions for timestamp %f", pc_msg->header.stamp.toSec());
            }
        }
    }

    ros::Time total_loop_end_time = ros::Time::now();
    ros::Duration total_loop_duration = total_loop_end_time - total_loop_start_time;
    ros::Duration bag_duration = last_msg_stamp - first_msg_stamp;

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