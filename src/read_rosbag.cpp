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

    // 1. Open the bag file for reading
    rosbag::Bag bag;
    std::string bag_file_path;
    std::string camera_depth_frame_id = "camera_depth_frame";
    geometry_msgs::TransformStamped transformStamped;
    ros::param::param<std::string>("~/bag_file_path", bag_file_path, "test.bag");
    ros::param::param<std::string>("~/camera_depth_frame_id", camera_depth_frame_id, "camera_depth_frame");

    try {
        bag.open(bag_file_path, rosbag::bagmode::Read);
    } catch(rosbag::BagIOException &ex){
        ROS_ERROR("Error opening bag file: %s", ex.what());
        return 1;
    }
    
    // 2. Define topics to read (tf and pointcloud topics)
    std::vector<std::string> tf_topics;
    tf_topics.push_back("/tf_static");
    tf_topics.push_back("/tf");
    // Also include pointcloud topic for later processing.
    std::vector<std::string> pc_topics;
    pc_topics.push_back("/lower_legs_camera/depth_registered/points");

    // 3. Create a view to iterate over tf messages
    rosbag::View tf_view(bag, rosbag::TopicQuery(tf_topics));

    // 4. Create a tf2 Buffer to hold transforms (populated manually)
    tf2_ros::Buffer tfBuffer;
    // No TransformListener is used since we add the transforms manually.
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

    // 5. Lookup the composite transform.
    try {
        transformStamped = tfBuffer.lookupTransform("base_link", camera_depth_frame_id, ros::Time(0));
        ROS_INFO("Transform from base_link to %s obtained.", camera_depth_frame_id.c_str());
    } catch (tf2::TransformException &ex) {
        ROS_ERROR("Error looking up transform: %s", ex.what());
        return 1;
    }

    // 8. Iterate over the bag and process pointclouds
    rosbag::View pc_view(bag, rosbag::TopicQuery(pc_topics));

    rosbag::Bag outBag;
    outBag.open("toe_positions.bag", rosbag::bagmode::Write);

    for (const rosbag::MessageInstance& m : pc_view) {
        sensor_msgs::PointCloud2::ConstPtr pc_msg = m.instantiate<sensor_msgs::PointCloud2>();
        if (pc_msg != nullptr) {
            // Conversion: sensor_msgs::PointCloud2 -> PCL Cloud.
            Cloud_ptr input_pcl = boost::make_shared<Cloud>();
            pcl_df::fromROSMsg(*pc_msg, *input_pcl);

            // Process the pointcloud: remove ground, split legs, then find toe positions.
            Cloud_ptr removedGround = removeGround(input_pcl);
            std::vector<Cloud_ptr> legs = splitLegs(removedGround);

            if (legs.size() == 2 && !legs[0]->empty() && !legs[1]->empty()) {
                geometry_msgs::PoseArray toe_positions;
                toe_positions.header.stamp = pc_msg->header.stamp;
                toe_positions.header.frame_id = "base_link";
                toe_positions.poses.resize(2);
                toe_positions.poses[0].position = findToe(legs[0]);
                toe_positions.poses[1].position = findToe(legs[1]);

                // Write the toe_positions message to the new bag on topic "toe_positions".
                outBag.write("toe_positions", pc_msg->header.stamp, toe_positions);
                ROS_INFO("Wrote toe positions for timestamp %f", pc_msg->header.stamp.toSec());
            }
        }
    }

    outBag.close();
    bag.close();
    ros::shutdown();
    return 0;
}