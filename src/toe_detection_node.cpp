#include <ros/ros.h>
#include "../include/toe_detection.h"

double MIN_Z, MAX_Z, MIN_Y, MAX_Y, CLUSTER_TOLERANCE, DONWSAMPLE_POINT_SIZE;
bool PUBLISH_DEBUG;
int MIN_CLUSTER_SIZE;
std::string INPUT_POINTCLOUD_TOPIC, CAMERA_DEPTH_FRAME_ID;

geometry_msgs::TransformStamped transformStamped;
ros::Publisher pub_left_leg, pub_right_leg, pub_left_toe, pub_right_toe, pub_debug, pub_toes;

// With splitLegs a vector will be filled where the first entry is left and and the second is right.
// If there is no Cluster then legs will stay empty.
// If both legs are one cluster it will get split in the middle
// If only one leg is in the frame its Cloud will be as expected. The other Cloud will be empty.
void cloud_cb(const sensor_msgs::PointCloud2 &input_cloud) {
    ros::Time start = ros::Time::now();

    // Conversion
    Cloud_ptr input_pcl = boost::make_shared<Cloud>();
    pcl_df::fromROSMsg(input_cloud, *input_pcl);

    // ros::Time conTime = ros::Time::now();
    // ROS_INFO("Conversion TOOK %f SECONDS", (conTime - start).toSec());
    Cloud_ptr removedGround = removeGround(input_pcl, DONWSAMPLE_POINT_SIZE, MIN_Z, MAX_Z, MIN_Y, MAX_Y, transformStamped);
    
    // ros::Time ground = ros::Time::now();
    // ROS_INFO("REMOVE GROUND TOOK %f SECONDS", (ground - conTime).toSec());
    if(PUBLISH_DEBUG) {
        pub_debug.publish(*removedGround);
    }

    std::vector<Cloud_ptr> legs = splitLegs(removedGround, CLUSTER_TOLERANCE, MIN_CLUSTER_SIZE);
    // ros::Time split = ros::Time::now();
    // ROS_INFO("SPLIT LEGS TOOK %f SECONDS", (split - ground).toSec());
    
    if (legs.size() == 2) {
        if (!legs[0]->empty() && !legs[1]->empty()) {
            geometry_msgs::PoseArray toe_positions;
            toe_positions.header.stamp = input_cloud.header.stamp;
            toe_positions.header.frame_id = "base_link";
            toe_positions.header.seq++;
            toe_positions.poses.resize(2);
            toe_positions.poses[0].position = findToe(legs[0]);
            toe_positions.poses[1].position = findToe(legs[1]);
            pub_toes.publish(toe_positions);
        }
    }
    ros::Time end = ros::Time::now();
    if (PUBLISH_DEBUG) {
        ROS_INFO("THIS CALLBACK TOOK %f SECONDS", (end - start).toSec());
    }
}

int main (int argc, char** argv) {
    // Initialize ROS
    ros::init (argc, argv, "toe_detection");
    ros::NodeHandle nh("~");

    nh.param("min_z", MIN_Z, 0.01);
    nh.param("max_z", MAX_Z, 0.3);
    nh.param("min_y", MIN_Y, -0.4);
    nh.param("max_y", MAX_Y, 0.4);
    nh.param("min_cluster_size", MIN_CLUSTER_SIZE, 150);
    nh.param("cluster_tolerance", CLUSTER_TOLERANCE, 0.03);
    nh.param("downsample_point_size", DONWSAMPLE_POINT_SIZE, 0.01);
    nh.param("publish_debug", PUBLISH_DEBUG, false);
    nh.param("input_pointcloud_topic", INPUT_POINTCLOUD_TOPIC, std::string("/camera/depth_registered/points"));
    nh.param("camera_depth_frame_id", CAMERA_DEPTH_FRAME_ID, std::string("camera_rgb_optical_frame"));

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);

    bool waitingPrinted = false;
    while (ros::ok()) {
        try {
            transformStamped = tfBuffer.lookupTransform("base_link", CAMERA_DEPTH_FRAME_ID, ros::Time(0), ros::Duration(5));
            break; // Successfully retrieved the transform.
        } catch (tf2::TransformException &ex) {
            if (!waitingPrinted) {
                ROS_ERROR("Waiting for base_link frame to appear...");
                waitingPrinted = true;
            }
            ros::Duration(0.1).sleep();
        }
    }

    // Create a ROS subscriber for the input point cloud
    ros::Subscriber sub = nh.subscribe (INPUT_POINTCLOUD_TOPIC, 1, cloud_cb);

    pub_toes = nh.advertise<geometry_msgs::PoseArray>("toe_positions", 1);
    // pub_left_toe = nh.advertise<geometry_msgs::PointStamped>("left_toe", 1);
    // pub_right_toe = nh.advertise<geometry_msgs::PointStamped>("right_toe", 1);

    if (PUBLISH_DEBUG) {
        pub_left_leg = nh.advertise<sensor_msgs::PointCloud2>("left_leg", 1);
        pub_right_leg = nh.advertise<sensor_msgs::PointCloud2>("right_leg", 1);
        pub_debug = nh.advertise<sensor_msgs::PointCloud2>("toe_debug", 1);
    }

    ros::spin();
    return 0;
}