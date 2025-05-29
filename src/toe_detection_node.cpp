#include "ros/ros.h"
#include "../include/pcl_types.h"
#include "../include/rosPointCloud2ToPCL.hpp"
// Drop in replacement for pcl::fromROSMsg (Davide Faconti)


double MIN_Z, MAX_Z, MIN_Y, MAX_Y, CLUSTER_TOLERANCE, DONWSAMPLE_POINT_SIZE;
bool PUBLISH_DEBUG;
int MIN_CLUSTER_SIZE;
std::string INPUT_POINTCLOUD_TOPIC, CAMERA_DEPTH_FRAME_ID;

geometry_msgs::TransformStamped transformStamped;
ros::Publisher pub_left_leg, pub_right_leg, pub_left_toe, pub_right_toe, pub_debug, pub_toes;

Cloud_ptr removeGround(const Cloud_ptr &input_pcl) {
    // ros::Time start = ros::Time::now();

    // Avoid copies by using shared pointers
    Cloud_ptr transformed_pcl   = boost::make_shared<Cloud>();
    Cloud_ptr filtered_z_pcl    = boost::make_shared<Cloud>();
    Cloud_ptr filtered_y_pcl    = boost::make_shared<Cloud>();
    Cloud_ptr downsampled_pcl   = boost::make_shared<Cloud>();

    // Downsampling
    pcl::VoxelGrid<Point> vg;
    vg.setInputCloud(input_pcl);
    vg.setLeafSize(DONWSAMPLE_POINT_SIZE, DONWSAMPLE_POINT_SIZE, DONWSAMPLE_POINT_SIZE);
    vg.filter(*downsampled_pcl);

    // ros::Time downsampling = ros::Time::now();
    // ROS_INFO("DOWNSAMPLING TOOK %f SECONDS", (downsampling - start).toSec());

    // Transform
    pcl::transformPointCloud(*downsampled_pcl, *transformed_pcl, tf2::transformToEigen(transformStamped).matrix());
    transformed_pcl->header.frame_id = "base_link";

    // ros::Time transTime = ros::Time::now();
    // ROS_INFO("Transformation TOOK %f SECONDS", (transTime - downsampling).toSec());

    // Remove ground with PassThrough (z-based)
    pcl::PassThrough<Point> pass;
    pass.setInputCloud(transformed_pcl);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(MIN_Z, MAX_Z);
    pass.filter(*filtered_z_pcl);

    // Further filter by y-range
    pcl::PassThrough<Point> pass_y;
    pass_y.setInputCloud(filtered_z_pcl);
    pass_y.setFilterFieldName("y");
    pass_y.setFilterLimits(-0.4, 0.4);
    pass_y.filter(*filtered_y_pcl);

    // ros::Time passTime = ros::Time::now();
    // ROS_INFO("PASSTHROUGH TOOK %f SECONDS", (passTime - transTime).toSec());

    return filtered_y_pcl;
}

std::vector<Cloud_ptr> findOrientation(Cloud_ptr fst_leg, Cloud_ptr snd_leg) {
    std::vector<Cloud_ptr> legs;
    Point fst_centroid, snd_centroid;

    pcl::computeCentroid(*fst_leg, fst_centroid);
    pcl::computeCentroid(*snd_leg, snd_centroid);

    if (fst_centroid.y > snd_centroid.y) {
        legs.push_back(fst_leg);
        legs.push_back(snd_leg);
    } else {
        legs.push_back(snd_leg);
        legs.push_back(fst_leg);
    }
    return legs;
}

std::vector<Cloud_ptr> splitCluster(Cloud_ptr both_legs) {
    std::vector<Cloud_ptr> legs;
    Point center;
    pcl::computeCentroid(*both_legs, center);

    // ROS_INFO("SIZE OF BOTH LEGS: %d", both_legs->points.size());
    if (both_legs->points.size() > 1000) {
        // if yes, there are both legs visible but in same cluster
        Indices left_inds, right_inds;
        for (size_t i = 0; i < both_legs->size(); i++) {
            float y = both_legs->points[i].y;
            (y > center.y) ? left_inds.push_back(i) : right_inds.push_back(i);
        }
        Cloud_ptr left_leg  = boost::make_shared<Cloud>(*both_legs, left_inds);
        Cloud_ptr right_leg = boost::make_shared<Cloud>(*both_legs, right_inds);
        legs.push_back(left_leg);
        legs.push_back(right_leg);
    } else {
        if (center.y >= 0) {
            ROS_INFO("ONLY LEFT LEG IN FRAME");
            legs.push_back(both_legs);
            legs.push_back(boost::make_shared<Cloud>()); // Empty right leg
        } else {
            ROS_INFO("ONLY RIGHT LEG IN FRAME");
            legs.push_back(boost::make_shared<Cloud>()); // Empty left leg
            legs.push_back(both_legs);
        }
    }
    return legs;
}

std::vector<Cloud_ptr> splitLegs(Cloud_ptr input_cloud_ptr) {
    std::vector<Cloud_ptr> legs;
    std::vector<Cloud_ptr> clusters;

    if (input_cloud_ptr->empty()) {
        ROS_INFO("Input is empty");
        return legs;
    }

    // Clustering
    pcl::search::KdTree<Point>::Ptr tree(new pcl::search::KdTree<Point>);
    tree->setInputCloud(input_cloud_ptr);
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<Point> ec;
    ec.setClusterTolerance(CLUSTER_TOLERANCE);
    ec.setMinClusterSize(MIN_CLUSTER_SIZE);
    ec.setSearchMethod(tree);
    ec.setInputCloud(input_cloud_ptr);
    ec.extract(cluster_indices);

    for (auto const &indices : cluster_indices) {
        Cloud_ptr cluster = boost::make_shared<Cloud>(*input_cloud_ptr, indices.indices);
        // ROS_INFO("SIZE OF CLUSTER: %d", cluster->points.size());
        clusters.push_back(cluster);
    }

    if (clusters.size() == 0) {
        ROS_INFO("NO CLUSTER FOUND");
    } else if (clusters.size() == 1) {
        // Split the single cluster
        auto splitted = splitCluster(clusters[0]);
        legs.insert(legs.end(), splitted.begin(), splitted.end());
    } else if (clusters.size() >= 2) {
        // Use the first two for orientation, ignore the rest
        auto oriented = findOrientation(clusters[0], clusters[1]);
        legs.insert(legs.end(), oriented.begin(), oriented.end());
    }
    return legs;
}

geometry_msgs::Point findToe(Cloud input_cloud) {

    geometry_msgs::Point maxX;
    if (!input_cloud.empty()) {
        maxX.x = input_cloud.points[0].x;
        maxX.y = input_cloud.points[0].y;
        maxX.z = input_cloud.points[0].z;

        for(size_t i = 0; i < input_cloud.size(); i++ ) {
            float X = input_cloud.points[i].x;
            if( X > maxX.x) {
                maxX.x = input_cloud.points[i].x;
                maxX.y = input_cloud.points[i].y;
                maxX.z = input_cloud.points[i].z;
            }
        }
        Indices inds;
        for (int i = 0; i < input_cloud.size(); i++) {
            if  ((input_cloud.points[i].x + 0.02) >= maxX.x) {
                inds.push_back(i);
            }
        }
        Cloud frontalPoints(input_cloud, inds);
        Point centroid;
        pcl::computeCentroid(frontalPoints, centroid);
        maxX.y = centroid.y;
    }
    return maxX;
}

//With splitLegs a vector will be filled where the first entry is left and and the second is right.
//If there is no Cluster then legs will stay empty.
//If both legs are one cluster it will get split in the middle
//If only one leg is in the frame its Cloud will be as expected. The other Cloud will be empty.
void cloud_cb(const sensor_msgs::PointCloud2 &input_cloud) {
    ros::Time start = ros::Time::now();

    // Conversion
    Cloud_ptr input_pcl = boost::make_shared<Cloud>();
    pcl_df::fromROSMsg(input_cloud, *input_pcl);

    // ros::Time conTime = ros::Time::now();
    // ROS_INFO("Conversion TOOK %f SECONDS", (conTime - start).toSec());

    Cloud_ptr removedGround = removeGround(input_pcl);

    // ros::Time ground = ros::Time::now();
    // ROS_INFO("REMOVE GROUND TOOK %f SECONDS", (ground - conTime).toSec());
    if(PUBLISH_DEBUG) {
        pub_debug.publish(*removedGround);
    }

    std::vector<Cloud_ptr> legs = splitLegs(removedGround);

    // ros::Time split = ros::Time::now();
    // ROS_INFO("SPLIT LEGS TOOK %f SECONDS", (split - ground).toSec());

    if (legs.size() == 2) {
        // if (!legs[0]->empty()) {
        //     geometry_msgs::PointStamped left_toe = findToe(*legs[0]);
        //     pub_left_toe.publish(left_toe);
        //     if(PUBLISH_DEBUG) {
        //         pub_left_leg.publish(*legs[0]);
        //     }
        // }
        // if (!legs[1]->empty()) {
        //     geometry_msgs::PointStamped right_toe = findToe(*legs[1]);
        //     pub_right_toe.publish(right_toe);
        //     if(PUBLISH_DEBUG) {
        //         pub_right_leg.publish(*legs[1]);
        //     }
        // }
        if (!legs[0]->empty() && !legs[1]->empty()) {
            geometry_msgs::PoseArray toe_positions;
            toe_positions.header.stamp = ros::Time::now();
            toe_positions.header.frame_id = "base_link";
            toe_positions.header.seq++;
            toe_positions.poses.resize(2);
            toe_positions.poses[0].position = findToe(*legs[0]);
            toe_positions.poses[1].position = findToe(*legs[1]);
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

    try {
        transformStamped = tfBuffer.lookupTransform("base_link", CAMERA_DEPTH_FRAME_ID, ros::Time(0), ros::Duration(5));
    } catch (tf2::TransformException &ex) {
        ROS_ERROR("%s", ex.what());
        return -1;
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
