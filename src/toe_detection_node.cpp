
#include "ros/ros.h"
#include "../include/pcl_types.h"


#define GND_LEVEL (0.01)
#define POINT_SIZE (0.01)
#define CLUSTER_TOLERANCE (0.03)


geometry_msgs::TransformStamped transformStamped;
ros::Publisher pub_left_leg, pub_right_leg, pub_left_toe, pub_right_toe;
// geometry_msgs::PointStamped old_right_toe, old_left_toe;
// float addedDistancesLeft = 0, addedDistancesRight = 0;
// int iterations = 0;

Cloud removeGround(sensor_msgs::PointCloud2 input_cloud) {
//     ROS_INFO("Tranform frame is %s und %s", transformStamped.header.frame_id.c_str(), transformStamped.child_frame_id.c_str());

    sensor_msgs::PointCloud2 sens_msg_input_cloud_tr;
    //Transformation
    tf2::doTransform(input_cloud, sens_msg_input_cloud_tr, transformStamped);


    Cloud input_cloud_tr;
    //Conversion from sensor_msgs::PointCloud2 to Cloud
    pcl::fromROSMsg(sens_msg_input_cloud_tr,input_cloud_tr);


    //Delete the GroundPoints
    Indices object_indices;
    for( size_t i = 0; i < input_cloud_tr.size(); i++ ) {
        float z = input_cloud_tr.points[i].z;
        if( z > GND_LEVEL) {
            object_indices.push_back(i);
        }
    }
    Cloud output(input_cloud_tr, object_indices );
    return output;
}


std::vector<Cloud> findOrientation(Cloud fst_leg, Cloud snd_leg) {
    std::vector<Cloud> legs;
    Point fst_centroid, snd_centroid;

    pcl::computeCentroid (fst_leg, fst_centroid);
    pcl::computeCentroid (snd_leg, snd_centroid);

    if (fst_centroid.y > snd_centroid.y) {
        legs.push_back(fst_leg);
        legs.push_back(snd_leg);
    }
    else {
        legs.push_back(snd_leg);
        legs.push_back(fst_leg);
    }
    return legs;
}

std::vector<Cloud> splitCluster(Cloud both_legs) {
//     ROS_INFO("CLUSTER HAS %lu POINTS", both_legs.points.size());
    std::vector<Cloud> legs;
    Point center;
    pcl::computeCentroid (both_legs, center);
    Cloud left_leg, right_leg;
    if (both_legs.points.size() > 1000) {
        Indices left_inds, right_inds;
        for( size_t i = 0; i < both_legs.size(); i++ ) {
            float y = both_legs.points[i].y;
            if( y > center.y) {
                left_inds.push_back(i);
            }
            else {
                right_inds.push_back(i);
            }
        }

        left_leg = Cloud(both_legs, left_inds);
        right_leg= Cloud(both_legs, right_inds);
        legs.push_back(left_leg);
        legs.push_back(right_leg);
        return legs;
    }
    if (center.y >= 0) {
        ROS_INFO("ONLY LEFT LEG IN FRAME");
        legs.push_back(both_legs);
        legs.push_back(right_leg);
    }
    else {
        ROS_INFO("ONLY RIGHT LEG IN FRAME");
        legs.push_back(left_leg);
        legs.push_back(both_legs);
    }
    return legs;
}


//side can only be left or right!
geometry_msgs:: PointStamped findToe(Cloud input_cloud) {

    geometry_msgs::PointStamped maxX;
    maxX.header.stamp = ros::Time::now();
    maxX.header.frame_id = "base_link";
    maxX.header.seq++;
    if (!input_cloud.empty()) {
        maxX.point.x = input_cloud.points[0].x;
        maxX.point.y = input_cloud.points[0].y;
        maxX.point.z = input_cloud.points[0].z;

        for(size_t i = 0; i < input_cloud.size(); i++ ) {
            float X = input_cloud.points[i].x;
            if( X > maxX.point.x) {
                maxX.point.x = input_cloud.points[i].x;
                maxX.point.y = input_cloud.points[i].y;
                maxX.point.z = input_cloud.points[i].z;
            }
        }
        Indices inds;
        for (int i = 0; i < input_cloud.size(); i++) {
            if  ((input_cloud.points[i].x + 0.02) >= maxX.point.x) {
                inds.push_back(i);
            }
        }
        Cloud frontalPoints(input_cloud, inds);
        Point centroid;
        pcl::computeCentroid(frontalPoints, centroid);
        maxX.point.y = centroid.y;
    }
    return maxX;
}

//With splitLegs a vector will be filled where the first entry is left and and the second is right.
//If there is no Cluster then legs will stay empty.
//If both legs are one cluster it will get split in the middle
//If only one leg is in the frame its Cloud will be as expected. The other Cloud will be empty.
std::vector<Cloud> splitLegs(Cloud input_cloud) {

    Cloud_ptr input_cloud_ptr = input_cloud.makeShared();

//     ROS_INFO("PointCloud before filtering has: %lu data points", input_cloud.points.size());
    // Create the filtering object: downsample the dataset using a leaf size of 1cm
    pcl::VoxelGrid<Point> vg;
    Cloud_ptr cloud_filtered(new Cloud);
    vg.setInputCloud (input_cloud_ptr);
    vg.setLeafSize (POINT_SIZE, POINT_SIZE, POINT_SIZE);
    vg.filter (*cloud_filtered);
//     ROS_INFO("PointCloud after filtering has: %lu data points", cloud_filtered->points.size());

    std::vector<Cloud> legs;


    if (cloud_filtered->empty()) {
        ROS_INFO("Input is empty");
        return legs;
    }

    // Creating the KdTree object for the search method of the extraction
    pcl::search::KdTree<Point>::Ptr tree (new pcl::search::KdTree<Point>);
    tree->setInputCloud (cloud_filtered);

    //Setting the parameters for cluster extraction
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<Point> ec;
    ec.setClusterTolerance(CLUSTER_TOLERANCE);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud_filtered);
    ec.setMinClusterSize(200);
    ec.extract(cluster_indices);


//     int i = 0;
    std::vector<Cloud> clusters;
    //Creating PointClouds for each cluster. clusters is sorted by the size of the cluster.
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
    {
//         ROS_INFO("Cluster %d has %lu points", i++, cluster_indices.at(i).indices.size());
        Cloud cloud_cluster(*cloud_filtered, it->indices );
        clusters.push_back(cloud_cluster);
    }

    if (clusters.size() == 0) {
//         ROS_INFO("NO CLUSTER FOUND");

    }
    else if (clusters.size() == 1) {
//         ROS_INFO("ONE CLUSTER with Size: %lu", clusters[0].size());
        legs = splitCluster(clusters[0]);
    }
    else if (clusters.size() == 2) {
        Cloud fst_leg = clusters[0];
        Cloud snd_leg = clusters[1];
        legs = findOrientation(fst_leg, snd_leg);
    }
    else {
//         ROS_INFO("More than 2 Clusters detected. Only the 2 largest will be published.");
        Cloud fst_leg = clusters[0];
        Cloud snd_leg = clusters[1];
        legs = findOrientation(fst_leg, snd_leg);
    }
    return legs;
}


void cloud_cb (sensor_msgs::PointCloud2 input_cloud) {
    ros::Time start = ros::Time::now();
    Cloud removedGround = removeGround(input_cloud);
    std::vector<Cloud> legs = splitLegs(removedGround);
    if (legs.size() == 2) {
        Cloud left_leg = legs[0];
        Cloud right_leg = legs[1];
//         iterations++;
        if (!left_leg.empty()) {
            geometry_msgs:: PointStamped left_toe = findToe(left_leg);
//             float distance = sqrt(pow(old_left_toe.point.x - left_toe.point.x, 2) + pow(old_left_toe.point.y - left_toe.point.y, 2) + pow(old_left_toe.point.z - left_toe.point.z, 2));
//             addedDistancesLeft += std::abs(distance);
//             old_left_toe = left_toe;
            pub_left_leg.publish(left_leg);
            pub_left_toe.publish(left_toe);
        }
        if (!right_leg.empty()) {
            geometry_msgs:: PointStamped right_toe = findToe(right_leg);
//             float distance = sqrt(pow(old_right_toe.point.x - right_toe.point.x, 2) + pow(old_right_toe.point.y - right_toe.point.y, 2) + pow(old_right_toe.point.z - right_toe.point.z, 2));
//             addedDistancesRight += std::abs(distance);

//             old_right_toe = right_toe;
            pub_right_leg.publish(right_leg);
            pub_right_toe.publish(right_toe);
        }
//         ROS_INFO("The average distance in the LEFT toe is %fm", addedDistancesLeft/iterations);
//         ROS_INFO("The average distance in the RIGHT toe is %fm", addedDistancesRight/iterations);
    ros::Time end = ros::Time::now();
    ROS_INFO("THIS CALLBACK TOOK %f SECONDS", (end - start).toSec());

    }
}

int main (int argc, char** argv) {
    // Initialize ROS
    ros::init (argc, argv, "toe_detection");
    ros::NodeHandle nh;

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);

    try {
        transformStamped = tfBuffer.lookupTransform("base_link", "camera_depth_optical_frame", ros::Time(0), ros::Duration(2));
    } catch (tf2::TransformException &ex) {
        ROS_WARN("%s", ex.what());
    }

    // Create a ROS subscriber for the input point cloud
    ros::Subscriber sub = nh.subscribe ("/camera/depth_registered/points", 1, cloud_cb);

    pub_left_leg = nh.advertise<sensor_msgs::PointCloud2>("left_leg", 1);
    pub_right_leg = nh.advertise<sensor_msgs::PointCloud2>("right_leg", 1);
    pub_left_toe = nh.advertise<geometry_msgs::PointStamped>("left_toe", 1);
    pub_right_toe = nh.advertise<geometry_msgs::PointStamped>("right_toe", 1);

    ros::spin();
    return 0;
}
