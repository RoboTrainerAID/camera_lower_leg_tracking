#include "ros/ros.h"
#include "../include/toe_detection.h"

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

geometry_msgs::Point findToe(Cloud_ptr input_cloud_ptr) {

    geometry_msgs::Point maxX;
    if (!input_cloud_ptr->empty()) {
        maxX.x = input_cloud_ptr->points[0].x;
        maxX.y = input_cloud_ptr->points[0].y;
        maxX.z = input_cloud_ptr->points[0].z;

        for(size_t i = 0; i < input_cloud_ptr->size(); i++ ) {
            float X = input_cloud_ptr->points[i].x;
            if( X > maxX.x) {
                maxX.x = input_cloud_ptr->points[i].x;
                maxX.y = input_cloud_ptr->points[i].y;
                maxX.z = input_cloud_ptr->points[i].z;
            }
        }
        Indices inds;
        for (int i = 0; i < input_cloud_ptr->size(); i++) {
            if  ((input_cloud_ptr->points[i].x + 0.02) >= maxX.x) {
                inds.push_back(i);
            }
        }
        Cloud frontalPoints(*input_cloud_ptr, inds);
        Point centroid;
        pcl::computeCentroid(frontalPoints, centroid);
        maxX.y = centroid.y;
    }
    return maxX;
}