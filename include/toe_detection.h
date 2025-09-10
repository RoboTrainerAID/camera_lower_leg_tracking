#pragma once

#include "pcl_types.h"  // decalres Cloud_ptr, Point, etc.
#include "rosPointCloud2ToPCL.hpp" // Drop in replacement for pcl::fromROSMsg (Davide Faconti)

// Declare processing functions.
Cloud_ptr removeGround(const Cloud_ptr &input_pcl, double downsample_point_size, double min_z, double max_z, double min_y, double max_y, geometry_msgs::TransformStamped transform);
std::vector<Cloud_ptr> splitLegs(Cloud_ptr input_cloud_ptr, double cluster_tolerance, int min_cluster_size);
std::vector<Cloud_ptr> splitCluster(Cloud_ptr both_legs);
std::vector<Cloud_ptr> findOrientation(Cloud_ptr fst_leg, Cloud_ptr snd_leg);
geometry_msgs::Point findToe(Cloud_ptr input_cloud_ptr);
