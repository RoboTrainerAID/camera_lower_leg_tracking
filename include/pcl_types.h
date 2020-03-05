 
#ifndef _PCL_TYPES_H
#define _PCL_TYPES_H

#include <math.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "ros/ros.h"

#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float64MultiArray.h>
#include <visualization_msgs/Marker.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/convert.h>
#include <tf2/transform_datatypes.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>

#include <pcl_ros/transforms.h>
#include "pcl_ros/point_cloud.h"
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/centroid.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/sac_model_cylinder.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

#include <pcl/registration/icp.h>

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <std_srvs/Trigger.h>

#include <iirob_filters/kalman_filter.h>

typedef pcl::PointXYZRGB Point;

typedef pcl::PointCloud<Point> Cloud;
typedef Cloud::Ptr Cloud_ptr;
typedef Cloud::ConstPtr Cloud_cptr;

typedef pcl::Normal Normal;
typedef pcl::PointCloud<Normal> Normals;
typedef Normals::Ptr Normals_ptr;
typedef Normals::ConstPtr Normals_cptr;

typedef std::vector<int> Indices;
typedef boost::shared_ptr<Indices> Indices_ptr;
typedef boost::shared_ptr<Indices const> Indices_cptr;

typedef iirob_filters::MultiChannelKalmanFilter<double> KalmanFilter;


#endif//_PCL_TYPES_H
