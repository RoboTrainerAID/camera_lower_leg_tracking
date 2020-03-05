#include <pcl_types.h>


ros::Publisher left_pub, right_pub;


geometry_msgs::PointStamped fillPoint(visualization_msgs::Marker vector) {
        geometry_msgs::PointStamped dir;
    dir.header.seq++;
    dir.header.frame_id ="base_link";
    dir.header.stamp = vector.header.stamp;
    
    dir.point.x = vector.points[0].x - vector.points[1].x;
    dir.point.y = vector.points[0].y - vector.points[1].y;
    dir.point.z = vector.points[0].z - vector.points[1].z;
    
    return dir;
}

void left_foot_axis_cb(visualization_msgs::Marker vector) {
    left_pub.publish(fillPoint(vector));
    
    return;
}

void right_foot_axis_cb(visualization_msgs::Marker vector) {
    right_pub.publish(fillPoint(vector));
    
    return;
}


int main(int argc, char **argv) {
  ros::init(argc, argv, "fix_foot_axis");

    ros::NodeHandle n;
    
    ros::Subscriber sub_left = n.subscribe("camera_lower_leg_tracking/left_foot_axis", 1, left_foot_axis_cb);
    ros::Subscriber sub_right = n.subscribe("camera_lower_leg_tracking/right_foot_axis", 1, right_foot_axis_cb);
    
    left_pub = n.advertise<geometry_msgs::PointStamped>("left_axis", 1);
    right_pub = n.advertise<geometry_msgs::PointStamped>("right_axis", 1);

    
    
    
    ros::spin();
    
    return 0;
    
}
