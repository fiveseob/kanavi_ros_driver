#include <ros/ros.h>
#include <iostream>

#include <time.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <netinet/in.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <chrono>
#include <Eigen/Eigen>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>


using namespace std;


struct EIGEN_ALIGN16 Point {
    PCL_ADD_POINT4D;
    float intensity;
    uint32_t t;
    uint16_t reflectivity;
    uint8_t ring;  
    uint16_t ambient;
    uint32_t range; 
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
}; // point format to publish 

POINT_CLOUD_REGISTER_POINT_STRUCT(Point,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    // use std::uint32_t to avoid conflicting with pcl::uint32_t
    (std::uint32_t, t, t)
    (std::uint16_t, reflectivity, reflectivity)
    (std::uint8_t, ring, ring)
    (std::uint16_t, ambient, ambient)
    (std::uint32_t, range, range)
)

sensor_msgs::PointCloud2 cloud_to_cloud_msg(int ww, int hh, const pcl::PointCloud<Point>& cloud, int timestamp,
											const std::string& frame) {
	sensor_msgs::PointCloud2 msg{};
	pcl::toROSMsg(cloud, msg);

	//msg.header.stamp.fromNSec(100000000);
	msg.header.stamp = ros::Time::now();

	msg.header.frame_id = frame;

	return msg;

}   // convert cloud_msg to ROS
