
#include "sensor_msgs/PointCloud2.h"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <ros_dvrk_vr/XYZcloud.h>
#include <sstream>
#include <ros/ros.h>


ros::Publisher pub;

void Callback(const ros_dvrk_vr::XYZcloud::ConstPtr& msg){
   pcl::PointCloud<pcl::PointXYZ> cloud;
   sensor_msgs::PointCloud2 cloud2;

   cloud.width = msg->X.size();
   cloud.height = 1;
   cloud.is_dense = false;
   cloud.points.resize (cloud.width * cloud.height);
   
   int i = 0;
   for (auto& point: cloud){
   point.x = msg->X[i];
   point.y = msg->Y[i];
   point.z = msg->Z[i];
   i++;
   }
   
   pcl::toROSMsg(cloud,cloud2);
   pub.publish(cloud2);
   
}

int main (int argc, char** argv){
   
   ros::init(argc, argv, "pcl_converter");
   ros::NodeHandle n;
   ros::Subscriber sub = n.subscribe("/unity/XYZCloud", 1000, Callback);

   pub = n.advertise<sensor_msgs::PointCloud2>("/unity/PointCloud", 1);
   
   ros::spin();

   return 0;
}