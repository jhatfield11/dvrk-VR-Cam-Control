#include <moveit_msgs/ChangeControlDimensions.h>
#include "ros/ros.h"
#include <ros/console.h>

int main(int argc, char **argv)
{
    std::string Arm;
    ros::init(argc, argv, "change_control_dimensions_client");
    ros::NodeHandle nh("~");
    nh.getParam("Arm", Arm);
    ROS_INFO("loaded arm %s", Arm);
    std::string service_name = "/"+Arm+"servo_server/change_control_dimensions";
    ros::service::waitForService(service_name);
    ros::ServiceClient client = nh.serviceClient<moveit_msgs::ChangeControlDimensions>(service_name);
    moveit_msgs::ChangeControlDimensions srv;

    srv.request.control_x_translation = true;
    srv.request.control_y_translation = true;
    srv.request.control_z_translation = true;
    srv.request.control_x_rotation = false;
    srv.request.control_y_rotation = false;
    srv.request.control_z_rotation = false;

    if (client.call(srv)) 
    {
        ROS_INFO("Removed rotation control dimension for Pose Tracking");
    }
    else 
    {
        ROS_ERROR("Failed to call service change_control_dimension - Remove rotation control.");
    }
}