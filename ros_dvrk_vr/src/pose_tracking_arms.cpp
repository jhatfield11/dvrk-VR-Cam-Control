#include <moveit_servo/servo.h>
#include <moveit_servo/pose_tracking.h>
#include <moveit_servo/status_codes.h>
#include <moveit_servo/make_shared_from_pool.h>
#include <thread>

#include "geometry_msgs/Pose.h"
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <moveit_msgs/ChangeControlDimensions.h>
#include <std_msgs/Int8.h>
#include "ros/ros.h"
#include <ros/console.h>
//#include <boost/bind.hpp>

static const std::string LOGNAME = "cpp_interface_example";
ros::Publisher target_pose_pub;
std::string Arm;
geometry_msgs::PoseStamped target_pose;

void callback(const geometry_msgs::Pose::ConstPtr& msg)
{
  geometry_msgs::Pose Pose_msg = *msg;
  target_pose.pose.orientation = Pose_msg.orientation;
  target_pose.pose.position.x = Pose_msg.position.x;
  target_pose.pose.position.y = Pose_msg.position.y;
  target_pose.pose.position.z = Pose_msg.position.z;
  target_pose.header.stamp = ros::Time::now();  

}

void publisherThread()
{
  ros::Rate rate(5); // ROS Rate at 5Hz
  
  while (ros::ok()) {
      target_pose_pub.publish(target_pose);
      rate.sleep();
  }
}



// Class for monitoring status of moveit_servo
class StatusMonitor
{
public:
  StatusMonitor(ros::NodeHandle& nh, const std::string& topic)
  {
    sub_ = nh.subscribe(topic, 1, &StatusMonitor::statusCB, this);
  }

private:
  void statusCB(const std_msgs::Int8ConstPtr& msg)
  {
    moveit_servo::StatusCode latest_status = static_cast<moveit_servo::StatusCode>(msg->data);
    if (latest_status != status_)
    {
      status_ = latest_status;
      const auto& status_str = moveit_servo::SERVO_STATUS_CODE_MAP.at(status_);
      ROS_INFO_STREAM_NAMED(LOGNAME, "Servo status: " << status_str);
    }
  }
  moveit_servo::StatusCode status_ = moveit_servo::StatusCode::INVALID;
  ros::Subscriber sub_;
};



int main(int argc, char** argv)
{
  ros::init(argc, argv, LOGNAME);
  ros::NodeHandle nh("~");
  nh.getParam("Arm", Arm);
  ros::MultiThreadedSpinner spinner(8);
  

  // Load the planning scene monitor
  planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor;
  planning_scene_monitor = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>("robot_description");
  if (!planning_scene_monitor->getPlanningScene())
  {
    ROS_ERROR_STREAM_NAMED(LOGNAME, "Error in setting up the PlanningSceneMonitor.");
    exit(EXIT_FAILURE);
  }

  planning_scene_monitor->startSceneMonitor();
  planning_scene_monitor->startWorldGeometryMonitor(
      planning_scene_monitor::PlanningSceneMonitor::DEFAULT_COLLISION_OBJECT_TOPIC,
      planning_scene_monitor::PlanningSceneMonitor::DEFAULT_PLANNING_SCENE_WORLD_TOPIC,
        false /* skip octomap monitor */);
  planning_scene_monitor->startStateMonitor();

  // Make a publisher for sending pose commands
  target_pose_pub = nh.advertise<geometry_msgs::PoseStamped>(Arm +"/target_pose", 0.1 /* queue */, true /* latch */);
  // Create the pose tracker
  moveit_servo::PoseTracking tracker(nh, planning_scene_monitor);

  StatusMonitor status_monitor(nh, "status");

  Eigen::Vector3d lin_tol{ 0.001, 0.001, 0.001 };
  double rot_tol = 0.1;

  geometry_msgs::TransformStamped current_ee_tf;
  tracker.getCommandFrameTransform(current_ee_tf);

  // Convert it to a Pose
  target_pose.header.frame_id = Arm + "_psm_base_link";
  target_pose.pose.position.x = current_ee_tf.transform.translation.x;
  target_pose.pose.position.y = current_ee_tf.transform.translation.y;
  target_pose.pose.position.z = current_ee_tf.transform.translation.z;
  target_pose.pose.orientation = current_ee_tf.transform.rotation;

  // Modify it a little bit
  target_pose.pose.position.z += 0.0001;

  // resetTargetPose() can be used to clear the target pose and wait for a new one, e.g. when moving between multiple
  // waypoints
  tracker.resetTargetPose();

  // Publish target pose
  target_pose.header.stamp = ros::Time::now();
  target_pose_pub.publish(target_pose);
  
  std::thread move_to_pose_thread(
    [&tracker, &lin_tol, &rot_tol] { tracker.moveToPose(lin_tol, rot_tol, 0.1 /* target pose timeout */); });

  std::thread worker(publisherThread); 
  std::string topic = "/unity/"+Arm+"/set_pose";

  ros::Subscriber sub = nh.subscribe("/unity/" +Arm+ "/set_pose", 1000, callback);

  
  spinner.spin();
  //ros::Subscriber sub=nh.subscribe<geometry_msg::Pose> (topic,1000, boost::bind(&callback, _1, &Arm));


  // Make sure the tracker is stopped and clean up
  tracker.stopMotion();
  move_to_pose_thread.join();
  worker.join();

  return EXIT_SUCCESS;
}