#include "ros/ros.h"
#include "ros/console.h"
#include "ros/callback_queue.h"
#include "std_msgs/String.h"

namespace a1_slam
{
  void random_callback(const std_msgs::String::ConstPtr &msg)
  {
    ROS_INFO("received the message: [%s]", msg->data.c_str());
    std::cout << "here" << std::endl;
  }

  class A1SlamNode
  {
  public:
    void launch_node()
    {
      ros::NodeHandle global_nh;

      // Check which sensors are selected to be used for SLAM.
      bool use_imu, use_2dlidar, use_3dlidar, use_depth;
      global_nh.getParam("/use_imu", use_imu);
      global_nh.getParam("/use_2dlidar", use_2dlidar);
      global_nh.getParam("/use_3dlidar", use_3dlidar);
      global_nh.getParam("/use_depth", use_depth);
      if (!use_imu && !use_2dlidar && !use_3dlidar && !use_depth)
      {
        ROS_ERROR("No sensors were selected");
      }
      ros::Subscriber lidar2d_sub;
      ros::CallbackQueue lidar2d_queue;
      // Spin up the callback threads for each subscriber
      ros::AsyncSpinner lidar2d_spinner(0, &lidar2d_queue);
      if (use_2dlidar)
      {
        ros::NodeHandle lidar2d_nh("/lidar2d");
        std::string lidar2d_topic;
        lidar2d_nh.getParam("topic", lidar2d_topic);
        lidar2d_nh.setCallbackQueue(&lidar2d_queue);
        lidar2d_sub = lidar2d_nh.subscribe(lidar2d_topic, 10, random_callback);
        lidar2d_spinner.start();
      }

      ros::waitForShutdown();
    };
  };
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "a1_slam_node");
  a1_slam::A1SlamNode node;
  node.launch_node();

  return 0;
}