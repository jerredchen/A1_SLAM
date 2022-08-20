#include "ros/ros.h"
#include "a1_slam/PreprocessCloud.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/point_cloud2_iterator.h"

bool preprocess_cloud_callback(a1_slam::PreprocessCloud::Request &request,
                               a1_slam::PreprocessCloud::Response &response)
{
  sensor_msgs::PointCloud2 cloud_msg = request.cloud;
  sensor_msgs::PointCloud2Iterator<float> iter_x(cloud_msg, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(cloud_msg, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(cloud_msg, "z");

  int ind = 0;
  for (auto it = iter_x; it != iter_x.end(); ++it) {
    response.x.push_back(*it);
    response.y.push_back(iter_y[ind]);
    response.z.push_back(iter_z[ind]);
    ind++;
  }

  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "preprocess_cloud_server");
  ros::NodeHandle n;

  ros::ServiceServer service = n.advertiseService("preprocess_cloud", preprocess_cloud_callback);
  ros::spin();

  return 0;
}
