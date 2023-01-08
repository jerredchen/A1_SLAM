#include "a1_slam/include/optimization/optimizer.h"

/* **************************  Helper Methods ************************** */

geometry_msgs::TransformStamped a1_slam::Optimizer::process_tf_message(gtsam::Key key)
{
  geometry_msgs::TransformStamped tf_msg;
  tf_msg.header.stamp = ros::Time::now();
  tf_msg.header.frame_id = "map";
  tf_msg.child_frame_id = "odom";
  gtsam::Pose3 pose_estimate = results.at<gtsam::Pose3>(key);
  tf_msg.transform.translation.x = pose_estimate.x();
  tf_msg.transform.translation.y = pose_estimate.y();
  tf_msg.transform.translation.z = pose_estimate.z();
  gtsam::Vector quaternion = pose_estimate.rotation().quaternion();
  tf_msg.transform.rotation.w = quaternion[0];
  tf_msg.transform.rotation.x = quaternion[1];
  tf_msg.transform.rotation.y = quaternion[2];
  tf_msg.transform.rotation.z = quaternion[3];
  return tf_msg;
}

/* ************************** Optimization Methods ************************** */

void a1_slam::Optimizer::add_priors()
{
  // Parse the prior pose estimate and standard deviation parameters.
  std::vector<double> param_estimate, param_sigmas;
  optimizer_nh.getParam("/prior_pose_estimate", param_estimate);
  optimizer_nh.getParam("/prior_pose_sigmas", param_sigmas);
  gtsam::Rot3 pose_rotation = gtsam::Rot3::Ypr(
      M_PI / 180 * param_estimate[2],
      M_PI / 180 * param_estimate[1],
      M_PI / 180 * param_estimate[0]);
  gtsam::Point3 pose_translation(param_estimate[3], param_estimate[4], param_estimate[5]);
  gtsam::Pose3 prior_pose(pose_rotation, pose_translation);
  Eigen::VectorXd sigmas(6);
  sigmas << M_PI / 180 * param_sigmas[3],
      M_PI / 180 * param_sigmas[4],
      M_PI / 180 * param_sigmas[5],
      param_sigmas[0],
      param_sigmas[1],
      param_sigmas[2];
  gtsam::noiseModel::Diagonal::shared_ptr prior_noise = gtsam::noiseModel::Diagonal::Sigmas(sigmas);

  // Create a GTSAM PriorFactor<Pose3> and add the factor to the factor graph.
  gtsam::PriorFactor<gtsam::Pose3> prior_pose_factor =
      gtsam::PriorFactor<gtsam::Pose3>(gtsam::symbol_shorthand::X(0), prior_pose, prior_noise);
  add_factor(prior_pose_factor);
  insert_initial_estimate(gtsam::symbol_shorthand::X(0), prior_pose_factor.prior());

  // Optimize the graph.
  optimize();
}

template <typename FACTOR_OR_CONTAINER>
void a1_slam::Optimizer::add_factor(const FACTOR_OR_CONTAINER &factor)
{
  std::lock_guard<std::mutex> lock(mtx);
  graph.add(factor);
}

template <typename VALUE>
void a1_slam::Optimizer::insert_initial_estimate(gtsam::Key key, const VALUE &value)
{
  std::lock_guard<std::mutex> lock(mtx);
  initial_estimates.insert(key, value);
  temp_key = std::max(temp_key, key);
}

void a1_slam::Optimizer::optimize()
{
  std::lock_guard<std::mutex> lock(mtx);
  // Perform an iSAM2 incremental update.
  isam2.update(graph, initial_estimates);
  results = isam2.calculateEstimate();

  // Update the global key with the temporary key.
  global_key = temp_key;

  // Clear the graph and initial estimates.
  graph.resize(0);
  initial_estimates.clear();
}

template <typename VALUE>
VALUE a1_slam::Optimizer::get_result_value(gtsam::Key key)
{
  std::lock_guard<std::mutex> lock(mtx);
  return results.at(key);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "optimizer_node");
  return 0;
}