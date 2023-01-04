#include "ros/ros.h"
#include "gtsam/geometry/Pose3.h"
#include "gtsam/geometry/Rot3.h"
#include "gtsam/geometry/Point3.h"
#include "gtsam/inference/Symbol.h"
#include "gtsam/nonlinear/ISAM2.h"
#include "gtsam/nonlinear/NonlinearFactorGraph.h"
#include "gtsam/slam/BetweenFactor.h"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "nav_msgs/Path.h"

using namespace gtsam;
using symbol_shorthand::X;

class Optimizer
{
private:
  // Relevant optimizer data structures.
  static std::deque<NoiseModelFactor> factor_queue;
  static std::deque<Pose3> pose_queue;

  // Factor graph data structures.
  static ISAM2 isam2;
  static NonlinearFactorGraph graph;
  static Values initial_estimates;
  static Values results;

  ros::NodeHandle optimizer_nh;
  static tf2_ros::TransformBroadcaster tf_broadcaster;

public:
  /* ************************** Optimization Methods ************************** */

  void add_priors()
  {
    // Parse the prior pose estimate and standard deviation parameters.
    std::vector<double> param_estimate, param_sigmas;
    optimizer_nh.getParam("/prior_pose_estimate", param_estimate);
    optimizer_nh.getParam("/prior_pose_sigmas", param_sigmas);
    Rot3 pose_rotation = Rot3::Ypr(
        M_PI / 180 * param_estimate[2],
        M_PI / 180 * param_estimate[1],
        M_PI / 180 * param_estimate[0]);
    Point3 pose_translation(param_estimate[3], param_estimate[4], param_estimate[5]);
    Pose3 prior_pose(pose_rotation, pose_translation);
    Eigen::VectorXd sigmas(6);
    sigmas << M_PI / 180 * param_sigmas[3],
        M_PI / 180 * param_sigmas[4],
        M_PI / 180 * param_sigmas[5],
        param_sigmas[0],
        param_sigmas[1],
        param_sigmas[2];
    noiseModel::Diagonal::shared_ptr prior_noise = noiseModel::Diagonal::Sigmas(sigmas);

    // Create a GTSAM PriorFactor<Pose3> and add the factor to the factor graph.
    PriorFactor<Pose3> prior_pose_factor =
        PriorFactor<Pose3>(X(0), prior_pose, prior_noise);
    graph.add(prior_pose_factor);
    initial_estimates.insert(X(0), prior_pose_factor.prior());

    // Optimize the graph.
    optimize();
  }

  void optimize()
  {
    // Perform an iSAM2 incremental update.
    isam2.update(graph, initial_estimates);
    results = isam2.calculateEstimate();

    // Clear the graph and initial estimates.
    graph.resize(0);
    initial_estimates.clear();
  }

  /* ************************** Callbacks ************************** */

  /* ************************** Preprocessing helpers ************************** */

  
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "optimizer_node");
  return 0;
}