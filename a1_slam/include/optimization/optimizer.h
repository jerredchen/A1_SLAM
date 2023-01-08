#ifndef A1SLAM_OPTIMIZATION_OPTIMIZER_H_
#define A1SLAM_OPTIMIZATION_OPTIMIZER_H_

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

namespace a1_slam
{

  class Optimizer
  {
  private:
    // Factor graph data structures.
    static gtsam::ISAM2 isam2;
    static gtsam::NonlinearFactorGraph graph;
    static gtsam::Values initial_estimates, results;
    static gtsam::Key global_key, temp_key;

    ros::NodeHandle optimizer_nh;
    static tf2_ros::TransformBroadcaster tf_broadcaster;

    /* **************************  Helper Methods ************************** */

    geometry_msgs::TransformStamped process_tf_message(gtsam::Key key);
  public:
    static std::mutex mtx;
    /* ************************** Optimization Methods ************************** */

    void add_priors();

    template <typename FACTOR_OR_CONTAINER>
    static void add_factor(const FACTOR_OR_CONTAINER &factor);

    template <typename VALUE>
    static void insert_initial_estimate(gtsam::Key key, const VALUE &value);

    static void optimize();

    template <typename VALUE>
    static VALUE get_result_value(gtsam::Key key);

    /* ************************** Callbacks ************************** */
  };

} // namespace a1_slam

#endif // A1SLAM_OPTIMIZATION_OPTIMIZER_H_