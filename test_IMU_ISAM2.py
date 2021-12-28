import unittest

import numpy as np

import gtsam
from gtsam.symbol_shorthand import B, V, X
from gtsam.utils.test_case import GtsamTestCase

class TestA1IMU(GtsamTestCase):

  def generate_only_gravity_data(self):
    accel_data = [(0, 0, 9.81) for _ in range(500)]
    gyro_data = [(0, 0, 0) for _ in range(500)]
    return accel_data, gyro_data

  def generate_biased_gravity_data(self):
    accel_data = [(0, 1, 9.81) for _ in range(500)]
    gyro_data = [(0, 0, 0) for _ in range(500)]
    return accel_data, gyro_data

  def generate_only_gravity_ground_truth(self):
    truth_pose = [gtsam.Pose3(gtsam.Pose2(0, 0, 0)) for _ in range(10)]
    truth_vel = [(0, 0, 0) for _ in range(10)]
    return truth_pose, truth_vel

  def A1_IMU_setup(self):
    # Instantiate the factor graph and initial estimates data structure.
    graph = gtsam.NonlinearFactorGraph()
    initial_estimate = gtsam.Values()

    # Instantiate the iSAM2 object to perform the optimization upon the factor graph.
    isam_params = gtsam.ISAM2Params()
    isam = gtsam.ISAM2(isam_params)

    # Instantiate the preintegration parameters and object to preintegrate IMU measurements.
    PIM_PARAMS = gtsam.PreintegrationParams.MakeSharedD(-9.81)
    I = np.eye(3,3)
    PIM_PARAMS.setAccelerometerCovariance(I * 1e-3)
    PIM_PARAMS.setGyroscopeCovariance(I * 1e-3)
    PIM_PARAMS.setIntegrationCovariance(I * 1e-3)
    pim = gtsam.PreintegratedImuMeasurements(PIM_PARAMS)

    # Instantiate the noise about the bias and the prior.
    PRIOR_BIAS_NOISE = gtsam.noiseModel.Isotropic.Sigma(6, 1e-4)
    PRIOR_POSE_NOISE = gtsam.noiseModel.Diagonal.Sigmas(np.array([1e-4, 1e-4, 1e-4, 1e-5, 1e-5, 1e-5]))
    PRIOR_VEL_NOISE = gtsam.noiseModel.Isotropic.Sigma(3, 1e-4)

    # Add the priors to the factor graph and initial estimates.
    graph.add(gtsam.PriorFactorPose3(X(0), gtsam.Pose3(), PRIOR_POSE_NOISE))
    initial_estimate.insert(X(0), gtsam.Pose3())
    graph.add(gtsam.PriorFactorVector(V(0), np.array([0, 0, 0]), PRIOR_VEL_NOISE))
    initial_estimate.insert(V(0), np.array([0, 0, 0]))

    graph.add(gtsam.PriorFactorConstantBias(B(0), gtsam.imuBias.ConstantBias(), PRIOR_BIAS_NOISE))
    initial_estimate.insert(B(0), gtsam.imuBias.ConstantBias())

    return graph, initial_estimate, isam, pim

  def A1_IMU_bias_setup(self):
    # Instantiate the factor graph and initial estimates data structure.
    graph = gtsam.NonlinearFactorGraph()
    initial_estimate = gtsam.Values()

    # Instantiate the iSAM2 object to perform the optimization upon the factor graph.
    isam_params = gtsam.ISAM2Params()
    isam = gtsam.ISAM2(isam_params)

    init_bias = gtsam.imuBias.ConstantBias(np.array([0.0, 1.0, 0.0]), np.array([0.0, 0.0, 0.0]))

    # Instantiate the preintegration parameters and object to preintegrate IMU measurements.
    PIM_PARAMS = gtsam.PreintegrationParams.MakeSharedD(-9.81)
    I = np.eye(3,3)
    PIM_PARAMS.setAccelerometerCovariance(I * 1e-3)
    PIM_PARAMS.setGyroscopeCovariance(I * 1e-3)
    PIM_PARAMS.setIntegrationCovariance(I * 1e-3)
    pim = gtsam.PreintegratedImuMeasurements(PIM_PARAMS, init_bias)

    # Instantiate the noise about the bias and the prior.
    PRIOR_BIAS_NOISE = gtsam.noiseModel.Isotropic.Sigma(6, 1e-4)
    PRIOR_POSE_NOISE = gtsam.noiseModel.Diagonal.Sigmas(np.array([1e-4, 1e-4, 1e-4, 1e-5, 1e-5, 1e-5]))
    PRIOR_VEL_NOISE = gtsam.noiseModel.Isotropic.Sigma(3, 1e-4)

    # Add the priors to the factor graph and initial estimates.
    graph.add(gtsam.PriorFactorPose3(X(0), gtsam.Pose3(), PRIOR_POSE_NOISE))
    initial_estimate.insert(X(0), gtsam.Pose3())
    graph.add(gtsam.PriorFactorVector(V(0), np.array([0, 0, 0]), PRIOR_VEL_NOISE))
    initial_estimate.insert(V(0), np.array([0, 0, 0]))

    # graph.add(gtsam.PriorFactorConstantBias(B(0), gtsam.imuBias.ConstantBias(), PRIOR_BIAS_NOISE))
    # initial_estimate.insert(B(0), gtsam.imuBias.ConstantBias())
    graph.add(gtsam.PriorFactorConstantBias(B(0), init_bias, PRIOR_BIAS_NOISE))
    initial_estimate.insert(B(0), init_bias)

    return graph, initial_estimate, isam, pim

  def test_only_gravity(self):
  
    # Setup factor graph with priors and initialized estimates, as well as iSAM2 and PIM.
    graph, initial_estimate, isam, pim = self.A1_IMU_setup()

    # Generate measurements.
    accel_data, gyro_data = self.generate_only_gravity_data()

    dt = 0.02
    i = 1
    state_prev = gtsam.NavState(gtsam.Pose3(), np.array([0, 0, 0]))
    bias_prev = gtsam.imuBias.ConstantBias()
    for k in range(len(accel_data)):
      measured_accel = np.array(accel_data[k])
      measured_omega = np.array(gyro_data[k])
      pim.integrateMeasurement(measured_accel, measured_omega, dt)
      if k % 50 == 0:
        imu_factor = gtsam.ImuFactor(X(i-1), V(i-1), X(i), V(i), B(0), pim)
        graph.push_back(imu_factor)
        predicted_state = pim.predict(state_prev, bias_prev)
        initial_estimate.insert(X(i), predicted_state.pose())
        initial_estimate.insert(V(i), predicted_state.velocity())
        pim.resetIntegration()

        isam.update(graph, initial_estimate)
        result = isam.calculateEstimate()

        # Reset the previous state and bias.
        state_prev = gtsam.NavState(result.atPose3(X(i)),
                                    result.atVector(V(i)))
        bias_prev = result.atConstantBias(B(0))
        i += 1
        graph = gtsam.NonlinearFactorGraph()
        initial_estimate.clear()

    # Generate ground truth poses.
    truth_pose, truth_vel = self.generate_only_gravity_ground_truth()
    for i in range(len(truth_pose)):
      pose_i = result.atPose3(X(i))
      self.gtsamAssertEquals(pose_i, truth_pose[i])
    for i in range(len(truth_vel)):
      vel_i = result.atVector(V(i))
      self.gtsamAssertEquals(vel_i, np.array(truth_vel[i]))

  def test_bias(self):
    # Setup factor graph with priors and initialized estimates, as well as iSAM2 and PIM.
    graph, initial_estimate, isam, pim = self.A1_IMU_bias_setup()

    # Generate measurements.
    accel_data, gyro_data = self.generate_biased_gravity_data()

    dt = 0.02
    i = 1
    state_prev = gtsam.NavState(gtsam.Pose3(), np.array([0, 0, 0]))
    bias_prev = gtsam.imuBias.ConstantBias(np.array([0, 1, 0]), np.array([0, 0, 0]))
    for k in range(len(accel_data)):
      measured_accel = np.array(accel_data[k])
      measured_omega = np.array(gyro_data[k])
      pim.integrateMeasurement(measured_accel, measured_omega, dt)
      if k % 50 == 0:
        imu_factor = gtsam.ImuFactor(X(i-1), V(i-1), X(i), V(i), B(0), pim)
        graph.push_back(imu_factor)
        predicted_state = pim.predict(state_prev, bias_prev)
        initial_estimate.insert(X(i), predicted_state.pose())
        initial_estimate.insert(V(i), predicted_state.velocity())
        pim.resetIntegration()

        isam.update(graph, initial_estimate)
        result = isam.calculateEstimate()

        # Reset the previous state and bias.
        state_prev = gtsam.NavState(result.atPose3(X(i)),
                                    result.atVector(V(i)))
        bias_prev = result.atConstantBias(B(0))
        i += 1
        graph = gtsam.NonlinearFactorGraph()
        initial_estimate.clear()

    # Generate ground truth poses.
    truth_pose, truth_vel = self.generate_only_gravity_ground_truth()
    for i in range(len(truth_pose)):
      pose_i = result.atPose3(X(i))
      self.gtsamAssertEquals(pose_i, truth_pose[i])
    for i in range(len(truth_vel)):
      vel_i = result.atVector(V(i))
      self.gtsamAssertEquals(vel_i, np.array(truth_vel[i]))

if __name__ == "__main__":
  unittest.main()