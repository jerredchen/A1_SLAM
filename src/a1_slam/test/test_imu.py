import gtsam
import numpy as np
from gtsam.symbol_shorthand import X, V, B
from gtsam.utils.test_case import GtsamTestCase
from optimization.optimizer import Optimizer
from sensors.imu import ImuWrapper
from unitree_legged_msgs.msg import HighState


class TestImuNode(GtsamTestCase):

    def setUp(self):
        self.optimizer = Optimizer()
        self.imu = ImuWrapper(self.optimizer)
        # Create default prior factors and covariances.
        self.setup_pose_prior = gtsam.PriorFactorPose3(
            X(0),
            gtsam.Pose3(),
            gtsam.noiseModel.Isotropic.Sigma(6, 1e-10)
        )
        self.setup_vel_prior = gtsam.PriorFactorVector(
            V(0),
            np.zeros(3),
            gtsam.noiseModel.Isotropic.Sigma(3, 1e-10)
        )
        self.setup_bias_prior = gtsam.PriorFactorConstantBias(
            B(0),
            gtsam.imuBias.ConstantBias(),
            gtsam.noiseModel.Isotropic.Sigma(6, 1e-3)
        )
        self.setup_accel_noise = np.diag([1e-3, 1e-3, 1e-3])
        self.setup_gyro_noise = np.diag([1e-5, 1e-5, 1e-5])
        self.setup_integration_noise = np.diag([1e-5, 1e-5, 1e-5])
        pim_params = gtsam.PreintegrationParams.MakeSharedU()
        self.imu.pim = gtsam.PreintegratedImuMeasurements(pim_params)

    def test_preintegrate(self):
        """Test preintegrating several measurements."""
        # Reset the optimizer attributes and pim.
        self.optimizer.results.clear()
        self.optimizer.isam = gtsam.ISAM2(gtsam.ISAM2Params())
        self.imu.pim.resetIntegration()

        highstate = HighState()
        highstate.imu.accelerometer = [1.0, 0.0, 0.0]
        for _ in range(50):
            self.imu.preintegrate_measurement(highstate, 0.02)

        np.testing.assert_array_almost_equal(
            self.imu.pim.deltaPij(), [0.5, 0, 0], decimal=1e-10)

    def test_create_IMU_factor(self):
        """Test adding an IMU factor to the graph."""
        # Reset the optimizer attributes and pim.
        self.optimizer.results.clear()
        self.optimizer.isam = gtsam.ISAM2(gtsam.ISAM2Params())
        self.imu.pim.resetIntegration()

        # Initialize the necessary parameters for adding the IMU factor.
        self.optimizer.add_factor(self.setup_pose_prior, (X(0), gtsam.Pose3()))
        self.optimizer.add_factor(self.setup_vel_prior, (V(0), np.zeros((3,))))
        self.optimizer.add_factor(
            self.setup_bias_prior, (B(0), gtsam.imuBias.ConstantBias()))
        self.optimizer.optimize()

        highstate = HighState()
        highstate.imu.accelerometer = [1.0, 0.0, 9.81]
        for _ in range(50):
            self.imu.preintegrate_measurement(highstate, 0.02)

        navstate = gtsam.NavState()
        imu_factor, aTb, predicted_state = self.imu.create_IMU_factor(
            0, 1, navstate)
        pose_estimate = (X(1), predicted_state.pose())
        velocity_estimate = (V(1), predicted_state.velocity())
        self.optimizer.add_factor(imu_factor, [pose_estimate, velocity_estimate])
        self.optimizer.optimize()

        expected_results = gtsam.Values()
        for i in range(2):
            expected_results.insert(X(i), gtsam.Pose3(
                gtsam.Rot3(), gtsam.Point3(0.5*i, 0.0, 0.0)))
            expected_results.insert(V(i), gtsam.Point3(i, 0, 0))
        expected_results.insert(B(0), gtsam.imuBias.ConstantBias())
        self.gtsamAssertEquals(self.optimizer.results, expected_results)
        expected_aTb = gtsam.Pose3(gtsam.Pose2(0.5, 0, 0))
        self.gtsamAssertEquals(aTb, expected_aTb)

    def test_get_aTb_estimate(self):
        """Test getting the aTb estimate from IMU measurements."""
        # Reset the optimizer attributes and pim.
        self.optimizer.results.clear()
        self.optimizer.isam = gtsam.ISAM2(gtsam.ISAM2Params())
        self.imu.pim.resetIntegration()

        # Add the necessary prior factors to the optimizer.
        self.optimizer.add_factor(self.setup_pose_prior, (X(0), gtsam.Pose3()))
        self.optimizer.add_factor(self.setup_vel_prior, (V(0), np.zeros((3,))))
        self.optimizer.add_factor(
            self.setup_bias_prior, (B(0), gtsam.imuBias.ConstantBias()))
        self.optimizer.optimize()

        highstate = HighState()
        highstate.imu.accelerometer = [1.0, 0.0, 9.81]
        for _ in range(50):
            self.imu.preintegrate_measurement(highstate, 0.02)

        expected = gtsam.Pose3(gtsam.Pose2(0.5, 0, 0))
        actual = self.imu.get_aTb_estimate(0, 1)
        self.gtsamAssertEquals(actual, expected)


if __name__ == '__main__':
    import rosunit
    rosunit.unitrun('a1_slam', 'test_imu', TestImuNode)
