
#include <gtsam/geometry/PinholeCamera.h>
#include <gtsam/geometry/Cal3_S2.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/navigation/Scenario.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/slam/BetweenFactor.h>

#include <vector>

using namespace std;
using namespace gtsam;

// Shorthand for velocity and pose variables
using symbol_shorthand::V;
using symbol_shorthand::X;

const double kGravity = 9.81;

/* ************************************************************************* */
int main(int argc, char* argv[]) {
  auto params = PreintegrationParams::MakeSharedU(kGravity);
  params->setAccelerometerCovariance(I_3x3 * 0.1);
  params->setGyroscopeCovariance(I_3x3 * 0.1);
  params->setIntegrationCovariance(I_3x3 * 0.1);
  params->setUse2ndOrderCoriolis(false);
  params->setOmegaCoriolis(Vector3(0, 0, 0));

  Pose3 delta(Rot3::Rodrigues(-0.1, 0.2, 0.25), Point3(0.05, -0.10, 0.20));

  // Start with a camera on x-axis looking at origin
  double radius = 30;
  const Point3 up(0, 0, 1), target(0, 0, 0);
  const Point3 position(radius, 0, 0);
  const auto camera = PinholeCamera<Cal3_S2>::Lookat(position, target, up);
  const auto pose_0 = camera.pose();

  // Now, create a constant-twist scenario that makes the camera orbit the
  // origin
  double angular_velocity = M_PI,  // rad/sec
      delta_t = 1.0 / 18;          // makes for 10 degrees per step
  Vector3 angular_velocity_vector(0, -angular_velocity, 0);
  Vector3 linear_velocity_vector(radius * angular_velocity, 0, 0);
  auto scenario = ConstantTwistScenario(angular_velocity_vector,
                                        linear_velocity_vector, pose_0);

  // Create a factor graph
  NonlinearFactorGraph newgraph;

  // Create (incremental) ISAM2 solver
  ISAM2 isam;

  // Create the initial estimate to the solution
  // Intentionally initialize the variables off from the ground truth
  Values initialEstimate, totalEstimate, result;

  // Add a prior on pose x0. This indirectly specifies where the origin is.
  // 0.1 rad std on roll, pitch, yaw, 30cm std on x,y,z.
  auto noise = noiseModel::Diagonal::Sigmas(
      (Vector(6) << Vector3::Constant(0.1), Vector3::Constant(0.3)).finished());
  newgraph.addPrior(X(0), pose_0, noise);

  // Add imu priors
  Key biasKey = Symbol('b', 0);
  auto biasnoise = noiseModel::Diagonal::Sigmas(Vector6::Constant(0.1));
  newgraph.addPrior(biasKey, imuBias::ConstantBias(), biasnoise);
  initialEstimate.insert(biasKey, imuBias::ConstantBias());
  auto velnoise = noiseModel::Diagonal::Sigmas(Vector3(0.1, 0.1, 0.1));

  Vector n_velocity(3);
  n_velocity << 0, angular_velocity * radius, 0;
  newgraph.addPrior(V(0), n_velocity, velnoise);

  initialEstimate.insert(V(0), n_velocity);

  // IMU preintegrator
  PreintegratedImuMeasurements accum(params);

  // Simulate poses and imu measurements, adding them to the factor graph
  for (size_t i = 0; i < 36; ++i) {
    double t = i * delta_t;
    if (i == 0) {  // First time add two poses
      auto pose_1 = scenario.pose(delta_t);
      initialEstimate.insert(X(0), pose_0.compose(delta));
      initialEstimate.insert(X(1), pose_1.compose(delta));
    } else if (i >= 2) {  // Add more poses as necessary
      auto pose_i = scenario.pose(t);
      initialEstimate.insert(X(i), pose_i.compose(delta));
    }

    if (i > 0) {
      // Add Bias variables periodically
      if (i % 5 == 0) {
        biasKey++;
        Symbol b1 = biasKey - 1;
        Symbol b2 = biasKey;
        Vector6 covvec;
        covvec << 0.1, 0.1, 0.1, 0.1, 0.1, 0.1;
        auto cov = noiseModel::Diagonal::Variances(covvec);
        auto f = boost::make_shared<BetweenFactor<imuBias::ConstantBias> >(
            b1, b2, imuBias::ConstantBias(), cov);
        newgraph.add(f);
        initialEstimate.insert(biasKey, imuBias::ConstantBias());
      }
      // Predict acceleration and gyro measurements in (actual) body frame
      Vector3 measuredAcc = scenario.acceleration_b(t) -
                            scenario.rotation(t).transpose() * params->n_gravity;
      Vector3 measuredOmega = scenario.omega_b(t);
      accum.integrateMeasurement(measuredAcc, measuredOmega, delta_t);

      // Add Imu Factor
      ImuFactor imufac(X(i - 1), V(i - 1), X(i), V(i), biasKey, accum);
      newgraph.add(imufac);

      // insert new velocity, which is wrong
      initialEstimate.insert(V(i), n_velocity);
      accum.resetIntegration();
    }

    // Incremental solution
    isam.update(newgraph, initialEstimate);
    result = isam.calculateEstimate();
    newgraph = NonlinearFactorGraph();
    initialEstimate.clear();
  }
  GTSAM_PRINT(result);
  return 0;
}
/* ************************************************************************* */
