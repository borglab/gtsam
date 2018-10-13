
#include <gtsam/geometry/SimpleCamera.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>

#include <vector>

#define GTSAM4

using namespace std;
using namespace gtsam;

/* ************************************************************************* */
vector<Pose3> createPoses() {
  // Create the set of ground-truth poses
  vector<Pose3> poses;
  double radius = 30.0;
  int i = 0;
  double theta = 0.0;
  Point3 up(0, 0, 1);
  Point3 target(0, 0, 0);
  for (; i < 80; ++i, theta += 2 * M_PI / 8) {
    Point3 position(radius * cos(theta), radius * sin(theta), 0.0);
    SimpleCamera camera = SimpleCamera::Lookat(position, target, up);
    poses.push_back(camera.pose());
  }
  return poses;
}

/* ************************************************************************* */
int main(int argc, char* argv[]) {
  // Create the set of ground-truth landmarks and poses
  vector<Pose3> poses = createPoses();

  // Create a factor graph
  NonlinearFactorGraph newgraph, totalgraph;

  // Create (incremental) ISAM2 solver
  ISAM2 isam;

  // Create the initial estimate to the solution
  // Intentionally initialize the variables off from the ground truth
  Values initialEstimate, totalEstimate, result;

  // Add a prior on pose x0. This indirectly specifies where the origin is.
  // 30cm std on x,y,z 0.1 rad on roll,pitch,yaw
  auto noise = noiseModel::Diagonal::Sigmas(
      (Vector(6) << Vector3::Constant(0.3), Vector3::Constant(0.1)).finished());
  newgraph.push_back(PriorFactor<Pose3>(0, poses[0], noise));
  totalgraph.push_back(PriorFactor<Pose3>(0, poses[0], noise));

  // Add imu priors
  int biasidx = 1000;
  auto biasnoise = noiseModel::Diagonal::Sigmas(Vector6::Constant(0.1));
  PriorFactor<imuBias::ConstantBias> biasprior(biasidx, imuBias::ConstantBias(),
                                               biasnoise);
  newgraph.push_back(biasprior);
  totalgraph.push_back(biasprior);
  initialEstimate.insert(biasidx, imuBias::ConstantBias());
  totalEstimate.insert(biasidx, imuBias::ConstantBias());
  auto velnoise = noiseModel::Diagonal::Sigmas(Vector3(0.1, 0.1, 0.1));

  Vector gravity(3), zero(3);
  gravity << 0, 0, -9.8;
  zero << 0, 0, 0;
#ifdef GTSAM4
  PriorFactor<Vector> velprior(100, zero, velnoise);
#else
  PriorFactor<LieVector> velprior(100, zero, velnoise);
#endif
  newgraph.push_back(velprior);
  totalgraph.push_back(velprior);

#ifdef GTSAM4
  initialEstimate.insert(100, zero);
  totalEstimate.insert(100, zero);
#else
  initialEstimate.insert(100, LieVector(zero));
  totalEstimate.insert(100, LieVector(zero));
#endif

  Matrix3 I;
  I << 1, 0, 0, 0, 1, 0, 0, 0, 1;
  Matrix3 accCov = I * 0.1;
  Matrix3 gyroCov = I * 0.1;
  Matrix3 intCov = I * 0.1;
  bool secOrder = false;
#ifdef GTSAM4
  // IMU preintegrator
  PreintegratedImuMeasurements accum(PreintegrationParams::MakeSharedD());
  accum.params()->setAccelerometerCovariance(accCov);
  accum.params()->setGyroscopeCovariance(gyroCov);
  accum.params()->setIntegrationCovariance(intCov);
  accum.params()->setUse2ndOrderCoriolis(secOrder);
  accum.params()->setOmegaCoriolis(Vector3(0, 0, 0));
#else
  ImuFactor::PreintegratedMeasurements accum(imuBias::ConstantBias(), accCov,
                                             gyroCov, intCov, secOrder);
#endif

  // Simulate poses and imu measurements, adding them to the factor graph
  for (size_t i = 0; i < poses.size(); ++i) {
#ifdef GTSAM4
    Pose3 delta(Rot3::Rodrigues(-0.1, 0.2, 0.25), Point3(0.05, -0.10, 0.20));
#else
    Pose3 delta(Rot3::ypr(-0.1, 0.2, 0.25), Point3(0.05, -0.10, 0.20));
#endif
    if (i == 0) {  // First time add two poses
      initialEstimate.insert(0, poses[0].compose(delta));
      initialEstimate.insert(1, poses[1].compose(delta));
      totalEstimate.insert(0, poses[0].compose(delta));
      totalEstimate.insert(1, poses[1].compose(delta));
    } else if (i >= 2) {  // Add more poses as necessary
      initialEstimate.insert(i, poses[i].compose(delta));
      totalEstimate.insert(i, poses[i].compose(delta));
    }

    if (i > 0) {
      // Add Bias variables periodically
      if (i % 5 == 0) {
        biasidx++;
        Symbol b1 = biasidx - 1;
        Symbol b2 = biasidx;
        Vector6 covvec;
        covvec << 0.1, 0.1, 0.1, 0.1, 0.1, 0.1;
        auto cov = noiseModel::Diagonal::Variances(covvec);
        Vector6 zerovec;
        zerovec << 0, 0, 0, 0, 0, 0;
        auto f = boost::make_shared<BetweenFactor<imuBias::ConstantBias> >(
            b1, b2, imuBias::ConstantBias(), cov);
        newgraph.add(f);
        totalgraph.add(f);
        initialEstimate.insert(biasidx, imuBias::ConstantBias());
        totalEstimate.insert(biasidx, imuBias::ConstantBias());
      }
      // Add Imu Factor
      accum.integrateMeasurement(gravity, zero, 0.5);
#ifdef GTSAM4
      ImuFactor imufac(i - 1, 100 + i - 1, i, 100 + i, biasidx, accum);
#else
      ImuFactor imufac(i - 1, 100 + i - 1, i, 100 + i, biasidx, accum, gravity,
                       zero);
#endif

      newgraph.add(imufac);
      totalgraph.add(imufac);

      // insert new velocity
#ifdef GTSAM4
      initialEstimate.insert(100 + i, gravity);
      totalEstimate.insert(100 + i, gravity);
#else
      initialEstimate.insert(100 + i, LieVector(gravity));
      totalEstimate.insert(100 + i, LieVector(gravity));
#endif
      accum.resetIntegration();
    }

    // Batch solution
    ISAM2 isam_full;
    isam_full.update(totalgraph, totalEstimate);
    result = isam_full.calculateEstimate();

    // Incremental solution
    isam.update(newgraph, initialEstimate);
    result = isam.calculateEstimate();
    newgraph = NonlinearFactorGraph();
    initialEstimate.clear();
  }

  return 0;
}
/* ************************************************************************* */
