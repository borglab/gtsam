
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/geometry/SimpleCamera.h>

#define GTSAM4

using namespace std;
using namespace gtsam;

/* ************************************************************************* */
std::vector<gtsam::Pose3> createPoses() {

  // Create the set of ground-truth poses
  std::vector<gtsam::Pose3> poses;
  double radius = 30.0;
  int i = 0;
  double theta = 0.0;
  gtsam::Point3 up(0,0,1);
  gtsam::Point3 target(0,0,0);
  for(; i < 80; ++i, theta += 2*M_PI/8) {
    gtsam::Point3 position = gtsam::Point3(radius*cos(theta), radius*sin(theta), 0.0);
    gtsam::SimpleCamera camera = gtsam::SimpleCamera::Lookat(position, target, up);
    poses.push_back(camera.pose());
  }
  return poses;
}
/* ************************************************************************* */

/* ************************************************************************* */
int main(int argc, char* argv[]) {

  // Create the set of ground-truth landmarks and poses
  vector<Pose3> poses = createPoses();

  // Create a factor graph
  NonlinearFactorGraph newgraph;
  NonlinearFactorGraph totalgraph;

  // Create ISAM2 solver
  ISAM2 isam, isam_full;

  // Create the initial estimate to the solution
  // Intentionally initialize the variables off from the ground truth
  Values initialEstimate, totalEstimate, result;

  // Add a prior on pose x0. This indirectly specifies where the origin is.
  // 30cm std on x,y,z 0.1 rad on roll,pitch,yaw
  noiseModel::Diagonal::shared_ptr noise = noiseModel::Diagonal::Sigmas(
      (Vector(6) << Vector3::Constant(0.3), Vector3::Constant(0.1)).finished());
  newgraph.push_back(PriorFactor<Pose3>(0, poses[0], noise));
  totalgraph.push_back(PriorFactor<Pose3>(0, poses[0], noise));

  // Add imu priors
  int biasidx = 1000;
  noiseModel::Diagonal::shared_ptr biasnoise = noiseModel::Diagonal::Sigmas((Vector(6) << 0.1, 0.1, 0.1, 0.1, 0.1, 0.1).finished());
  PriorFactor<imuBias::ConstantBias> biasprior(biasidx,imuBias::ConstantBias(),biasnoise);
  newgraph.push_back(biasprior);
  totalgraph.push_back(biasprior);
  initialEstimate.insert(biasidx, imuBias::ConstantBias());
  totalEstimate.insert(biasidx, imuBias::ConstantBias());
  noiseModel::Diagonal::shared_ptr velnoise = noiseModel::Diagonal::Sigmas((Vector(3) << 0.1, 0.1, 0.1).finished());
#ifdef GTSAM4
  PriorFactor<Vector> velprior(100,(Vector(3) << 0, 0, 0).finished(), velnoise);
#else
  PriorFactor<LieVector> velprior(100,(Vector(3) << 0, 0, 0).finished(), velnoise);
#endif
  newgraph.push_back(velprior);
  totalgraph.push_back(velprior);

#ifdef GTSAM4
  initialEstimate.insert(100, (Vector(3) << 0, 0, 0).finished());
  totalEstimate.insert(100, (Vector(3) << 0, 0, 0).finished());
#else
  initialEstimate.insert(100, LieVector((Vector(3) << 0, 0, 0).finished()));
  totalEstimate.insert(100, LieVector((Vector(3) << 0, 0, 0).finished()));
#endif

  Matrix3 I;
  I << 1, 0, 0, 0, 1, 0, 0, 0, 1;
  Matrix3 accCov = I*0.1;
  Matrix3 gyroCov = I*0.1;
  Matrix3 intCov = I*0.1;
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
  ImuFactor::PreintegratedMeasurements accum(imuBias::ConstantBias(), accCov, gyroCov, intCov, secOrder);
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
        Symbol b1 = biasidx-1;
        Symbol b2 = biasidx;
        imuBias::ConstantBias basebias = imuBias::ConstantBias();
        Vector6 covvec;
        covvec << 0.1, 0.1, 0.1, 0.1, 0.1, 0.1;
        noiseModel::Diagonal::shared_ptr cov = noiseModel::Diagonal::Variances(covvec);
        Vector6 zerovec;
        zerovec << 0, 0, 0, 0, 0, 0;
        BetweenFactor<imuBias::ConstantBias>::shared_ptr f(new BetweenFactor<imuBias::ConstantBias>(b1, b2, imuBias::ConstantBias(), cov));
        newgraph.add(f);
        totalgraph.add(f);
        initialEstimate.insert(biasidx, imuBias::ConstantBias());
        totalEstimate.insert(biasidx, imuBias::ConstantBias());
      }
      // Add Imu Factor
      accum.integrateMeasurement((Vector(3) << 0.0, 0.0, -9.8).finished(), (Vector(3) << 0, 0, 0).finished(), 0.5);
#ifdef GTSAM4
      ImuFactor imufac(i-1, 100+i-1,i,100+i, biasidx, accum);
#else
      ImuFactor imufac(i-1, 100+i-1,i,100+i, biasidx, accum, (Vector(3) << 0.0, 0.0, -9.8).finished(), (Vector(3) << 0.0, 0.0, 0.0).finished());
#endif

      newgraph.add(imufac);
      totalgraph.add(imufac);
#ifdef GTSAM4
      initialEstimate.insert(100+i, (Vector(3) << 0.0, 0.0, -9.8).finished());  // insert new velocity
      totalEstimate.insert(100+i, (Vector(3) << 0.0, 0.0, -9.8).finished());  // insert new velocity
#else
      initialEstimate.insert(100+i, LieVector((Vector(3) << 0.0, 0.0, -9.8).finished()));  // insert new velocity
      totalEstimate.insert(100+i, LieVector((Vector(3) << 0.0, 0.0, -9.8).finished()));  // insert new velocity
#endif
      accum.resetIntegration();
    }

    // Batch solution
    isam_full = ISAM2();
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

