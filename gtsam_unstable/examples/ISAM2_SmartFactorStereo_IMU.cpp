/**
 * @file ISAM2_SmartFactorStereo_IMU.cpp
 * @brief test of iSAM2 with smart stereo factors and IMU preintegration,
 * originally used to debug valgrind invalid reads with Eigen
 * @author Nghia Ho
 *
 * Setup is a stationary stereo camera with an IMU attached.
 * The data file is at examples/Data/ISAM2_SmartFactorStereo_IMU.txt
 * It contains 5 frames of stereo matches and IMU data.
 */
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam_unstable/slam/SmartStereoProjectionPoseFactor.h>

#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

using namespace std;
using namespace gtsam;
using symbol_shorthand::X;
using symbol_shorthand::V;
using symbol_shorthand::B;

struct IMUHelper {
  IMUHelper() {
    {
      auto gaussian = noiseModel::Diagonal::Sigmas(
          (Vector(6) << Vector3::Constant(5.0e-2), Vector3::Constant(5.0e-3))
              .finished());
      auto huber = noiseModel::Robust::Create(
          noiseModel::mEstimator::Huber::Create(1.345), gaussian);

      biasNoiseModel = huber;
    }

    {
      auto gaussian = noiseModel::Isotropic::Sigma(3, 0.01);
      auto huber = noiseModel::Robust::Create(
          noiseModel::mEstimator::Huber::Create(1.345), gaussian);

      velocityNoiseModel = huber;
    }

    // expect IMU to be rotated in image space co-ords
    auto p = std::make_shared<PreintegratedCombinedMeasurements::Params>(
        Vector3(0.0, 9.8, 0.0));

    p->accelerometerCovariance =
        I_3x3 * pow(0.0565, 2.0);  // acc white noise in continuous
    p->integrationCovariance =
        I_3x3 * 1e-9;  // integration uncertainty continuous
    p->gyroscopeCovariance =
        I_3x3 * pow(4.0e-5, 2.0);  // gyro white noise in continuous
    p->biasAccCovariance = I_3x3 * pow(0.00002, 2.0);  // acc bias in continuous
    p->biasOmegaCovariance =
        I_3x3 * pow(0.001, 2.0);  // gyro bias in continuous
    p->biasAccOmegaInt = Matrix::Identity(6, 6) * 1e-5;

    // body to IMU rotation
    Rot3 iRb(0.036129, -0.998727, 0.035207,
             0.045417, -0.033553, -0.998404,
             0.998315, 0.037670, 0.044147);

    // body to IMU translation (meters)
    Point3 iTb(0.03, -0.025, -0.06);

    // body in this example is the left camera
    p->body_P_sensor = Pose3(iRb, iTb);

    Rot3 prior_rotation = Rot3(I_3x3);
    Pose3 prior_pose(prior_rotation, Point3(0, 0, 0));

    Vector3 acc_bias(0.0, -0.0942015, 0.0);  // in camera frame
    Vector3 gyro_bias(-0.00527483, -0.00757152, -0.00469968);

    priorImuBias = imuBias::ConstantBias(acc_bias, gyro_bias);

    prevState = NavState(prior_pose, Vector3(0, 0, 0));
    propState = prevState;
    prevBias = priorImuBias;

    preintegrated = new PreintegratedCombinedMeasurements(p, priorImuBias);
  }

  imuBias::ConstantBias priorImuBias;  // assume zero initial bias
  noiseModel::Robust::shared_ptr velocityNoiseModel;
  noiseModel::Robust::shared_ptr biasNoiseModel;
  NavState prevState;
  NavState propState;
  imuBias::ConstantBias prevBias;
  PreintegratedCombinedMeasurements* preintegrated;
};

int main(int argc, char* argv[]) {
  if (argc != 2) {
    cout << "./ISAM2_SmartFactorStereo_IMU [data.txt]\n";
    return 0;
  }

  ifstream in(argv[1]);

  if (!in) {
    cerr << "error opening: " << argv[1] << "\n";
    return 1;
  }

  // Camera parameters
  double fx = 822.37;
  double fy = 822.37;
  double cx = 538.73;
  double cy = 579.10;
  double baseline = 0.372;  // meters

  Cal3_S2Stereo::shared_ptr K(new Cal3_S2Stereo(fx, fy, 0.0, cx, cy, baseline));

  ISAM2Params parameters;
  parameters.relinearizeThreshold = 0.1;
  ISAM2 isam(parameters);

  // Create a factor graph
  std::map<size_t, SmartStereoProjectionPoseFactor::shared_ptr> smartFactors;
  NonlinearFactorGraph graph;
  Values initialEstimate;
  IMUHelper imu;

  // Pose prior - at identity
  auto priorPoseNoise = noiseModel::Diagonal::Sigmas(
      (Vector(6) << Vector3::Constant(0.1), Vector3::Constant(0.1)).finished());
  graph.addPrior(X(1), Pose3::Identity(), priorPoseNoise);
  initialEstimate.insert(X(0), Pose3::Identity());

  // Bias prior
  graph.addPrior(B(1), imu.priorImuBias,
                                               imu.biasNoiseModel);
  initialEstimate.insert(B(0), imu.priorImuBias);

  // Velocity prior - assume stationary
  graph.addPrior(V(1), Vector3(0, 0, 0), imu.velocityNoiseModel);
  initialEstimate.insert(V(0), Vector3(0, 0, 0));

  int lastFrame = 1;
  int frame;

  while (true) {
    char line[1024];

    in.getline(line, sizeof(line));
    stringstream ss(line);
    char type;

    ss >> type;
    ss >> frame;

    if (frame != lastFrame || in.eof()) {
      cout << "Running iSAM for frame: " << lastFrame << "\n";

      initialEstimate.insert(X(lastFrame), Pose3::Identity());
      initialEstimate.insert(V(lastFrame), Vector3(0, 0, 0));
      initialEstimate.insert(B(lastFrame), imu.prevBias);

      CombinedImuFactor imuFactor(X(lastFrame - 1), V(lastFrame - 1),
                                  X(lastFrame), V(lastFrame), B(lastFrame - 1),
                                  B(lastFrame), *imu.preintegrated);

      graph.add(imuFactor);

      isam.update(graph, initialEstimate);

      Values currentEstimate = isam.calculateEstimate();

      imu.propState = imu.preintegrated->predict(imu.prevState, imu.prevBias);
      imu.prevState = NavState(currentEstimate.at<Pose3>(X(lastFrame)),
                               currentEstimate.at<Vector3>(V(lastFrame)));
      imu.prevBias = currentEstimate.at<imuBias::ConstantBias>(B(lastFrame));
      imu.preintegrated->resetIntegrationAndSetBias(imu.prevBias);

      graph.resize(0);
      initialEstimate.clear();

      if (in.eof()) {
        break;
      }
    }

    if (type == 'i') {  // Process IMU measurement
      double ax, ay, az;
      double gx, gy, gz;
      double dt = 1 / 800.0;  // IMU at ~800Hz

      ss >> ax;
      ss >> ay;
      ss >> az;

      ss >> gx;
      ss >> gy;
      ss >> gz;

      Vector3 acc(ax, ay, az);
      Vector3 gyr(gx, gy, gz);

      imu.preintegrated->integrateMeasurement(acc, gyr, dt);
    } else if (type == 's') {  // Process stereo measurement
      int landmark;
      double xl, xr, y;

      ss >> landmark;
      ss >> xl;
      ss >> xr;
      ss >> y;

      if (smartFactors.count(landmark) == 0) {
        auto gaussian = noiseModel::Isotropic::Sigma(3, 1.0);

        SmartProjectionParams params(HESSIAN, ZERO_ON_DEGENERACY);

        smartFactors[landmark] = SmartStereoProjectionPoseFactor::shared_ptr(
            new SmartStereoProjectionPoseFactor(gaussian, params));
        graph.push_back(smartFactors[landmark]);
      }

      smartFactors[landmark]->add(StereoPoint2(xl, xr, y), X(frame), K);
    } else {
      throw runtime_error("unexpected data type: " + string(1, type));
    }

    lastFrame = frame;
  }

  return 0;
}
