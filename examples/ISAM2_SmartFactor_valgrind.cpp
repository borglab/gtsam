#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam_unstable/slam/SmartStereoProjectionPoseFactor.h>
#include <gtsam/navigation/CombinedImuFactor.h>

#include <iostream>
#include <vector>
#include <sstream>
#include <fstream>

using namespace std;
using namespace gtsam;
using symbol_shorthand::X;
using symbol_shorthand::V;
using symbol_shorthand::B;

struct IMUHelper
{
    IMUHelper()
    {
        gtsam::Matrix33 measured_acc_cov      = gtsam::Matrix33::Identity(3,3) * pow(0.113, 2.0);
        gtsam::Matrix33 measured_omega_cov    = gtsam::Matrix33::Identity(3,3) * pow(4.0e-5, 2.0);
        gtsam::Matrix33 bias_acc_cov          = gtsam::Matrix33::Identity(3,3) * pow( 0.00002, 2.0);
        gtsam::Matrix33 bias_omega_cov        = gtsam::Matrix33::Identity(3,3) * pow(0.001, 2.0);
        gtsam::Matrix33 integration_error_cov = gtsam::Matrix33::Identity(3,3)*1e-9; // error committed in integrating position from velocities
        gtsam::Matrix bias_acc_omega_int      = gtsam::Matrix::Identity(6,6)*1e-5; // error in the bias used for preintegration

        {
            auto gaussian = gtsam::noiseModel::Diagonal::Sigmas(
                (gtsam::Vector(6) <<
                gtsam::Vector3::Constant(5.0e-2),
                gtsam::Vector3::Constant(5.0e-3)).finished());

            auto huber = gtsam::noiseModel::Robust::Create(gtsam::noiseModel::mEstimator::Huber::Create(1.345), gaussian);

            bias_noise_model = huber;
        }

        {
            auto gaussian = gtsam::noiseModel::Isotropic::Sigma(3,0.01);
            auto huber = gtsam::noiseModel::Robust::Create(gtsam::noiseModel::mEstimator::Huber::Create(1.345), gaussian);

            velocity_noise_model = huber;
        }

        // expect IMU to be rotated in image space co-ords
        auto p = boost::make_shared<gtsam::PreintegratedCombinedMeasurements::Params>(gtsam::Vector3(0.0, 9.8, 0.0));

        p->accelerometerCovariance = measured_acc_cov; // acc white noise in continuous
        p->integrationCovariance = integration_error_cov; // integration uncertainty continuous
        p->gyroscopeCovariance = measured_omega_cov; // gyro white noise in continuous
        p->biasAccCovariance = bias_acc_cov; // acc bias in continuous
        p->biasOmegaCovariance = bias_omega_cov; // gyro bias in continuous
        p->biasAccOmegaInt = bias_acc_omega_int;

         // from SVD comparing gyro and vision estimates, no bias modelled
        gtsam::Rot3 body_to_imu_rot(
            3.612861008216737e-02,  -9.987267865568920e-01,   3.520695026944293e-02,
            4.541686330383411e-02,  -3.355264881270600e-02,  -9.984044913186698e-01,
            9.983145957368207e-01,   3.766995581886975e-02,   4.414682737675374e-02);

        gtsam::Point3 body_to_imu_trans(0.03, -0.025, -0.06);

        p->body_P_sensor = gtsam::Pose3(body_to_imu_rot, body_to_imu_trans);

        gtsam::Rot3 prior_rotation = gtsam::Rot3(gtsam::Matrix33::Identity());
        gtsam::Pose3 prior_pose(prior_rotation, gtsam::Point3(0,0,0));

        gtsam::Vector3 acc_bias(2.05998e-18, -0.0942015, 1.17663e-16);
        gtsam::Vector3 gyro_bias(0,0,0);

        prior_imu_bias = gtsam::imuBias::ConstantBias(acc_bias, gyro_bias);

        prev_state = gtsam::NavState(prior_pose, gtsam::Vector3(0,0,0));
        prop_state = prev_state;
        prev_bias = prior_imu_bias;

        preintegrated = new gtsam::PreintegratedCombinedMeasurements(p, prior_imu_bias);
    }

    gtsam::imuBias::ConstantBias prior_imu_bias; // assume zero initial bias
    gtsam::noiseModel::Robust::shared_ptr velocity_noise_model;
    gtsam::noiseModel::Robust::shared_ptr bias_noise_model;
    gtsam::NavState prev_state;
    gtsam::NavState prop_state;
    gtsam::imuBias::ConstantBias prev_bias;
    gtsam::PreintegratedCombinedMeasurements *preintegrated;
};

int main(int argc, char* argv[]) {
    if (argc != 2) {
        cout << "./main [data.txt]\n";
        return 0;
    }

    double fx = 8.2237229542137766e+02;
    double fy = fx;
    double cx = 5.3872524261474609e+02;
    double cy = 5.7909587860107422e+02;
    double baseline = 7.4342307430851651e-01;

    Cal3_S2Stereo::shared_ptr K(new Cal3_S2Stereo(fx, fy, 0.0, cx, cy, baseline));

    ISAM2Params parameters;
    parameters.relinearizeThreshold = 0.1;
    ISAM2 isam(parameters);

    // Create a factor graph
    std::map<size_t, gtsam::SmartStereoProjectionPoseFactor::shared_ptr> smartFactors;
    NonlinearFactorGraph graph;
    Values initialEstimate;
    IMUHelper imu;

    // Pose prior
    auto priorPoseNoise = noiseModel::Diagonal::Sigmas((Vector(6) << Vector3::Constant(0.1), Vector3::Constant(0.1)).finished());
    graph.emplace_shared<PriorFactor<Pose3>>(X(1), Pose3::identity(), priorPoseNoise);
    initialEstimate.insert(X(0), Pose3::identity());

    // Bias prior
    graph.add(PriorFactor<imuBias::ConstantBias>(B(1), imu.prior_imu_bias, imu.bias_noise_model));
    initialEstimate.insert(B(0), imu.prior_imu_bias);

    // Velocity prior
    graph.add(PriorFactor<Vector3>(V(1), Vector3(0,0,0), imu.velocity_noise_model));
    initialEstimate.insert(V(0), Vector3(0,0,0));

    ifstream in(argv[1]);

    if (!in) {
        cerr << "error opening: " << argv[1] << "\n";
        return 1;
    }

    int last_frame = 1;
    int frame;

    while (true) {
        char line[1024];

        in.getline(line, sizeof(line));
        stringstream ss(line);
        char type;

        ss >> type;
        ss >> frame;

        if (frame != last_frame || in.eof()) {
            cout << "Running isam for frame: " << last_frame << "\n";

            initialEstimate.insert(X(last_frame), Pose3::identity());
            initialEstimate.insert(V(last_frame), Vector3(0,0,0));
            initialEstimate.insert(B(last_frame), imu.prev_bias);

            CombinedImuFactor imu_factor(
                X(last_frame-1), V(last_frame-1),
                X(last_frame), V(last_frame),
                B(last_frame-1), B(last_frame),
                *imu.preintegrated);

            graph.add(imu_factor);

            isam.update(graph, initialEstimate);

            Values currentEstimate = isam.calculateEstimate();
            //currentEstimate.print("Current estimate: ");

            imu.prop_state = imu.preintegrated->predict(imu.prev_state, imu.prev_bias);
            imu.prev_state = NavState(currentEstimate.at<Pose3>(X(last_frame)), currentEstimate.at<Vector3>(V(last_frame)));
            imu.prev_bias = currentEstimate.at<imuBias::ConstantBias>(B(last_frame));
            imu.preintegrated->resetIntegrationAndSetBias(imu.prev_bias);

            graph.resize(0);
            initialEstimate.clear();

            if (in.eof()) {
                break;
            }
        }

        if (type == 'i') {
            double ax, ay, az;
            double gx, gy, gz;
            double dt = 1/800.0;

            ss >> ax;
            ss >> ay;
            ss >> az;

            ss >> gx;
            ss >> gy;
            ss >> gz;

            Vector3 acc(ax, ay, az);
            Vector3 gyr(gx, gy, gz);

            imu.preintegrated->integrateMeasurement(acc, gyr, dt);
        } else if (type == 's') {
            int landmark;
            double xl, xr, y;

            ss >> landmark;
            ss >> xl;
            ss >> xr;
            ss >> y;

            if (smartFactors.count(landmark) == 0) {
                auto gaussian = noiseModel::Isotropic::Sigma(3, 1.0);

                SmartProjectionParams params(HESSIAN, ZERO_ON_DEGENERACY);

                smartFactors[landmark] = SmartStereoProjectionPoseFactor::shared_ptr(new SmartStereoProjectionPoseFactor(gaussian, params));
                graph.push_back(smartFactors[landmark]);
            }

            smartFactors[landmark]->add(StereoPoint2(xl, xr, y), X(frame), K);
        }

        last_frame = frame;
    }

    return 0;
}
