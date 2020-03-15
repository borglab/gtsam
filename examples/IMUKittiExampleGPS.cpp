/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file IMUKittiExampleGPS
 * @brief Example of application of ISAM2 for GPS-aided navigation on the KITTI VISION BENCHMARK SUITE
 * @author Ported by Thomas Jespersen (thomasj@tkjelectronics.dk), TKJ Electronics
 */

// GTSAM related includes.
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/slam/dataset.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/ISAM2Params.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/inference/Symbol.h>
#include <cstring>
#include <fstream>
#include <iostream>

using namespace gtsam;
using namespace std;

using symbol_shorthand::X; // Pose3 (x,y,z,r,p,y)
using symbol_shorthand::V; // Vel   (xdot,ydot,zdot)
using symbol_shorthand::B; // Bias  (ax,ay,az,gx,gy,gz)

typedef struct {
    double Time;
    double dt;
    Vector3 Accelerometer;
    Vector3 Gyroscope; // omega
} imuMeasurement_t;

typedef struct {
    double Time;
    Vector3 Position; // x,y,z
} gpsMeasurement_t;

const string output_filename = "IMUKittyExampleGPSResults.csv";

int main(int argc, char* argv[])
{
    string line;

    // Read IMU metadata and compute relative sensor pose transforms
    // BodyPtx BodyPty BodyPtz BodyPrx BodyPry BodyPrz AccelerometerSigma GyroscopeSigma IntegrationSigma AccelerometerBiasSigma GyroscopeBiasSigma AverageDeltaT
    // 0 0 0 0 0 0 0.01 0.000175 0 0.000167 2.91e-006 0.0100395199348279
    string IMU_metadata_file = findExampleDataFile("KittiEquivBiasedImu_metadata.txt");
    ifstream IMU_metadata(IMU_metadata_file.c_str());

    printf("-- Reading sensor metadata\n");

    getline(IMU_metadata, line, '\n'); // ignore the first line

    double BodyPtx, BodyPty, BodyPtz, BodyPrx, BodyPry, BodyPrz, AccelerometerSigma, GyroscopeSigma, IntegrationSigma, AccelerometerBiasSigma, GyroscopeBiasSigma, AverageDeltaT;
    getline(IMU_metadata, line, '\n');
    sscanf(line.c_str(), "%lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf", &BodyPtx, &BodyPty, &BodyPtz, &BodyPrx, &BodyPry, &BodyPrz, &AccelerometerSigma, &GyroscopeSigma, &IntegrationSigma, &AccelerometerBiasSigma, &GyroscopeBiasSigma, &AverageDeltaT);
    printf("IMU metadata: %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf\n", BodyPtx, BodyPty, BodyPtz, BodyPrx, BodyPry, BodyPrz, AccelerometerSigma, GyroscopeSigma, IntegrationSigma, AccelerometerBiasSigma, GyroscopeBiasSigma, AverageDeltaT);

    Vector6 BodyP = (Vector(6) << BodyPtx, BodyPty, BodyPtz, BodyPrx, BodyPry, BodyPrz).finished();
    auto body_T_imu = Pose3::Expmap(BodyP);
    if (!body_T_imu.equals(Pose3(), 1e-5)) {
        printf("Currently only support IMUinBody is identity, i.e. IMU and body frame are the same");
        exit(-1);
    }

    // Read IMU data
    // Time dt accelX accelY accelZ omegaX omegaY omegaZ
    // 46534.47837579 46534.47837579 1.7114864219577 0.1717911743144 9.80533438749 -0.0032006241515747 0.031231284764596 -0.0063569265706488
    vector<imuMeasurement_t> IMU_measurements;
    string IMU_data_file = findExampleDataFile("KittiEquivBiasedImu.txt");

    printf("-- Reading IMU measurements from file\n");
    {
        ifstream IMU_data(IMU_data_file.c_str());
        getline(IMU_data, line, '\n'); // ignore the first line

        double time = 0, dt = 0, acc_x = 0, acc_y = 0, acc_z = 0, gyro_x = 0, gyro_y = 0, gyro_z = 0;
        while (!IMU_data.eof()) {
            getline(IMU_data, line, '\n');
            sscanf(line.c_str(), "%lf %lf %lf %lf %lf %lf %lf %lf", &time, &dt, &acc_x, &acc_y, &acc_z, &gyro_x,
                   &gyro_y, &gyro_z);

            imuMeasurement_t measurement;
            measurement.Time = time;
            measurement.dt = dt;
            measurement.Accelerometer = Vector3(acc_x, acc_y, acc_z);
            measurement.Gyroscope = Vector3(gyro_x, gyro_y, gyro_z);
            IMU_measurements.push_back(measurement);
        }
    }

    // Read GPS data
    // Time,X,Y,Z
    // 46534.478375790000428,-6.8269361350059405424,-11.868164241239471224,0.040306091310000624617
    vector<gpsMeasurement_t> GPS_measurements;
    string GPS_data_file = findExampleDataFile("KittiGps_converted.txt");

    printf("-- Reading GPS measurements from file\n");
    {
        ifstream GPS_data(GPS_data_file.c_str());
        getline(GPS_data, line, '\n'); // ignore the first line

        double time = 0, gps_x = 0, gps_y = 0, gps_z = 0;
        while (!GPS_data.eof()) {
            getline(GPS_data, line, '\n');
            sscanf(line.c_str(), "%lf,%lf,%lf,%lf", &time, &gps_x, &gps_y, &gps_z);

            gpsMeasurement_t measurement;
            measurement.Time = time;
            measurement.Position = Vector3(gps_x, gps_y, gps_z);
            GPS_measurements.push_back(measurement);
        }
    }

    // Configure different variables
    double tOffset = GPS_measurements[0].Time;
    size_t firstGPSPose = 1;
    size_t GPSskip = 10; // Skip this many GPS measurements each time
    double g = 9.8;
    auto w_coriolis = Vector3(); // zero vector

    // Configure noise models
    noiseModel::Diagonal::shared_ptr noiseModelGPS = noiseModel::Diagonal::Precisions((Vector(6) << Vector3::Constant(0), Vector3::Constant(1.0/0.07)).finished());

    // Set initial conditions for the estimated trajectory
    auto currentPoseGlobal = Pose3(Rot3(), GPS_measurements[firstGPSPose].Position); // initial pose is the reference frame (navigation frame)
    auto currentVelocityGlobal = Vector3(); // the vehicle is stationary at the beginning at position 0,0,0
    auto currentBias = imuBias::ConstantBias(); // init with zero bias

    noiseModel::Diagonal::shared_ptr sigma_init_x = noiseModel::Isotropic::Precisions((Vector(6) << Vector3::Constant(0), Vector3::Constant(1.0)).finished());
    noiseModel::Diagonal::shared_ptr sigma_init_v = noiseModel::Isotropic::Sigma(3, 1000.0);
    noiseModel::Diagonal::shared_ptr sigma_init_b = noiseModel::Isotropic::Sigmas((Vector(6) << Vector3::Constant(0.100), Vector3::Constant(5.00e-05)).finished());

    // Set IMU preintegration parameters
    Matrix33 measured_acc_cov = Matrix33::Identity(3,3) * pow(AccelerometerSigma,2);
    Matrix33 measured_omega_cov = Matrix33::Identity(3,3) * pow(GyroscopeSigma,2);
    Matrix33 integration_error_cov = Matrix33::Identity(3,3) * pow(IntegrationSigma,2); // error committed in integrating position from velocities

    boost::shared_ptr<PreintegratedImuMeasurements::Params> IMU_params = PreintegratedImuMeasurements::Params::MakeSharedU(g);
    IMU_params->accelerometerCovariance = measured_acc_cov; // acc white noise in continuous
    IMU_params->integrationCovariance = integration_error_cov; // integration uncertainty continuous
    IMU_params->gyroscopeCovariance = measured_omega_cov; // gyro white noise in continuous
    IMU_params->omegaCoriolis = w_coriolis;

    std::shared_ptr<PreintegratedImuMeasurements> currentSummarizedMeasurement = nullptr;

    // Set ISAM2 parameters and create ISAM2 solver object
    ISAM2Params isamParams;
    isamParams.factorization = ISAM2Params::CHOLESKY;
    isamParams.relinearizeSkip = 10;

    ISAM2 isam(isamParams);

    // Create the factor graph and values object that will store new factors and values to add to the incremental graph
    NonlinearFactorGraph newFactors;
    Values newValues; // values storing the initial estimates of new nodes in the factor graph

    /// Main loop:
    /// (1) we read the measurements
    /// (2) we create the corresponding factors in the graph
    /// (3) we solve the graph to obtain and optimal estimate of robot trajectory
    printf("-- Starting main loop: inference is performed at each time step, but we plot trajectory every 10 steps\n");
    size_t imuMeasurementIndex = 0;
    for (size_t gpsMeasurementIndex = firstGPSPose; gpsMeasurementIndex < GPS_measurements.size() - 1; gpsMeasurementIndex++) {
        // At each non=IMU measurement we initialize a new node in the graph
        auto currentPoseKey = X(gpsMeasurementIndex);
        auto currentVelKey = V(gpsMeasurementIndex);
        auto currentBiasKey = B(gpsMeasurementIndex);
        double t = GPS_measurements[gpsMeasurementIndex].Time;

        if (gpsMeasurementIndex == firstGPSPose) {
            // Create initial estimate and prior on initial pose, velocity, and biases
            newValues.insert(currentPoseKey, currentPoseGlobal);
            newValues.insert(currentVelKey, currentVelocityGlobal);
            newValues.insert(currentBiasKey, currentBias);
            newFactors.add(PriorFactor<Pose3>(currentPoseKey, currentPoseGlobal, sigma_init_x));
            newFactors.add(PriorFactor<Vector3>(currentVelKey, currentVelocityGlobal, sigma_init_v));
            newFactors.add(PriorFactor<imuBias::ConstantBias>(currentBiasKey, currentBias, sigma_init_b));
        } else {
            double t_previous = GPS_measurements[gpsMeasurementIndex-1].Time;

            // Summarize IMU data between the previous GPS measurement and now
            currentSummarizedMeasurement = std::make_shared<PreintegratedImuMeasurements>(IMU_params, currentBias);
            static size_t includedIMUmeasurementCount = 0;
            while (imuMeasurementIndex < IMU_measurements.size() && IMU_measurements[imuMeasurementIndex].Time <= t) {
                if (IMU_measurements[imuMeasurementIndex].Time >= t_previous) {
                    currentSummarizedMeasurement->integrateMeasurement(IMU_measurements[imuMeasurementIndex].Accelerometer, IMU_measurements[imuMeasurementIndex].Gyroscope, IMU_measurements[imuMeasurementIndex].dt);
                    includedIMUmeasurementCount++;
                }
                imuMeasurementIndex++;
            }

            // Create IMU factor
            auto previousPoseKey = X(gpsMeasurementIndex-1);
            auto previousVelKey = V(gpsMeasurementIndex-1);
            auto previousBiasKey = B(gpsMeasurementIndex-1);

            newFactors.add(ImuFactor(
                                 previousPoseKey, previousVelKey,
                                 currentPoseKey, currentVelKey,
                                 previousBiasKey, *currentSummarizedMeasurement));

            // Bias evolution as given in the IMU metadata
            noiseModel::Diagonal::shared_ptr sigma_between_b = noiseModel::Diagonal::Sigmas((Vector(6) << Vector3::Constant(sqrt(includedIMUmeasurementCount) * AccelerometerBiasSigma), Vector3::Constant(sqrt(includedIMUmeasurementCount) * GyroscopeBiasSigma)).finished());
            newFactors.add(BetweenFactor<imuBias::ConstantBias>(previousBiasKey, currentBiasKey, imuBias::ConstantBias(), sigma_between_b));

            // Create GPS factor
            auto GPSPose = Pose3(currentPoseGlobal.rotation(), GPS_measurements[gpsMeasurementIndex].Position);
            if ((gpsMeasurementIndex % GPSskip) == 0) {
                newFactors.add(PriorFactor<Pose3>(currentPoseKey, GPSPose, noiseModelGPS));
                newValues.insert(currentPoseKey, GPSPose);

                printf("################ POSE INCLUDED AT TIME %lf ################\n", t);
                GPSPose.translation().print();
                printf("\n\n");
            } else {
                newValues.insert(currentPoseKey, currentPoseGlobal);
            }

            // Add initial values for velocity and bias based on the previous estimates
            newValues.insert(currentVelKey, currentVelocityGlobal);
            newValues.insert(currentBiasKey, currentBias);

            // Update solver
            // =======================================================================
            // We accumulate 2*GPSskip GPS measurements before updating the solver at
            //first so that the heading becomes observable.
            if (gpsMeasurementIndex > (firstGPSPose + 2*GPSskip)) {
                printf("################ NEW FACTORS AT TIME %lf ################\n", t);
                newFactors.print();

                isam.update(newFactors, newValues);

                // Reset the newFactors and newValues list
                newFactors.resize(0);
                newValues.clear();

                // Extract the result/current estimates
                Values result = isam.calculateEstimate();

                currentPoseGlobal = result.at<Pose3>(currentPoseKey);
                currentVelocityGlobal = result.at<Vector3>(currentVelKey);
                currentBias = result.at<imuBias::ConstantBias>(currentBiasKey);

                printf("\n################ POSE AT TIME %lf ################\n", t);
                currentPoseGlobal.print();
                printf("\n\n");
            }
        }
    }

    // Save results to file
    printf("\nWriting results to file...\n");
    FILE* fp_out = fopen(output_filename.c_str(), "w+");
    fprintf(fp_out, "#time(s),x(m),y(m),z(m),qx,qy,qz,qw,gt_x(m),gt_y(m),gt_z(m)\n");

    Values result = isam.calculateEstimate();
    for (size_t gpsMeasurementIndex = firstGPSPose; gpsMeasurementIndex < GPS_measurements.size() - 1; gpsMeasurementIndex++) {
        auto poseKey = X(gpsMeasurementIndex);
        auto velKey = V(gpsMeasurementIndex);
        auto biasKey = B(gpsMeasurementIndex);

        auto pose = result.at<Pose3>(poseKey);
        auto velocity = result.at<Vector3>(velKey);
        auto bias = result.at<imuBias::ConstantBias>(biasKey);

        auto pose_quat = pose.rotation().toQuaternion();
        auto gps = GPS_measurements[gpsMeasurementIndex].Position;

        fprintf(fp_out, "%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n",
                GPS_measurements[gpsMeasurementIndex].Time,
                pose.x(), pose.y(), pose.z(),
                pose_quat.x(), pose_quat.y(), pose_quat.z(), pose_quat.w(),
                gps(0), gps(1), gps(2));
    }

    fclose(fp_out);
}