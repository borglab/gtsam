/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file GalileanFilterExample.cpp
 * @brief Run the GalileanFilter on a simulated flight with delayed GNSS.
 *
 * Reproduces the setting of Delama et al. (RSS 2026), Sec. 4.2: circular
 * motion with vertical waves, 200 Hz IMU, 20 Hz GNSS arriving with an unknown
 * delay. The filter initializes from the first IMU sample (δ̂₀ = 0) and
 * estimates attitude, velocity, position, IMU biases and the delay.
 *
 * Usage: GalileanFilterExample [delay_seconds=0.2] [duration_seconds=60]
 *
 * @author Moses Adewolu
 */

#include <gtsam_unstable/navigation/GalileanFilter.h>

#include <cmath>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <random>
#include <vector>

using namespace gtsam;
using namespace gtsam::galilean;

namespace {

/// Simulation constants.
struct Scenario {
  double imuRate = 200.0;
  int gnssDivider = 10;  ///< GNSS at imuRate / gnssDivider = 20 Hz
  double radius = 10.0;
  double angularRate = 0.5;
  double sigmaGyro = 1e-3;  ///< per-sample gyro noise [rad/s]
  double sigmaAcc = 1e-2;   ///< per-sample accelerometer noise [m/s²]
  double sigmaGnss = 0.05;  ///< GNSS noise [m]
  Vector3 gyroBias = Vector3(0.004, -0.003, 0.002);
  Vector3 accelBias = Vector3(0.05, -0.04, 0.03);
};

/// True world acceleration and body rate at time t.
void trajectory(const Scenario& s, double t, Vector3* accWorld,
                Vector3* omega) {
  const double r = s.radius, w = s.angularRate;
  *accWorld = Vector3(-r * w * w * std::cos(w * t), -r * w * w * std::sin(w * t),
                      -2.0 * std::sin(2.0 * t));
  *omega = Vector3(0.4 * std::sin(0.9 * t), 0.3 * std::cos(0.6 * t),
                   0.2 + 0.2 * std::sin(0.3 * t));
}

}  // namespace

int main(int argc, char* argv[]) {
  const double trueDelay = argc > 1 ? std::atof(argv[1]) : 0.2;
  const double duration = argc > 2 ? std::atof(argv[2]) : 60.0;
  const Scenario s;
  const double dt = 1.0 / s.imuRate;
  const Vector3 gravity(0, 0, -9.81);

  // ---- Filter parameters --------------------------------------------------
  GalileanFilterParams params;
  params.gravity = gravity;
  params.antennaLever = Point3(0.0, 0.1, -0.05);
  params.bufferWindow = std::max(0.6, trueDelay + 0.2);
  params.gyroNoiseDensity = s.sigmaGyro * std::sqrt(dt);
  params.accelNoiseDensity = s.sigmaAcc * std::sqrt(dt);
  params.gyroBiasRandomWalk = 1e-4;
  params.accelBiasRandomWalk = 3e-4;
  params.initialPositionSigma = 2.0;
  params.initialVelocitySigma = 6.0;
  params.initialAttitudeSigma = 0.05;
  params.initialDelaySigma = std::max(0.2, trueDelay);
  params.gnssOutlierThreshold = 16.27;  // χ²(3) at 99.9%

  // ---- Ground truth starts at rest-like attitude, moving on the circle -----
  const Vector10 gN = EarthInput(gravity, Vector3::Zero());
  Gal3 T(Rot3(), Point3(s.radius, 0, 0),
         Vector3(0, s.radius * s.angularRate, 1.0), 0.0);
  std::vector<Gal3> truth{T};
  std::vector<IMUInput> inputs;
  const auto truthAt = [&](double time) {  // exact under zero-order hold
    const size_t k = static_cast<size_t>(std::floor(time / dt + 1e-9));
    return Dynamics(State(truth[k], Vector10::Zero()), inputs[k], gN,
                    time - k * dt)
        .first;
  };

  std::mt19937 rng(2026);
  std::normal_distribution<double> n(0.0, 1.0);
  const auto noise3 = [&](double sigma) {
    return Vector3(sigma * n(rng), sigma * n(rng), sigma * n(rng));
  };

  GalileanFilter filter(params);

  std::cout << "Galilean EqF, true GNSS delay " << 1000 * trueDelay
            << " ms, " << duration << " s\n\n"
            << std::setw(7) << "t [s]" << std::setw(16) << "delay [ms]"
            << std::setw(12) << "± [ms]" << std::setw(13) << "pos err [m]"
            << std::setw(15) << "vel err [m/s]" << std::setw(14)
            << "att err [deg]" << std::setw(10) << "NIS\n";

  int rejected = 0, fixes = 0;
  const int steps = static_cast<int>(duration / dt);
  for (int k = 0; k < steps; ++k) {
    const double t = k * dt;
    Vector3 accWorld, omega;
    trajectory(s, t, &accWorld, &omega);
    const IMUInput imu(omega, T.rotation().unrotate(accWorld - gravity), t);
    inputs.push_back(imu);
    T = Dynamics(State(T, Vector10::Zero()), imu, gN, dt).first;
    truth.push_back(T);

    // Sensor: biased, noisy IMU.
    const IMUInput measured(imu.gyr + s.gyroBias + noise3(s.sigmaGyro),
                            imu.acc + s.accelBias + noise3(s.sigmaAcc), t);
    if (!filter.isInitialized()) {
      // The vehicle is already moving, so initializeFromIMU() (which assumes
      // rest) does not apply. Start instead from a rough pose, as from a first
      // GNSS fix and a coarse heading: 1.5 m, 0.5 m/s and ~3 deg off, δ̂ = 0.
      filter.initialize(Gal3(T.rotation() * Rot3::RzRyRx(0.03, -0.03, 0.04),
                             T.translation() + Point3(1.0, -1.0, 0.5),
                             T.velocity() + Vector3(0.3, -0.3, 0.2), 0.0));
    }
    filter.predict(measured, dt);

    const double tNow = (k + 1) * dt;
    if ((k + 1) % s.gnssDivider == 0 && tNow > trueDelay + dt) {
      const Gal3 Td = truthAt(tNow - trueDelay);
      const Point3 y = Td.rotation() * params.antennaLever + Td.translation() +
                       noise3(s.sigmaGnss);
      ++fixes;
      if (!filter.update(GnssMeasurement(
              y, Matrix3::Identity() * s.sigmaGnss * s.sigmaGnss, tNow))) {
        ++rejected;
      }
    }

    if ((k + 1) % static_cast<int>(5.0 / dt) == 0) {
      const Gal3 est = filter.navState();
      std::cout << std::fixed << std::setprecision(1) << std::setw(7) << tNow
                << std::setprecision(1) << std::setw(16)
                << 1000 * filter.delay() << std::setw(12)
                << 1000 * filter.delaySigma() << std::setprecision(3)
                << std::setw(13)
                << (est.translation() - T.translation()).norm()
                << std::setw(15) << (est.velocity() - T.velocity()).norm()
                << std::setw(14)
                << Rot3::Logmap(T.rotation().between(est.rotation())).norm() *
                       180.0 / M_PI
                << std::setprecision(2) << std::setw(10) << filter.lastNis()
                << "\n";
    }
  }

  std::cout << std::setprecision(4);
  std::cout << "\nGyro bias  true " << s.gyroBias.transpose() << "\n"
            << "           est. " << filter.gyroBias().transpose() << "\n"
            << "Accel bias true " << s.accelBias.transpose() << "\n"
            << "           est. " << filter.accelBias().transpose() << "\n"
            << "GNSS fixes rejected by NIS gate: " << rejected << " / "
            << fixes << "\n";
  return 0;
}
