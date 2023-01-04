/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    imuFactorTesting.h
 * @brief   Common testing infrastructure
 * @author  Frank Dellaert
 */

#pragma once

#include <gtsam/navigation/ImuBias.h>
#include <gtsam/inference/Symbol.h>

using namespace std;
using namespace gtsam;

// Convenience for named keys
using symbol_shorthand::X;
using symbol_shorthand::V;
using symbol_shorthand::B;

namespace {
static const Vector3 kZero = Z_3x1;
typedef imuBias::ConstantBias Bias;
static const Bias kZeroBiasHat, kZeroBias;

static const Vector3 kZeroOmegaCoriolis(0, 0, 0);
static const Vector3 kNonZeroOmegaCoriolis(0, 0.1, 0.1);

static const double kGravity = 10;
static const Vector3 kGravityAlongNavZDown(0, 0, kGravity);

// Realistic MEMS white noise characteristics. Angular and velocity random walk
// expressed in degrees respectively m/s per sqrt(hr).
auto radians = [](double t) { return t * M_PI / 180; };
static const double kGyroSigma = radians(0.5) / 60;  // 0.5 degree ARW
static const double kAccelSigma = 0.1 / 60;          // 10 cm VRW
}

namespace testing {

struct ImuMeasurement {
  ImuMeasurement(const Vector3& acc, const Vector3& gyro, double dt)
      : acc(acc), gyro(gyro), dt(dt) {}
  const Vector3 acc, gyro;
  const double dt;
};

template <typename PIM>
void integrateMeasurements(const vector<ImuMeasurement>& measurements,
                           PIM* pim) {
  for (const auto& m : measurements)
    pim->integrateMeasurement(m.acc, m.gyro, m.dt);
}

struct SomeMeasurements : vector<ImuMeasurement> {
  SomeMeasurements() {
    reserve(102);
    const double dt = 0.01, pi100 = M_PI / 100;
    emplace_back(Vector3(0.1, 0, 0), Vector3(pi100, 0, 0), dt);
    emplace_back(Vector3(0.1, 0, 0), Vector3(pi100, 0, 0), dt);
    for (int i = 1; i < 100; i++) {
      emplace_back(Vector3(0.05, 0.09, 0.01),
                   Vector3(pi100, pi100 * 3, 2 * pi100), dt);
    }
  }
};

}  // namespace testing
