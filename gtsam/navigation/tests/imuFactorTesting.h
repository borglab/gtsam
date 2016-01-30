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

typedef imuBias::ConstantBias Bias;
static const Vector3 Z_3x1 = Vector3::Zero();
static const Bias kZeroBiasHat, kZeroBias;

static const Vector3 kZeroOmegaCoriolis(0, 0, 0);
static const Vector3 kNonZeroOmegaCoriolis(0, 0.1, 0.1);

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
