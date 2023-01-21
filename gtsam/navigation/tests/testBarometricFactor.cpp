/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    testBarometricFactor.cpp
 * @brief   Unit test for BarometricFactor
 * @author  Peter Milani
 * @date   16 Dec, 2021
 */

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/navigation/BarometricFactor.h>

#include <boost/bind/bind.hpp>

using namespace std::placeholders;
using namespace std;
using namespace gtsam;

// *************************************************************************
namespace example {}

double metersToBaro(const double& meters) {
    double temp = 15.04 - 0.00649 * meters;
    return 101.29 * std::pow(((temp + 273.1) / 288.08), 5.256);
}

// *************************************************************************
TEST(BarometricFactor, Constructor) {
    using namespace example;

    // meters to barometric.

    double baroMeasurement = metersToBaro(10.);

    // Factor
    Key key(1);
    Key key2(2);
    SharedNoiseModel model = noiseModel::Isotropic::Sigma(1, 0.25);
    BarometricFactor factor(key, key2, baroMeasurement, model);

    // Create a linearization point at zero error
    Pose3 T(Rot3::RzRyRx(0., 0., 0.), Point3(0., 0., 10.));
    double baroBias = 0.;
    Vector1 zero;
    zero << 0.;
    EXPECT(assert_equal(zero, factor.evaluateError(T, baroBias), 1e-5));

    // Calculate numerical derivatives
    Matrix expectedH = numericalDerivative21<Vector, Pose3, double>(
        [&factor](const Pose3& p, const double& d) {return factor.evaluateError(p, d);},
		T, baroBias);

    Matrix expectedH2 = numericalDerivative22<Vector, Pose3, double>(
        [&factor](const Pose3& p, const double& d) {return factor.evaluateError(p, d);},
        T, baroBias);

    // Use the factor to calculate the derivative
    Matrix actualH, actualH2;
    factor.evaluateError(T, baroBias, actualH, actualH2);

    // Verify we get the expected error
    EXPECT(assert_equal(expectedH, actualH, 1e-8));
    EXPECT(assert_equal(expectedH2, actualH2, 1e-8));
}

// *************************************************************************

//***************************************************************************
TEST(BarometricFactor, nonZero) {
    using namespace example;

    // meters to barometric.

    double baroMeasurement = metersToBaro(10.);

    // Factor
    Key key(1);
    Key key2(2);
    SharedNoiseModel model = noiseModel::Isotropic::Sigma(1, 0.25);
    BarometricFactor factor(key, key2, baroMeasurement, model);

    Pose3 T(Rot3::RzRyRx(0.5, 1., 1.), Point3(20., 30., 1.));
    double baroBias = 5.;

    // Calculate numerical derivatives
    Matrix expectedH = numericalDerivative21<Vector, Pose3, double>(
        [&factor](const Pose3& p, const double& d) {return factor.evaluateError(p, d);},
        T, baroBias);

    Matrix expectedH2 = numericalDerivative22<Vector, Pose3, double>(
        [&factor](const Pose3& p, const double& d) {return factor.evaluateError(p, d);},
        T, baroBias);

    // Use the factor to calculate the derivative and the error
    Matrix actualH, actualH2;
    Vector error = factor.evaluateError(T, baroBias, actualH, actualH2);
    Vector actual = (Vector(1) << -4.0).finished();

    // Verify we get the expected error
    EXPECT(assert_equal(expectedH, actualH, 1e-8));
    EXPECT(assert_equal(expectedH2, actualH2, 1e-8));
    EXPECT(assert_equal(error, actual, 1e-8));
}

// *************************************************************************
int main() {
    TestResult tr;
    return TestRegistry::runAllTests(tr);
}
// *************************************************************************
