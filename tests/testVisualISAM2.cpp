/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    testVisualISAM2.cpp
 * @brief   Test convergence of visualSLAM example.
 * @author  Duy-Nguyen Ta
 * @author  Frank Dellaert
 */

#include <CppUnitLite/TestHarness.h>

#include <examples/SFMdata.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/ProjectionFactor.h>

#include <vector>

using namespace std;
using namespace gtsam;

/* ************************************************************************* */
TEST(testVisualISAM2, all)
{
    Cal3_S2::shared_ptr K(new Cal3_S2(50.0, 50.0, 0.0, 50.0, 50.0));

    auto measurementNoise = noiseModel::Isotropic::Sigma(2, 1.0);

    // Create ground truth data
    vector<Point3> points = createPoints();
    vector<Pose3> poses = createPoses();

    // Set the parameters
    ISAM2Params parameters;
    parameters.relinearizeThreshold = 0.01;
    parameters.relinearizeSkip = 1;
    ISAM2 isam(parameters);

    // Create a Factor Graph and Values to hold the new data
    NonlinearFactorGraph graph;
    Values initialEstimate;

    // Loop over the poses, adding the observations to iSAM incrementally
    for (size_t i = 0; i < poses.size(); ++i)
    {
        // Add factors for each landmark observation
        for (size_t j = 0; j < points.size(); ++j)
        {
            PinholeCamera<Cal3_S2> camera(poses[i], *K);
            Point2 measurement = camera.project(points[j]);
            graph.emplace_shared<GenericProjectionFactor<Pose3, Point3, Cal3_S2>>(
                measurement, measurementNoise, Symbol('x', i), Symbol('l', j), K);
        }

        // Add an initial guess for the current pose
        // Intentionally initialize the variables off from the ground truth
        static Pose3 kDeltaPose(Rot3::Rodrigues(-0.1, 0.2, 0.25),
                                Point3(0.05, -0.10, 0.20));
        initialEstimate.insert(Symbol('x', i), poses[i] * kDeltaPose);

        // Treat first iteration as special case
        if (i == 0)
        {
            // Add a prior on pose x0, 30cm std on x,y,z and 0.1 rad on roll,pitch,yaw
            static auto kPosePrior = noiseModel::Diagonal::Sigmas(
                (Vector(6) << Vector3::Constant(0.1), Vector3::Constant(0.3))
                    .finished());
            graph.addPrior(Symbol('x', 0), poses[0], kPosePrior);

            // Add a prior on landmark l0
            static auto kPointPrior = noiseModel::Isotropic::Sigma(3, 0.1);
            graph.addPrior(Symbol('l', 0), points[0], kPointPrior);

            // Add initial guesses to all observed landmarks
            // Intentionally initialize the variables off from the ground truth
            static Point3 kDeltaPoint(-0.25, 0.20, 0.15);
            for (size_t j = 0; j < points.size(); ++j)
                initialEstimate.insert<Point3>(Symbol('l', j), points[j] + kDeltaPoint);
        }
        else
        {
            // Update iSAM with the new factors
            isam.update(graph, initialEstimate);

            // Do an extra update to converge withing these 8 iterations
            isam.update();

            // Optimize
            Values currentEstimate = isam.calculateEstimate();

            // reset for next iteration
            graph.resize(0);
            initialEstimate.clear();
        }
    } // for loop

    auto result = isam.calculateEstimate();
    EXPECT_LONGS_EQUAL(16, result.size());
    for (size_t j = 0; j < points.size(); ++j)
    {
        Symbol key('l', j);
        EXPECT(assert_equal(points[j], result.at<Point3>(key), 0.01));
    }
}

/* ************************************************************************* */
int main()
{
    TestResult tr;
    return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
