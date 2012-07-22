/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file LocalizationExample.cpp
 * @brief Simple robot localization example, with three "GPS-like" measurements
 * @author Frank Dellaert
 */

/**
 * A simple 2D pose slam example with "GPS" measurements
 *  - The robot moves forward 2 meter each iteration
 *  - The robot initially faces along the X axis (horizontal, to the right in 2D)
 *  - We have full odometry between pose
 *  - We have "GPS-like" measurements implemented with a custom factor
 */

// As this is a planar SLAM example, we will use Pose2 variables (x, y, theta) to represent
// the robot positions
#include <gtsam/geometry/Pose2.h>

// Each variable in the system (poses) must be identified with a unique key.
// We can either use simple integer keys (1, 2, 3, ...) or symbols (X1, X2, L1).
// Here we will use simple integer keys
#include <gtsam/nonlinear/Key.h>

// In GTSAM, measurement functions are represented as 'factors'. Several common factors
// have been provided with the library for solving robotics/SLAM/Bundle Adjustment problems.
// Here we will use Between factors for the relative motion described by odometry measurements.
// Because we have global measurements in the form of "GPS-like" measurements, we don't
// actually need to provide an initial position prior in this example. We will create our
// custom factor shortly.
#include <gtsam/slam/BetweenFactor.h>

// When the factors are created, we will add them to a Factor Graph. As the factors we are using
// are nonlinear factors, we will need a Nonlinear Factor Graph.
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

// The nonlinear solvers within GTSAM are iterative solvers, meaning they linearize the
// nonlinear functions around an initial linearization point, then solve the linear system
// to update the linearization point. This happens repeatedly until the solver converges
// to a consistent set of variable values. This requires us to specify an initial guess
// for each variable, held in a Values container.
#include <gtsam/nonlinear/Values.h>

// Finally, once all of the factors have been added to our factor graph, we will want to
// solve/optimize to graph to find the best (Maximum A Posteriori) set of variable values.
// GTSAM includes several nonlinear optimizers to perform this step. Here we will use the
// standard Levenberg-Marquardt solver
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>

// Once the optimized values have been calculated, we can also calculate the marginal covariance
// of desired variables
#include <gtsam/nonlinear/Marginals.h>


using namespace std;
using namespace gtsam;


// Before we begin the example, we must create a custom unary factor to implement a
// "GPS-like" functionality. Because standard GPS measurements provide information
// only on the position, and not on the orientation, we cannot use a simple prior to
// properly model this measurement.
//
// The factor will be a unary factor, affect only a single system variable. It will
// also use a standard Gaussian noise model. Hence, we will derive our new factor from
// the NoiseModelFactor1.
#include <gtsam/nonlinear/NonlinearFactor.h>
class UnaryFactor: public NoiseModelFactor1<Pose2> {

  // The factor will hold a measurement consisting of an (X,Y) location
  Point2 measurement_;

public:
  /// shorthand for a smart pointer to a factor
  typedef boost::shared_ptr<UnaryFactor> shared_ptr;

  // The constructor requires the variable key, the (X, Y) measurement value, and the noise model
  UnaryFactor(Key j, double x, double y, const SharedNoiseModel& model):
    NoiseModelFactor1<Pose2>(model, j), measurement_(x, y) {}

  virtual ~UnaryFactor() {}

  // By using the NoiseModelFactor base classes, the only two function that must be overridden.
  // The first is the 'evaluateError' function. This function implements the desired measurement
  // function, returning a vector of errors when evaluated at the provided variable value. It
  // must also calculate the Jacobians for this measurement function, if requested.
  Vector evaluateError(const Pose2& pose, boost::optional<Matrix&> H = boost::none) const
  {
    // The measurement function for a GPS-like measurement is simple:
    // error_x = pose.x - measurement.x
    // error_y = pose.y - measurement.y
    // Consequently, the Jacobians are:
    // [ derror_x/dx  derror_x/dy  derror_x/dtheta ] = [1 0 0]
    // [ derror_y/dx  derror_y/dy  derror_y/dtheta ] = [0 1 0]
    if (H)
      (*H) = Matrix_(2,3, 1.0,0.0,0.0, 0.0,1.0,0.0);

    return Vector_(2, pose.x() - measurement_.x(), pose.y() - measurement_.y());
  }

  // The second is a 'clone' function that allows the factor to be copied. Under most
  // circumstances, the following code that employs the default copy constructor should
  // work fine.
  virtual gtsam::NonlinearFactor::shared_ptr clone() const {
    return boost::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new UnaryFactor(*this))); }

  // Additionally, custom factors should really provide specific implementations of
  // 'equals' to ensure proper operation will all GTSAM functionality, and a custom
  // 'print' function, if desired.
  virtual bool equals(const NonlinearFactor& expected, double tol=1e-9) const {
    const UnaryFactor* e = dynamic_cast<const UnaryFactor*> (&expected);
    return e != NULL && NoiseModelFactor1::equals(*e, tol) && this->measurement_.equals(e->measurement_, tol);
  }

  virtual void print(const std::string& s, const KeyFormatter& keyFormatter = DefaultKeyFormatter) const {
    std::cout << s << "UnaryFactor(" << keyFormatter(this->key()) << ")\n";
    measurement_.print("  measurement: ");
    this->noiseModel_->print("  noise model: ");
  }

};



int main(int argc, char** argv) {

  // 1. Create a factor graph container and add factors to it
  NonlinearFactorGraph graph;

  // 2a. Add odometry factors
  // For simplicity, we will use the same noise model for each odometry factor
  noiseModel::Diagonal::shared_ptr odometryNoise = noiseModel::Diagonal::Sigmas(Vector_(3, 0.2, 0.2, 0.1));
  // Create odometry (Between) factors between consecutive poses
  graph.add(BetweenFactor<Pose2>(1, 2, Pose2(2.0, 0.0, 0.0), odometryNoise));
  graph.add(BetweenFactor<Pose2>(2, 3, Pose2(2.0, 0.0, 0.0), odometryNoise));

  // 2b. Add "GPS-like" measurements
  // We will use our custom UnaryFactor for this.
  noiseModel::Diagonal::shared_ptr unaryNoise = noiseModel::Diagonal::Sigmas(Vector_(2, 0.1, 0.1)); // 10cm std on x,y
  graph.add(UnaryFactor(1, 0.0, 0.0, unaryNoise));
  graph.add(UnaryFactor(3, 4.0, 0.0, unaryNoise));
  graph.print("\nFactor Graph:\n"); // print

  // 3. Create the data structure to hold the initialEstimate estimate to the solution
  // For illustrative purposes, these have been deliberately set to incorrect values
  Values initialEstimate;
  initialEstimate.insert(1, Pose2(0.5, 0.0, 0.2));
  initialEstimate.insert(2, Pose2(2.3, 0.1, -0.2));
  initialEstimate.insert(3, Pose2(4.1, 0.1, 0.1));
  initialEstimate.print("\nInitial Estimate:\n"); // print

  // 4. Optimize using Levenberg-Marquardt optimization. The optimizer
  // accepts an optional set of configuration parameters, controlling
  // things like convergence criteria, the type of linear system solver
  // to use, and the amount of information displayed during optimization.
  // Here we will use the default set of parameters.  See the
  // documentation for the full set of parameters.
  LevenbergMarquardtOptimizer optimizer(graph, initialEstimate);
  Values result = optimizer.optimize();
  result.print("Final Result:\n");

  // 5. Calculate and print marginal covariances for all variables
  Marginals marginals(graph, result);
  cout << "Pose 1 covariance:\n" << marginals.marginalCovariance(1) << endl;
  cout << "Pose 2 covariance:\n" << marginals.marginalCovariance(2) << endl;
  cout << "Pose 3 covariance:\n" << marginals.marginalCovariance(3) << endl;

  return 0;
}
