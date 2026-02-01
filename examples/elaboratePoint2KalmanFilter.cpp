/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file elaboratePoint2KalmanFilter.cpp
 *
 * simple linear Kalman filter on a moving 2D point, but done using factor graphs
 * This example manually creates all of the needed data structures
 *
 * @date Aug 19, 2011
 * @author Frank Dellaert
 * @author Stephen Williams
 */

#include <gtsam/nonlinear/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/GaussianBayesNet.h>
#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/base/Vector.h>

#include <cassert>

using namespace std;
using namespace gtsam;
using symbol_shorthand::X;

int main() {

  // [code below basically does SRIF with Cholesky]

  // Create a factor graph to perform the inference
  GaussianFactorGraph::shared_ptr linearFactorGraph(new GaussianFactorGraph);

  // Create the desired ordering
  Ordering::shared_ptr ordering(new Ordering);

  // Create a structure to hold the linearization points
  Values linearizationPoints;

  // Ground truth example
  // Start at origin, move to the right (x-axis): 0,0  0,1  0,2
  // Motion model is just moving to the right (x'-x)^2
  // Measurements are GPS like, (x-z)^2, where z is a 2D measurement
  // i.e., we should get 0,0  0,1  0,2 if there is no noise

  // Create new state variable
  ordering->push_back(X(0));

  // Initialize state x0 (2D point) at origin by adding a prior factor, i.e., Bayes net P(x0)
  // This is equivalent to x_0 and P_0
  Point2 x_initial(0,0);
  SharedDiagonal P_initial = noiseModel::Isotropic::Sigma(2, 0.1);

  // Linearize the factor and add it to the linear factor graph
  linearizationPoints.insert(X(0), x_initial);
  linearFactorGraph->add(X(0), 
    P_initial->R(), 
    Vector::Zero(2),
    noiseModel::Unit::Create(2));

  // Now predict the state at t=1, i.e. argmax_{x1} P(x1) = P(x1|x0) P(x0)
  // In Kalman Filter notation, this is x_{t+1|t} and P_{t+1|t}
  // For the Kalman Filter, this requires a motion model, f(x_{t}) = x_{t+1|t)
  // Assuming the system is linear, this will be of the form f(x_{t}) = F*x_{t} + B*u_{t} + w
  // where F is the state transition model/matrix, B is the control input model,
  // and w is zero-mean, Gaussian white noise with covariance Q
  // Note, in some models, Q is actually derived as G*w*G^T where w models uncertainty of some
  // physical property, such as velocity or acceleration, and G is derived from physics
  //
  // For the purposes of this example, let us assume we are using a constant-position model and
  // the controls are driving the point to the right at 1 m/s. Then, F = [1 0 ; 0 1], B = [1 0 ; 0 1]
  // and u = [1 ; 0]. Let us also assume that the process noise Q = [0.1 0 ; 0 0.1];
  //
  // In the case of factor graphs, the factor related to the motion model would be defined as
  // f2 = (f(x_{t}) - x_{t+1}) * Q^-1 * (f(x_{t}) - x_{t+1})^T
  // Conveniently, there is a factor type, called a BetweenFactor, that can generate this factor
  // given the expected difference, f(x_{t}) - x_{t+1}, and Q.
  // so, difference = x_{t+1} - x_{t} = F*x_{t} + B*u_{t} - I*x_{t}
  //                                  = (F - I)*x_{t} + B*u_{t}
  //                                  = B*u_{t} (for our example)
  ordering->push_back(X(1));

  Point2 difference(1,0);
  SharedDiagonal Q = noiseModel::Isotropic::Sigma(2, 0.1);
  BetweenFactor<Point2> factor2(X(0), X(1), difference, Q);
  // Linearize the factor and add it to the linear factor graph
  linearizationPoints.insert(X(1), x_initial);
  linearFactorGraph->push_back(factor2.linearize(linearizationPoints));

  // We have now made the small factor graph f1-(x0)-f2-(x1)
  // where factor f1 is just the prior from time t0, P(x0)
  // and   factor f2 is from the motion model
  // Eliminate this in order x0, x1, to get Bayes net P(x0|x1)P(x1)
  // As this is a filter, all we need is the posterior P(x1), so we just keep the root of the Bayes net
  //
  // Because of the way GTSAM works internally, we have used nonlinear class even though this example
  // system is linear. We first convert the nonlinear factor graph into a linear one, using the specified
  // ordering. Linear factors are simply numbered, and are not accessible via named key like the nonlinear
  // variables. Also, the nonlinear factors are linearized around an initial estimate. For a true linear
  // system, the initial estimate is not important.

  // Solve the linear factor graph, converting it into a linear Bayes Network ( P(x0,x1) = P(x0|x1)*P(x1) )
  GaussianBayesNet::shared_ptr bayesNet = linearFactorGraph->eliminateSequential(*ordering);
  const GaussianConditional::shared_ptr& x1Conditional = bayesNet->back(); // This should be P(x1)

  // Extract the current estimate of x1,P1 from the Bayes Network
  VectorValues result = bayesNet->optimize();
  Point2 x1_predict = linearizationPoints.at<Point2>(X(1)) + result[X(1)];
  traits<Point2>::Print(x1_predict, "X1 Predict");

  // Update the new linearization point to the new estimate
  linearizationPoints.update(X(1), x1_predict);



  // Create a new, empty graph and add the prior from the previous step
  linearFactorGraph = GaussianFactorGraph::shared_ptr(new GaussianFactorGraph);

  // Convert the root conditional, P(x1) in this case, into a Prior for the next step
  // Some care must be done here, as the linearization point in future steps will be different
  // than what was used when the factor was created.
  // f = || F*dx1' - (F*x0 - x1) ||^2, originally linearized at x1 = x0
  // After this step, the factor needs to be linearized around x1 = x1_predict
  // This changes the factor to f = || F*dx1'' - b'' ||^2
  //                              = || F*(dx1' - (dx1' - dx1'')) - b'' ||^2
  //                              = || F*dx1' - F*(dx1' - dx1'') - b'' ||^2
  //                              = || F*dx1' - (b'' + F(dx1' - dx1'')) ||^2
  //                              -> b' = b'' + F(dx1' - dx1'')
  //                              -> b'' = b' - F(dx1' - dx1'')
  //                              = || F*dx1'' - (b'  - F(dx1' - dx1'')) ||^2
  //                              = || F*dx1'' - (b'  - F(x_predict - x_inital)) ||^2

  // Create a new, empty graph and add the new prior
  linearFactorGraph = GaussianFactorGraph::shared_ptr(new GaussianFactorGraph);
  linearFactorGraph->add(
	X(1), 
    x1Conditional->R(),
    x1Conditional->d() - x1Conditional->R() * result[X(1)],
    x1Conditional->get_model());
 
  // Reset ordering for the next step
  ordering = Ordering::shared_ptr(new Ordering);
  ordering->push_back(X(1));

  // Now, a measurement, z1, has been received, and the Kalman Filter should be "Updated"/"Corrected"
  // This is equivalent to saying P(x1|z1) ~ P(z1|x1)*P(x1) ~ f3(x1)*f4(x1;z1)
  // where f3 is the prior from the previous step, and
  // where f4 is a measurement factor
  //
  // So, now we need to create the measurement factor, f4
  // For the Kalman Filter, this is the measurement function, h(x_{t}) = z_{t}
  // Assuming the system is linear, this will be of the form h(x_{t}) = H*x_{t} + v
  // where H is the observation model/matrix, and v is zero-mean, Gaussian white noise with covariance R
  //
  // For the purposes of this example, let us assume we have something like a GPS that returns
  // the current position of the robot. For this simple example, we can use a PriorFactor to model the
  // observation as it depends on only a single state variable, x1. To model real sensor observations
  // generally requires the creation of a new factor type. For example, factors for range sensors, bearing
  // sensors, and camera projections have already been added to GTSAM.
  //
  // In the case of factor graphs, the factor related to the measurements would be defined as
  // f4 = (h(x_{t}) - z_{t}) * R^-1 * (h(x_{t}) - z_{t})^T
  //    = (x_{t} - z_{t}) * R^-1 * (x_{t} - z_{t})^T
  // This can be modeled using the PriorFactor, where the mean is z_{t} and the covariance is R.
  Point2 z1(1.0, 0.0);
  SharedDiagonal R1 = noiseModel::Isotropic::Sigma(2, 0.25);
  PriorFactor<Point2> factor4(X(1), z1, R1);
  // Linearize the factor and add it to the linear factor graph
  linearFactorGraph->push_back(factor4.linearize(linearizationPoints));

  // We have now made the small factor graph f3-(x1)-f4
  // where factor f3 is the prior from previous time ( P(x1) )
  // and   factor f4 is from the measurement, z1 ( P(x1|z1) )
  // Eliminate this in order x1, to get Bayes net P(x1)
  // As this is a filter, all we need is the posterior P(x1), so we just keep the root of the Bayes net
  // We solve as before...

  // Solve the linear factor graph, converting it into a linear Bayes Network ( P(x0,x1) = P(x0|x1)*P(x1) )
  GaussianBayesNet::shared_ptr updatedBayesNet = linearFactorGraph->eliminateSequential(*ordering);
  const GaussianConditional::shared_ptr& updatedConditional = updatedBayesNet->back();

  // Extract the current estimate of x1 from the Bayes Network
  VectorValues updatedResult = updatedBayesNet->optimize();
  Point2 x1_update = linearizationPoints.at<Point2>(X(1)) + updatedResult[X(1)];
  traits<Point2>::Print(x1_update, "X1 Update");

  // Update the linearization point to the new estimate
  linearizationPoints.update(X(1), x1_update);






  // Wash, rinse, repeat for another time step
  // Create a new, empty graph and add the prior from the previous step
  linearFactorGraph = GaussianFactorGraph::shared_ptr(new GaussianFactorGraph);

  // Convert the root conditional, P(x1) in this case, into a Prior for the next step
  // The linearization point of this prior must be moved to the new estimate of x, and the key/index needs to be reset to 0,
  // the first key in the next iteration
  linearFactorGraph->add(
	X(1), 
    updatedConditional->R(),
    updatedConditional->d() - updatedConditional->R() * updatedResult[X(1)],
    updatedConditional->get_model());

  // Create the desired ordering
  ordering = Ordering::shared_ptr(new Ordering);
  ordering->push_back(X(1));
  ordering->push_back(X(2));
  
  // Create a nonlinear factor describing the motion model (moving right again)
  Point2 difference2(1,0);
  SharedDiagonal Q2 = noiseModel::Isotropic::Sigma(2, 0.1);
  BetweenFactor<Point2> factor6(X(1), X(2), difference2, Q2);

  // Linearize the factor and add it to the linear factor graph
  linearizationPoints.insert(X(2), x1_update);
  linearFactorGraph->push_back(factor6.linearize(linearizationPoints));

  // Solve the linear factor graph, converting it into a linear Bayes Network ( P(x1,x2) = P(x1|x2)*P(x2) )
  GaussianBayesNet::shared_ptr predictionBayesNet2 = linearFactorGraph->eliminateSequential(*ordering);
  const GaussianConditional::shared_ptr& x2Conditional = predictionBayesNet2->back();
   
  // Extract the predicted state
  VectorValues prediction2Result = predictionBayesNet2->optimize();
  Point2 x2_predict = linearizationPoints.at<Point2>(X(2)) + prediction2Result[X(2)];
  traits<Point2>::Print(x2_predict, "X2 Predict");

  // Update the linearization point to the new estimate
  linearizationPoints.update(X(2), x2_predict);

  // Now add the next measurement
  // Create a new, empty graph and add the prior from the previous step
  linearFactorGraph = GaussianFactorGraph::shared_ptr(new GaussianFactorGraph);

  // Convert the root conditional, P(x1) in this case, into a Prior for the next step
  linearFactorGraph->add(
	X(2), 
    x2Conditional->R(),
    x2Conditional->d() - x2Conditional->R() * prediction2Result[X(2)],
    x2Conditional->get_model());

  // Create the desired ordering
  ordering = Ordering::shared_ptr(new Ordering);
  ordering->push_back(X(2));

  // And update using z2 ...
  Point2 z2(2.0, 0.0);
  SharedDiagonal R2 = noiseModel::Diagonal::Sigmas((gtsam::Vector2() << 0.25, 0.25).finished());
  PriorFactor<Point2> factor8(X(2), z2, R2);

  // Linearize the factor and add it to the linear factor graph
  linearFactorGraph->push_back(factor8.linearize(linearizationPoints));

  // We have now made the small factor graph f7-(x2)-f8
  // where factor f7 is the prior from previous time ( P(x2) )
  // and   factor f8 is from the measurement, z2 ( P(x2|z2) )
  // Eliminate this in order x2, to get Bayes net P(x2)
  // As this is a filter, all we need is the posterior P(x2), so we just keep the root of the Bayes net
  // We solve as before...

  // Solve the linear factor graph, converting it into a linear Bayes Network ( P(x0,x1) = P(x0|x1)*P(x1) )
  GaussianBayesNet::shared_ptr updatedBayesNet2 = linearFactorGraph->eliminateSequential(*ordering);
  const GaussianConditional::shared_ptr& updatedConditional2 = updatedBayesNet2->back();

  // Extract the current estimate of x2 from the Bayes Network
  VectorValues updatedResult2 = updatedBayesNet2->optimize();
  Point2 x2_update = linearizationPoints.at<Point2>(X(2)) + updatedResult2[X(2)];
  traits<Point2>::Print(x2_update, "X2 Update");

  // Update the linearization point to the new estimate
  linearizationPoints.update(X(2), x2_update);






  // Wash, rinse, repeat for a third time step
  // Create a new, empty graph and add the prior from the previous step
  linearFactorGraph = GaussianFactorGraph::shared_ptr(new GaussianFactorGraph);

  // Convert the root conditional, P(x1) in this case, into a Prior for the next step
  linearFactorGraph->add(
	X(2), 
    updatedConditional2->R(),
    updatedConditional2->d() - updatedConditional2->R() * updatedResult2[X(2)],
    updatedConditional2->get_model());

  // Create the desired ordering
  ordering = Ordering::shared_ptr(new Ordering);
  ordering->push_back(X(2));
  ordering->push_back(X(3));

  // Create a nonlinear factor describing the motion model
  Point2 difference3(1,0);
  SharedDiagonal Q3 = noiseModel::Isotropic::Sigma(2, 0.1);
  BetweenFactor<Point2> factor10(X(2), X(3), difference3, Q3);

  // Linearize the factor and add it to the linear factor graph
  linearizationPoints.insert(X(3), x2_update);
  linearFactorGraph->push_back(factor10.linearize(linearizationPoints));

  // Solve the linear factor graph, converting it into a linear Bayes Network ( P(x1,x2) = P(x1|x2)*P(x2) )
  GaussianBayesNet::shared_ptr predictionBayesNet3 = linearFactorGraph->eliminateSequential(*ordering);
  const GaussianConditional::shared_ptr& x3Conditional = predictionBayesNet3->back();

  // Extract the current estimate of x3 from the Bayes Network
  VectorValues prediction3Result = predictionBayesNet3->optimize();
  Point2 x3_predict = linearizationPoints.at<Point2>(X(3)) + prediction3Result[X(3)];
  traits<Point2>::Print(x3_predict, "X3 Predict");

  // Update the linearization point to the new estimate
  linearizationPoints.update(X(3), x3_predict);



  // Now add the next measurement
  // Create a new, empty graph and add the prior from the previous step
  linearFactorGraph = GaussianFactorGraph::shared_ptr(new GaussianFactorGraph);

  // Convert the root conditional, P(x1) in this case, into a Prior for the next step
  linearFactorGraph->add(
	X(3), 
    x3Conditional->R(),
    x3Conditional->d() - x3Conditional->R() * prediction3Result[X(3)],
    x3Conditional->get_model());
 
  // Create the desired ordering
  ordering = Ordering::shared_ptr(new Ordering);
  ordering->push_back(X(3));

  // And update using z3 ...
  Point2 z3(3.0, 0.0);
  SharedDiagonal R3 = noiseModel::Isotropic::Sigma(2, 0.25);
  PriorFactor<Point2> factor12(X(3), z3, R3);

  // Linearize the factor and add it to the linear factor graph
  linearFactorGraph->push_back(factor12.linearize(linearizationPoints));

  // We have now made the small factor graph f11-(x3)-f12
  // where factor f11 is the prior from previous time ( P(x3) )
  // and   factor f12 is from the measurement, z3 ( P(x3|z3) )
  // Eliminate this in order x3, to get Bayes net P(x3)
  // As this is a filter, all we need is the posterior P(x3), so we just keep the root of the Bayes net
  // We solve as before...

  // Solve the linear factor graph, converting it into a linear Bayes Network ( P(x0,x1) = P(x0|x1)*P(x1) )
  GaussianBayesNet::shared_ptr updatedBayesNet3 = linearFactorGraph->eliminateSequential(*ordering);
  const GaussianConditional::shared_ptr& updatedConditional3 = updatedBayesNet3->back();

  // Extract the current estimate of x2 from the Bayes Network
  VectorValues updatedResult3 = updatedBayesNet3->optimize();
  Point2 x3_update = linearizationPoints.at<Point2>(X(3)) + updatedResult3[X(3)];
  traits<Point2>::Print(x3_update, "X3 Update");

  // Update the linearization point to the new estimate
  linearizationPoints.update(X(3), x3_update);



  return 0;
}
