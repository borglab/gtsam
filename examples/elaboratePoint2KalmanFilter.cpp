/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/*
 * Point2KalmanFilter.cpp
 *
 * simple linear Kalman filter on a moving 2D point, but done using factor graphs
 *
 *  Created on: Aug 19, 2011
 *  @Author: Frank Dellaert
 *  @Author: Stephen Williams
 */

using namespace std;
using namespace gtsam;

int main() {

	// [code below basically does SRIF with LDL]

	// Ground truth example
	// Start at origin, move to the right (x-axis): 0,0  0,1  0,2
	// Motion model is just moving to the right (x'-x)^2
	// Measurements are GPS like, (x-z)^2, where z is a 2D measurement
	// i.e., we should get 0,0  0,1  0,2 if there is no noise

	// Init state x1 (2D point) at origin, i.e., Bayes net P(x1)

	// Update using z1, P(x1|z1) ~ P(x1)P(z1|x1) ~ f1(x1) f2(x1;z1)
	// Hence, make small factor graph f1-(x1)-f2
	// Eliminate to get Bayes net P(x1|z1)

	// Predict x2 P(x2|z1) = \int P(x2|x1) P(x1|z1)
	// Make a small factor graph f1-(x1)-f2-(x2)
	// where factor f1 is just the posterior from time t1, P(x1|z1)
	// where factor f2 is the motion model (x'-x)^2
	// Now, eliminate this in order x1, x2, to get Bayes net P(x1|x2)P(x2)
	// so, we just keep the root of the Bayes net, P(x2) which is really P(x2|z1)

	// Update using z2, yielding Bayes net P(x2|z1,z2)
	// repeat....

	// Combined predict-update is more efficient
	// Predict x3, update using z3
	// P(x3|z1,z2,z3) ~ P(z3|x3) \int_{x2} P(x3|x2) P(x2|z1,z2)
	// form factor graph f3-(x3)-f2-(x2)-f1
	// where f3 ~ P(z3|x3), f2 ~ P(x3|x2), and f1 ~ P(x2|z1,z2)
	// Now, eliminate this in order x2, x3, to get Bayes net P(x2|x3)P(x3)
	// and again, just keep the root of the Bayes net, P(x3) which is really P(x3|z1,z2,z3)


	// print out posterior on x3


}
