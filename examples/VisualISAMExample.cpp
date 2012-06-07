/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    VisualISAMExample.cpp
 * @brief   An ISAM example for synthesis sequence, single camera
 * @author  Duy-Nguyen Ta
 */

#include <gtsam/nonlinear/Symbol.h>
#include <gtsam/nonlinear/NonlinearISAM.h>
#include <gtsam/slam/visualSLAM.h>
#include <gtsam/slam/BetweenFactor.h>
#include "VisualSLAMData.h"

using namespace std;
using namespace gtsam;

// Convenience for named keys
using symbol_shorthand::X;
using symbol_shorthand::L;

/* ************************************************************************* */
int main(int argc, char* argv[]) {

	VisualSLAMExampleData data = VisualSLAMExampleData::generate();

  /* 1. Create a NonlinearISAM which will be relinearized and reordered after every "relinearizeInterval" updates */
  int relinearizeInterval = 3;
  NonlinearISAM isam(relinearizeInterval);

  /* 2. At each frame (poseId) with new camera pose and set of associated measurements,
   * create a graph of new factors and update ISAM */

  // Store the current best estimate from ISAM
	Values currentEstimate;

  // First two frames:
	// Add factors and initial values for the first two poses and landmarks then update ISAM.
	// Note: measurements from the first pose only are not enough to update ISAM:
	//       the system is underconstrained.
  {
  	visualSLAM::Graph newFactors;

  	// First pose with prior factor
  	newFactors.addPosePrior(X(0), data.poses[0], data.noiseX);

  	// Second pose with odometry measurement
  	newFactors.addOdometry(X(0), X(1), data.odometry, data.noiseX);

  	// Visual measurements at both poses
  	for (size_t i=0; i<2; ++i) {
			for (size_t j=0; j<data.z[i].size(); ++j) {
				newFactors.addMeasurement(data.z[i][j], data.noiseZ, X(i), L(j), data.sK);
			}
  	}

  	// Initial values for the first two poses, simulated with Gaussian noise
  	Values initials;
  	initials.insert(X(0), data.poses[0]);
  	initials.insert(X(1), data.poses[0]*data.odometry);

  	// Initial values for the landmarks
  	for (size_t j=0; j<data.points.size(); ++j)
  		initials.insert(L(j), data.points[j]);

  	// Update ISAM the first time and obtain the current estimate
  	isam.update(newFactors, initials);
  	currentEstimate = isam.estimate();
  	cout << "Frame 0 and 1: " << endl;
  	currentEstimate.print("Current estimate: ");
  }

  // Subsequent frames: Add new odometry and measurement factors and initial values,
  // then update ISAM at each frame
  for (size_t i=2; i<data.poses.size(); ++i) {
  	visualSLAM::Graph newFactors;
  	// Factor for odometry measurements, simulated by adding Gaussian noise to the ground-truth.
  	Pose3 odoMeasurement =  data.odometry;
  	newFactors.addOdometry(X(i-1), X(i), data.odometry, data.noiseX);
  	// Factors for visual measurements
  	for (size_t j=0; j<data.z[i].size(); ++j) {
  		newFactors.addMeasurement(data.z[i][j], data.noiseZ, X(i), L(j), data.sK);
  	}

    // Initial estimates for the new node Xi, simulated by Gaussian noises
  	Values initials;
  	initials.insert(X(i), currentEstimate.at<Pose3>(X(i-1))*data.odometry);

  	// update ISAM
  	isam.update(newFactors, initials);
  	currentEstimate = isam.estimate();
  	cout << "****************************************************" << endl;
  	cout << "Frame " << i << ": " << endl;
  	currentEstimate.print("Current estimate: ");
  }

  return 0;
}
/* ************************************************************************* */

