/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    vSLAMexample.cpp
 * @brief   An vSLAM example for synthesis sequence
 * single camera
 * @author  Duy-Nguyen
 */

#include <boost/shared_ptr.hpp>
using namespace boost;

#define GTSAM_MAGIC_KEY

#include <gtsam/nonlinear/NonlinearFactorGraph-inl.h>
#include <gtsam/nonlinear/NonlinearOptimizer-inl.h>
#include <gtsam/inference/graph-inl.h>
#include <gtsam/slam/visualSLAM.h>
#include <gtsam/slam/PriorFactor.h>

#include <gtsam/inference/ISAM-inl.h>
#include <gtsam/linear/GaussianISAM.h>
#include "ISAMLoop.h"
#include "ISAMLoop-inl.h"

#include "vSLAMutils.h"
#include "Feature2D.h"

using namespace std;
using namespace gtsam;
using namespace gtsam::visualSLAM;
using namespace boost;

/* ************************************************************************* */
#define CALIB_FILE          "calib.txt"
#define LANDMARKS_FILE      "landmarks.txt"
#define POSES_FILE          "posesISAM.txt"
#define MEASUREMENTS_FILE    "measurementsISAM.txt"

// Base data folder
string g_dataFolder;

// Store groundtruth values, read from files
shared_ptrK g_calib;
map<int, Point3> g_landmarks;        // map: <landmark_id, landmark_position>
std::vector<Pose3> g_poses;          // prior camera poses at each frame
std::vector<std::vector<Feature2D> > g_measurements;    // feature sets detected at each frame

// Noise models
SharedGaussian measurementSigma(noiseModel::Isotropic::Sigma(2, 5.0f));
SharedGaussian poseSigma(noiseModel::Unit::Create(1));


/* ************************************************************************* */
/**
 * Read all data: calibration file, landmarks, poses, and all features measurements
 * Data is stored in global variables, which are used later to simulate incremental updates.
 */
void readAllDataISAM() {
  g_calib = readCalibData(g_dataFolder + CALIB_FILE);

  // Read groundtruth landmarks' positions. These will be used later as intial estimates and priors for landmark nodes.
  g_landmarks = readLandMarks(g_dataFolder + LANDMARKS_FILE);

  // Read groundtruth camera poses. These will be used later as intial estimates for pose nodes.
  g_poses = readPosesISAM(g_dataFolder, POSES_FILE);

  // Read all 2d measurements. Those will become factors linking their associating pose and the corresponding landmark.
  g_measurements = readAllMeasurementsISAM(g_dataFolder, MEASUREMENTS_FILE);
}

/* ************************************************************************* */
/**
 * Setup newFactors and initialValues for each new pose and set of measurements at each frame.
 */
void createNewFactors(shared_ptr<Graph>& newFactors, boost::shared_ptr<Values>& initialValues,
    int pose_id, Pose3& pose, std::vector<Feature2D>& measurements, SharedGaussian measurementSigma, shared_ptrK calib) {

  // Create a graph of newFactors with new measurements
  newFactors = shared_ptr<Graph> (new Graph());
  for (size_t i = 0; i < measurements.size(); i++) {
    newFactors->addMeasurement(
        measurements[i].m_p,
        measurementSigma,
        pose_id,
        measurements[i].m_idLandmark,
        calib);
  }

  // ... we need priors on the new pose and all new landmarks
  newFactors->addPosePrior(pose_id, pose, poseSigma);
  for (size_t i = 0; i < measurements.size(); i++) {
    newFactors->addPointPrior(measurements[i].m_idLandmark, g_landmarks[measurements[i].m_idLandmark]);
  }

  // Create initial values for all nodes in the newFactors
  initialValues = shared_ptr<Values> (new Values());
  initialValues->insert(pose_id, pose);
  for (size_t i = 0; i < measurements.size(); i++) {
    initialValues->insert(measurements[i].m_idLandmark, g_landmarks[measurements[i].m_idLandmark]);
  }
}

/* ************************************************************************* */
int main(int argc, char* argv[]) {
  if (argc < 2) {
    cout << "Usage: vISAMexample <DataFolder>" << endl << endl;
    cout << "\tPlease specify <DataFolder>, which contains calibration file, initial\n"
      "\tlandmarks, initial poses, and feature data." << endl;
    cout << "\tSample folder is in $gtsam_source_folder$/examples/vSLAMexample/Data/" << endl << endl;
    cout << "Example usage: vISAMexample '$gtsam_source_folder$/examples/vSLAMexample/Data/'" << endl;
    exit(0);
  }

  g_dataFolder = string(argv[1]) + "/";
  readAllDataISAM();

  // Create an ISAMLoop which will be relinearized and reordered after every "relinearizeInterval" updates
  int relinearizeInterval = 3;
  ISAMLoop<Values> isam(relinearizeInterval);

  // At each frame i with new camera pose and new set of measurements associated with it,
  // create a graph of new factors and update ISAM
  for (size_t i = 0; i < g_measurements.size(); i++) {
    shared_ptr<Graph> newFactors;
    shared_ptr<Values> initialValues;
    createNewFactors(newFactors, initialValues, i, g_poses[i], g_measurements[i], measurementSigma, g_calib);

    isam.update(*newFactors, *initialValues);
    Values currentEstimate = isam.estimate();
    cout << "****************************************************" << endl;
    currentEstimate.print("Current estimate: ");
  }

  return 0;
}
/* ************************************************************************* */

