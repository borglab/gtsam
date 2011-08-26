/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    vSFMexample.cpp
 * @brief   An vSFM example for synthesis sequence
 * single camera
 * @author  Duy-Nguyen Ta
 */

#include <boost/shared_ptr.hpp>
using namespace boost;

// Magically casts strings like "x3" to a Symbol('x',3) key, see Key.h
#define GTSAM_MAGIC_KEY

#include <gtsam/nonlinear/NonlinearFactorGraph-inl.h>
#include <gtsam/nonlinear/NonlinearOptimizer-inl.h>
#include <gtsam/inference/graph-inl.h>
#include <gtsam/slam/visualSLAM.h>
#include <gtsam/slam/PriorFactor.h>

#include "vSLAMutils.h"
#include "Feature2D.h"

using namespace std;
using namespace gtsam;
using namespace gtsam::visualSLAM;
using namespace boost;

/* ************************************************************************* */
#define CALIB_FILE          "calib.txt"
#define LANDMARKS_FILE      "landmarks.txt"
#define POSES_FILE          "poses.txt"
#define MEASUREMENTS_FILE    "measurements.txt"

// Base data folder
string g_dataFolder;

// Store groundtruth values, read from files
shared_ptrK g_calib;
map<int, Point3> g_landmarks;        // map: <landmark_id, landmark_position>
map<int, Pose3> g_poses;            // map: <camera_id, pose>
std::vector<Feature2D> g_measurements;    // Feature2D: {camera_id, landmark_id, 2d feature_position}

// Noise models
SharedNoiseModel measurementSigma(noiseModel::Isotropic::Sigma(2, 5.0f));

/* ************************************************************************* */
/**
 * Read all data: calibration file, landmarks, poses, and all features measurements
 * Data is stored in global variables.
 */
void readAllData() {

  g_calib = readCalibData(g_dataFolder + "/" + CALIB_FILE);

  // Read groundtruth landmarks' positions. These will be used later as intial estimates for landmark nodes.
  g_landmarks = readLandMarks(g_dataFolder + "/" + LANDMARKS_FILE);

  // Read groundtruth camera poses. These will be used later as intial estimates for pose nodes.
  g_poses = readPoses(g_dataFolder, POSES_FILE);

  // Read all 2d measurements. Those will become factors linking their associating pose and the corresponding landmark.
  g_measurements = readAllMeasurements(g_dataFolder, MEASUREMENTS_FILE);
}

/* ************************************************************************* */
/**
 * Setup vSLAM graph
 * by adding and linking 2D features (measurements) detected in each captured image
 * with their corresponding landmarks.
 */
Graph setupGraph(std::vector<Feature2D>& measurements, SharedNoiseModel measurementSigma, shared_ptrK calib) {

  Graph g;

  cout << "Built graph: " << endl;
  for (size_t i = 0; i < measurements.size(); i++) {
    measurements[i].print();

    g.addMeasurement(
        measurements[i].m_p,
        measurementSigma,
        measurements[i].m_idCamera,
        measurements[i].m_idLandmark,
        calib);
  }

  return g;
}

/* ************************************************************************* */
/**
 * Create a structure of initial estimates for all nodes (landmarks and poses) in the graph.
 * The returned Values structure contains all initial values for all nodes.
 */
Values initialize(std::map<int, Point3> landmarks, std::map<int, Pose3> poses) {

  Values initValues;

  // Initialize landmarks 3D positions.
  for (map<int, Point3>::iterator lmit = landmarks.begin(); lmit != landmarks.end(); lmit++)
    initValues.insert(lmit->first, lmit->second);

  // Initialize camera poses.
  for (map<int, Pose3>::iterator poseit = poses.begin(); poseit != poses.end(); poseit++)
    initValues.insert(poseit->first, poseit->second);

  return initValues;
}

/* ************************************************************************* */
int main(int argc, char* argv[]) {
  if (argc < 2) {
    cout << "Usage: vSFMexample <DataFolder>" << endl << endl;
    cout << "\tPlease specify <DataFolder>, which contains calibration file, initial\n"
          "\tlandmarks, initial poses, and feature data." << endl;
    cout << "\tSample folder is in $gtsam_source_folder$/examples/vSLAMexample/Data" << endl << endl;
    cout << "Example usage: vSFMexample '$gtsam_source_folder$/examples/vSLAMexample/Data'" << endl;
    exit(0);
  }

  g_dataFolder = string(argv[1]) + "/";
  readAllData();

  // Create a graph using the 2D measurements (features) and the calibration data
  boost::shared_ptr<Graph> graph(new Graph(setupGraph(g_measurements, measurementSigma, g_calib)));

  // Create an initial Values structure using groundtruth values as the initial estimates
  boost::shared_ptr<Values> initialEstimates(new Values(initialize(g_landmarks, g_poses)));
  cout << "*******************************************************" << endl;
  initialEstimates->print("INITIAL ESTIMATES: ");

  // Add prior factor for all poses in the graph
  map<int, Pose3>::iterator poseit = g_poses.begin();
  for (; poseit != g_poses.end(); poseit++)
    graph->addPosePrior(poseit->first, poseit->second, noiseModel::Unit::Create(1));

  // Optimize the graph
  cout << "*******************************************************" << endl;
  NonlinearOptimizationParameters::sharedThis params = NonlinearOptimizationParameters::newVerbosity(Optimizer::Parameters::DAMPED);
  Optimizer::shared_values result = Optimizer::optimizeGN(graph, initialEstimates, params);

  // Print final results
  cout << "*******************************************************" << endl;
  result->print("FINAL RESULTS: ");

  return 0;
}
/* ************************************************************************* */

