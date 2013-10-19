
/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file KITTItoBALConverter.cpp
 * @brief Program for reading KITTI files and convert these ones to BAL format
 * @date October, 2013
 * @author Pablo F. Alcantarilla
 */

// Both relative poses and recovered trajectory poses will be stored as Pose3 objects
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Cal3DS2.h>

// Each variable in the system (poses and landmarks) must be identified with a unique key.
// We can either use simple integer keys (1, 2, 3, ...) or symbols (X1, X2, L1).
// Here we will use Symbols
#include <gtsam/inference/Symbol.h>

// We want to use iSAM2 to solve the range-SLAM problem incrementally
#include <gtsam/nonlinear/ISAM2.h>

// iSAM2 requires as input a set set of new factors to be added stored in a factor graph,
// and initial guesses for any new variables used in the added factors
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

// We will use a non-liear solver to batch-inituialize from the first 150 frames
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>

// In GTSAM, measurement functions are represented as 'factors'. Several common factors
// have been provided with the library for solving robotics SLAM problems.
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/ProjectionFactor.h>
#include <gtsam_unstable/slam/SmartProjectionHessianFactor.h>

// We need to use SFM_data to save it to BAL format
#include <gtsam/slam/dataset.h>

// Standard headers, added last, so we know headers above work on their own
#include <boost/foreach.hpp>
#include <boost/assign.hpp>
#include <boost/assign/std/vector.hpp>
#include <fstream>
#include <iostream>

using namespace std;
using namespace gtsam;
using namespace boost::assign;
namespace NM = gtsam::noiseModel;

using symbol_shorthand::X;
using symbol_shorthand::L;

typedef PriorFactor<Pose3> Pose3Prior;

/* ************************************************************************* */

//// Helper functions taken from VO code
// Loaded all pose values into list
Values::shared_ptr loadPoseValues(const string& filename) {
  Values::shared_ptr values(new Values());
  bool addNoise = false;
  Pose3 noise_pose = Pose3(Rot3::ypr(-M_PI/10, 0., -M_PI/10), gtsam::Point3(0.5,0.1,0.3));

  // read in camera poses
  string full_filename = filename;
  ifstream fin;
  fin.open(full_filename.c_str());

  int pose_id;
  while (fin >> pose_id) {
    double pose_matrix[16];
    for (int i = 0; i < 16; i++) {
      fin >> pose_matrix[i];
    }

    if (addNoise) {
      values->insert(Symbol('x',pose_id), Pose3(Matrix_(4, 4, pose_matrix)).compose(noise_pose));
    } else {
      values->insert(Symbol('x',pose_id), Pose3(Matrix_(4, 4, pose_matrix)));
    }
  }

  fin.close();
  return values;
}

/* ************************************************************************* */

// Loaded specific pose values that are in key list
Values::shared_ptr loadPoseValues(const string& filename, list<Key> keys) {
  Values::shared_ptr values(new Values());
  std::list<Key>::iterator kit;

  // read in camera poses
  string full_filename = filename;
  ifstream fin;
  fin.open(full_filename.c_str());

  int pose_id;
  while (fin >> pose_id) {
    double pose_matrix[16];
    for (int i = 0; i < 16; i++) {
      fin >> pose_matrix[i];
    }
    kit = find (keys.begin(), keys.end(), X(pose_id));
    if (kit != keys.end()) {
      cout << " Adding " << X(pose_id) << endl;
      values->insert(Symbol('x',pose_id), Pose3(Matrix_(4, 4, pose_matrix)));
    }
  }

  fin.close();
  return values;
}

/* ************************************************************************* */

// Load calibration info
Cal3_S2::shared_ptr loadCalibration(const string& filename) {
  string full_filename = filename;
  ifstream fin;
  fin.open(full_filename.c_str());

  // try loading from parent directory as backup
  if(!fin) {
    cerr << "Could not load " << full_filename;
    exit(1);
  }

  double fx, fy, s, u, v, b;
  fin >> fx >> fy >> s >> u >> v >> b;
  fin.close();

  Cal3_S2::shared_ptr K(new Cal3_S2(fx, fy, s, u, v));

  return K;
}

/* ************************************************************************* */

void writeValues(string directory_, const Values& values){
  string filename = directory_ + "camera_poses.txt";
  ofstream fout;
  fout.open(filename.c_str());
  fout.precision(20);

  // write out camera poses
  BOOST_FOREACH(Values::ConstFiltered<Pose3>::value_type key_value, values.filter<Pose3>()) {
    fout << Symbol(key_value.key).index();
    const gtsam::Matrix& matrix= key_value.value.matrix();
    for (size_t row=0; row < 4; ++row) {
      for (size_t col=0; col < 4; ++col) {
        fout << " " << matrix(row, col);
      }
    }
    fout << endl;
  }
  fout.close();

  if(values.filter<Point3>().size() > 0) {
    // write landmarks
    filename = directory_ + "landmarks.txt";
    fout.open(filename.c_str());

    BOOST_FOREACH(Values::ConstFiltered<Point3>::value_type key_value, values.filter<Point3>()) {
      fout << Symbol(key_value.key).index();
      fout << " " << key_value.value.x();
      fout << " " << key_value.value.y();
      fout << " " << key_value.value.z();
      fout << endl;
    }
    fout.close();

  }
}

/* ************************************************************************* */

int main(int argc, char** argv) {


  SfM_data kitti_sfm;

  int ncameras = 0, npoints = 0, nobservations = 0;

  bool debug = false;

  // Minimum number of views of a landmark before it is added to the graph (SmartProjectionFactor case only)
  unsigned int minimumNumViews = 1;

  string HOME = getenv("HOME");
  string input_dir = HOME + "/Research/datasets/kitti/loop_closures_merged/";
  string output_file = HOME + "/Research/datasets/kitti/loop_closures_merged/loop_closures_merged_bal.txt";

  typedef GenericProjectionFactor<Pose3, Point3, Cal3_S2> ProjectionFactor;
  NonlinearFactorGraph graph;

  // Load calibration
  boost::shared_ptr<Cal3_S2> K = loadCalibration(input_dir+"calibration.txt");
  Cal3Bundler Kd(K->fx(),0,0,0,0);
  K->print("Calibration");

  // Load values from VO camera poses output
  gtsam::Values::shared_ptr loaded_values = loadPoseValues(input_dir+"camera_poses.txt");

  // Load camera poses
  BOOST_FOREACH(Values::ConstFiltered<Pose3>::value_type key_value, loaded_values->filter<Pose3>()) {
    Pose3 pose = key_value.value;
    kitti_sfm.cameras.push_back(SfM_Camera(pose,Kd));
    ncameras++;
  }

  // Read in kitti dataset
  ifstream fin;
  fin.open((input_dir+"sorted_contiguous_stereo_factors.txt").c_str());
  if(!fin) {
    cerr << "Could not open sorted_contiguous_stereo_factors.txt" << endl;
    exit(1);
  }

  // Read all measurements tracked by VO stereo
  cout << "Loading sorted_contiguous_stereo_factors.txt" << endl;
  Key r, l, currentLandmark = 0;
  std::list<Key> allViews;
  std::vector<Key> views;
  std::vector<Point2> measurements;
  Values values;

  float uL, uR, v, x, y, z;
  while (fin >> r >> l >> uL >> uR >> v >> x >> y >> z){

    if (debug) cout << "CurrentLandmark " << currentLandmark << " Landmark " << l << std::endl;

    if (loaded_values->exists<Point3>(L(l)) == boost::none) {
      Pose3 camera = loaded_values->at<Pose3>(X(r));
      Point3 worldPoint = camera.transform_from(Point3(x, y, z));
      loaded_values->insert(L(l), worldPoint); // add point;
      npoints++;

      // Create a track without observations
      SfM_Track track;
      track.p = worldPoint;
      track.r = .4;
      track.g = .4;
      track.b = .4;
      kitti_sfm.tracks.push_back(track);
    }

    nobservations++;
  }

  fin.close();

  // Open again the file to store the measurements
  fin.open((input_dir+"sorted_contiguous_stereo_factors.txt").c_str());
  if(!fin) {
    cerr << "Could not open sorted_contiguous_stereo_factors.txt" << endl;
    exit(1);
  }
  while (fin >> r >> l >> uL >> uR >> v >> x >> y >> z){

    SfM_Measurement observation = make_pair(r,Point2(uL,v));
    kitti_sfm.tracks[l].measurements.push_back(observation);
  }
  fin.close();

  cout << "ncameras " << ncameras << " npoints " << npoints << endl;
  cout << "nobservations " << nobservations << endl;

  writeBAL(output_file,kitti_sfm);

  exit(0);
}

