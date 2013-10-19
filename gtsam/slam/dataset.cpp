/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file dataset.cpp
 * @date Jan 22, 2010
 * @author nikai, Luca Carlone
 * @brief utility functions for loading datasets
 */

#include <fstream>
#include <sstream>
#include <cstdlib>

#include <boost/filesystem.hpp>

#include <gtsam/geometry/Pose2.h>
#include <gtsam/linear/Sampler.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/slam/dataset.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/BearingRangeFactor.h>

using namespace std;
namespace fs = boost::filesystem;
using namespace gtsam::symbol_shorthand;

#define LINESIZE 81920

namespace gtsam {

#ifndef MATLAB_MEX_FILE
/* ************************************************************************* */
string findExampleDataFile(const string& name) {
  // Search source tree and installed location
  vector<string> rootsToSearch;
  rootsToSearch.push_back(GTSAM_SOURCE_TREE_DATASET_DIR); // Defined by CMake, see gtsam/gtsam/CMakeLists.txt
  rootsToSearch.push_back(GTSAM_INSTALLED_DATASET_DIR);   // Defined by CMake, see gtsam/gtsam/CMakeLists.txt

  // Search for filename as given, and with .graph and .txt extensions
  vector<string> namesToSearch;
  namesToSearch.push_back(name);
  namesToSearch.push_back(name + ".graph");
  namesToSearch.push_back(name + ".txt");
  namesToSearch.push_back(name + ".out");

  // Find first name that exists
  BOOST_FOREACH(const fs::path& root, rootsToSearch) {
    BOOST_FOREACH(const fs::path& name, namesToSearch) {
      if(fs::is_regular_file(root / name))
        return (root / name).string();
    }
  }

  // If we did not return already, then we did not find the file
  throw std::invalid_argument(
      "gtsam::findExampleDataFile could not find a matching file in\n"
      SOURCE_TREE_DATASET_DIR " or\n"
      INSTALLED_DATASET_DIR " named\n" +
      name + ", " + name + ".graph, or " + name + ".txt");
}
#endif

/* ************************************************************************* */
pair<NonlinearFactorGraph::shared_ptr, Values::shared_ptr> load2D(
    pair<string, boost::optional<noiseModel::Diagonal::shared_ptr> > dataset,
    int maxID, bool addNoise, bool smart) {
  return load2D(dataset.first, dataset.second, maxID, addNoise, smart);
}

/* ************************************************************************* */
pair<NonlinearFactorGraph::shared_ptr, Values::shared_ptr> load2D(
    const string& filename, boost::optional<noiseModel::Diagonal::shared_ptr> model, int maxID,
    bool addNoise, bool smart) {
  cout << "Will try to read " << filename << endl;
  ifstream is(filename.c_str());
  if (!is)
    throw std::invalid_argument("load2D: can not find the file!");

  Values::shared_ptr initial(new Values);
  NonlinearFactorGraph::shared_ptr graph(new NonlinearFactorGraph);

  string tag;

  // load the poses
  while (is) {
    if(! (is >> tag))
      break;

    if ((tag == "VERTEX2") || (tag == "VERTEX")) {
      int id;
      double x, y, yaw;
      is >> id >> x >> y >> yaw;
      // optional filter
      if (maxID && id >= maxID)
        continue;
      initial->insert(id, Pose2(x, y, yaw));
    }
    is.ignore(LINESIZE, '\n');
  }
  is.clear(); /* clears the end-of-file and error flags */
  is.seekg(0, ios::beg);

  // Create a sampler with random number generator
  Sampler sampler(42u);

  // load the factors
  bool haveLandmark = false;
  while (is) {
    if(! (is >> tag))
      break;

    if ((tag == "EDGE2") || (tag == "EDGE") || (tag == "ODOMETRY")) {
      int id1, id2;
      double x, y, yaw;
      double v1, v2, v3, v4, v5, v6;

      is >> id1 >> id2 >> x >> y >> yaw;
      is >> v1 >> v2 >> v3 >> v4 >> v5 >> v6;

      // Try to guess covariance matrix layout
      Matrix m(3,3);
      if(v1 != 0.0 && v2 == 0.0 && v3 != 0.0 && v4 != 0.0 && v5 == 0.0 && v6 == 0.0)
      {
        // Looks like [ v1 v2 v5; v2' v3 v6; v5' v6' v4 ]
        m <<  v1, v2, v5,  v2, v3, v6,  v5, v6, v4;
      }
      else if(v1 != 0.0 && v2 == 0.0 && v3 == 0.0 && v4 != 0.0 && v5 == 0.0 && v6 != 0.0)
      {
        // Looks like [ v1 v2 v3; v2' v4 v5; v3' v5' v6 ]
        m << v1, v2, v3,  v2, v4, v5,  v3, v5, v6;
      }
      else
      {
        throw std::invalid_argument("load2D: unrecognized covariance matrix format in dataset file");
      }

      // optional filter
      if (maxID && (id1 >= maxID || id2 >= maxID))
        continue;

      Pose2 l1Xl2(x, y, yaw);

      // SharedNoiseModel noise = noiseModel::Gaussian::Covariance(m, smart);
      if (!model) {
        Vector variances = Vector_(3, m(0, 0), m(1, 1), m(2, 2));
        model = noiseModel::Diagonal::Variances(variances, smart);
      }

      if (addNoise)
        l1Xl2 = l1Xl2.retract(sampler.sampleNewModel(*model));

      // Insert vertices if pure odometry file
      if (!initial->exists(id1))
        initial->insert(id1, Pose2());
      if (!initial->exists(id2))
        initial->insert(id2, initial->at<Pose2>(id1) * l1Xl2);

      NonlinearFactor::shared_ptr factor(
          new BetweenFactor<Pose2>(id1, id2, l1Xl2, *model));
      graph->push_back(factor);
    }
    if (tag == "BR") {
      int id1, id2;
      double bearing, range, bearing_std, range_std;

      is >> id1 >> id2 >> bearing >> range >> bearing_std >> range_std;

      // optional filter
      if (maxID && (id1 >= maxID || id2 >= maxID))
        continue;

      noiseModel::Diagonal::shared_ptr measurementNoise =
          noiseModel::Diagonal::Sigmas(Vector_(2, bearing_std, range_std));
      *graph += BearingRangeFactor<Pose2, Point2>(id1, id2, bearing, range, measurementNoise);

      // Insert poses or points if they do not exist yet
      if (!initial->exists(id1))
        initial->insert(id1, Pose2());
      if (!initial->exists(id2)) {
        Pose2 pose = initial->at<Pose2>(id1);
        Point2 local(cos(bearing)*range,sin(bearing)*range);
        Point2 global = pose.transform_from(local);
        initial->insert(id2, global);
      }
    }
    if (tag == "LANDMARK") {
      int id1, id2;
      double lmx, lmy;
      double v1, v2, v3;

      is >> id1 >> id2 >> lmx >> lmy >> v1 >> v2 >> v3;

      // Convert x,y to bearing,range
      double bearing = std::atan2(lmy, lmx);
      double range = std::sqrt(lmx*lmx + lmy*lmy);

      // In our experience, the x-y covariance on landmark sightings is not very good, so assume
      // that it describes the uncertainty at a range of 10m, and convert that to bearing/range
      // uncertainty.
      SharedDiagonal measurementNoise;
      if(std::abs(v1 - v3) < 1e-4)
      {
        double rangeVar = v1;
        double bearingVar = v1 / 10.0;
        measurementNoise = noiseModel::Diagonal::Sigmas(Vector_(2, bearingVar, rangeVar));
      }
      else
      {
        if(!haveLandmark) {
          cout << "Warning: load2D is a very simple dataset loader and is ignoring the\n"
              "non-uniform covariance on LANDMARK measurements in this file." << endl;
          haveLandmark = true;
        }
      }

      // Add to graph
      *graph += BearingRangeFactor<Pose2, Point2>(id1, L(id2), bearing, range, measurementNoise);
    }
    is.ignore(LINESIZE, '\n');
  }

  cout << "load2D read a graph file with " << initial->size()
                    << " vertices and " << graph->nrFactors() << " factors" << endl;

  return make_pair(graph, initial);
}

/* ************************************************************************* */
void save2D(const NonlinearFactorGraph& graph, const Values& config,
    const noiseModel::Diagonal::shared_ptr model, const string& filename) {

  fstream stream(filename.c_str(), fstream::out);

  // save poses
  BOOST_FOREACH(const Values::ConstKeyValuePair& key_value, config)
  {
    const Pose2& pose = dynamic_cast<const Pose2&>(key_value.value);
    stream << "VERTEX2 " << key_value.key << " " << pose.x() << " "
        << pose.y() << " " << pose.theta() << endl;
  }

  // save edges
  Matrix R = model->R();
  Matrix RR = trans(R) * R; //prod(trans(R),R);
  BOOST_FOREACH(boost::shared_ptr<NonlinearFactor> factor_, graph)
  {
    boost::shared_ptr<BetweenFactor<Pose2> > factor =
        boost::dynamic_pointer_cast<BetweenFactor<Pose2> >(factor_);
    if (!factor)
      continue;

    Pose2 pose = factor->measured().inverse();
    stream << "EDGE2 " << factor->key2() << " " << factor->key1() << " "
        << pose.x() << " " << pose.y() << " " << pose.theta() << " "
        << RR(0, 0) << " " << RR(0, 1) << " " << RR(1, 1) << " "
        << RR(2, 2) << " " << RR(0, 2) << " " << RR(1, 2) << endl;
  }

  stream.close();
}

/* ************************************************************************* */
bool load3D(const string& filename) {
  ifstream is(filename.c_str());
  if (!is)
    return false;

  while (is) {
    char buf[LINESIZE];
    is.getline(buf, LINESIZE);
    istringstream ls(buf);
    string tag;
    ls >> tag;

    if (tag == "VERTEX3") {
      int id;
      double x, y, z, roll, pitch, yaw;
      ls >> id >> x >> y >> z >> roll >> pitch >> yaw;
    }
  }
  is.clear(); /* clears the end-of-file and error flags */
  is.seekg(0, ios::beg);

  while (is) {
    char buf[LINESIZE];
    is.getline(buf, LINESIZE);
    istringstream ls(buf);
    string tag;
    ls >> tag;

    if (tag == "EDGE3") {
      int id1, id2;
      double x, y, z, roll, pitch, yaw;
      ls >> id1 >> id2 >> x >> y >> z >> roll >> pitch >> yaw;
      Matrix m = eye(6);
      for (int i = 0; i < 6; i++)
        for (int j = i; j < 6; j++)
          ls >> m(i, j);
    }
  }
  return true;
}

/* ************************************************************************* */
pair<NonlinearFactorGraph::shared_ptr, Values::shared_ptr> load2D_robust(
    const string& filename, noiseModel::Base::shared_ptr& model, int maxID) {
  cout << "Will try to read " << filename << endl;
  ifstream is(filename.c_str());
  if (!is)
    throw std::invalid_argument("load2D: can not find the file!");

  Values::shared_ptr initial(new Values);
  NonlinearFactorGraph::shared_ptr graph(new NonlinearFactorGraph);

  string tag;

  // load the poses
  while (is) {
    is >> tag;

    if ((tag == "VERTEX2") || (tag == "VERTEX")) {
      int id;
      double x, y, yaw;
      is >> id >> x >> y >> yaw;
      // optional filter
      if (maxID && id >= maxID)
        continue;
      initial->insert(id, Pose2(x, y, yaw));
    }
    is.ignore(LINESIZE, '\n');
  }
  is.clear(); /* clears the end-of-file and error flags */
  is.seekg(0, ios::beg);

  // Create a sampler with random number generator
  Sampler sampler(42u);

  // load the factors
  while (is) {
    is >> tag;

    if ((tag == "EDGE2") || (tag == "EDGE") || (tag == "ODOMETRY")) {
      int id1, id2;
      double x, y, yaw;

      is >> id1 >> id2 >> x >> y >> yaw;
      Matrix m = eye(3);
      is >> m(0, 0) >> m(0, 1) >> m(1, 1) >> m(2, 2) >> m(0, 2) >> m(1, 2);
      m(2, 0) = m(0, 2);
      m(2, 1) = m(1, 2);
      m(1, 0) = m(0, 1);

      // optional filter
      if (maxID && (id1 >= maxID || id2 >= maxID))
        continue;

      Pose2 l1Xl2(x, y, yaw);

      // Insert vertices if pure odometry file
      if (!initial->exists(id1))
        initial->insert(id1, Pose2());
      if (!initial->exists(id2))
        initial->insert(id2, initial->at<Pose2>(id1) * l1Xl2);

      NonlinearFactor::shared_ptr factor(
          new BetweenFactor<Pose2>(id1, id2, l1Xl2, model));
      graph->push_back(factor);
    }
    if (tag == "BR") {
      int id1, id2;
      double bearing, range, bearing_std, range_std;

      is >> id1 >> id2 >> bearing >> range >> bearing_std >> range_std;

      // optional filter
      if (maxID && (id1 >= maxID || id2 >= maxID))
        continue;

      noiseModel::Diagonal::shared_ptr measurementNoise =
          noiseModel::Diagonal::Sigmas(Vector_(2, bearing_std, range_std));
      *graph += BearingRangeFactor<Pose2, Point2>(id1, id2, bearing, range, measurementNoise);

      // Insert poses or points if they do not exist yet
      if (!initial->exists(id1))
        initial->insert(id1, Pose2());
      if (!initial->exists(id2)) {
        Pose2 pose = initial->at<Pose2>(id1);
        Point2 local(cos(bearing)*range,sin(bearing)*range);
        Point2 global = pose.transform_from(local);
        initial->insert(id2, global);
      }
    }
    is.ignore(LINESIZE, '\n');
  }

  cout << "load2D read a graph file with " << initial->size()
                    << " vertices and " << graph->nrFactors() << " factors" << endl;

  return make_pair(graph, initial);
}

/* ************************************************************************* */
Rot3 openGLFixedRotation(){ // this is due to different convention for cameras in gtsam and openGL
  /* R = [ 1   0   0
   *       0  -1   0
   *       0   0  -1]
   */
  Matrix3  R_mat = Matrix3::Zero(3,3);
  R_mat(0,0) = 1.0;  R_mat(1,1) = -1.0; R_mat(2,2) = -1.0;
  return Rot3(R_mat);
}

/* ************************************************************************* */
Pose3 openGL2gtsam(const Rot3& R, double tx, double ty, double tz)
{
  Rot3 R90 = openGLFixedRotation();
  Rot3 wRc = (  R.inverse() ).compose(R90);

  // Our camera-to-world translation wTc = -R'*t
  return Pose3 (wRc, R.unrotate(Point3(-tx,-ty,-tz)));
}

/* ************************************************************************* */
Pose3 gtsam2openGL(const Rot3& R, double tx, double ty, double tz)
{
  Rot3 R90 = openGLFixedRotation();
  Rot3 cRw_openGL = R90.compose(  R.inverse() );
  Point3 t_openGL = cRw_openGL.rotate(Point3(-tx,-ty,-tz));
  return Pose3(cRw_openGL, t_openGL);
}

/* ************************************************************************* */
Pose3 gtsam2openGL(const Pose3& PoseGTSAM)
{
  return gtsam2openGL(PoseGTSAM.rotation(), PoseGTSAM.x(), PoseGTSAM.y(), PoseGTSAM.z());
}

/* ************************************************************************* */
bool readBundler(const string& filename, SfM_data &data)
{
  // Load the data file
  ifstream is(filename.c_str(),ifstream::in);
  if(!is)
  {
    cout << "Error in readBundler: can not find the file!!" << endl;
    return false;
  }

  // Ignore the first line
  char aux[500];
  is.getline(aux,500);

  // Get the number of camera poses and 3D points
  size_t nrPoses, nrPoints;
  is >> nrPoses >> nrPoints;

  // Get the information for the camera poses
  for( size_t i = 0; i < nrPoses; i++ )
  {
    // Get the focal length and the radial distortion parameters
    float f, k1, k2;
    is >> f >> k1 >> k2;
    Cal3Bundler K(f, k1, k2);

    // Get the rotation matrix
    float r11, r12, r13;
    float r21, r22, r23;
    float r31, r32, r33;
    is >> r11 >> r12 >> r13
    >> r21 >> r22 >> r23
    >> r31 >> r32 >> r33;

    // Bundler-OpenGL rotation matrix
    Rot3 R(
        r11, r12, r13,
        r21, r22, r23,
        r31, r32, r33);

    // Check for all-zero R, in which case quit
    if(r11==0 && r12==0 && r13==0)
    {
      cout << "Error in readBundler: zero rotation matrix for pose " << i << endl;
      return false;
    }

    // Get the translation vector
    float tx, ty, tz;
    is >> tx >> ty >> tz;

    Pose3 pose = openGL2gtsam(R,tx,ty,tz);

    data.cameras.push_back(SfM_Camera(pose,K));
  }

  // Get the information for the 3D points
  for( size_t j = 0; j < nrPoints; j++ )
  {
    SfM_Track track;

    // Get the 3D position
    float x, y, z;
    is >> x >> y >> z;
    track.p = Point3(x,y,z);

    // Get the color information
    float r, g, b;
    is >> r >> g >> b;
    track.r = r/255.0;
    track.g = g/255.0;
    track.b = b/255.0;

    // Now get the visibility information
    size_t nvisible = 0;
    is >> nvisible;

    for( size_t k = 0; k < nvisible; k++ )
    {
      size_t cam_idx = 0, point_idx = 0;
      float u, v;
      is >> cam_idx >> point_idx >> u >> v;
      track.measurements.push_back(make_pair(cam_idx,Point2(u,-v)));
    }

    data.tracks.push_back(track);
  }

  is.close();
  return true;
}

/* ************************************************************************* */
bool readBAL(const string& filename, SfM_data &data)
{
  // Load the data file
  ifstream is(filename.c_str(),ifstream::in);
  if(!is)
  {
    cout << "Error in readBAL: can not find the file!!" << endl;
    return false;
  }

  // Get the number of camera poses and 3D points
  size_t nrPoses, nrPoints, nrObservations;
  is >> nrPoses >> nrPoints >> nrObservations;

  data.tracks.resize(nrPoints);

  // Get the information for the observations
  for( size_t k = 0; k < nrObservations; k++ )
  {
    size_t i = 0, j = 0;
    float u, v;
    is >> i >> j >> u >> v;
    data.tracks[j].measurements.push_back(make_pair(i,Point2(u,-v)));
  }

  // Get the information for the camera poses
  for( size_t i = 0; i < nrPoses; i++ )
  {
    // Get the rodriguez vector
    float wx, wy, wz;
    is >> wx >> wy >> wz;
    Rot3 R = Rot3::rodriguez(wx, wy, wz);// BAL-OpenGL rotation matrix

    // Get the translation vector
    float tx, ty, tz;
    is >> tx >> ty >> tz;

    Pose3 pose = openGL2gtsam(R,tx,ty,tz);

    // Get the focal length and the radial distortion parameters
    float f, k1, k2;
    is >> f >> k1 >> k2;
    Cal3Bundler K(f, k1, k2);

    data.cameras.push_back(SfM_Camera(pose,K));
  }

  // Get the information for the 3D points
  for( size_t j = 0; j < nrPoints; j++ )
  {
    // Get the 3D position
    float x, y, z;
    is >> x >> y >> z;
    SfM_Track& track = data.tracks[j];
    track.p = Point3(x,y,z);
    track.r = 0.4;
    track.g = 0.4;
    track.b = 0.4;
  }

  is.close();
  return true;
}

/* ************************************************************************* */
bool writeBAL(const string& filename, SfM_data &data)
{
  // Open the output file
  ofstream os;
  os.open(filename.c_str());
  os.precision(20);
  if (!os.is_open()) {
    cout << "Error in writeBAL: can not open the file!!" << endl;
    return false;
  }

  // Write the number of camera poses and 3D points
  int nrObservations=0;
  for (size_t j = 0; j < data.number_tracks(); j++){
    nrObservations += data.tracks[j].number_measurements();
  }

  // Write observations
  os <<  data.number_cameras() << " " << data.number_tracks() << " " << nrObservations << endl;
  os << endl;

  for (size_t j = 0; j < data.number_tracks(); j++){ // for each 3D point j
    SfM_Track track = data.tracks[j];

    for(size_t k = 0; k < track.number_measurements(); k++){ // for each observation of the 3D point j
      int i = track.measurements[k].first; // camera id
      double u0 = data.cameras[i].calibration().u0();
      double v0 = data.cameras[i].calibration().v0();

      if(u0 != 0 || v0 != 0){cout<< "writeBAL has not been tested for calibration with nonzero (u0,v0)"<< endl;}

      double pixelBALx = track.measurements[k].second.x() - u0; // center of image is the origin
      double pixelBALy = - (track.measurements[k].second.y() - v0); // center of image is the origin
      Point2 pixelMeasurement(pixelBALx, pixelBALy);
      os <<  i /*camera id*/ << " " << j /*point id*/ << " "
          << pixelMeasurement.x() /*u of the pixel*/ << " " << pixelMeasurement.y() /*v of the pixel*/ << endl;
    }
  }
  os << endl;

  // Write cameras
  for (size_t i = 0; i < data.number_cameras(); i++){ // for each camera
    Pose3 poseGTSAM = data.cameras[i].pose();
    Cal3Bundler cameraCalibration = data.cameras[i].calibration();
    Pose3 poseOpenGL = gtsam2openGL(poseGTSAM);
    os <<  Rot3::Logmap(poseOpenGL.rotation()) << endl;
    os <<  poseOpenGL.translation().vector() << endl;
    os <<  cameraCalibration.fx() << endl;
    os <<  cameraCalibration.k1() << endl;
    os <<  cameraCalibration.k2() << endl;
    os << endl;
  }

  // Write the points
  for (size_t j = 0; j < data.number_tracks(); j++){ // for each 3D point j
    Point3 point = data.tracks[j].p;
    os << point.x() << endl;
    os << point.y() << endl;
    os << point.z() << endl;
    os << endl;
  }

  os.close();
  return true;
}

bool writeBALfromValues(const string& filename, SfM_data &data, Values& values){

  // CHECKS
  Values valuesPoses = values.filter<Pose3>();
  if( valuesPoses.size() != data.number_cameras()){
    cout << "writeBALfromValues: different number of cameras in SfM_data (#cameras= " << data.number_cameras()
        <<") and values (#cameras " << valuesPoses.size() << ")!!" << endl;
    return false;
  }
  Values valuesPoints = values.filter<Point3>();
  if( valuesPoints.size() != data.number_tracks()){
    cout << "writeBALfromValues: different number of points in SfM_data (#points= " << data.number_tracks()
        <<") and values (#points " << valuesPoints.size() << ")!!" << endl;
    return false;
  }
  if(valuesPoints.size() + valuesPoses.size() != values.size()){
    cout << "writeBALfromValues write only poses and points values!!" << endl;
    return false;
  }
  if(valuesPoints.size()==0 || valuesPoses.size()==0){
    cout << "writeBALfromValues: No point or pose in values!!" << endl;
    return false;
  }

  for (size_t i = 0; i < data.number_cameras(); i++){ // for each camera
    Key poseKey = symbol('x',i);
    Pose3 pose = values.at<Pose3>(poseKey);
    Cal3Bundler K = data.cameras[i].calibration();
    PinholeCamera<Cal3Bundler> camera(pose, K);
    data.cameras[i] = camera;
  }

  for (size_t j = 0; j < data.number_tracks(); j++){ // for each point
    Key pointKey = symbol('l',j);
    Point3 point = values.at<Point3>(pointKey);
    data.tracks[j].p = point;
  }

  return writeBAL(filename, data);
}


} // \namespace gtsam
