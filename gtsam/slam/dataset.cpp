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
 * @author Kai Ni, Luca Carlone, Frank Dellaert
 * @brief utility functions for loading datasets
 */

#include <gtsam/sam/BearingRangeFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/dataset.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/inference/FactorGraph.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/Values-inl.h>
#include <gtsam/linear/Sampler.h>
#include <gtsam/base/GenericValue.h>
#include <gtsam/base/Lie.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/types.h>
#include <gtsam/base/Value.h>
#include <gtsam/base/Vector.h>

#include <boost/assign/list_inserter.hpp>
#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/path.hpp>
#include <boost/optional.hpp>

#include <cmath>
#include <fstream>
#include <iostream>
#include <stdexcept>

using namespace std;
namespace fs = boost::filesystem;
using namespace gtsam::symbol_shorthand;

#define LINESIZE 81920

namespace gtsam {

/* ************************************************************************* */
string findExampleDataFile(const string& name) {
  // Search source tree and installed location
  vector<string> rootsToSearch;

  // Constants below are defined by CMake, see gtsam/gtsam/CMakeLists.txt
  rootsToSearch.push_back(GTSAM_SOURCE_TREE_DATASET_DIR);
  rootsToSearch.push_back(GTSAM_INSTALLED_DATASET_DIR);

  // Search for filename as given, and with .graph and .txt extensions
  vector<string> namesToSearch;
  namesToSearch.push_back(name);
  namesToSearch.push_back(name + ".graph");
  namesToSearch.push_back(name + ".txt");
  namesToSearch.push_back(name + ".out");
  namesToSearch.push_back(name + ".xml");

  // Find first name that exists
  for(const fs::path& root: rootsToSearch) {
    for(const fs::path& name: namesToSearch) {
      if (fs::is_regular_file(root / name))
        return (root / name).string();
    }
  }

  // If we did not return already, then we did not find the file
  throw invalid_argument(
      "gtsam::findExampleDataFile could not find a matching file in\n"
      GTSAM_SOURCE_TREE_DATASET_DIR " or\n"
      GTSAM_INSTALLED_DATASET_DIR " named\n" + name + ", " + name
          + ".graph, or " + name + ".txt");
}

/* ************************************************************************* */
string createRewrittenFileName(const string& name) {
  // Search source tree and installed location
  if (!exists(fs::path(name))) {
    throw invalid_argument(
        "gtsam::createRewrittenFileName could not find a matching file in\n"
            + name);
  }

  fs::path p(name);
  fs::path newpath = fs::path(p.parent_path().string())
      / fs::path(p.stem().string() + "-rewritten.txt");

  return newpath.string();
}

/* ************************************************************************* */
GraphAndValues load2D(pair<string, SharedNoiseModel> dataset, int maxID,
    bool addNoise, bool smart, NoiseFormat noiseFormat,
    KernelFunctionType kernelFunctionType) {
  return load2D(dataset.first, dataset.second, maxID, addNoise, smart,
      noiseFormat, kernelFunctionType);
}

/* ************************************************************************* */
// Read noise parameters and interpret them according to flags
static SharedNoiseModel readNoiseModel(ifstream& is, bool smart,
    NoiseFormat noiseFormat, KernelFunctionType kernelFunctionType) {
  double v1, v2, v3, v4, v5, v6;
  is >> v1 >> v2 >> v3 >> v4 >> v5 >> v6;

  if (noiseFormat == NoiseFormatAUTO) {
    // Try to guess covariance matrix layout
    if (v1 != 0.0 && v2 == 0.0 && v3 != 0.0 && v4 != 0.0 && v5 == 0.0
        && v6 == 0.0) {
      // NoiseFormatGRAPH
      noiseFormat = NoiseFormatGRAPH;
    } else if (v1 != 0.0 && v2 == 0.0 && v3 == 0.0 && v4 != 0.0 && v5 == 0.0
        && v6 != 0.0) {
      // NoiseFormatCOV
      noiseFormat = NoiseFormatCOV;
    } else {
      throw std::invalid_argument(
          "load2D: unrecognized covariance matrix format in dataset file. Please specify the noise format.");
    }
  }

  // Read matrix and check that diagonal entries are non-zero
  Matrix M(3, 3);
  switch (noiseFormat) {
  case NoiseFormatG2O:
  case NoiseFormatCOV:
    // i.e., [ v1 v2 v3; v2' v4 v5; v3' v5' v6 ]
    if (v1 == 0.0 || v4 == 0.0 || v6 == 0.0)
      throw runtime_error(
          "load2D::readNoiseModel looks like this is not G2O matrix order");
    M << v1, v2, v3, v2, v4, v5, v3, v5, v6;
    break;
  case NoiseFormatTORO:
  case NoiseFormatGRAPH:
    // http://www.openslam.org/toro.html
    // inf_ff inf_fs inf_ss inf_rr inf_fr inf_sr
    // i.e., [ v1 v2 v5; v2' v3 v6; v5' v6' v4 ]
    if (v1 == 0.0 || v3 == 0.0 || v4 == 0.0)
      throw invalid_argument(
          "load2D::readNoiseModel looks like this is not TORO matrix order");
    M << v1, v2, v5, v2, v3, v6, v5, v6, v4;
    break;
  default:
    throw runtime_error("load2D: invalid noise format");
  }

  // Now, create a Gaussian noise model
  // The smart flag will try to detect a simpler model, e.g., unit
  SharedNoiseModel model;
  switch (noiseFormat) {
  case NoiseFormatG2O:
  case NoiseFormatTORO:
    // In both cases, what is stored in file is the information matrix
    model = noiseModel::Gaussian::Information(M, smart);
    break;
  case NoiseFormatGRAPH:
  case NoiseFormatCOV:
    // These cases expect covariance matrix
    model = noiseModel::Gaussian::Covariance(M, smart);
    break;
  default:
    throw invalid_argument("load2D: invalid noise format");
  }

  switch (kernelFunctionType) {
  case KernelFunctionTypeNONE:
    return model;
    break;
  case KernelFunctionTypeHUBER:
    return noiseModel::Robust::Create(
        noiseModel::mEstimator::Huber::Create(1.345), model);
    break;
  case KernelFunctionTypeTUKEY:
    return noiseModel::Robust::Create(
        noiseModel::mEstimator::Tukey::Create(4.6851), model);
    break;
  default:
    throw invalid_argument("load2D: invalid kernel function type");
  }
}

/* ************************************************************************* */
boost::optional<IndexedPose> parseVertex(istream& is, const string& tag) {
  if ((tag == "VERTEX2") || (tag == "VERTEX_SE2") || (tag == "VERTEX")) {
    Key id;
    double x, y, yaw;
    is >> id >> x >> y >> yaw;
    return IndexedPose(id, Pose2(x, y, yaw));
  } else {
    return boost::none;
  }
}

/* ************************************************************************* */
boost::optional<IndexedEdge> parseEdge(istream& is, const string& tag) {
  if ((tag == "EDGE2") || (tag == "EDGE") || (tag == "EDGE_SE2")
      || (tag == "ODOMETRY")) {

    Key id1, id2;
    double x, y, yaw;
    is >> id1 >> id2 >> x >> y >> yaw;
    return IndexedEdge(pair<Key, Key>(id1, id2), Pose2(x, y, yaw));
  } else {
    return boost::none;
  }
}

/* ************************************************************************* */
GraphAndValues load2D(const string& filename, SharedNoiseModel model, Key maxID,
    bool addNoise, bool smart, NoiseFormat noiseFormat,
    KernelFunctionType kernelFunctionType) {

  ifstream is(filename.c_str());
  if (!is)
    throw invalid_argument("load2D: can not find file " + filename);

  Values::shared_ptr initial(new Values);
  NonlinearFactorGraph::shared_ptr graph(new NonlinearFactorGraph);

  string tag;

  // load the poses
  while (!is.eof()) {
    if (!(is >> tag))
      break;

    const auto indexed_pose = parseVertex(is, tag);
    if (indexed_pose) {
      Key id = indexed_pose->first;

      // optional filter
      if (maxID && id >= maxID)
        continue;

      initial->insert(id, indexed_pose->second);
    }
    is.ignore(LINESIZE, '\n');
  }
  is.clear(); /* clears the end-of-file and error flags */
  is.seekg(0, ios::beg);

  // If asked, create a sampler with random number generator
  std::unique_ptr<Sampler> sampler;
  if (addNoise) {
    noiseModel::Diagonal::shared_ptr noise;
    if (model)
      noise = boost::dynamic_pointer_cast<noiseModel::Diagonal>(model);
    if (!noise)
      throw invalid_argument(
          "gtsam::load2D: invalid noise model for adding noise"
              "(current version assumes diagonal noise model)!");
    sampler.reset(new Sampler(noise));
  }

  // Parse the pose constraints
  Key id1, id2;
  bool haveLandmark = false;
  const bool useModelInFile = !model;
  while (!is.eof()) {
    if (!(is >> tag))
      break;

    auto between_pose = parseEdge(is, tag);
    if (between_pose) {
      std::tie(id1, id2) = between_pose->first;
      Pose2& l1Xl2 = between_pose->second;

      // read noise model
      SharedNoiseModel modelInFile = readNoiseModel(is, smart, noiseFormat,
          kernelFunctionType);

      // optional filter
      if (maxID && (id1 >= maxID || id2 >= maxID))
        continue;

      if (useModelInFile)
        model = modelInFile;

      if (addNoise)
        l1Xl2 = l1Xl2.retract(sampler->sample());

      // Insert vertices if pure odometry file
      if (!initial->exists(id1))
        initial->insert(id1, Pose2());
      if (!initial->exists(id2))
        initial->insert(id2, initial->at<Pose2>(id1) * l1Xl2);

      NonlinearFactor::shared_ptr factor(
          new BetweenFactor<Pose2>(id1, id2, l1Xl2, model));
      graph->push_back(factor);
    }
    // Parse measurements
    double bearing, range, bearing_std, range_std;

    // A bearing-range measurement
    if (tag == "BR") {
      is >> id1 >> id2 >> bearing >> range >> bearing_std >> range_std;
    }

    // A landmark measurement, TODO Frank says: don't know why is converted to bearing-range
    if (tag == "LANDMARK") {
      double lmx, lmy;
      double v1, v2, v3;

      is >> id1 >> id2 >> lmx >> lmy >> v1 >> v2 >> v3;

      // Convert x,y to bearing,range
      bearing = atan2(lmy, lmx);
      range = sqrt(lmx * lmx + lmy * lmy);

      // In our experience, the x-y covariance on landmark sightings is not very good, so assume
      // it describes the uncertainty at a range of 10m, and convert that to bearing/range uncertainty.
      if (std::abs(v1 - v3) < 1e-4) {
        bearing_std = sqrt(v1 / 10.0);
        range_std = sqrt(v1);
      } else {
        bearing_std = 1;
        range_std = 1;
        if (!haveLandmark) {
          cout
              << "Warning: load2D is a very simple dataset loader and is ignoring the\n"
                  "non-uniform covariance on LANDMARK measurements in this file."
              << endl;
          haveLandmark = true;
        }
      }
    }

    // Do some common stuff for bearing-range measurements
    if (tag == "LANDMARK" || tag == "BR") {

      // optional filter
      if (maxID && id1 >= maxID)
        continue;

      // Create noise model
      noiseModel::Diagonal::shared_ptr measurementNoise =
          noiseModel::Diagonal::Sigmas((Vector(2) << bearing_std, range_std).finished());

      // Add to graph
      *graph += BearingRangeFactor<Pose2, Point2>(id1, L(id2), bearing, range,
          measurementNoise);

      // Insert poses or points if they do not exist yet
      if (!initial->exists(id1))
        initial->insert(id1, Pose2());
      if (!initial->exists(L(id2))) {
        Pose2 pose = initial->at<Pose2>(id1);
        Point2 local(cos(bearing) * range, sin(bearing) * range);
        Point2 global = pose.transformFrom(local);
        initial->insert(L(id2), global);
      }
    }
    is.ignore(LINESIZE, '\n');
  }

  return make_pair(graph, initial);
}

/* ************************************************************************* */
GraphAndValues load2D_robust(const string& filename,
    noiseModel::Base::shared_ptr& model, int maxID) {
  return load2D(filename, model, maxID);
}

/* ************************************************************************* */
void save2D(const NonlinearFactorGraph& graph, const Values& config,
    const noiseModel::Diagonal::shared_ptr model, const string& filename) {

  fstream stream(filename.c_str(), fstream::out);

  // save poses

  for(const Values::ConstKeyValuePair& key_value: config) {
    const Pose2& pose = key_value.value.cast<Pose2>();
    stream << "VERTEX2 " << key_value.key << " " << pose.x() << " " << pose.y()
        << " " << pose.theta() << endl;
  }

  // save edges
  Matrix R = model->R();
  Matrix RR = trans(R) * R; //prod(trans(R),R);
  for(boost::shared_ptr<NonlinearFactor> factor_: graph) {
    boost::shared_ptr<BetweenFactor<Pose2> > factor =
        boost::dynamic_pointer_cast<BetweenFactor<Pose2> >(factor_);
    if (!factor)
      continue;

    Pose2 pose = factor->measured().inverse();
    stream << "EDGE2 " << factor->key2() << " " << factor->key1() << " "
        << pose.x() << " " << pose.y() << " " << pose.theta() << " " << RR(0, 0)
        << " " << RR(0, 1) << " " << RR(1, 1) << " " << RR(2, 2) << " "
        << RR(0, 2) << " " << RR(1, 2) << endl;
  }

  stream.close();
}

/* ************************************************************************* */
GraphAndValues readG2o(const string& g2oFile, const bool is3D,
                       KernelFunctionType kernelFunctionType) {
  if (is3D) {
    return load3D(g2oFile);
  } else {
    // just call load2D
    int maxID = 0;
    bool addNoise = false;
    bool smart = true;
    return load2D(g2oFile, SharedNoiseModel(), maxID, addNoise, smart,
                  NoiseFormatG2O, kernelFunctionType);
  }
}

/* ************************************************************************* */
void writeG2o(const NonlinearFactorGraph& graph, const Values& estimate,
    const string& filename) {
  fstream stream(filename.c_str(), fstream::out);

  // save 2D & 3D poses
  for (const auto& key_value : estimate) {
    auto p = dynamic_cast<const GenericValue<Pose2>*>(&key_value.value);
    if (!p) continue;
    const Pose2& pose = p->value();
    stream << "VERTEX_SE2 " << key_value.key << " " << pose.x() << " "
        << pose.y() << " " << pose.theta() << endl;
  }

  for(const auto& key_value: estimate) {
      auto p = dynamic_cast<const GenericValue<Pose3>*>(&key_value.value);
      if (!p) continue;
      const Pose3& pose = p->value();
      const Point3 t = pose.translation();
      const auto q = pose.rotation().toQuaternion();
      stream << "VERTEX_SE3:QUAT " << key_value.key << " " << t.x() << " "
             << t.y() << " " << t.z() << " " << q.x() << " " << q.y() << " "
             << q.z() << " " << q.w() << endl;
  }

  // save edges (2D or 3D)
  for(const auto& factor_: graph) {
    boost::shared_ptr<BetweenFactor<Pose2> > factor =
        boost::dynamic_pointer_cast<BetweenFactor<Pose2> >(factor_);
    if (factor){
      SharedNoiseModel model = factor->noiseModel();
      boost::shared_ptr<noiseModel::Gaussian> gaussianModel =
          boost::dynamic_pointer_cast<noiseModel::Gaussian>(model);
      if (!gaussianModel){
        model->print("model\n");
        throw invalid_argument("writeG2o: invalid noise model!");
      }
      Matrix Info = gaussianModel->R().transpose() * gaussianModel->R();
      Pose2 pose = factor->measured(); //.inverse();
      stream << "EDGE_SE2 " << factor->key1() << " " << factor->key2() << " "
          << pose.x() << " " << pose.y() << " " << pose.theta();
      for (int i = 0; i < 3; i++){
        for (int j = i; j < 3; j++){
          stream << " " << Info(i, j);
        }
      }
      stream << endl;
    }

    boost::shared_ptr< BetweenFactor<Pose3> > factor3D =
        boost::dynamic_pointer_cast< BetweenFactor<Pose3> >(factor_);

    if (factor3D){
      SharedNoiseModel model = factor3D->noiseModel();

      boost::shared_ptr<noiseModel::Gaussian> gaussianModel =
          boost::dynamic_pointer_cast<noiseModel::Gaussian>(model);
      if (!gaussianModel){
        model->print("model\n");
        throw invalid_argument("writeG2o: invalid noise model!");
      }
      Matrix Info = gaussianModel->R().transpose() * gaussianModel->R();
      const Pose3 pose3D = factor3D->measured();
      const Point3 p = pose3D.translation();
      const auto q = pose3D.rotation().toQuaternion();
      stream << "EDGE_SE3:QUAT " << factor3D->key1() << " " << factor3D->key2()
             << " " << p.x() << " " << p.y() << " " << p.z() << " " << q.x()
             << " " << q.y() << " " << q.z() << " " << q.w();

      Matrix InfoG2o = I_6x6;
      InfoG2o.block(0,0,3,3) = Info.block(3,3,3,3); // cov translation
      InfoG2o.block(3,3,3,3) = Info.block(0,0,3,3); // cov rotation
      InfoG2o.block(0,3,3,3) = Info.block(0,3,3,3); // off diagonal
      InfoG2o.block(3,0,3,3) = Info.block(3,0,3,3); // off diagonal

      for (int i = 0; i < 6; i++){
        for (int j = i; j < 6; j++){
          stream << " " << InfoG2o(i, j);
        }
      }
      stream << endl;
    }
  }
  stream.close();
}

/* ************************************************************************* */
static Rot3 NormalizedRot3(double w, double x, double y, double z) {
  const double norm = sqrt(w * w + x * x + y * y + z * z), f = 1.0 / norm;
  return Rot3::Quaternion(f * w, f * x, f * y, f * z);
}
/* ************************************************************************* */
std::map<Key, Pose3> parse3DPoses(const string& filename) {
  ifstream is(filename.c_str());
  if (!is)
    throw invalid_argument("parse3DPoses: can not find file " + filename);

  std::map<Key, Pose3> poses;
  while (!is.eof()) {
    char buf[LINESIZE];
    is.getline(buf, LINESIZE);
    istringstream ls(buf);
    string tag;
    ls >> tag;

    if (tag == "VERTEX3") {
      Key id;
      double x, y, z, roll, pitch, yaw;
      ls >> id >> x >> y >> z >> roll >> pitch >> yaw;
      poses.emplace(id, Pose3(Rot3::Ypr(yaw, pitch, roll), {x, y, z}));
    }
    if (tag == "VERTEX_SE3:QUAT") {
      Key id;
      double x, y, z, qx, qy, qz, qw;
      ls >> id >> x >> y >> z >> qx >> qy >> qz >> qw;
      poses.emplace(id, Pose3(NormalizedRot3(qw, qx, qy, qz), {x, y, z}));
    }
  }
  return poses;
}

/* ************************************************************************* */
BetweenFactorPose3s parse3DFactors(
    const string& filename,
    const noiseModel::Diagonal::shared_ptr& corruptingNoise) {
  ifstream is(filename.c_str());
  if (!is) throw invalid_argument("parse3DFactors: can not find file " + filename);

  boost::optional<Sampler> sampler;
  if (corruptingNoise) {
    sampler = Sampler(corruptingNoise);
  }

  std::vector<BetweenFactor<Pose3>::shared_ptr> factors;
  while (!is.eof()) {
    char buf[LINESIZE];
    is.getline(buf, LINESIZE);
    istringstream ls(buf);
    string tag;
    ls >> tag;

    if (tag == "EDGE3") {
      Key id1, id2;
      double x, y, z, roll, pitch, yaw;
      ls >> id1 >> id2 >> x >> y >> z >> roll >> pitch >> yaw;
      Matrix m(6, 6);
      for (size_t i = 0; i < 6; i++)
        for (size_t j = i; j < 6; j++) ls >> m(i, j);
      SharedNoiseModel model = noiseModel::Gaussian::Information(m);
      factors.emplace_back(new BetweenFactor<Pose3>(
          id1, id2, Pose3(Rot3::Ypr(yaw, pitch, roll), {x, y, z}), model));
    }
    if (tag == "EDGE_SE3:QUAT") {
      Key id1, id2;
      double x, y, z, qx, qy, qz, qw;
      ls >> id1 >> id2 >> x >> y >> z >> qx >> qy >> qz >> qw;
      Matrix m(6, 6);
      for (size_t i = 0; i < 6; i++) {
        for (size_t j = i; j < 6; j++) {
          double mij;
          ls >> mij;
          m(i, j) = mij;
          m(j, i) = mij;
        }
      }
      Matrix mgtsam(6, 6);

      mgtsam.block<3, 3>(0, 0) = m.block<3, 3>(3, 3);  // cov rotation
      mgtsam.block<3, 3>(3, 3) = m.block<3, 3>(0, 0);  // cov translation
      mgtsam.block<3, 3>(0, 3) = m.block<3, 3>(0, 3);  // off diagonal
      mgtsam.block<3, 3>(3, 0) = m.block<3, 3>(3, 0);  // off diagonal

      SharedNoiseModel model = noiseModel::Gaussian::Information(mgtsam);
      auto R12 = NormalizedRot3(qw, qx, qy, qz);
      if (sampler) {
        R12 = R12.retract(sampler->sample());
      }

      factors.emplace_back(new BetweenFactor<Pose3>(
          id1, id2, Pose3(R12, {x, y, z}), model));
    }
  }
  return factors;
}

/* ************************************************************************* */
GraphAndValues load3D(const string& filename) {
  const auto factors = parse3DFactors(filename);
  NonlinearFactorGraph::shared_ptr graph(new NonlinearFactorGraph);
  for (const auto& factor : factors) {
    graph->push_back(factor);
  }

  const auto poses = parse3DPoses(filename);
  Values::shared_ptr initial(new Values);
  for (const auto& key_pose : poses) {
    initial->insert(key_pose.first, key_pose.second);
  }

  return make_pair(graph, initial);
}

/* ************************************************************************* */
Rot3 openGLFixedRotation() { // this is due to different convention for cameras in gtsam and openGL
  /* R = [ 1   0   0
   *       0  -1   0
   *       0   0  -1]
   */
  Matrix3 R_mat = Matrix3::Zero(3, 3);
  R_mat(0, 0) = 1.0;
  R_mat(1, 1) = -1.0;
  R_mat(2, 2) = -1.0;
  return Rot3(R_mat);
}

/* ************************************************************************* */
Pose3 openGL2gtsam(const Rot3& R, double tx, double ty, double tz) {
  Rot3 R90 = openGLFixedRotation();
  Rot3 wRc = (R.inverse()).compose(R90);

  // Our camera-to-world translation wTc = -R'*t
  return Pose3(wRc, R.unrotate(Point3(-tx, -ty, -tz)));
}

/* ************************************************************************* */
Pose3 gtsam2openGL(const Rot3& R, double tx, double ty, double tz) {
  Rot3 R90 = openGLFixedRotation();
  Rot3 cRw_openGL = R90.compose(R.inverse());
  Point3 t_openGL = cRw_openGL.rotate(Point3(-tx, -ty, -tz));
  return Pose3(cRw_openGL, t_openGL);
}

/* ************************************************************************* */
Pose3 gtsam2openGL(const Pose3& PoseGTSAM) {
  return gtsam2openGL(PoseGTSAM.rotation(), PoseGTSAM.x(), PoseGTSAM.y(),
      PoseGTSAM.z());
}

/* ************************************************************************* */
bool readBundler(const string& filename, SfmData &data) {
  // Load the data file
  ifstream is(filename.c_str(), ifstream::in);
  if (!is) {
    cout << "Error in readBundler: can not find the file!!" << endl;
    return false;
  }

  // Ignore the first line
  char aux[500];
  is.getline(aux, 500);

  // Get the number of camera poses and 3D points
  size_t nrPoses, nrPoints;
  is >> nrPoses >> nrPoints;

  // Get the information for the camera poses
  for (size_t i = 0; i < nrPoses; i++) {
    // Get the focal length and the radial distortion parameters
    float f, k1, k2;
    is >> f >> k1 >> k2;
    Cal3Bundler K(f, k1, k2);

    // Get the rotation matrix
    float r11, r12, r13;
    float r21, r22, r23;
    float r31, r32, r33;
    is >> r11 >> r12 >> r13 >> r21 >> r22 >> r23 >> r31 >> r32 >> r33;

    // Bundler-OpenGL rotation matrix
    Rot3 R(r11, r12, r13, r21, r22, r23, r31, r32, r33);

    // Check for all-zero R, in which case quit
    if (r11 == 0 && r12 == 0 && r13 == 0) {
      cout << "Error in readBundler: zero rotation matrix for pose " << i
          << endl;
      return false;
    }

    // Get the translation vector
    float tx, ty, tz;
    is >> tx >> ty >> tz;

    Pose3 pose = openGL2gtsam(R, tx, ty, tz);

    data.cameras.emplace_back(pose, K);
  }

  // Get the information for the 3D points
  data.tracks.reserve(nrPoints);
  for (size_t j = 0; j < nrPoints; j++) {
    SfmTrack track;

    // Get the 3D position
    float x, y, z;
    is >> x >> y >> z;
    track.p = Point3(x, y, z);

    // Get the color information
    float r, g, b;
    is >> r >> g >> b;
    track.r = r / 255.f;
    track.g = g / 255.f;
    track.b = b / 255.f;

    // Now get the visibility information
    size_t nvisible = 0;
    is >> nvisible;

    track.measurements.reserve(nvisible);
    track.siftIndices.reserve(nvisible);
    for (size_t k = 0; k < nvisible; k++) {
      size_t cam_idx = 0, point_idx = 0;
      float u, v;
      is >> cam_idx >> point_idx >> u >> v;
      track.measurements.emplace_back(cam_idx, Point2(u, -v));
      track.siftIndices.emplace_back(cam_idx, point_idx);
    }

    data.tracks.push_back(track);
  }

  is.close();
  return true;
}

/* ************************************************************************* */
bool readBAL(const string& filename, SfmData &data) {
  // Load the data file
  ifstream is(filename.c_str(), ifstream::in);
  if (!is) {
    cout << "Error in readBAL: can not find the file!!" << endl;
    return false;
  }

  // Get the number of camera poses and 3D points
  size_t nrPoses, nrPoints, nrObservations;
  is >> nrPoses >> nrPoints >> nrObservations;

  data.tracks.resize(nrPoints);

  // Get the information for the observations
  for (size_t k = 0; k < nrObservations; k++) {
    size_t i = 0, j = 0;
    float u, v;
    is >> i >> j >> u >> v;
    data.tracks[j].measurements.emplace_back(i, Point2(u, -v));
  }

  // Get the information for the camera poses
  for (size_t i = 0; i < nrPoses; i++) {
    // Get the Rodrigues vector
    float wx, wy, wz;
    is >> wx >> wy >> wz;
    Rot3 R = Rot3::Rodrigues(wx, wy, wz); // BAL-OpenGL rotation matrix

    // Get the translation vector
    float tx, ty, tz;
    is >> tx >> ty >> tz;

    Pose3 pose = openGL2gtsam(R, tx, ty, tz);

    // Get the focal length and the radial distortion parameters
    float f, k1, k2;
    is >> f >> k1 >> k2;
    Cal3Bundler K(f, k1, k2);

    data.cameras.emplace_back(pose, K);
  }

  // Get the information for the 3D points
  for (size_t j = 0; j < nrPoints; j++) {
    // Get the 3D position
    float x, y, z;
    is >> x >> y >> z;
    SfmTrack& track = data.tracks[j];
    track.p = Point3(x, y, z);
    track.r = 0.4f;
    track.g = 0.4f;
    track.b = 0.4f;
  }

  is.close();
  return true;
}

/* ************************************************************************* */
bool writeBAL(const string& filename, SfmData &data) {
  // Open the output file
  ofstream os;
  os.open(filename.c_str());
  os.precision(20);
  if (!os.is_open()) {
    cout << "Error in writeBAL: can not open the file!!" << endl;
    return false;
  }

  // Write the number of camera poses and 3D points
  size_t nrObservations = 0;
  for (size_t j = 0; j < data.number_tracks(); j++) {
    nrObservations += data.tracks[j].number_measurements();
  }

  // Write observations
  os << data.number_cameras() << " " << data.number_tracks() << " "
      << nrObservations << endl;
  os << endl;

  for (size_t j = 0; j < data.number_tracks(); j++) { // for each 3D point j
    const SfmTrack& track = data.tracks[j];

    for (size_t k = 0; k < track.number_measurements(); k++) { // for each observation of the 3D point j
      size_t i = track.measurements[k].first; // camera id
      double u0 = data.cameras[i].calibration().u0();
      double v0 = data.cameras[i].calibration().v0();

      if (u0 != 0 || v0 != 0) {
        cout
            << "writeBAL has not been tested for calibration with nonzero (u0,v0)"
            << endl;
      }

      double pixelBALx = track.measurements[k].second.x() - u0; // center of image is the origin
      double pixelBALy = -(track.measurements[k].second.y() - v0); // center of image is the origin
      Point2 pixelMeasurement(pixelBALx, pixelBALy);
      os << i /*camera id*/<< " " << j /*point id*/<< " "
          << pixelMeasurement.x() /*u of the pixel*/<< " "
          << pixelMeasurement.y() /*v of the pixel*/<< endl;
    }
  }
  os << endl;

  // Write cameras
  for (size_t i = 0; i < data.number_cameras(); i++) { // for each camera
    Pose3 poseGTSAM = data.cameras[i].pose();
    Cal3Bundler cameraCalibration = data.cameras[i].calibration();
    Pose3 poseOpenGL = gtsam2openGL(poseGTSAM);
    os << Rot3::Logmap(poseOpenGL.rotation()) << endl;
    os << poseOpenGL.translation().x() << endl;
    os << poseOpenGL.translation().y() << endl;
    os << poseOpenGL.translation().z() << endl;
    os << cameraCalibration.fx() << endl;
    os << cameraCalibration.k1() << endl;
    os << cameraCalibration.k2() << endl;
    os << endl;
  }

  // Write the points
  for (size_t j = 0; j < data.number_tracks(); j++) { // for each 3D point j
    Point3 point = data.tracks[j].p;
    os << point.x() << endl;
    os << point.y() << endl;
    os << point.z() << endl;
    os << endl;
  }

  os.close();
  return true;
}

bool writeBALfromValues(const string& filename, const SfmData &data,
    Values& values) {
  using Camera = PinholeCamera<Cal3Bundler>;
  SfmData dataValues = data;

  // Store poses or cameras in SfmData
  size_t nrPoses = values.count<Pose3>();
  if (nrPoses == dataValues.number_cameras()) { // we only estimated camera poses
    for (size_t i = 0; i < dataValues.number_cameras(); i++) { // for each camera
      Key poseKey = symbol('x', i);
      Pose3 pose = values.at<Pose3>(poseKey);
      Cal3Bundler K = dataValues.cameras[i].calibration();
      Camera camera(pose, K);
      dataValues.cameras[i] = camera;
    }
  } else {
    size_t nrCameras = values.count<Camera>();
    if (nrCameras == dataValues.number_cameras()) { // we only estimated camera poses and calibration
      for (size_t i = 0; i < nrCameras; i++) { // for each camera
        Key cameraKey = i; // symbol('c',i);
        Camera camera = values.at<Camera>(cameraKey);
        dataValues.cameras[i] = camera;
      }
    } else {
      cout
          << "writeBALfromValues: different number of cameras in SfM_dataValues (#cameras "
          << dataValues.number_cameras() << ") and values (#cameras "
          << nrPoses << ", #poses " << nrCameras << ")!!"
          << endl;
      return false;
    }
  }

  // Store 3D points in SfmData
  size_t nrPoints = values.count<Point3>(), nrTracks = dataValues.number_tracks();
  if (nrPoints != nrTracks) {
    cout
        << "writeBALfromValues: different number of points in SfM_dataValues (#points= "
        << nrTracks << ") and values (#points "
        << nrPoints << ")!!" << endl;
  }

  for (size_t j = 0; j < nrTracks; j++) { // for each point
    Key pointKey = P(j);
    if (values.exists(pointKey)) {
      Point3 point = values.at<Point3>(pointKey);
      dataValues.tracks[j].p = point;
    } else {
      dataValues.tracks[j].r = 1.0;
      dataValues.tracks[j].g = 0.0;
      dataValues.tracks[j].b = 0.0;
      dataValues.tracks[j].p = Point3(0,0,0);
    }
  }

  // Write SfmData to file
  return writeBAL(filename, dataValues);
}

Values initialCamerasEstimate(const SfmData& db) {
  Values initial;
  size_t i = 0; // NO POINTS:  j = 0;
  for(const SfmCamera& camera: db.cameras)
    initial.insert(i++, camera);
  return initial;
}

Values initialCamerasAndPointsEstimate(const SfmData& db) {
  Values initial;
  size_t i = 0, j = 0;
  for(const SfmCamera& camera: db.cameras)
    initial.insert((i++), camera);
  for(const SfmTrack& track: db.tracks)
    initial.insert(P(j++), track.p);
  return initial;
}

} // \namespace gtsam
