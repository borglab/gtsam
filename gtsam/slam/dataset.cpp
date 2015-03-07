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
 * @author nikai
 * @brief utility functions for loading datasets
 */

#include <fstream>
#include <sstream>
#include <cstdlib>

#include <boost/filesystem.hpp>

#include <gtsam/geometry/Pose2.h>
#include <gtsam/linear/Sampler.h>
#include <gtsam/nonlinear/Symbol.h>
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
      graph->add(BearingRangeFactor<Pose2, Point2>(id1, id2, bearing, range, measurementNoise));

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
      graph->add(BearingRangeFactor<Pose2, Point2>(id1, L(id2), bearing, range, measurementNoise));
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
      graph->add(BearingRangeFactor<Pose2, Point2>(id1, id2, bearing, range, measurementNoise));

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

} // \namespace gtsam
