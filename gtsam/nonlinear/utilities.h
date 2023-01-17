/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    utilities.h
 * @brief   Contains *generic* global functions designed particularly for the matlab interface
 * @author  Stephen Williams
 */

#pragma once

#include <gtsam/inference/Symbol.h>
#include <gtsam/slam/ProjectionFactor.h>
#include <gtsam/linear/Sampler.h>
#include <gtsam/linear/VectorValues.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Cal3_S2.h>
#include <gtsam/geometry/PinholeCamera.h>

#include <exception>

namespace gtsam {

namespace utilities {

// Create a KeyList from indices
FastList<Key> createKeyList(const Vector& I) {
  FastList<Key> set;
  for (int i = 0; i < I.size(); i++)
    set.push_back(I[i]);
  return set;
}

// Create a KeyList from indices using symbol
FastList<Key> createKeyList(std::string s, const Vector& I) {
  FastList<Key> set;
  char c = s[0];
  for (int i = 0; i < I.size(); i++)
    set.push_back(Symbol(c, I[i]));
  return set;
}

// Create a KeyVector from indices
KeyVector createKeyVector(const Vector& I) {
  KeyVector set;
  for (int i = 0; i < I.size(); i++)
    set.push_back(I[i]);
  return set;
}

// Create a KeyVector from indices using symbol
KeyVector createKeyVector(std::string s, const Vector& I) {
  KeyVector set;
  char c = s[0];
  for (int i = 0; i < I.size(); i++)
    set.push_back(Symbol(c, I[i]));
  return set;
}

// Create a KeySet from indices
KeySet createKeySet(const Vector& I) {
  KeySet set;
  for (int i = 0; i < I.size(); i++)
    set.insert(I[i]);
  return set;
}

// Create a KeySet from indices using symbol
KeySet createKeySet(std::string s, const Vector& I) {
  KeySet set;
  char c = s[0];
  for (int i = 0; i < I.size(); i++)
    set.insert(symbol(c, I[i]));
  return set;
}

/// Extract all Point2 values into a single matrix [x y]
Matrix extractPoint2(const Values& values) {
  const auto points = values.extract<gtsam::Point2>();
  // Point2 is aliased as a gtsam::Vector in the wrapper
  const auto points2 = values.extract<gtsam::Vector>();

  Matrix result(points.size() + points2.size(), 2);

  size_t j = 0;
  for (const auto& key_value : points) {
    result.row(j++) = key_value.second;
  }
  for (const auto& key_value : points2) {
    if (key_value.second.rows() == 2) {
      result.row(j++) = key_value.second;
    }
  }
  return result;
}

/// Extract all Point3 values into a single matrix [x y z]
Matrix extractPoint3(const Values& values) {
  const auto points = values.extract<gtsam::Point3>();
  // Point3 is aliased as a gtsam::Vector in the wrapper
  const auto points2 = values.extract<gtsam::Vector>();

  Matrix result(points.size() + points2.size(), 3);

  size_t j = 0;
  for (const auto& key_value : points) {
    result.row(j++) = key_value.second;
  }
  for (const auto& key_value : points2) {
    if (key_value.second.rows() == 3) {
      result.row(j++) = key_value.second;
    }
  }
  return result;
}

/// Extract all Pose3 values
Values allPose2s(const Values& values) {
  Values result;
  for(const auto& key_value: values.extract<Pose2>())
    result.insert(key_value.first, key_value.second);
  return result;
}

/// Extract all Pose2 values into a single matrix [x y theta]
Matrix extractPose2(const Values& values) {
  const auto poses = values.extract<Pose2>();
  Matrix result(poses.size(), 3);
  size_t j = 0;
  for(const auto& key_value: poses)
    result.row(j++) << key_value.second.x(), key_value.second.y(), key_value.second.theta();
  return result;
}

/// Extract all Pose3 values
Values allPose3s(const Values& values) {
  Values result;
  for(const auto& key_value: values.extract<Pose3>())
    result.insert(key_value.first, key_value.second);
  return result;
}

/// Extract all Pose3 values into a single matrix [r11 r12 r13 r21 r22 r23 r31 r32 r33 x y z]
Matrix extractPose3(const Values& values) {
  const auto poses = values.extract<Pose3>();
  Matrix result(poses.size(), 12);
  size_t j = 0;
  for(const auto& key_value: poses) {
    result.row(j).segment(0, 3) << key_value.second.rotation().matrix().row(0);
    result.row(j).segment(3, 3) << key_value.second.rotation().matrix().row(1);
    result.row(j).segment(6, 3) << key_value.second.rotation().matrix().row(2);
    result.row(j).tail(3) = key_value.second.translation();
    j++;
  }
  return result;
}

/// Extract all Vector values with a given symbol character into an mxn matrix,
/// where m is the number of symbols that match the character and n is the
/// dimension of the variables.  If not all variables have dimension n, then a
/// runtime error will be thrown.  The order of returned values are sorted by
/// the symbol.
/// For example, calling extractVector(values, 'x'), where values contains 200
/// variables x1, x2, ..., x200 of type Vector each 5-dimensional, will return a
/// 200x5 matrix with row i containing xi.
Matrix extractVectors(const Values& values, char c) {
  const auto vectors = values.extract<Vector>(Symbol::ChrTest(c));
  if (vectors.size() == 0) {
    return Matrix();
  }
  auto dim = vectors.begin()->second.size();
  Matrix result(vectors.size(), dim);
  Eigen::Index rowi = 0;
  for (const auto& kv : vectors) {
    if (kv.second.size() != dim) {
      throw std::runtime_error(
          "Tried to extract different-sized vectors into a single matrix");
    }
    result.row(rowi) = kv.second;
    ++rowi;
  }
  return result;
}

/// Perturb all Point2 values using normally distributed noise
void perturbPoint2(Values& values, double sigma, int32_t seed = 42u) {
  noiseModel::Isotropic::shared_ptr model =
      noiseModel::Isotropic::Sigma(2, sigma);
  Sampler sampler(model, seed);
  for (const auto& key_value : values.extract<Point2>()) {
    values.update<Point2>(key_value.first,
                          key_value.second + Point2(sampler.sample()));
  }
  for (const auto& key_value : values.extract<gtsam::Vector>()) {
    if (key_value.second.rows() == 2) {
      values.update<gtsam::Vector>(key_value.first,
                                   key_value.second + Point2(sampler.sample()));
    }
  }
}

/// Perturb all Pose2 values using normally distributed noise
void perturbPose2(Values& values, double sigmaT, double sigmaR, int32_t seed =
    42u) {
  noiseModel::Diagonal::shared_ptr model = noiseModel::Diagonal::Sigmas(
      Vector3(sigmaT, sigmaT, sigmaR));
  Sampler sampler(model, seed);
  for(const auto& key_value: values.extract<Pose2>()) {
    values.update<Pose2>(key_value.first, key_value.second.retract(sampler.sample()));
  }
}

/// Perturb all Point3 values using normally distributed noise
void perturbPoint3(Values& values, double sigma, int32_t seed = 42u) {
  noiseModel::Isotropic::shared_ptr model =
      noiseModel::Isotropic::Sigma(3, sigma);
  Sampler sampler(model, seed);
  for (const auto& key_value : values.extract<Point3>()) {
    values.update<Point3>(key_value.first,
                          key_value.second + Point3(sampler.sample()));
  }
  for (const auto& key_value : values.extract<gtsam::Vector>()) {
    if (key_value.second.rows() == 3) {
      values.update<gtsam::Vector>(key_value.first,
                                   key_value.second + Point3(sampler.sample()));
    }
  }
}

/**
 * @brief Insert a number of initial point values by backprojecting
 * 
 * @param values The values dict to insert the backprojections to.
 * @param camera The camera model.
 * @param J Vector of key indices.
 * @param Z 2*J matrix of pixel values.
 * @param depth Initial depth value.
 */
void insertBackprojections(Values& values, const PinholeCamera<Cal3_S2>& camera,
    const Vector& J, const Matrix& Z, double depth) {
  if (Z.rows() != 2)
    throw std::invalid_argument("insertBackProjections: Z must be 2*J");
  if (Z.cols() != J.size())
    throw std::invalid_argument(
        "insertBackProjections: J and Z must have same number of entries");
  for (int k = 0; k < Z.cols(); k++) {
    Point2 p(Z(0, k), Z(1, k));
    Point3 P = camera.backproject(p, depth);
    values.insert(J(k), P);
  }
}

/**
 * @brief Insert multiple projection factors for a single pose key
 * 
 * @param graph The nonlinear factor graph to add the factors to.
 * @param i Camera key.
 * @param J Vector of key indices.
 * @param Z 2*J matrix of pixel values.
 * @param model Factor noise model.
 * @param K Calibration matrix.
 * @param body_P_sensor Pose of the camera sensor in the body frame.
 */
void insertProjectionFactors(NonlinearFactorGraph& graph, Key i,
    const Vector& J, const Matrix& Z, const SharedNoiseModel& model,
    const Cal3_S2::shared_ptr K, const Pose3& body_P_sensor = Pose3()) {
  if (Z.rows() != 2)
    throw std::invalid_argument("addMeasurements: Z must be 2*K");
  if (Z.cols() != J.size())
    throw std::invalid_argument(
        "addMeasurements: J and Z must have same number of entries");
  for (int k = 0; k < Z.cols(); k++) {
    graph.push_back(
        std::make_shared<GenericProjectionFactor<Pose3, Point3> >(
            Point2(Z(0, k), Z(1, k)), model, i, Key(J(k)), K, body_P_sensor));
  }
}

/// Calculate the errors of all projection factors in a graph
Matrix reprojectionErrors(const NonlinearFactorGraph& graph,
    const Values& values) {
  // first count
  size_t K = 0, k = 0;
  for(const NonlinearFactor::shared_ptr& f: graph)
    if (boost::dynamic_pointer_cast<const GenericProjectionFactor<Pose3, Point3> >(
        f))
      ++K;
  // now fill
  Matrix errors(2, K);
  for(const NonlinearFactor::shared_ptr& f: graph) {
    std::shared_ptr<const GenericProjectionFactor<Pose3, Point3> > p =
        boost::dynamic_pointer_cast<const GenericProjectionFactor<Pose3, Point3> >(
            f);
    if (p)
      errors.col(k++) = p->unwhitenedError(values);
  }
  return errors;
}

/// Convert from local to world coordinates
Values localToWorld(const Values& local, const Pose2& base,
    const KeyVector user_keys = KeyVector()) {

  Values world;

  // if no keys given, get all keys from local values
  KeyVector keys(user_keys);
  if (keys.size()==0)
    keys = local.keys();

  // Loop over all keys
  for(Key key: keys) {
    try {
      // if value is a Pose2, compose it with base pose
      Pose2 pose = local.at<Pose2>(key);
      world.insert(key, base.compose(pose));
    } catch (const std::exception& e1) {
      try {
        // if value is a Point2, transform it from base pose
        Point2 point = local.at<Point2>(key);
        world.insert(key, base.transformFrom(point));
      } catch (const std::exception& e2) {
        // if not Pose2 or Point2, do nothing
        #ifndef NDEBUG
          std::cerr << "Values[key] is neither Pose2 nor Point2, so skip" << std::endl;
        #endif
      }
    }
  }
  return world;
}

} // namespace utilities

}

