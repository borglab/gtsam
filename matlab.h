/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    matlab.h
 * @brief   Contains *generic* global functions designed particularly for the matlab interface
 * @author  Stephen Williams
 */

#pragma once

#include <gtsam/slam/ProjectionFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/SimpleCamera.h>

#include <exception>

namespace gtsam {

  namespace utilities {

    /// Extract all Point2 values into a single matrix [x y]
    Matrix extractPoint2(const Values& values) {
      size_t j=0;
      Values::ConstFiltered<Point2> points = values.filter<Point2>();
      Matrix result(points.size(),2);
      BOOST_FOREACH(const Values::ConstFiltered<Point2>::KeyValuePair& key_value, points)
        result.row(j++) = key_value.value.vector();
      return result;
    }

    /// Extract all Point3 values into a single matrix [x y z]
    Matrix extractPoint3(const Values& values) {
      size_t j=0;
      Values::ConstFiltered<Point3> points = values.filter<Point3>();
      Matrix result(points.size(),3);
      BOOST_FOREACH(const Values::ConstFiltered<Point3>::KeyValuePair& key_value, points)
        result.row(j++) = key_value.value.vector();
      return result;
    }

    /// Extract all Pose2 values into a single matrix [x y theta]
    Matrix extractPose2(const Values& values) {
      size_t j=0;
      Values::ConstFiltered<Pose2> poses = values.filter<Pose2>();
      Matrix result(poses.size(),3);
      BOOST_FOREACH(const Values::ConstFiltered<Pose2>::KeyValuePair& key_value, poses)
        result.row(j++) << key_value.value.x(), key_value.value.y(), key_value.value.theta();
      return result;
    }

    /// Extract all Pose3 values into a single matrix [r11 r12 r13 r21 r22 r23 r31 r32 r33 x y z]
    Matrix extractPose3(const Values& values) {
      size_t j=0;
      Values::ConstFiltered<Pose3> poses = values.filter<Pose3>();
      Matrix result(poses.size(),12);
      BOOST_FOREACH(const Values::ConstFiltered<Pose3>::KeyValuePair& key_value, poses) {
        result.row(j).segment(0, 3) << key_value.value.rotation().matrix().row(0);
        result.row(j).segment(3, 3) << key_value.value.rotation().matrix().row(1);
        result.row(j).segment(6, 3) << key_value.value.rotation().matrix().row(2);
        result.row(j).tail(3) = key_value.value.translation().vector();
        j++;
      }
      return result;
    }


    /// perturb all Point2 using normally distributed noise
    void perturbPoint2(Values& values, double sigma, int32_t seed = 42u) {
      noiseModel::Isotropic::shared_ptr model = noiseModel::Isotropic::Sigma(2,sigma);
      Sampler sampler(model, seed);
      BOOST_FOREACH(const Values::ConstFiltered<Point2>::KeyValuePair& key_value, values.filter<Point2>()) {
        values.update(key_value.key, key_value.value.retract(sampler.sample()));
      }
    }

    /// perturb all Point3 using normally distributed noise
    void perturbPoint3(Values& values, double sigma, int32_t seed = 42u) {
      noiseModel::Isotropic::shared_ptr model = noiseModel::Isotropic::Sigma(3,sigma);
      Sampler sampler(model, seed);
      BOOST_FOREACH(const Values::ConstFiltered<Point3>::KeyValuePair& key_value, values.filter<Point3>()) {
        values.update(key_value.key, key_value.value.retract(sampler.sample()));
      }
    }

    /// insert a number of initial point values by backprojecting
    void insertBackprojections(Values& values, const SimpleCamera& camera, const Vector& J, const Matrix& Z, double depth) {
      if (Z.rows() != 2) throw std::invalid_argument("insertBackProjections: Z must be 2*K");
      if (Z.cols() != J.size()) throw std::invalid_argument("insertBackProjections: J and Z must have same number of entries");
      for(int k=0;k<Z.cols();k++) {
        Point2 p(Z(0,k),Z(1,k));
        Point3 P = camera.backproject(p, depth);
        values.insert(J(k), P);
      }
    }

    /// insert multiple projection factors for a single pose key
    void insertProjectionFactors(NonlinearFactorGraph& graph, Key i, const Vector& J, const Matrix& Z,
        const SharedNoiseModel& model, const shared_ptrK K) {
      if (Z.rows() != 2) throw std::invalid_argument("addMeasurements: Z must be 2*K");
      if (Z.cols() != J.size()) throw std::invalid_argument(
            "addMeasurements: J and Z must have same number of entries");
      for (int k = 0; k < Z.cols(); k++) {
        graph.push_back(
            boost::make_shared<GenericProjectionFactor<Pose3, Point3> >
            (Point2(Z(0, k), Z(1, k)), model, i, Key(J(k)), K));
      }
    }

    /// calculate the errors of all projection factors in a graph
    Matrix reprojectionErrors(const NonlinearFactorGraph& graph, const Values& values) {
      // first count
      size_t K = 0, k=0;
      BOOST_FOREACH(const NonlinearFactor::shared_ptr& f, graph)
        if (boost::dynamic_pointer_cast<const GenericProjectionFactor<Pose3, Point3> >(f)) ++K;
      // now fill
      Matrix errors(2,K);
      BOOST_FOREACH(const NonlinearFactor::shared_ptr& f, graph) {
        boost::shared_ptr<const GenericProjectionFactor<Pose3, Point3> > p = boost::dynamic_pointer_cast<const GenericProjectionFactor<Pose3, Point3> >(f);
        if (p) errors.col(k++) = p->unwhitenedError(values);
      }
      return errors;
    }

  }

}

