/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file visualSLAM.cpp
 * @date Jan 14, 2010
 * @author richard
 */

#include <gtsam/slam/visualSLAM.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/linear/Sampler.h>
#include <boost/make_shared.hpp>

namespace visualSLAM {

  using boost::make_shared;

  /* ************************************************************************* */
  void Values::insertBackprojections(const SimpleCamera& camera,
      const Vector& J, const Matrix& Z, double depth) {
    if (Z.rows() != 2) throw std::invalid_argument("insertBackProjections: Z must be 2*K");
    if (Z.cols() != J.size()) throw std::invalid_argument(
          "insertBackProjections: J and Z must have same number of entries");
    for(int k=0;k<Z.cols();k++) {
      Point2 p(Z(0,k),Z(1,k));
      Point3 P = camera.backproject(p, depth);
      insertPoint(J(k), P);
    }
  }

  /* ************************************************************************* */
  void Values::perturbPoints(double sigma, int32_t seed) {
    ConstFiltered<Point3> points = allPoints();
    noiseModel::Isotropic::shared_ptr model = noiseModel::Isotropic::Sigma(3,sigma);
    Sampler sampler(model, seed);
    BOOST_FOREACH(const ConstFiltered<Point3>::KeyValuePair& keyValue, points) {
      update(keyValue.key, keyValue.value.retract(sampler.sample()));
    }
  }

  /* ************************************************************************* */
  Matrix Values::points() const {
    size_t j=0;
    ConstFiltered<Point3> points = allPoints();
    Matrix result(points.size(),3);
    BOOST_FOREACH(const ConstFiltered<Point3>::KeyValuePair& keyValue, points)
      result.row(j++) = keyValue.value.vector();
    return result;
  }

  /* ************************************************************************* */
  void Graph::addPointConstraint(Key pointKey, const Point3& p) {
  	push_back(make_shared<NonlinearEquality<Point3> >(pointKey, p));
  }

  /* ************************************************************************* */
  void Graph::addPointPrior(Key pointKey, const Point3& p, const SharedNoiseModel& model) {
  	push_back(make_shared<PriorFactor<Point3> >(pointKey, p, model));
  }

  /* ************************************************************************* */
  void Graph::addMeasurement(const Point2& measured, const SharedNoiseModel& model,
       Key poseKey, Key pointKey, const shared_ptrK K) {
    push_back(
        make_shared<GenericProjectionFactor<Pose3, Point3> >
          (measured, model, poseKey, pointKey, K));
  }

  /* ************************************************************************* */
  void Graph::addMeasurements(Key i, const Vector& J, const Matrix& Z,
      const SharedNoiseModel& model, const shared_ptrK K) {
    if (Z.rows() != 2) throw std::invalid_argument("addMeasurements: Z must be 2*K");
    if (Z.cols() != J.size()) throw std::invalid_argument(
          "addMeasurements: J and Z must have same number of entries");
    for (int k = 0; k < Z.cols(); k++)
      addMeasurement(Point2(Z(0, k), Z(1, k)), model, i, J(k), K);
  }

  /* ************************************************************************* */
  void Graph::addStereoMeasurement(const StereoPoint2& measured, const SharedNoiseModel& model,
       Key poseKey, Key pointKey, const shared_ptrKStereo K) {
  	push_back(make_shared<GenericStereoFactor<Pose3, Point3> >(measured, model, poseKey, pointKey, K));
  }

  /* ************************************************************************* */
  void Graph::addRangeFactor(Key poseKey, Key pointKey, double range, const SharedNoiseModel& model) {
    push_back(make_shared<gtsam::RangeFactor<Pose3, Point3> >(poseKey, pointKey, range, model));
  }

  /* ************************************************************************* */
  Matrix Graph::reprojectionErrors(const Values& values) const {
    // first count
    size_t K = 0, k=0;
    BOOST_FOREACH(const sharedFactor& f, *this)
      if (boost::dynamic_pointer_cast<const ProjectionFactor>(f)) ++K;
    // now fill
    Matrix errors(2,K);
    BOOST_FOREACH(const sharedFactor& f, *this) {
      boost::shared_ptr<const ProjectionFactor> p =
          boost::dynamic_pointer_cast<const ProjectionFactor>(f);
      if (p) errors.col(k++) = p->unwhitenedError(values);
    }
    return errors;
  }
  /* ************************************************************************* */
}
