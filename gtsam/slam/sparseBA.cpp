/**
 * @file   sparseBA.cpp
 * @brief  
 * @date   Jul 6, 2012
 * @author Yong-Dian Jian
 */

#include <gtsam/slam/sparseBA.h>

namespace sparseBA {

/* ************************************************************************* */
void Graph::addSimpleCameraConstraint(Key cameraKey, const SimpleCamera &camera) {
  addCameraConstraint<SimpleCamera>(cameraKey, camera);
}

/* ************************************************************************* */
void Graph::addSimpleCameraPrior(Key cameraKey, const SimpleCamera &camera, SharedNoiseModel &model) {
  addCameraPrior<SimpleCamera>(cameraKey, camera, model);
}

/* ************************************************************************* */
void Graph::addSimpleCameraMeasurement(const Point2 &z, SharedNoiseModel& model, Index cameraKey, Index pointKey) {
  addMeasurement<SimpleCamera>(z, model, cameraKey, pointKey);
}

/* ************************************************************************* */
Matrix Graph::reprojectionErrors(const Values& values) const {

  // TODO: support the other calibration objects. Now it only works for Cal3_S2.

  typedef GeneralSFMFactor<SimpleCamera, Point3> SFMFactor;
  typedef GeneralSFMFactor2<Cal3_S2> SFMFactor2;

  // first count
  size_t K = 0, k=0;
  BOOST_FOREACH(const sharedFactor& f, *this)
    if (boost::dynamic_pointer_cast<const SFMFactor>(f)) ++K;
    else if (boost::dynamic_pointer_cast<const SFMFactor2>(f)) ++K;

  // now fill
  Matrix errors(2,K);
  BOOST_FOREACH(const sharedFactor& f, *this) {
    boost::shared_ptr<const SFMFactor> p = boost::dynamic_pointer_cast<const SFMFactor>(f);
    if (p) {
      errors.col(k++) = p->unwhitenedError(values);
      continue;
    }

    boost::shared_ptr<const SFMFactor2> p2 = boost::dynamic_pointer_cast<const SFMFactor2>(f);
    if (p2) {
      errors.col(k++) = p2->unwhitenedError(values);
    }
  }
  return errors;
}
/* ************************************************************************* */
}


