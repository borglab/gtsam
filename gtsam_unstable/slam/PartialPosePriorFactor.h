/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file PartialPosePriorFactor.h
 * @brief Factor for setting a partial rotation or translation prior on a pose.
 * @author Milo Knowles
 */
#pragma once

#include <gtsam_unstable/slam/PartialPriorFactor.h>

namespace gtsam {

template <typename POSE>
class PartialPosePriorFactor : public PartialPriorFactor<POSE> {
 private:
  typedef PartialPriorFactor<POSE> Base;
  typedef PartialPosePriorFactor<POSE> This;
  typedef typename POSE::Translation Translation;
  typedef typename POSE::Rotation Rotation;

 public:
  GTSAM_CONCEPT_LIE_TYPE(POSE)

  using Base::Base;
  using Base::evaluateError;

  gtsam::NonlinearFactor::shared_ptr clone() const override {
    return std::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new This(*this))); }

 private:
  /**
   * Maps a POSE input x to a parameter vector h(x). In this case, the "parameter"
   * vector is the same as the tangent vector representation (Logmap(x)) for
   * rotation but is different for translation.
   *
   * For Pose2, the parameter vector is [ t, theta ].
   * For Pose3, the parameter vector is [ r t ], where r is angle-axis
   * representation of rotation, and t is the translation part of x.
  */
  virtual Vector Parameterize(const POSE& x, Matrix* H = nullptr) const override {
    const int rDim = traits<Rotation>::GetDimension(x.rotation());
    const int tDim = traits<Translation>::GetDimension(x.translation());
    const int xDim = traits<POSE>::GetDimension(x);

    Vector p(xDim); // The output parameter vector.

    const std::pair<size_t, size_t> transInterval = POSE::translationInterval();
    const std::pair<size_t, size_t> rotInterval = POSE::rotationInterval();

    p.middleRows(transInterval.first, tDim) = x.translation();

    Matrix H_rot;
    p.middleRows(rotInterval.first, rDim) = (H) ? Rotation::Logmap(x.rotation(), H_rot)
                                                : Rotation::Logmap(x.rotation());

    if (H) {
      *H = Matrix::Zero(xDim, xDim);
      (*H).block(rotInterval.first, rotInterval.first, rDim, rDim) = H_rot;
      (*H).block(transInterval.first, transInterval.first, tDim, tDim) = x.rotation().matrix();
    }

    return p;
  }
};

}
