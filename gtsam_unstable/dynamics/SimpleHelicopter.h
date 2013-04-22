/*
 * @file SimpleHelicopter.h
 * @brief Implement SimpleHelicopter discrete dynamics model and variational integrator,
 *        following [Kobilarov09siggraph]
 * @author Duy-Nguyen Ta
 */

#pragma once

#include <gtsam/geometry/Pose3.h>
#include <gtsam/base/LieVector.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

namespace gtsam {

/**
 * Implement the Reconstruction equation: \f$ g_{k+1} = g_k \exp (h\xi_k) \f$, where
 *      \f$ h \f$: timestep (parameter)
 *      \f$ g_{k+1}, g_{k} \f$: poses at the current and the next timestep
 *      \f$ \xi_k \f$: the body-fixed velocity (Lie algebra)
 * It is somewhat similar to BetweenFactor, but treats the body-fixed velocity
 * \f$ \xi_k \f$ as a variable. So it is a three-way factor.
 */
class Reconstruction : public NoiseModelFactor3<Pose3, Pose3, LieVector>  {

  double h_;  // time step
  typedef NoiseModelFactor3<Pose3, Pose3, LieVector> Base;
public:
  Reconstruction(Key gKey1, Key gKey, Key xiKey, double h, double mu = 1000.0) :
    Base(noiseModel::Constrained::All(Pose3::Dim()*3, fabs(mu)), gKey1, gKey,
        xiKey), h_(h) {
  }
  virtual ~Reconstruction() {}

  /// @return a deep copy of this factor
  virtual gtsam::NonlinearFactor::shared_ptr clone() const {
    return boost::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new Reconstruction(*this))); }

  /** \f$ log((g_k\exp(h\xi_k))^{-1}g_{k+1}) = 0, with optional derivatives */
  Vector evaluateError(const Pose3& gk1, const Pose3& gk, const LieVector& xik,
      boost::optional<Matrix&> H1 = boost::none,
      boost::optional<Matrix&> H2 = boost::none,
      boost::optional<Matrix&> H3 = boost::none) const {

    static const bool debug = false;

    Matrix gkxiHgk, gkxiHexpxi;
    Pose3 gkxi = gk.compose(Pose3::Expmap(h_*xik), gkxiHgk, gkxiHexpxi);

    Matrix hxHgk1, hxHgkxi;
    Pose3 hx = gkxi.between(gk1, hxHgkxi, hxHgk1);

    if (H1) {
      *H1 = hxHgk1;
    }
    if (H2) {
      Matrix hxHgk = hxHgkxi*gkxiHgk;
      *H2 = hxHgk;
    }

    if (H3) {
      Matrix expxiHxi = Pose3::dExpInv_TLN(xik);
      Matrix hxHxi = hxHgkxi*gkxiHexpxi*expxiHxi;
      *H3 = hxHxi;
    }

    return Pose3::Logmap(hx);
  }

};

} /* namespace gtsam */
