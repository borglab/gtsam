/*
 * @file SimpleHelicopter.h
 * @brief Implement SimpleHelicopter discrete dynamics model and variational integrator,
 *        following [Kobilarov09siggraph]
 * @author Duy-Nguyen Ta
 */

#pragma once

#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/base/numericalDerivative.h>
#include <cmath>

namespace gtsam {

/**
 * Implement the Reconstruction equation: \f$ g_{k+1} = g_k \exp (h\xi_k) \f$, where
 *      \f$ h \f$: timestep (parameter)
 *      \f$ g_{k+1}, g_{k} \f$: poses at the current and the next timestep
 *      \f$ \xi_k \f$: the body-fixed velocity (Lie algebra)
 * It is somewhat similar to BetweenFactor, but treats the body-fixed velocity
 * \f$ \xi_k \f$ as a variable. So it is a three-way factor.
 * Note: this factor is necessary if one needs to smooth the entire graph. It's not needed
 *  in sequential update method.
 */
class Reconstruction : public NoiseModelFactor3<Pose3, Pose3, Vector6>  {

  double h_;  // time step
  typedef NoiseModelFactor3<Pose3, Pose3, Vector6> Base;
public:
  Reconstruction(Key gKey1, Key gKey, Key xiKey, double h, double mu = 1000.0) :
    Base(noiseModel::Constrained::All(6, std::abs(mu)), gKey1, gKey,
        xiKey), h_(h) {
  }
  ~Reconstruction() override {}

  /// @return a deep copy of this factor
  gtsam::NonlinearFactor::shared_ptr clone() const override {
    return boost::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new Reconstruction(*this))); }

  /** \f$ log((g_k\exp(h\xi_k))^{-1}g_{k+1}) = 0, with optional derivatives */
  Vector evaluateError(const Pose3& gk1, const Pose3& gk, const Vector6& xik,
      boost::optional<Matrix&> H1 = boost::none,
      boost::optional<Matrix&> H2 = boost::none,
      boost::optional<Matrix&> H3 = boost::none) const override {

    Matrix6 D_exphxi_xi;
    Pose3 exphxi = Pose3::Expmap(h_ * xik, H3 ? &D_exphxi_xi : 0);

    Matrix6 D_gkxi_gk, D_gkxi_exphxi;
    Pose3 gkxi = gk.compose(exphxi, D_gkxi_gk, H3 ? &D_gkxi_exphxi : 0);

    Matrix6 D_hx_gk1, D_hx_gkxi;
    Pose3 hx = gkxi.between(gk1, (H2 || H3) ? &D_hx_gkxi : 0, H1 ? &D_hx_gk1 : 0);

    Matrix6 D_log_hx;
    Vector error = Pose3::Logmap(hx, D_log_hx);

    if (H1) *H1 = D_log_hx * D_hx_gk1;
    if (H2 || H3) {
      Matrix6 D_log_gkxi = D_log_hx * D_hx_gkxi;
      if (H2) *H2 = D_log_gkxi * D_gkxi_gk;
      if (H3) *H3 = D_log_gkxi * D_gkxi_exphxi * D_exphxi_xi * h_;
    }

    return error;
  }

};

/**
 * Implement the Discrete Euler-Poincare' equation:
 */
class DiscreteEulerPoincareHelicopter : public NoiseModelFactor3<Vector6, Vector6, Pose3>  {

  double h_;  /// time step
  Matrix Inertia_;  /// Inertia tensors Inertia = [ J 0; 0 M ]
  Vector Fu_;   /// F is the 6xc Control matrix, where c is the number of control variables uk, which directly change the vehicle pose (e.g., gas/brake/speed)
  /// F(.) is actually a function of the shape variables, which do not change the pose, but affect the vehicle's shape, e.g. steering wheel.
  /// Fu_ encodes everything we need to know about the vehicle's dynamics.
  double m_;  /// mass. For gravity external force f_ext, which has a fixed formula in this case.

  // TODO: Fk_ and f_ext should be generalized as functions (factor nodes) on control signals and poses/velocities.
  // This might be needed in control or system identification problems.
  // We treat them as constant here, since the control inputs are to specify.

  typedef NoiseModelFactor3<Vector6, Vector6, Pose3> Base;

public:
  DiscreteEulerPoincareHelicopter(Key xiKey1, Key xiKey_1, Key gKey,
      double h, const Matrix& Inertia, const Vector& Fu, double m,
      double mu = 1000.0) :
        Base(noiseModel::Constrained::All(6, std::abs(mu)), xiKey1, xiKey_1, gKey),
        h_(h), Inertia_(Inertia), Fu_(Fu), m_(m) {
  }
  ~DiscreteEulerPoincareHelicopter() override {}

  /// @return a deep copy of this factor
  gtsam::NonlinearFactor::shared_ptr clone() const override {
    return boost::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new DiscreteEulerPoincareHelicopter(*this))); }

  /** DEP, with optional derivatives
   * pk - pk_1 - h_*Fu_ - h_*f_ext = 0
   * where pk = CT_TLN(h*xi_k)*Inertia*xi_k
   *       pk_1 = CT_TLN(-h*xi_k_1)*Inertia*xi_k_1
   * */
  Vector evaluateError(const Vector6& xik, const Vector6& xik_1, const Pose3& gk,
      boost::optional<Matrix&> H1 = boost::none,
      boost::optional<Matrix&> H2 = boost::none,
      boost::optional<Matrix&> H3 = boost::none) const override {

    Vector muk = Inertia_*xik;
    Vector muk_1 = Inertia_*xik_1;

    // Apply the inverse right-trivialized tangent (derivative) map of the exponential map,
    // using the trapezoidal Lie-Newmark (TLN) scheme, to a vector.
    // TLN is just a first order approximation of the dExpInv_exp above, detailed in [Kobilarov09siggraph]
    // C_TLN formula: I6 - 1/2 ad[xi].
    Matrix D_adjThxik_muk, D_adjThxik1_muk1;
    Vector pk = muk - 0.5*Pose3::adjointTranspose(h_*xik, muk, D_adjThxik_muk);
    Vector pk_1 = muk_1 - 0.5*Pose3::adjointTranspose(-h_*xik_1, muk_1, D_adjThxik1_muk1);

    Matrix D_gravityBody_gk;
    Point3 gravityBody = gk.rotation().unrotate(Point3(0.0, 0.0, -9.81*m_), D_gravityBody_gk, boost::none);
    Vector f_ext = (Vector(6) << 0.0, 0.0, 0.0, gravityBody.x(), gravityBody.y(), gravityBody.z()).finished();

    Vector hx = pk - pk_1 - h_*Fu_ - h_*f_ext;

    if (H1) {
      Matrix D_pik_xi = Inertia_-0.5*(h_*D_adjThxik_muk + Pose3::adjointMap(h_*xik).transpose()*Inertia_);
      *H1 = D_pik_xi;
    }

    if (H2) {
      Matrix D_pik1_xik1 = Inertia_-0.5*(-h_*D_adjThxik1_muk1 + Pose3::adjointMap(-h_*xik_1).transpose()*Inertia_);
      *H2 = -D_pik1_xik1;
    }

    if (H3) {
      *H3 = Z_6x6;
      insertSub(*H3, -h_*D_gravityBody_gk, 3, 0);
    }

    return hx;
  }

#if 0
  Vector computeError(const Vector6& xik, const Vector6& xik_1, const Pose3& gk) const {
    Vector pk = Pose3::dExpInv_exp(h_*xik).transpose()*Inertia_*xik;
    Vector pk_1 = Pose3::dExpInv_exp(-h_*xik_1).transpose()*Inertia_*xik_1;

    Point3 gravityBody = gk.rotation().unrotate(Point3(0.0, 0.0, -9.81*m_));
    Vector f_ext = (Vector(6) << 0.0, 0.0, 0.0, gravityBody.x(), gravityBody.y(), gravityBody.z());

    Vector hx = pk - pk_1 - h_*Fu_ - h_*f_ext;

    return hx;
  }

  Vector evaluateError(const Vector6& xik, const Vector6& xik_1, const Pose3& gk,
      boost::optional<Matrix&> H1 = boost::none,
      boost::optional<Matrix&> H2 = boost::none,
      boost::optional<Matrix&> H3 = boost::none) const {
    if (H1) {
      (*H1) = numericalDerivative31(
          std::function<Vector(const Vector6&, const Vector6&, const Pose3&)>(
              std::bind(&DiscreteEulerPoincareHelicopter::computeError, *this, _1, _2, _3)
          ),
          xik, xik_1, gk, 1e-5
      );
    }
    if (H2) {
      (*H2) = numericalDerivative32(
          std::function<Vector(const Vector6&, const Vector6&, const Pose3&)>(
              std::bind(&DiscreteEulerPoincareHelicopter::computeError, *this, _1, _2, _3)
          ),
          xik, xik_1, gk, 1e-5
      );
    }
    if (H3) {
      (*H3) = numericalDerivative33(
          std::function<Vector(const Vector6&, const Vector6&, const Pose3&)>(
              std::bind(&DiscreteEulerPoincareHelicopter::computeError, *this, _1, _2, _3)
          ),
          xik, xik_1, gk, 1e-5
      );
    }

    return computeError(xik, xik_1, gk);
  }
#endif

};


} /* namespace gtsam */
