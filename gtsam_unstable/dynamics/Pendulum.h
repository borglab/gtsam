/**
 * @file Pendulum.h
 * @brief Three-way factors for the pendulum dynamics as in [Stern06siggraph] for
 *        (1) explicit Euler method, (2) implicit Euler method, and (3) sympletic Euler method.
 *        Note that all methods use the same formulas for the factors. They are only different in
 *        the way we connect variables using those factors in the graph.
 * @author Duy-Nguyen Ta
 */

#pragma once

#include <gtsam/nonlinear/NonlinearFactor.h>

namespace gtsam {

//*************************************************************************
/**
 * This class implements the first constraint.
 *    - For explicit Euler method:  q_{k+1} = q_k + h*v_k
 *    - For implicit Euler method:  q_{k+1} = q_k + h*v_{k+1}
 *    - For sympletic Euler method: q_{k+1} = q_k + h*v_{k+1}
 */
class PendulumFactor1: public NoiseModelFactorN<double, double, double> {
public:

protected:
  typedef NoiseModelFactorN<double, double, double> Base;

  /** default constructor to allow for serialization */
  PendulumFactor1() {}

  double h_;  // time step

public:

  // Provide access to the Matrix& version of evaluateError:
  using Base::evaluateError;

  typedef std::shared_ptr<PendulumFactor1> shared_ptr;

  ///Constructor.  k1: q_{k+1}, k: q_k, velKey: velocity variable depending on the chosen method, h: time step
  PendulumFactor1(Key k1, Key k, Key velKey, double h, double mu = 1000.0)
  : Base(noiseModel::Constrained::All(1, std::abs(mu)), k1, k, velKey), h_(h) {}

  /// @return a deep copy of this factor
  gtsam::NonlinearFactor::shared_ptr clone() const override {
    return std::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new PendulumFactor1(*this))); }

  /** q_k + h*v - q_k1 = 0, with optional derivatives */
  Vector evaluateError(const double& qk1, const double& qk, const double& v,
      OptionalMatrixType H1, OptionalMatrixType H2,
      OptionalMatrixType H3) const override {
    const size_t p = 1;
    if (H1) *H1 = -Matrix::Identity(p,p);
    if (H2) *H2 = Matrix::Identity(p,p);
    if (H3) *H3 = Matrix::Identity(p,p)*h_;
    return (Vector(1) << qk+v*h_-qk1).finished();
  }

}; // \PendulumFactor1


//*************************************************************************
/**
 * This class implements the second constraint the
 *    - For explicit Euler method:  v_{k+1} = v_k - h*g/L*sin(q_k)
 *    - For implicit Euler method:  v_{k+1} = v_k - h*g/L*sin(q_{k+1})
 *    - For sympletic Euler method: v_{k+1} = v_k - h*g/L*sin(q_k)
 */
class PendulumFactor2: public NoiseModelFactorN<double, double, double> {
public:

protected:
  typedef NoiseModelFactorN<double, double, double> Base;

  /** default constructor to allow for serialization */
  PendulumFactor2() {}

  double h_;
  double g_;
  double r_;

public:

  // Provide access to the Matrix& version of evaluateError:
  using Base::evaluateError;

  typedef std::shared_ptr<PendulumFactor2 > shared_ptr;

  ///Constructor.  vk1: v_{k+1}, vk: v_k, qkey: q's key depending on the chosen method, h: time step
  PendulumFactor2(Key vk1, Key vk, Key qkey, double h, double r = 1.0, double g = 9.81, double mu = 1000.0)
  : Base(noiseModel::Constrained::All(1, std::abs(mu)), vk1, vk, qkey), h_(h), g_(g), r_(r) {}

  /// @return a deep copy of this factor
  gtsam::NonlinearFactor::shared_ptr clone() const override {
    return std::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new PendulumFactor2(*this))); }

  /**  v_k - h*g/L*sin(q) - v_k1 = 0, with optional derivatives */
  Vector evaluateError(const double & vk1, const double & vk, const double & q,
      OptionalMatrixType H1, OptionalMatrixType H2,
      OptionalMatrixType H3) const override {
    const size_t p = 1;
    if (H1) *H1 = -Matrix::Identity(p,p);
    if (H2) *H2 = Matrix::Identity(p,p);
    if (H3) *H3 = -Matrix::Identity(p,p)*h_*g_/r_*cos(q);
    return (Vector(1) << vk - h_ * g_ / r_ * sin(q) - vk1).finished();
  }

}; // \PendulumFactor2


//*************************************************************************
/**
 * This class implements the first position-momentum update rule
 *  \f$ p_k = -D_1 L_d(q_k,q_{k+1},h) = \frac{1}{h}mr^{2}\left(q_{k+1}-q_{k}\right)+mgrh(1-\alpha)\,\sin\left((1-\alpha)q_{k}+\alpha q_{k+1}\right) \f$
 *  \f$ = (1/h)mr^2 (q_{k+1}-q_k) + mgrh(1-alpha) sin ((1-alpha)q_k+\alpha q_{k+1}) \f$
 */
class PendulumFactorPk: public NoiseModelFactorN<double, double, double> {
public:

protected:
  typedef NoiseModelFactorN<double, double, double> Base;

  /** default constructor to allow for serialization */
  PendulumFactorPk() {}

  double h_;  //! time step
  double m_;  //! mass
  double r_;  //! length
  double g_;  //! gravity
  double alpha_; //! in [0,1], define the mid-point between [q_k,q_{k+1}] for approximation. The sympletic rule above can be obtained as a special case when alpha = 0.

public:

  // Provide access to the Matrix& version of evaluateError:
  using Base::evaluateError;

  typedef std::shared_ptr<PendulumFactorPk > shared_ptr;

  ///Constructor
  PendulumFactorPk(Key pKey, Key qKey, Key qKey1,
      double h, double m = 1.0, double r = 1.0, double g = 9.81, double alpha = 0.0, double mu = 1000.0)
  : Base(noiseModel::Constrained::All(1, std::abs(mu)), pKey, qKey, qKey1),
    h_(h), m_(m), r_(r), g_(g), alpha_(alpha) {}

  /// @return a deep copy of this factor
  gtsam::NonlinearFactor::shared_ptr clone() const override {
    return std::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new PendulumFactorPk(*this))); }

  /**  1/h mr^2 (qk1-qk)+mgrh (1-a) sin((1-a)pk + a*pk1) - pk = 0, with optional derivatives */
  Vector evaluateError(const double & pk, const double & qk, const double & qk1,
      OptionalMatrixType H1, OptionalMatrixType H2,
      OptionalMatrixType H3) const override {
    const size_t p = 1;

    double qmid = (1-alpha_)*qk + alpha_*qk1;
    double mr2_h = 1/h_*m_*r_*r_;
    double mgrh  = m_*g_*r_*h_;

    if (H1) *H1 = -Matrix::Identity(p,p);
    if (H2) *H2 = Matrix::Identity(p,p)*(-mr2_h + mgrh*(1-alpha_)*(1-alpha_)*cos(qmid));
    if (H3) *H3 = Matrix::Identity(p,p)*( mr2_h + mgrh*(1-alpha_)*(alpha_)*cos(qmid));

    return (Vector(1) << mr2_h * (qk1 - qk) + mgrh * (1 - alpha_) * sin(qmid) - pk).finished();
  }

}; // \PendulumFactorPk

//*************************************************************************
/**
 * This class implements the second position-momentum update rule
 *  \f$ p_k1 = D_2 L_d(q_k,q_{k+1},h) = \frac{1}{h}mr^{2}\left(q_{k+1}-q_{k}\right)-mgrh\alpha\sin\left((1-\alpha)q_{k}+\alpha q_{k+1}\right) \f$
 *  \f$ = (1/h)mr^2 (q_{k+1}-q_k) - mgrh alpha sin ((1-alpha)q_k+\alpha q_{k+1}) \f$
 */
class PendulumFactorPk1: public NoiseModelFactorN<double, double, double> {
public:

protected:
  typedef NoiseModelFactorN<double, double, double> Base;

  /** default constructor to allow for serialization */
  PendulumFactorPk1() {}

  double h_;  //! time step
  double m_;  //! mass
  double r_;  //! length
  double g_;  //! gravity
  double alpha_; //! in [0,1], define the mid-point between [q_k,q_{k+1}] for approximation. The sympletic rule above can be obtained as a special case when alpha = 0.

public:

  // Provide access to the Matrix& version of evaluateError:
  using Base::evaluateError;

  typedef std::shared_ptr<PendulumFactorPk1 > shared_ptr;

  ///Constructor
  PendulumFactorPk1(Key pKey1, Key qKey, Key qKey1,
      double h, double m = 1.0, double r = 1.0, double g = 9.81, double alpha = 0.0, double mu = 1000.0)
  : Base(noiseModel::Constrained::All(1, std::abs(mu)), pKey1, qKey, qKey1),
    h_(h), m_(m), r_(r), g_(g), alpha_(alpha) {}

  /// @return a deep copy of this factor
  gtsam::NonlinearFactor::shared_ptr clone() const override {
    return std::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new PendulumFactorPk1(*this))); }

  /**  1/h mr^2 (qk1-qk) - mgrh a sin((1-a)pk + a*pk1) - pk1 = 0, with optional derivatives */
  Vector evaluateError(const double & pk1, const double & qk, const double & qk1,
      OptionalMatrixType H1, OptionalMatrixType H2,
      OptionalMatrixType H3) const override {
    const size_t p = 1;

    double qmid = (1-alpha_)*qk + alpha_*qk1;
    double mr2_h = 1/h_*m_*r_*r_;
    double mgrh  = m_*g_*r_*h_;

    if (H1) *H1 = -Matrix::Identity(p,p);
    if (H2) *H2 = Matrix::Identity(p,p)*(-mr2_h - mgrh*(1-alpha_)*alpha_*cos(qmid));
    if (H3) *H3 = Matrix::Identity(p,p)*( mr2_h - mgrh*alpha_*alpha_*cos(qmid));

    return (Vector(1) << mr2_h * (qk1 - qk) - mgrh * alpha_ * sin(qmid) - pk1).finished();
  }

}; // \PendulumFactorPk1

}
