/**
 * @file Pendulum.h
 * @brief Three-way factors for the pendulum dynamics as in [Stern06siggraph] for
 *        (1) explicit Euler method, (2) implicit Euler method, and (3) sympletic Euler method.
 *        Note that all methods use the same formulas for the factors. They are only different in
 *        the way we connect variables using those factors in the graph.
 * @author Duy-Nguyen Ta
 */

#pragma once

#include <gtsam/base/LieScalar.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

namespace gtsam {

/**
 * This class implements the first constraint.
 *    - For explicit Euler method:  q_{k+1} = q_k + dt*v_k
 *    - For implicit Euler method:  q_{k+1} = q_k + dt*v_{k+1}
 *    - For sympletic Euler method: q_{k+1} = q_k + dt*v_{k+1}
 */
class PendulumFactor1: public NoiseModelFactor3<LieScalar, LieScalar, LieScalar> {
public:

protected:
  typedef NoiseModelFactor3<LieScalar, LieScalar, LieScalar> Base;

  /** default constructor to allow for serialization */
  PendulumFactor1() {}

  double dt_;

public:

  typedef boost::shared_ptr<PendulumFactor1> shared_ptr;

  ///Constructor.  k1: q_{k+1}, k: q_k, velKey: velocity variable depending on the chosen method, dt: time step
  PendulumFactor1(Key k1, Key k, Key velKey, double dt, double mu = 1000.0)
  : Base(noiseModel::Constrained::All(LieScalar::Dim(), fabs(mu)), k1, k, velKey), dt_(dt) {}

  /// @return a deep copy of this factor
  virtual gtsam::NonlinearFactor::shared_ptr clone() const {
    return boost::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new PendulumFactor1(*this))); }

  /** q_k + dt*v - q_k1 = 0, with optional derivatives */
  Vector evaluateError(const LieScalar& qk1, const LieScalar& qk, const LieScalar& v,
      boost::optional<Matrix&> H1 = boost::none,
      boost::optional<Matrix&> H2 = boost::none,
      boost::optional<Matrix&> H3 = boost::none) const {
    const size_t p = LieScalar::Dim();
    if (H1) *H1 = -eye(p);
    if (H2) *H2 = eye(p);
    if (H3) *H3 = eye(p)*dt_;
    return qk1.localCoordinates(qk.compose(LieScalar(v*dt_)));
  }

}; // \PendulumFactor1


/**
 * This class implements the second constraint the
 *    - For explicit Euler method:  v_{k+1} = v_k - dt*g/L*sin(q_k)
 *    - For implicit Euler method:  v_{k+1} = v_k - dt*g/L*sin(q_{k+1})
 *    - For sympletic Euler method: v_{k+1} = v_k - dt*g/L*sin(q_k)
 */
class PendulumFactor2: public NoiseModelFactor3<LieScalar, LieScalar, LieScalar> {
public:

protected:
  typedef NoiseModelFactor3<LieScalar, LieScalar, LieScalar> Base;

  /** default constructor to allow for serialization */
  PendulumFactor2() {}

  double dt_;
  double g_;
  double L_;

public:

  typedef boost::shared_ptr<PendulumFactor2 > shared_ptr;

  ///Constructor.  vk1: v_{k+1}, vk: v_k, qkey: q's key depending on the chosen method, dt: time step
  PendulumFactor2(Key vk1, Key vk, Key qkey, double dt, double L = 1.0, double g = 9.81, double mu = 1000.0)
  : Base(noiseModel::Constrained::All(LieScalar::Dim(), fabs(mu)), vk1, vk, qkey), dt_(dt), g_(g), L_(L) {}

  /// @return a deep copy of this factor
  virtual gtsam::NonlinearFactor::shared_ptr clone() const {
    return boost::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new PendulumFactor2(*this))); }

  /**  v_k - dt*g/L*sin(q) - v_k1 = 0, with optional derivatives */
  Vector evaluateError(const LieScalar& vk1, const LieScalar& vk, const LieScalar& q,
      boost::optional<Matrix&> H1 = boost::none,
      boost::optional<Matrix&> H2 = boost::none,
      boost::optional<Matrix&> H3 = boost::none) const {
    const size_t p = LieScalar::Dim();
    if (H1) *H1 = -eye(p);
    if (H2) *H2 = eye(p);
    if (H3) *H3 = -eye(p)*dt_*g_/L_*cos(q.value());
    return vk1.localCoordinates(LieScalar(vk - dt_*g_/L_*sin(q)));
  }

}; // \PendulumFactor2

}
