/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file GroupAction.h
 * @brief Group action concept and CRTP base class
 * @author Frank Dellaert
 */

#pragma once

#include <gtsam/base/Manifold.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/OptionalJacobian.h>
#include <gtsam/base/Testable.h>

#include <type_traits>

namespace gtsam {

/// Enum to specify whether the action is a Left or Right action
enum class ActionType { Left, Right };

namespace group_action {

/**
 * Orbit: The map g -> m, defined by fixing m0.
 * For Left Action: g -> g * m0
 * For Right Action: g -> m0 * g
 *
 * Think of this as "partially applying" the action at a base point m0 to
 * produce a function of the group element alone. This is often what is needed
 * when you want to differentiate trajectories on the group while tracking a
 * single point on the manifold.
 */
template <typename Action>
struct Orbit : public Action {
  using G = typename Action::Group;
  using M = typename Action::Manifold;
  static constexpr int DimG = traits<G>::dimension;
  static constexpr int DimM = traits<M>::dimension;

  M m0;

  explicit Orbit(const M& m) : m0(m) {}

  /// Constructor binding an explicit action instance (for stateful actions)
  Orbit(const M& m, const Action& a) : Action(a), m0(m) {}

  /// @brief  Apply the orbit to a group element.
  M operator()(const G& g) const {
    if constexpr (Action::type == ActionType::Left) {
      return Action::operator()(g, m0);
    } else {
      return Action::operator()(m0, g);
    }
  }

  // Version with Jacobian request: requires the action to provide Jacobians.
  template <typename A = Action,
            typename = std::enable_if_t<
                std::is_invocable_r_v<M, const A&, const G&, const M&,
                                      OptionalJacobian<DimM, DimG>,
                                      OptionalJacobian<DimM, DimM>> ||
                std::is_invocable_r_v<M, const A&, const M&, const G&,
                                      OptionalJacobian<DimM, DimM>,
                                      OptionalJacobian<DimM, DimG>>>>
  M operator()(const G& g, OptionalJacobian<DimM, DimG> H) const {
    if constexpr (Action::type == ActionType::Left) {
      return Action::operator()(g, m0, H, {});
    } else {
      return Action::operator()(m0, g, {}, H);
    }
  }
};

/**
 * Diffeomorphism: The map m -> m, defined by fixing g0.
 * (Also known as the translation map L_g or R_g).
 * For Left Action: m -> g0 * m
 * For Right Action: m -> m * g0
 *
 * Inherits from Action to allow for Empty Base Optimization (EBO).
 * If Action is stateless, this struct will have the same size as G.
 */
template <typename Action>
struct Diffeomorphism : public Action {
  using G = typename Action::Group;
  using M = typename Action::Manifold;
  static constexpr int DimG = traits<G>::dimension;
  static constexpr int DimM = traits<M>::dimension;

  G g0;

  explicit Diffeomorphism(const G& g) : g0(g) {}

  /// Constructor with explicit action instance
  Diffeomorphism(const G& g, const Action& a) : Action(a), g0(g) {}

  /// @brief Apply the diffeomorphism to a manifold point.
  M operator()(const M& m) const {
    if constexpr (Action::type == ActionType::Left) {
      return Action::operator()(g0, m);
    } else {
      return Action::operator()(m, g0);
    }
  }

  // Version with Jacobian request: requires the action to provide Jacobians.
  template <typename A = Action,
            typename = std::enable_if_t<
                std::is_invocable_r_v<M, const A&, const G&, const M&,
                                      OptionalJacobian<DimM, DimG>,
                                      OptionalJacobian<DimM, DimM>> ||
                std::is_invocable_r_v<M, const A&, const M&, const G&,
                                      OptionalJacobian<DimM, DimM>,
                                      OptionalJacobian<DimM, DimG>>>>
  M operator()(const M& m, OptionalJacobian<DimM, DimM> H) const {
    if constexpr (Action::type == ActionType::Left) {
      return Action::operator()(g0, m, {}, H);
    } else {
      return Action::operator()(m, g0, H, {});
    }
  }

  /**
   * @brief Push-forward a tangent vector through the diffeomorphism.
   *
   * Given a tangent vector v at m, this returns the translated vector in the
   * tangent space at φ_g0(m) using the Jacobian of the action. This is the
   * standard way to transport vectors between tangent spaces induced by the
   * group action.
   */
  template <typename A = Action,
            typename = std::enable_if_t<
                std::is_invocable_r_v<M, const A&, const G&, const M&,
                                      OptionalJacobian<DimM, DimG>,
                                      OptionalJacobian<DimM, DimM>> ||
                std::is_invocable_r_v<M, const A&, const M&, const G&,
                                      OptionalJacobian<DimM, DimM>,
                                      OptionalJacobian<DimM, DimG>>>>
  typename traits<M>::TangentVector pushforward(
      const M& m, const typename traits<M>::TangentVector& v,
      OptionalJacobian<DimM, DimM> H = {}) const {
    Eigen::Matrix<double, DimM, DimM> D;
    OptionalJacobian<DimM, DimM> H_arg = H ? H : &D;
    this->operator()(m, H_arg);
    const auto& J = H ? *H : D;
    return J * v;
  }
};

/**
 * @brief Induced action on vector fields via push-forward.
 *
 * For a fixed group element g and vector field f this functor evaluates
 * the induced field (g ⋅ f)(ξ) = Dφ_g(φ_{g^{-1}}(ξ)) f(φ_{g^{-1}}(ξ))
 * where φ is the underlying group action.
 *
 * In plainer terms: it moves a vector field along the group action so it stays
 * consistent with the symmetry, which is exactly what equivariant filters need
 * when transforming dynamics or measurements.
 */
template <typename Action, typename VectorField>
struct InducedVectorField : public Action {
  using G = typename Action::Group;
  using M = typename Action::Manifold;
  using TangentVector = typename traits<M>::TangentVector;

  G g_;
  VectorField field_;

  InducedVectorField(const G& g, const VectorField& f) : g_(g), field_(f) {}

  InducedVectorField(const G& g, const VectorField& f, const Action& action)
      : Action(action), g_(g), field_(f) {}

  TangentVector operator()(const M& xi) const {
    const Action& action = static_cast<const Action&>(*this);
    const Orbit<Action> orbit(xi, action);
    const M transported = orbit(g_.inverse());
    const TangentVector v = field_(transported);
    const Diffeomorphism<Action> phi_g0(g_, action);
    return phi_g0.pushforward(transported, v);
  }
};

}  // namespace group_action

/**
 * GroupAction CRTP base class.
 *
 * Defines a group action of a Group G on a Manifold M.
 * Users should derive from this class and implement the operator():
 *
 *   M operator()(const G& g, const M& m,
 *                OptionalJacobian<DimM, DimG> Hg = {},
 *                OptionalJacobian<DimM, DimM> Hm = {}) const;
 *   (for Left action)
 *
 *   or
 *
 *   M operator()(const M& m, const G& g,
 *                OptionalJacobian<DimM, DimM> Hm = {},
 *                OptionalJacobian<DimM, DimG> Hg = {}) const;
 *   (for Right action)
 *
 * @tparam Derived The user's action functor.
 * @tparam G The group type.
 * @tparam M The manifold type.
 *
 * The provided aliases make common derived constructions easier:
 *  - Orbit(m0) yields g ↦ φ(g, m0)
 *  - Diffeomorphism(g0) yields m ↦ φ(g0, m) (or φ(m, g0) for right actions)
 *  - InducedVectorField(g0, f) yields the pushed-forward vector field g0 ⋅ f
 * These are lightweight functors; build them directly where needed.
 */
template <typename Derived, typename G, typename M>
struct GroupAction {
  /// Access derived class
  const Derived& derived() const { return static_cast<const Derived&>(*this); }

  // Helper typedefs
  using Group = G;
  using Manifold = M;
  static constexpr int DimG = traits<G>::dimension;
  static constexpr int DimM = traits<M>::dimension;

  // --------------------------------------------------------------------------
  // Function Objects for Partial Application
  // --------------------------------------------------------------------------

  using Orbit = group_action::Orbit<Derived>;
  using Diffeomorphism = group_action::Diffeomorphism<Derived>;
  template <typename VectorField>
  struct InducedVectorField
      : public group_action::InducedVectorField<Derived, VectorField> {
    using Base = group_action::InducedVectorField<Derived, VectorField>;
    using Base::Base;  // inherit default/aux constructors

    InducedVectorField(const Group& g, const VectorField& f) : Base(g, f) {}

    InducedVectorField(const Group& g, const VectorField& f,
                       const Derived& action)
        : Base(g, f, action) {}
  };
  // These wrappers keep call sites short while allowing CTAD on the
  // VectorField type: `Derived::InducedVectorField f(g, f_vecfield);`
};

// --------------------------------------------------------------------------
// Test Helpers
// --------------------------------------------------------------------------

/// Check right action property: φ(m, g1 g2) = φ(φ(m, g1), g2).
template <class Action, class M_, class G_>
inline bool rightActionEqual(const Action& phi, const M_& m, const G_& g1,
                             const G_& g2) {
  const M_ left = phi(m, g1 * g2);
  const M_ right = phi(phi(m, g1), g2);
  return assert_equal(left, right);
}

/// Check left action property: φ(g1 g2, m) = φ(g1, φ(g2, m)).
template <class Action, class G_, class M_>
inline bool leftActionEqual(const Action& phi, const G_& g1, const G_& g2,
                            const M_& m) {
  const M_ left = phi(g1 * g2, m);
  const M_ right = phi(g1, phi(g2, m));
  return assert_equal(left, right);
}

#define EXPECT_RIGHT_ACTION(phi, m, g1, g2)                    \
  do {                                                         \
    EXPECT(::gtsam::rightActionEqual((phi), (m), (g1), (g2))); \
  } while (0)

#define EXPECT_LEFT_ACTION(phi, g1, g2, m)                    \
  do {                                                        \
    EXPECT(::gtsam::leftActionEqual((phi), (g1), (g2), (m))); \
  } while (0)

}  // namespace gtsam
