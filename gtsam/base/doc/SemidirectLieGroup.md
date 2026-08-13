# `SemidirectLieGroup`: Action-Coupled Products

`SemidirectLieGroup<G,H,Action>` constructs the left semidirect product
`G ⋉ H`, where `G` acts on the vector space `H`. Include it with:

```cpp
#include <gtsam/base/SemidirectLieGroup.h>
```

## Defining the action

Derive the action from `GroupAction`, implement its optional Jacobians, and
provide its infinitesimal generator:

```cpp
struct Rot3VectorAction
    : public gtsam::GroupAction<Rot3VectorAction, gtsam::Rot3,
                                gtsam::Vector3> {
  static constexpr gtsam::ActionType type = gtsam::ActionType::Left;

  gtsam::Vector3 operator()(const gtsam::Rot3& R, const gtsam::Vector3& p,
                            gtsam::OptionalJacobian<3, 3> HR = {},
                            gtsam::OptionalJacobian<3, 3> Hp = {}) const {
    return R.rotate(p, HR, Hp);
  }

  static gtsam::Matrix3 generator(const gtsam::Vector3& omega) {
    return gtsam::skewSymmetric(omega);
  }
};

using SE3Like = gtsam::SemidirectLieGroup<
    gtsam::Rot3, gtsam::Vector3, Rot3VectorAction>;
```

The action must be a default-constructible left `GroupAction`; `H` must be a
fixed-size Eigen vector; and `Action::generator(u)` must return the fixed-size
matrix representing the infinitesimal action.

## Group and Lie operations

Writing the action as `phi(g,h)`, the group law is:

```text
(g1,h1) * (g2,h2) = (g1*g2, h1 + phi(g1,h2))
(g,h)^-1           = (g^-1, phi(g^-1,-h))
```

With `A(u) = Action::generator(u)` and
`phi1(A) = sum(A^k/(k+1)!)`, the Lie maps are:

```text
Exp(u,v) = (Exp_G(u), phi1(A(u))*v)
Log(g,h) = (u, phi1(A(u))^-1*h),  u = Log_G(g)
```

The implementation evaluates `phi1` through an augmented matrix exponential,
which remains well behaved when `A` is singular or close to zero.

## Jacobians and adjoints

All optional Jacobians use GTSAM's right-Jacobian convention. When `G`
provides static `adjointMap(u)`, the complete exponential Jacobian uses
`Jr(xi) = phi1(-ad_xi)` and reuses its lower-right block for the transported
value. A reduced vector-only Fréchet construction preserves compatibility with
custom base groups that omit static `adjointMap`.

`AdjointMap()` is derived from the action's two Jacobians. Static
`adjointMap(xi)` is available when the base group provides its algebra
adjoint.

For the special adjoint action of a group on its own algebra, prefer
[`TangentLieGroup`](TangentLieGroup.md), whose repeated-block structure admits
faster kernels and simpler direct formulas.

The implementation is exercised by
[`testSemidirectLieGroup.cpp`](../../../tests/testSemidirectLieGroup.cpp).
