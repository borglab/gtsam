# `ProductLieGroup`: Direct and Semidirect Products

`ProductLieGroup` constructs a Lie group from two existing Lie groups. With
its default third template argument it represents the direct product
`G × H`; with a left group action it represents the semidirect product
`G ⋉ H`. In both cases, the product provides GTSAM's standard group, manifold,
Lie-group, adjoint, and optional-Jacobian interfaces.

Both component types must satisfy GTSAM's Lie-group requirements described in
the [`LieGroup` guide](LieGroup.md).

Include the class with:

```cpp
#include <gtsam/base/ProductLieGroup.h>
```

## Direct products

A direct product keeps the two component groups independent:

```cpp
using RotationPose = gtsam::ProductLieGroup<gtsam::Rot3, gtsam::Pose2>;

const RotationPose x(gtsam::Rot3::Rz(0.2), gtsam::Pose2(1.0, 2.0, 0.3));
const gtsam::Rot3& rotation = x.first;
const gtsam::Pose2& pose = x.second;
```

For elements `(g1,h1)` and `(g2,h2)`, the operations are componentwise:

```text
(g1,h1) * (g2,h2) = (g1*g2, h1*h2)
(g,h)^-1           = (g^-1, h^-1)
Exp(u,v)           = (Exp_G(u), Exp_H(v))
Log(g,h)           = (Log_G(g), Log_H(h))
```

The tangent vector concatenates the two component tangents as `[u;v]`. For
fixed-dimensional components, the product dimension is known at compile time
and equals `dim(G) + dim(H)`. Direct products also support dynamically sized
components, with dimensions obtained from the stored values.

The group and algebra adjoints are block diagonal:

```text
Ad_(g,h) = diag(Ad_g, Ad_h)
ad_(u,v) = diag(ad_u, ad_v)
```

Eigen vector-space factors are abelian, so their algebra-adjoint block is
zero.

## Semidirect products

A semidirect product couples the second component to the first through a left
action `phi(g,h)`. Declare an action by deriving from `GroupAction`, providing
its optional Jacobians, and providing the infinitesimal generator
`A(u) = d/dt phi(Exp_G(t*u), .)|t=0`:

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

using SE3Like =
    gtsam::ProductLieGroup<gtsam::Rot3, gtsam::Vector3, Rot3VectorAction>;
```

The resulting group law is:

```text
(g1,h1) * (g2,h2) = (g1*g2, h1 + phi(g1,h2))
(g,h)^-1           = (g^-1, phi(g^-1,-h))
```

The current generic semidirect implementation requires:

- `Action` to be default-constructible and derived from
  `GroupAction<Action,G,H>`;
- `Action::type` to be `ActionType::Left`;
- `H` to be a fixed-size Eigen column vector, whose group operation is
  addition; and
- `Action::generator(u)` to return the fixed-size action generator.

These requirements let `ProductLieGroup` derive the exponential and logarithm
analytically. With `A(u) = Action::generator(u)` and
`phi1(A) = sum(A^k/(k+1)!)`, they are:

```text
Exp(u,v) = (Exp_G(u), phi1(A(u))*v)
Log(g,h) = (u, phi1(A(u))^-1*h),  u = Log_G(g)
```

The implementation evaluates `phi1` through an augmented matrix exponential,
avoiding formulas that become unstable when `A` is singular or close to zero.

## Jacobians and adjoints

All optional Jacobians use GTSAM's right-trivialized chart convention. Direct
products assemble the component Jacobians into independent blocks.
Semidirect-product Jacobians include the coupling induced by the action.

When the base group exposes a static `G::adjointMap(u)`, the complete Expmap
Jacobian is computed from the universal right-Jacobian identity
`Jr(xi) = phi1(-ad_xi)`. A directional Fréchet construction remains available
for compatible custom base groups without that static operation.

`AdjointMap()` is derived from the action's two Jacobians. The static
`adjointMap(xi)` is also available when non-vector component groups provide
their static algebra adjoint.

## Dynamic dimensions

The one-vector `Expmap([u;v])` and `adjointMap([u;v])` overloads can infer the
split when at most one component is dynamically sized. If both components are
dynamic, use the two-vector `Expmap(u,v)` overload. For the special direct
product of two dynamic vector spaces, `adjointMap` is unambiguously zero.

## Choosing the appropriate construction

Use a direct product when the component group laws are independent. Use a
semidirect product when the first group transforms the second component during
composition, such as rotations acting on translations. Use a dedicated Lie
group class when the representation needs additional invariants, storage, or
more specialized closed-form operations.

For the important adjoint-action semidirect product `G ⋉ 𝔤`, use
[`TangentLieGroup`](TangentLieGroup.md). The implementation is exercised by
[`testProductLieGroup.cpp`](../../../tests/testProductLieGroup.cpp) and
[`testActionProductLieGroup.cpp`](../../../tests/testActionProductLieGroup.cpp).
