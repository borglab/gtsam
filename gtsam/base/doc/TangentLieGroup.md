# `TangentLieGroup`: A Lie Group Acting on Its Algebra

`TangentLieGroup<G>` constructs the tangent group `TG = G ⋉ 𝔤`, where a Lie
group acts on its own Lie algebra through the adjoint action. It is a dedicated
class with all standard GTSAM Lie-group operations and optional Jacobians.

Include it with:

```cpp
#include <gtsam/base/TangentLieGroup.h>
```

An element `(g,xi)` stores `g` in `.first` and the algebra vector `xi` in
`.second`. The first element acts on the second through `Ad_g`:

```text
(g1,xi1) * (g2,xi2) = (g1*g2, xi1 + Ad_g1*xi2)
(g,xi)^-1            = (g^-1, -Ad_(g^-1)*xi)
```

`G` must have a fixed-dimensional tangent space and provide both the group
adjoint `AdjointMap()` and static algebra adjoint `adjointMap(u)`.

Typical aliases are:

```cpp
using TGSO3 = gtsam::TangentLieGroup<gtsam::Rot3>;  // dimension 6
using TGSE3 = gtsam::TangentLieGroup<gtsam::Pose3>; // dimension 12
using TGGal3 = gtsam::TangentLieGroup<gtsam::Gal3>; // dimension 20
using TGTGSE3 = gtsam::TangentLieGroup<TGSE3>;      // dimension 24
```

## Exponential and logarithm

Write a tangent vector as `[u;v]`, and let `g = Exp_G(u)`. If `Jr_G(u)` is the
base group's right Jacobian, then its left Jacobian is
`Jl_G(u) = Ad_g*Jr_G(u)`. The tangent-group exponential reuses these base-group
quantities:

```text
Exp_TG(u,v) = (g, Jl_G(u)*v)
            = (g, Ad_g*Jr_G(u)*v)
```

The inverse operation uses the base Logmap Jacobian
`Jr_G(u)^-1`:

```text
Log_TG(g,h) = (u, Jr_G(u)^-1*Ad_(g^-1)*h),  u = Log_G(g)
```

Reusing the base group's optimized Expmap and Logmap derivatives avoids a
separate matrix exponential and linear solve for tangent-group values.

## Algebra and group adjoints

For `(u,v)` in the tangent-group algebra, the algebra adjoint has a repeated
lower-triangular structure:

```text
ad_(u,v) = [ ad_u   0   ]
           [ ad_v  ad_u ]
```

For a group element `(g,xi)`, the group adjoint is:

```text
Ad_(g,xi) = [ Ad_g             0   ]
            [ ad_xi*Ad_g      Ad_g ]
```

These operations make tangent groups valid bases for recursively constructed
tangent groups such as `TangentLieGroup<TangentLieGroup<Pose3>>`.

## Structured right Jacobian

Applying `phi1` to the algebra adjoint preserves its triangular form:

```text
Jr_TG(u,v) = [ J  0 ]
             [ Q  J ]

J = Jr_G(u) = phi1(-ad_u)
Q = L_phi1(-ad_u, -ad_v)
```

The generic implementation computes the single directional Fréchet block `Q`
with one fixed-size `3n × 3n` augmented exponential, rather than applying a
generic `4n × 4n` exponential to the complete tangent-group algebra. Logmap
inverts the same structure blockwise:

```text
Jr_TG^-1 = [ J^-1                0    ]
            [ -J^-1*Q*J^-1      J^-1 ]
```

This path is shared by tangent groups based on `Rot3`, `Pose3`, `Gal3`, direct
products, and other compatible fixed-dimensional Lie groups.

## Closed form for `TangentLieGroup<Rot3>`

Under the standard `[omega;v]` coordinates, `TangentLieGroup<Rot3>` is
isomorphic to `Pose3`: the algebra component `v` is the translation. The
`Rot3` implementation therefore specializes the tangent-group right Jacobian
with the existing SO(3) dexp kernel, producing the same block Jacobian as
`Pose3::Expmap` without an augmented matrix exponential.

The specialization is an implementation detail; the public API and numerical
conventions are identical to the generic tangent-group path.

## API and usage

`TangentLieGroup` exposes the complete Lie-group interface, including
`Identity`, composition,
inverse, `between`, `retract`, `localCoordinates`, `Expmap`, `Logmap`,
`AdjointMap`, and static `adjointMap`. The concatenated tangent ordering is
always the base tangent followed by the transported algebra component.

Use a tangent group when a state contains a Lie-group element together with an
algebra-valued quantity that transforms through the group's adjoint action. See
[`SemidirectLieGroup`](SemidirectLieGroup.md) for general action-coupled
products. The implementation and its analytic Jacobians are exercised by
[`testTangentLieGroup.cpp`](../../../tests/testTangentLieGroup.cpp).
