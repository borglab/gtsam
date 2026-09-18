# `ProductLieGroup`: Direct Products

`ProductLieGroup<G,H>` constructs the independent direct product `G × H`
from two existing Lie groups. It provides GTSAM's standard group, manifold,
Lie-group, adjoint, and optional-Jacobian interfaces.

Both component types must satisfy the requirements in the
[`LieGroup` guide](LieGroup.md). Include the class with:

```cpp
#include <gtsam/base/ProductLieGroup.h>
```

## Usage and operations

```cpp
using RotationPose = gtsam::ProductLieGroup<gtsam::Rot3, gtsam::Pose2>;

const RotationPose x(gtsam::Rot3::Rz(0.2), gtsam::Pose2(1.0, 2.0, 0.3));
const gtsam::Rot3& rotation = x.first;
const gtsam::Pose2& pose = x.second;
```

All operations are componentwise:

```text
(g1,h1) * (g2,h2) = (g1*g2, h1*h2)
(g,h)^-1           = (g^-1, h^-1)
Exp(u,v)           = (Exp_G(u), Exp_H(v))
Log(g,h)           = (Log_G(g), Log_H(h))
```

The tangent vector concatenates the component tangents as `[u;v]`. The group
and algebra adjoints are block diagonal:

```text
Ad_(g,h) = diag(Ad_g, Ad_h)
ad_(u,v) = diag(ad_u, ad_v)
```

Eigen vector-space factors are abelian, so their algebra-adjoint block is
zero. All optional Jacobians follow GTSAM's right-Jacobian convention.

## Dynamic dimensions

Fixed-dimensional components produce fixed-size tangent vectors and
Jacobians. Dynamically sized components are also supported, with dimensions
obtained from the stored values.

The one-vector `Expmap([u;v])` and `adjointMap([u;v])` overloads can infer the
split when at most one component is dynamically sized. If both components are
dynamic, use `Expmap(u,v)`. For two dynamic vector spaces, `adjointMap` is
unambiguously zero.

## Related constructions

Use [`SemidirectLieGroup`](SemidirectLieGroup.md) when the first group acts on
the second component during composition. Use
[`TangentLieGroup`](TangentLieGroup.md) for the important adjoint-action
construction `G ⋉ 𝔤`, which has additional structured kernels.

The implementation is exercised by
[`testProductLieGroup.cpp`](../../../tests/testProductLieGroup.cpp).
