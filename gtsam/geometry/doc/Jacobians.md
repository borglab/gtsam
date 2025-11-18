# Jacobians via Kernels...

#### ...and their connection with Fréchet Derivatives
Frank Dellaert, August 2025

## Part 1 - SO(3) kernels

This note shows how to compute Jacobians for SO(3) - and later for semidirect products like SE(3), SE₂(3), and Gal(3) - using a compact **kernel** representation. The key is that many SO(3) operators are polynomials in \(\Omega=[\omega]_\times\):
\[
K(\omega) \;=\; a\,I \; \pm \; b(\theta)\,\Omega \; + \; c(\theta)\,\Omega^2,\qquad \theta=\|\omega\|.
\]
We call this the **kernel**. Once we can apply \(K(\omega)\) and its **closed-form derivative with respect to \(\omega\)**, we can reuse it everywhere.

### SO(3) essentials
Let \(\Omega = [\omega]_\times\), \(\theta=\|\omega\|\). Define (Rodrigues/Eade)
\[
A=\frac{\sin\theta}{\theta},\qquad
B=\frac{1-\cos\theta}{\theta^2},\qquad
C=\frac{1-A}{\theta^2},\qquad
G=\frac{1-2B}{2\theta^2}.
\]
Small-angle series: \(A=1-\theta^2/6+\cdots\), \(B=1/2-\theta^2/24+\cdots\), \(C=1/6-\theta^2/120+\cdots\), \(G=1/24-\theta^2/720+\cdots\).

Three important kernels:
- **Rodrigues kernel**: \(C(\omega) = \exp(\pm\Omega) = I \pm A\,\Omega + B\,\Omega^2\).
- **Jacobian kernel**: \(J(\omega) = I \pm B\,\Omega + C\,\Omega^2\).
- **Position/second kernel**: \(\Gamma(\omega) = \tfrac12 I \pm C\,\Omega + G\,\Omega^2\).

### Applying a kernel: y = K(ω) v
For any \(v\in\mathbb R^3\), we can either explicitly calculate the $3\times 3$ matrix $K(\omega)$ and multiply,
\[
\boxed{\;k(\omega,v) \;=\; K(\omega) v \,=\, a v + b\,(\Omega v) + c\,(\Omega^2 v).\;}
\]
*or* make use of the fact that \(\Omega v = \omega\times v\) and \(\Omega^2 v = \omega\times(\omega\times v)= (\omega\cdot v)\,\omega - \|\omega\|^2 v\) and write
\[
k(\omega, v) = a v + b (\omega \times v) + c \big((\omega \cdot v)\,\omega - \|\omega\|^2 v\big).
\]

### Closed-form derivatives of K(ω) v
Denote the **radial derivatives** \(d_b = b'(\theta)/\theta\), \(d_c = c'(\theta)/\theta\). Then the Jacobians of $k(.,)$ in its two arguments are:
\[
\boxed{\;\frac{\partial y}{\partial \omega}
= (d_b\,\Omega v + d_c\,\Omega^2 v)\,\omega^\top\; -\,b\,[v]_\times
\;+\; c\big(\,\omega v^\top + (\omega\!\cdot\! v)\,I - 2 v\,\omega^\top\big)
\;}
\]
\[
\boxed{\;\frac{\partial y}{\partial v}
= a I + b\,\Omega + c\,\Omega^2\;\equiv\; K(\omega).\;}
\]

### GTSAM Kernel API
The GTSAM implementation is centered around two concepts in the `gtsam::so3` namespace: `ExpmapFunctor` and `DexpFunctor`.

The `so3::ExpmapFunctor` class is a context object, constructed for a specific rotation vector `omega`. It efficiently caches geometric quantities like \(\theta\), \(\Omega=[\omega]_\times\), and \(\Omega^2\), and provides the rotation matrix \(R(\omega)\) via `expmap()`.

The `so3::DexpFunctor` class builds on `ExpmapFunctor` by lazily computing the coefficients \(A, B, C, \dots\) and their derivatives when needed. It provides the kernels for various SO(3) operations:
- `Rodrigues()`: returns the kernel for the SO(3) exponential map $\exp(\pm \Omega)$.
- `Jacobian()`: returns the kernel for the SO(3) Jacobians \(J_\ell, J_r\).
- `InvJacobian()`: returns the kernel for the inverse Jacobians  \(J_\ell^{-1}, J_r^{-1}\).
- `Gamma()`: returns the kernel for the "second-order" \(\Gamma\) matrices.

The `so3::Kernel` struct, produced by `DexpFunctor`, holds the \(a, b, c\) coefficients and their radial derivatives. It provides the core functionality for applying kernels and computing their derivatives.
```c++
struct GTSAM_EXPORT Kernel {
  // ... fields a, b, c, db, dc ...

  Matrix3 left() const;   // a I + b W + c WW
  Matrix3 right() const;  // a I - b W + c WW

  Vector3 applyLeft(const Vector3& v, OptionalJacobian<3, 3> Hw = {},
                    OptionalJacobian<3, 3> Hv = {}) const;
  Vector3 applyRight(const Vector3& v, OptionalJacobian<3, 3> Hw = {},
                     OptionalJacobian<3, 3> Hv = {}) const;
};
```
  
### Using the kernels in Other groups: SE(3) example
In SE(3), we reuse the SO(3) **kernels**. The kernel viewpoint emphasizes that the off-diagonal block is just the derivative \(dK(\omega)/d\omega\) from Part 1 applied to the translation vector. In the API, this is exactly what the `apply()` method returns in its Jacobian `H1`.

The **right Jacobian** has the block form:
\[
J_r^{SE(3)}(\omega,\rho)
= \begin{pmatrix}
J_r(\omega) & 0\\[4pt]
Q_r(\omega,\rho) & J_r(\omega)
\end{pmatrix}.
\]
The off-diagonal block \(Q_r\) is obtained by applying the SO(3) kernel \(J_r\) to the translation vector and taking the derivative with respect to \(\omega\). In the API, this corresponds to calling `apply()` on the kernel, with the Jacobian `H1` capturing the derivative for the off-diagonal block. This approach leverages the closed-form kernel derivatives developed in Part 1, requiring no additional series or integrals.

Additionally, constructing an SE(3) object from a rotation matrix and a translation introduces another \(R^\top\). This leads to the following expression for the \(Q_r\) block:
\[
Q_r(\omega, \rho) = R^\top \cdot \mathcal{L}_{J_r}(\Omega)[-[\rho]_\times],
\]
where \(Q_r\) is expressed in the body frame, and \(R^T = \exp(-\Omega)\) ensures the correct frame transformation.

## Part 2 - A Cookbook for Group Jacobians

With the theoretical foundation from Part 1 in place, we now have a powerful recipe. We know that for semidirect products, the Jacobians are block-triangular, and the off-diagonal blocks are given by the Fréchet derivative of the corresponding $SO(3)$ kernel. This allows us to construct Jacobians for complex groups using our simple kernel building blocks.

This section provides a practical "cookbook" for several common groups. For each group, we will:
1.  Define the tangent vector $\xi$ and the exponential map $\exp(\xi)$.
2.  Show the resulting block structure of the right Jacobian $J_r(\xi)$.
3.  Provide the formulas for each block.

### $\text{SE}(3)$ - Special Euclidean Group

The group of rigid body motions. This section uses the convention adopted by GTSAM for its `Pose3` class, which is physically motivated by integrating body-centric velocities.

-   **Tangent Vector:** $\xi = (\omega, v) \in \mathbb{R}^6$, where $\omega$ is angular velocity and $v$ is linear velocity, both expressed in the body frame.
-   **Exponential Map:** The map uses the **left Jacobian** of $\text{SO}(3)$ to compute the final translation, which corresponds to integrating a body-fixed velocity $v$.
    $$
    \exp(\xi) = (R, t) \quad \text{where} \quad R = \exp(\omega), \quad t = J_l(\omega)v
    $$
-   **Right Jacobian Structure:** We aim to compute the **right Jacobian** $J_r(\xi)$, which relates perturbations in the body frame to the final pose. This is the standard convention for uncertainty propagation in GTSAM.
    $$
    J_r(\xi) =
    \begin{pmatrix}
    J_r(\omega) & 0 \\
    Q_r(\omega, v) & J_r(\omega)
    \end{pmatrix}
    $$
-   **Block Formulas:** The diagonal blocks are the standard right Jacobian of $\text{SO}(3)$, $J_r(\omega)$. The off-diagonal block $Q_r$ requires a two-step process to derive:
    1.  **World-Frame Derivative ($Q_l$):** First, we compute the derivative of the translation $t$ with respect to the rotation $\omega$. Since $t$ is a point in the world frame, this derivative is also in the world frame. This is, by definition, the off-diagonal block of the **left Jacobian** of $\text{SE}(3)$, which we can call $Q_l$. It is computed using the Fréchet derivative of the $J_l$ kernel.
        $$
        Q_l = \frac{\partial t}{\partial \omega} = \frac{\partial (J_l(\omega)v)}{\partial \omega} = \mathcal{L}_{J_l}(\Omega)[-[v]_\times]
        $$
        In the GTSAM API, this is `local.Jl().applyFrechet(v)`, or obtained via the `OptionalJacobian` argument of `applyLeftJacobian`.

    2.  **Frame Transformation:** To get the required $Q_r$ for the right Jacobian, we must transform this world-frame derivative into the body frame. This is done by left-multiplying by $R^T = \exp(-\Omega)$.
        $$
        Q_r(\omega, v) = R^T \cdot Q_l = \exp(-\Omega) \cdot \mathcal{L}_{J_l}(\Omega)[-[v]_\times]
        $$
        This two-step process is the correct and principled way to compute the right Jacobian for this choice of exponential map.

### $\text{SE}_2(3)$ - The `NavState`

The `NavState` class in GTSAM implements a specific, influential definition of the $SE_2(3)$ Lie group from the robotics literature (e.g., Barrau, 2012).

This group models a state with three components—rotation, position, and velocity—that evolve in parallel, swept along by the rotation.

> **Note**: *currently* in GTSAM we use $T = (R,t,v)$ as opposed to the $(R,v,t)$ convention from Barrau et al. We plan to switch to that more accepted convention in the future.

-   **Tangent Vector:** $\xi = (\omega, \rho, \nu) \in \mathbb{R}^9$. To align with `NavState`, we use the convention where $\omega$ is angular velocity, $\rho$ is the tangent for position, and $\nu$ is the tangent for velocity.

-   **Exponential Map:** The group structure is defined by its Lie algebra `Hat` operator, which has a "parallel transport" structure with zeros in the lower-left block.
    $$
    \hat{\xi} := \begin{pmatrix} [\omega]_\times & \rho & \nu \\ 0 & 0 & 0 \\ 0 & 0 & 0 \end{pmatrix}
    $$
    The exponential map is the standard matrix exponential of this matrix, $\exp(\xi) := \exp_m(\hat{\xi})$. This yields the closed-form expression used in `NavState`:
    $$
    \exp(\xi) = (R, t, v) \quad \text{where} \quad
    \begin{cases}
    R = \exp(\omega) \\
    t = J_l(\omega)\rho \\
    v = J_l(\omega)\nu
    \end{cases}
    $$

-   **Right Jacobian Structure:** We compute the right Jacobian $J_r(\xi)$ for this group. The tangent vector ordering is $(\omega, \rho, \nu)$ and the state ordering is $(R, t, v)$.
    $$
    J_r(\xi) =
    \begin{pmatrix}
    J_r(\omega) & 0 & 0 \\
    (J_r)_{t,\omega} & (J_r)_{t,\rho} & 0 \\
    (J_r)_{v,\omega} & 0 & (J_r)_{v,\nu}
    \end{pmatrix}
    $$

-   **Block Formulas:**
    -   **Diagonal Blocks:** The diagonal blocks represent the influence of a tangent component on its corresponding state variable, expressed in the body frame. This requires rotating the left Jacobian by $R^T$.
        $$
        (J_r)_{t,\rho} = (J_r)_{v,\nu} = R^T J_l(\omega) = J_r(\omega)
        $$
    -   **Off-Diagonal Blocks (Rotation Coupling):** These blocks capture the influence of rotation on translation and velocity. The logic is identical to `SE(3)`: we compute the world-frame derivative (the left-Jacobian's Fréchet derivative) and rotate it into the body frame.
        $$
        (J_r)_{t,\omega} = R^T \cdot \frac{\partial(J_l(\omega)\rho)}{\partial\omega} = R^T \cdot \mathcal{L}_{J_l}(\Omega)[-[\rho]_\times]
        $$
        $$
        (J_r)_{v,\omega} = R^T \cdot \frac{\partial(J_l(\omega)\nu)}{\partial\omega} = R^T \cdot \mathcal{L}_{J_l}(\Omega)[-[\nu]_\times]
        $$

### $\text{Gal}(3)$ - The Galilean group

The `Gal3` class implements Galilean relativity, adding time to $SE_2(3)$.  Its Jacobian, as implemented and tested in GTSAM, is the result of formal Lie-theoretic derivations.

-   **Tangent Vector:** $\xi = (\omega, \nu, \rho, \alpha) \in \mathbb{R}^{10}$.
    -   $\omega$: angular velocity
    -   $\nu$: velocity tangent
    -   $\rho$: position tangent
    -   $\alpha$: time interval

-   **Exponential Map:** State Order = $(R, v, p, t)$.
    $$
    \exp(\xi) = (R, v, p, t) \quad \text{where} \quad
    \begin{cases}
    R = \exp(\omega) \\
    v = J_l(\omega)\nu \\
    p = J_l(\omega)\rho + \alpha \, \Gamma_l(\omega)\nu \\
    t = \alpha
    \end{cases}
    $$

-   **Right Jacobian Structure:** We compute the right Jacobian $J_r(\xi)$. The tangent order is $(\omega, \nu, \rho, \alpha)$ and the internal state order is $(R, v, p, t)$.
    $$
    J_r(\xi) =
    \begin{pmatrix}
    J_r(\omega) & 0 & 0 & 0 \\
    (J_r)_{v,\omega} & J_r(\omega) & 0 & 0 \\
    (J_r)_{p,\omega} & (J_r)_{p,\nu} & J_r(\omega) & (J_r)_{p,\alpha} \\
    (J_r)_{t,\omega} & (J_r)_{t,\nu} & (J_r)_{t,\rho} & 1
    \end{pmatrix}
    $$

-   **Block Formulas (As Implemented in GTSAM):**
    -   **Row 2 (v):** is just as in $\text{SE}_2(3)$:
        $$
        (J_r)_{v,\omega} = R^T \cdot \mathcal{L}_{J_l}(\Omega)[-[\nu]_\times]
        $$
    -   **Row 3 (p):** The first block contains two straightforward Fréchet derivatives applications, rotated back:
        $$
        (J_r)_{p,\omega} = R^T \left( \mathcal{L}_{J_l}(\Omega)[-[\rho]_\times] + \alpha \mathcal{L}_{\Gamma_l}(\Omega)[-[\nu]_\times] \right)
        $$

        The second is straightforward, also rotated back:
        $$
        (J_r)_{p,\nu} = R^T \cdot \alpha \, \Gamma_l(\omega)
        $$
        
        This derivative might be unexpected but can be proven when considering the group action of $\omega, \alpha$ on $r$:
        $$
        (J_r)_{p,\alpha} = -\Gamma_r(\omega)\nu
        $$

### $Sim(3)$ - The Similarity Group (GTSAM Convention)

The `Similarity3` class in GTSAM models transformations involving rotation, translation, and uniform scaling. Its exponential map is defined by a specialized, scale-aware kernel, implemented in the `VFunctor`. This functor dynamically creates an kernel adapted to the specific rotation $\omega$ and log-scale $\lambda$.

-   **The Scale-Aware Kernel:** The `VFunctor` calculates three coefficients, `P`, `Q`, and `R`, which are functions of both the rotation angle $\theta = \|\omega\|$ and the log-scale $\lambda$. These coefficients define a specialized "left-acting" kernel `V_l` that follows the standard polynomial structure:
    $$
    V_l(\omega, \lambda) = P(\theta, \lambda)I + Q(\theta, \lambda)\Omega + R(\theta, \lambda)\Omega^2
    $$
    This kernel is defined by the integral $V_l = \int_0^1 \exp(s\lambda) \exp(s\Omega) ds$. The `VFunctor` provides a closed-form evaluation of this integral.

-   **Tangent Vector:** $\xi = (\omega, u, \lambda) \in \mathbb{R}^7$, where $\omega$ is angular velocity, $u$ is the tangent for translation, and $\lambda$ is the log-scale.

-   **Exponential Map:** The map uses the scale-aware kernel $V_l$ to compute the world-frame translation.
    $$
    \exp(\xi) = (R, p, s) \quad \text{where} \quad
    \begin{cases}
    R = \exp(\omega) \\
    p = V_l(\omega, \lambda)u \\
    s = \exp(\lambda)
    \end{cases}
    $$

-   **Right Jacobian Structure:** We compute the right Jacobian $J_r(\xi)$. It is a 7x7 matrix with the following block structure, corresponding to the tangent order $(\omega, u, \lambda)$:
    $$
    J_r(\xi) =
    \begin{pmatrix}
    J_r(\omega) & 0 & 0 \\
    (J_r)_{p,\omega} & (J_r)_{p,u} & (J_r)_{p,\lambda} \\
    0 & 0 & 1
    \end{pmatrix}
    $$

-   **Block Formulas:** The logic remains consistent: compute the world-frame derivative of the `Expmap`, then rotate it into the body frame with $R^T$.

    -   **$(J_r)_{p,u}$ (Translation Diagonal):** The partial derivative of $p = V_l u$ with respect to $u$ is simply the kernel matrix $V_l$. We then rotate it into the body frame.
        $$
        (J_r)_{p,u} = R^T \cdot \frac{\partial p}{\partial u} = R^T V_l(\omega, \lambda)
        $$

    -   **$(J_r)_{p,\omega}$ (Rotation Coupling):** This is the Fréchet derivative with respect to $\omega$. The `VFunctor::kernel()` method explicitly computes the necessary radial derivatives (`dQ`, `dR`) to construct a valid `so3::ABCKernel`. This allows us to directly apply the Fréchet machinery to our specialized kernel.
        $$
        (J_r)_{p,\omega} = R^T \cdot \frac{\partial p}{\partial \omega} = R^T \cdot \mathcal{L}_{V_l}(\omega, \lambda)[-[u]_\times]
        $$

    -   **$(J_r)_{p,\lambda}$ (Scale Coupling):** This is the derivative with respect to the scalar $\lambda$. We must differentiate the kernel itself with respect to $\lambda$:
        $$
        \frac{\partial V_l}{\partial \lambda} = \frac{\partial P}{\partial \lambda}I + \frac{\partial Q}{\partial \lambda}\Omega + \frac{\partial R}{\partial \lambda}\Omega^2
        $$
        This defines a **new kernel**, let's call it $W_l(\omega, \lambda)$, whose coefficients are the partial derivatives of the `P, Q, R` coefficients with respect to $\lambda$. The final Jacobian block is this new kernel applied to `u` and rotated into the body frame.
        $$
        (J_r)_{p,\lambda} = R^T \cdot \frac{\partial p}{\partial \lambda} = R^T \cdot W_l(\omega, \lambda) u
        $$
        This demonstrates how the derivatives for `Sim(3)` require two distinct derivative operations on the underlying `VFunctor` coefficients: radial derivatives for the $\omega$ Jacobian and partial derivatives with respect to $\lambda$ for the scale Jacobian.
        
### API

Mainly for unit-testing, the ABCKernel API also defines:
```c++

  /// Fréchet derivative of left-kernel K(ω) in the direction X ∈ so(3)
  /// L_M(Ω)[X] = b X + c (Ω X + X Ω) + s (db Ω + dc Ω²), with s = -½ tr(Ω X)
  Matrix3 frechet(const Matrix3& X) const;

  /// Apply Fréchet derivative to vector (left specialization)
  Matrix3 applyFrechet(const Vector3& v) const;
```

A call to `applyFrechet` should match the `H1` derivative of `Jacobian().left().apply()`.

## References
- Eade, *Lie Groups for 2D and 3D Transformations*.
- Chirikjian, *Stochastic Models, Information Theory, and Lie Groups*, Vol. 2.
- Barfoot, *State Estimation for Robotics*.
- GTSAM sources (`ExpmapFunctor`, `ABCKernel`, `DexpFunctor`, `GammaFunctor`).
