# Jacobians via Kernels...

#### ...and their connection with Fréchet Derivatives
Frank Dellaert, August 2025

## Part 1 - SO(3) kernels

This note shows how to compute Jacobians for SO(3) - and later for semidirect products like SE(3), SE₂(3), and Gal(3) - using a compact **kernel** representation. The key is that many SO(3) operators are polynomials in \(\Omega=[\omega]_\times\):
\[
M(\omega) \;=\; a\,I \; + \; b(\theta)\,\Omega \; + \; c(\theta)\,\Omega^2,\qquad \theta=\|\omega\|.
\]
We call this the **kernel**. Once we can apply \(M(\omega)\) and its **closed-form derivative with respect to \(\omega\)**, we can reuse it everywhere.

### SO(3) essentials
Let \(\Omega = [\omega]_\times\), \(\theta=\|\omega\|\). Define (Rodrigues/Eade)
\[
A=\frac{\sin\theta}{\theta},\qquad
B=\frac{1-\cos\theta}{\theta^2},\qquad
C=\frac{1-A}{\theta^2},\qquad
G=\frac{1-2B}{2\theta^2}.
\]
Small-angle series: \(A=1-\theta^2/6+\cdots\), \(B=1/2-\theta^2/24+\cdots\), \(C=1/6-\theta^2/120+\cdots\), \(G=1/24-\theta^2/720+\cdots\).

Typical kernels:
- **SO(3) Jacobians**: \(J_\ell = I + B\,\Omega + C\,\Omega^2\), \(J_r = I - B\,\Omega + C\,\Omega^2\).
- **Position/second kernel**: \(\Gamma_\ell = \tfrac12 I + C\,\Omega + G\,\Omega^2\), \(\Gamma_r = \tfrac12 I - C\,\Omega + G\,\Omega^2\).

### Apply the kernel: y = M(ω) v
For any \(v\in\mathbb R^3\):
\[
\boxed{\;y(\omega,v) \;=\; M(\omega) v \,=\, a v + b\,(\Omega v) + c\,(\Omega^2 v).\;}
\]
We will also need \(\Omega v = \omega\times v\) and \(\Omega^2 v = \omega\times(\omega\times v)= (\omega\cdot v)\,\omega - \|\omega\|^2 v\).

### Closed-form derivatives of M(ω) v
Let \(y(\omega,v)=a v + b\,\Omega v + c\,\Omega^2 v\). Denote the **radial derivatives** \(d_b = b'(\theta)/\theta\), \(d_c = c'(\theta)/\theta\). Then the Jacobians are:
\[
\boxed{\;\frac{\partial y}{\partial \omega}
= -\,b\,[v]_\times
\;+\; c\big(\,\omega v^\top + (\omega\!\cdot\! v)\,I - 2 v\,\omega^\top\big)
\;+\; (d_b\,\Omega v + d_c\,\Omega^2 v)\,\omega^\top\;}
\]
\[
\boxed{\;\frac{\partial y}{\partial v}
= a I + b\,\Omega + c\,\Omega^2\;\equiv\; M(\omega).\;}
\]
**Derivation sketch.** Differentiate each term using \(\partial(\Omega v)/\partial\omega = -[v]_\times\) and
\(\partial(\Omega^2 v)/\partial\omega = \omega v^\top + (\omega\!\cdot\! v)I - 2 v\,\omega^\top\). Radial dependence enters via \(\partial b/\partial\omega = d_b\,\omega\), \(\partial c/\partial\omega = d_c\,\omega\), producing the rank-1 outer product with \(\omega^\top\).

### GTSAM Kernel API

The GTSAM implementation is centered around two concepts in the `gtsam::so3` namespace: `Local` and `Kernel`.

The `so3::Local` class is a context object, constructed for a specific rotation vector `omega`. It efficiently caches geometric quantities like \(\theta\), \(\Omega=[\omega]_\times\), and \(\Omega^2\), and lazily computes the `A, B, C, ...` coefficients and their derivatives when needed.

`so3::Local` then acts as a factory for `so3::Kernel` objects:
- `local.expmap()`: returns the rotation matrix \(R(\omega)\).
- `local.Jacobian()`: returns the kernel for the SO(3) Jacobians \(J_\ell, J_r\).
- `local.InvJacobian()`: returns the kernel for the inverse Jacobians.
- `local.Gamma()`: returns the kernel for the "second-order" \(\Gamma\) matrices.

The `so3::Kernel` struct holds the \(a,b,c\) coefficients and their radial derivatives. It provides the core functionality:
```c++
struct GTSAM_EXPORT Kernel {
  // ... fields a, b, c, db, dc ...

  Matrix3 left() const;   // a I + b W + c WW
  Matrix3 right() const;  // a I - b W + c WW

  Vector3 applyLeft(const Vector3& v, OptionalJacobian<3, 3> Hw = {},
                    OptionalJacobian<3, 3> Hv = {}) const;
  Vector3 applyRight(const Vector3& v, OptionalJacobian<3, 3> Hw = {},
                     OptionalJacobian<3, 3> Hv = {}) const;

  // Shortcuts: K * v == left,  v * K == right
  Vector3 operator*(const Vector3& v) const;
  friend Vector3 operator*(const Vector3& v, const Kernel& K);
};
```
  
**Summary.** The kernel view gives three primitives:
- `operator*` for fast application \(v \mapsto M(\omega)v\),
- `matrix()` to materialize \(M(\omega)\), and
- `apply(·)` allows you to obtain off-diagonal blocks via H1 and H2.
No series or integrals are required to implement these.

### Using the kernels in Other groups: SE(3) example

In SE(3) (ω-first ordering), we reuse the SO(3) **kernels** from Part 1. The kernel viewpoint emphasizes that the off-diagonal block is just the derivative \(dM(\omega)/d\omega\) from Part 1 applied to the translation vector. In the API, this is exactly what the `apply()` method returns in its Jacobian `H1`.

The **right** and **left** Jacobians have the block forms
\[
J_r^{SE(3)}(\omega,\rho)
= \begin{pmatrix}
J_r(\omega) & 0\\[4pt]
Q_r(\omega,\rho) & J_r(\omega)
\end{pmatrix},
\qquad
J_\ell^{SE(3)}(\omega,\rho)
= \begin{pmatrix}
J_\ell(\omega) & 0\\[4pt]
Q_\ell(\omega,\rho) & J_\ell(\omega)
\end{pmatrix}.
\]
The off-diagonal blocks \(Q_r\) and \(Q_\ell\) are obtained by applying the corresponding SO(3) kernel from Part 1 (i.e., \(J_r\) or \(J_\ell\)) to the translation vector, and taking the derivative with respect to \(\omega\). In the API, this corresponds to calling `apply()` on the kernel, with the Jacobian `H1` capturing the derivative for the off-diagonal block. This approach leverages the closed-form kernel derivatives developed in Part 1, and requires no additional series or integrals.

---
## Part 2 - Connection with Fréchet Derivatives

Part 1 gave us a powerful and efficient "recipe": the kernel. We saw that by plugging in coefficients, we can compute $SO(3)$ Jacobians and their derivatives with simple, closed-form expressions.

You might now be asking: *Why does this work?* And more importantly, *how can we confidently use this simple $\text{SO}(3)$ tool to build the much more complex Jacobians for groups like $\text{SE}(3)$, $\text{SE}_2(3)$, and $\text{Gal}(3)$?*

This section bridges that gap. It will connect our practical kernel recipe to the formal definitions from Lie theory. The goal is **not** to force you through complex derivations, but to give you the high-level understanding of *why* the kernel approach is correct and powerful. We will see that the intimidating integrals that formally define Jacobians have an elegant, closed-form solution for our kernels, and this solution is precisely the **Fréchet derivative**.

### An integral identity for $\exp$

We can also define the exponential map by defining a trajectory \( g(s) = \exp(s\,\xi) \), which satisfies 
- $\dot g (s) = \frac{d}{ds} \exp(s\,\xi) = \exp(s\,\xi)\,\xi$
-  \( g(0) = I \). 

Hence we have the useful identity
\[
\exp(\xi) = I + \int_{0}^{1} \exp(s\,\xi)\,\xi \; ds.
\]

This holds for any Lie group.

### Defining Jacobians via the adjoint \( \mathrm{ad} \)

For any Lie group \(G\) with Lie algebra \(\mathfrak g\), the **adjoint map**
\[
\mathrm{ad}_\xi : \mathfrak g \to \mathfrak g
\]

is the linear operator defined by
\[
\mathrm{ad}_\xi(\eta) \;=\; [\xi,\,\eta],
\]

the Lie bracket of \(\xi\) with \(\eta\). It encodes “how the infinitesimal motion \(\xi\) changes another infinitesimal motion \(\eta\)”.

In Lie theory, the Jacobians of the exponential map are formally defined using an integral. These definitions are fundamental but computationally impractical to use directly. The key idea is the **adjoint map**, $\mathrm{ad}_\xi(\eta) = [\xi, \eta]$ (the Lie bracket), which captures how an infinitesimal motion $\xi$ affects another motion $\eta$.

The left and right Jacobians are then defined as the following integrals of the exponentiated adjoint operator:
$$
J_\ell(\xi)=\int_{0}^{1} \exp(s\,\mathrm{ad}_\xi)\,ds, \qquad J_r(\xi)=\int_{0}^{1} \exp(-s\,\mathrm{ad}_\xi)\,ds.
$$
Think of these integrals as a sophisticated "averaging" process along the Lie group trajectory generated by $\xi$. While the full derivation (see optional box below) is involved, the critical takeaway for us is that **this is the formal ground truth**. Any closed-form expression we use, like our kernel, must be equivalent to this definition.

The (optional) exposition in the box below explains why.

>
> Let \(\xi \in \mathbb{R}^n\) be coordinates in the Lie algebra, and \(\hat{\xi} \in \mathfrak{g}\) its matrix form (e.g., skew-symmetric for \(\mathfrak{so}(3)\)). The derivative \(D\exp(\hat{\xi})[\hat{\eta}]\) tells us how \(\exp(\hat{\xi})\) changes when we perturb \(\hat{\xi}\) in the direction \(\hat{\eta}\).
>
> This derivative lives in the tangent space *at* \(\exp(\hat{\xi})\), but we want to express it back *at the identity* so it can be written as a matrix acting directly on the coordinate vector \(\eta\).
>
> Definition: The **right Jacobian** \(J_r(\xi)\) is the linear map such that
> \[
> \boxed{J_r(\xi)\,\eta \;:=\; D\exp(\hat{\xi})[\hat{\eta}]\,\exp(-\hat{\xi})}
> \]
>
> That is, we compute the derivative at \(\exp(\hat{\xi})\) and then *right-translate* it back to the identity by multiplying on the right by \(\exp(-\hat{\xi})\).
>
> Using the integral identity for the exponential map, differentiating in the direction \(\hat{\eta}\) gives
> \[
> D\exp(\hat{\xi})[\hat{\eta}] = \int_0^1 \exp(s\,\hat{\xi})\,\hat{\eta}\; ds,
> \]
> 
> hence
> \[
> J_r(\xi)\,\eta
> = \left( \int_0^1 \exp(s\,\hat{\xi})\,\hat{\eta}\; ds \right) \exp(-\hat{\xi}).
> \]
> 
> Using conjugation and a change of variables $t=1-s$ we arrive at
> \[
> J_r(\xi)\,\eta
> = \int_0^1 \mathrm{Ad}_{\exp(-t\,\hat{\xi})}(\hat{\eta})\; dt.
> \]
>
> In operator form, this reveals the right Jacobian as
> \[
> \boxed{J_r(\xi) = \int_0^1 \exp(-t\,\mathrm{ad}_{\hat{\xi}})\; dt}
> \]
>
> which can be seen as an *average of infinitesimal conjugations* along the path \(s \mapsto \exp(s\,\hat{\xi})\).


## The Jacobians for semidirect product groups

The real power of this formalism appears when we look at semidirect product groups like $\text{SE}(3) = SO(3) \ltimes \mathbb{R}^3$. An element of the Lie algebra is a pair $\xi = (\omega, v)$. The magic is that the adjoint operator $\mathrm{ad}_\xi$ for these groups has a **block lower-triangular structure**:
$$
\mathrm{ad}_{(\omega,v)} =
\begin{pmatrix}
\mathrm{ad}_\omega & 0 \\
\mathrm{ad}_v & \mathrm{ad}_\omega
\end{pmatrix}
$$
Because matrix functions (like the exponential and integral) preserve block-triangularity, the resulting Jacobian $J_r(\xi)$ will also be block lower-triangular:
$$
J_r(\omega,v)
= \int_0^1 \exp(-s\,\mathrm{ad}_{(\omega,v)})\,ds = \begin{pmatrix}
J_r(\omega) & 0 \\
Q_r(\omega,v) & J_r(\omega)
\end{pmatrix}
$$
This is a profound result! It tells us:
1.  **The diagonal blocks** are just the familiar $\text{SO}(3)$ Jacobians, $J_r(\omega)$.
2.  **The off-diagonal block $Q_r$** captures the coupling between rotation $\omega$ and the vector part $v$. It is this block that we need a simple way to compute.

#### The Fréchet Derivative: A Practical Tool for Matrix Derivatives

Before solving for $Q_r$, let's introduce our main computational tool. The **Fréchet derivative** is the generalization of the familiar derivative to functions that take a matrix as input and produce a matrix as output. For a function $M(\Omega)$, its Fréchet derivative $\mathcal{L}_M(\Omega)[X]$ tells us how $M$ changes in the linear direction $X$.

For our kernel $M(\omega) = aI + b(\theta)\Omega + c(\theta)\Omega^2$, the Fréchet derivative in a skew-symmetric direction $X = [\delta\omega]_\times$ has a clean, closed-form expression:
$$
\boxed{\mathcal{L}_{M}(\Omega)[X] = b\,X + c(\Omega X + X \Omega) + s\,(d_b\,\Omega + d_c\,\Omega^2), \quad s=-\tfrac{1}{2}\mathrm{tr}(\Omega X)}
$$
Notice this is just an algebraic formula using the same $b, c, d_b, d_c$ coefficients from Part 1. It is a concrete recipe, not an abstract concept.

#### The Connection: Fréchet Derivatives as the Solution to the Integral

Now we can state the central connection. The off-diagonal block $Q_r$ from the integral can be shown to be an operator acting on the vector part $v$. For semidirect products, this operator takes the form of the Fréchet derivative of the corresponding $\text{SO}(3)$ kernel, evaluated in the direction $X = -[v]_\times$.

**In other words, the Fréchet derivative is the closed-form evaluation of the complex $\mathrm{ad}$-integral for our family of kernels.**

Let's break this down for $\text{SE}(3)$:
1.  The formal definition gives the off-diagonal block $Q_r$ as a complicated integral involving $\mathrm{ad}_{(\omega,\rho)}$.
2.  Lie theory proves this integral is equivalent to applying a linear operator to the translation vector $\rho$.
3.  Our key insight is that this linear operator is *exactly* the Fréchet derivative of the $\text{SO}(3)$ Jacobian kernel, $J_r(\omega)$, acting on $\rho$.

$$
\text{Off-Diagonal Block for SE(3):} \quad Q_r(\omega, \rho) = \mathcal{L}_{J_r}(\Omega)[-[\rho]_\times]
$$

This is the punchline. We can completely bypass the complicated integrals. The Fréchet derivative, which has a simple formula for our kernels, gives us the exact off-diagonal blocks we need. This provides the theoretical justification for using our simple $\text{SO}(3)$ tools to build the Jacobians for more complex groups, which we will do in Part 3.

> **(Optional) Proof: How the Adjoint Integral is equal to the Fréchet Derivative**
> TBD

---
## Part 3 - A Cookbook for Group Jacobians

With the theoretical foundation from Part 2 in place, we now have a powerful recipe. We know that for semidirect products, the Jacobians are block-triangular, and the off-diagonal blocks are given by the Fréchet derivative of the corresponding $SO(3)$ kernel. This allows us to construct Jacobians for complex groups using our simple kernel building blocks.

This section provides a practical "cookbook" for several common groups. For each group, we will:
1.  Define the tangent vector $\xi$ and the exponential map $\exp(\xi)$.
2.  Show the resulting block structure of the right Jacobian $J_r(\xi)$.
3.  Provide the formulas for each block.

All formulas are **right-trivialized** unless marked. Left-trivialized versions are found by substituting $J_r \to J_\ell$ and $\Gamma_r \to \Gamma_\ell$.

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

    2.  **Frame Transformation:** To get the required $Q_r$ for the right Jacobian, we must transform this world-frame derivative into the body frame. This is done by left-multiplying by $R^T = \exp(-\omega)$.
        $$
        Q_r(\omega, v) = R^T \cdot Q_l = \exp(-\omega) \cdot \mathcal{L}_{J_l}(\Omega)[-[v]_\times]
        $$
        This two-step process is the correct and principled way to compute the right Jacobian for this choice of exponential map.

### $\text{SE}_2(3)$ - The `NavState`

The `NavState` class in GTSAM implements a specific, influential definition of the $SE_2(3)$ Lie group, sourced from the robotics literature (e.g., Barrau, 2012). It is crucial to understand that this definition is a **deliberate modeling choice** and differs from other canonical, physics-based definitions of a constant velocity model.

This group models a state with three components—rotation, position, and velocity—that evolve in parallel, swept along by the rotation. It is a mathematically consistent Lie group whose implementation in GTSAM is correct and verified.

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
    Critically, note that there is no $\Gamma_l$ term. The final position $t$ is independent of the velocity tangent $\nu$.

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
    -   **Zero Blocks:** The blocks $(J_r)_{t,\nu}$ and $(J_r)_{v,\rho}$ are zero. This is a direct and correct consequence of the group's definition: the exponential map for position $t$ does not depend on the velocity tangent $\nu$, and vice-versa.

This definition results in a simpler algebraic structure than the canonical "integrated velocity" model, which is advantageous for certain filtering applications and is the one faithfully implemented and tested in GTSAM.

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
        
        This derivative is unexpected:
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

  /// Fréchet derivative of left-kernel M(ω) in the direction X ∈ so(3)
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
