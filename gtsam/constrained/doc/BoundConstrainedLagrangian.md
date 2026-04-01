# BoundConstrainedLagrangian

This note explains `gtsam::BoundConstrainedLagrangian` using the augmented-Lagrangian convention from Nocedal and Wright, then describes the bound-constrained Lagrangian (BCL) globalization idea in those same conventions, and finally points to the corresponding code paths in `gtsam`.

## References and Conventions

This write-up follows the Nocedal and Wright augmented-Lagrangian convention from *Numerical Optimization*, Chapter 17, but uses our notation `h(x)` for equality constraints:

$$
\min_x \; \phi(x) = \frac{1}{2}\|r(x)\|^2
\quad \text{subject to} \quad h(x) = 0,
$$

with augmented Lagrangian

$$
\mathcal{L}_A(x, \lambda; \mu)
= \phi(x) - \lambda^\top h(x) + \frac{\mu}{2}\|h(x)\|^2.
$$

This is the same sign convention documented in [AugmentedLagrangian.h](../AugmentedLagrangian.h). Completing the square gives

$$
\mathcal{L}_A(x, \lambda; \mu)
= \phi(x) + \frac{\mu}{2}\left\|h(x) - \frac{\lambda}{\mu}\right\|^2 - \frac{1}{2\mu}\|\lambda\|^2.
$$

So each outer iteration approximately minimizes a shifted penalty problem, then decides whether the multiplier estimate is trustworthy enough to update. In these conventions, larger `mu` means a stronger quadratic penalty.

The main references for the discussion below are:

- Nocedal and Wright, *Numerical Optimization*, 2nd ed., Chapter 17, especially Equation 17.36 and Algorithm 17.4.
- Conn, Gould, and Toint, "A globally convergent augmented Lagrangian algorithm for optimization with general constraints and simple bounds," *SIAM Journal on Numerical Analysis*, 28(2):545-572, 1991.

At the textbook level, the outer loop is:

1. Fix $(\lambda_k, \mu_k)$.
2. Approximately minimize $\mathcal{L}_A(x, \lambda_k; \mu_k)$ to obtain $x_{k+1}$.
3. Decide, from stationarity and feasibility tests, whether to update the multiplier or instead strengthen the penalty.

Written in terms of `h(x)`, the usual accepted ALM multiplier update is

$$
\lambda_{k+1} = \lambda_k - \mu_k h(x_{k+1}).
$$

In the `gtsam` implementation, the update is written directly in terms of the stored constraint-violation vector returned by `whitenedError()`, so the assignment appears in C++ as `lambdaEq += muEq * violation`.

## BCL Globalization Strategy

The BCL idea from Conn, Gould, and Toint is not a different merit function. It is an outer-loop globalization rule for deciding when the multiplier iteration should be trusted and when the algorithm should fall back to a stronger penalty step.

In the present convention, a simple BCL-style iteration uses two outer tolerances:

- $\omega_k$ for approximate first-order stationarity of the inner subproblem
- $\eta_k$ for constraint feasibility

The practical logic is:

1. Approximately minimize the current augmented Lagrangian until the inner solve is accurate enough relative to $\omega_k$.
2. Check whether the resulting point is feasible enough, meaning

$$
\|h(x_{k+1})\|_\infty \le \eta_k.
$$

3. If both tests are satisfied, accept a multiplier update and keep the penalty fixed.
4. Otherwise, keep the multiplier estimate and increase the penalty parameter.

Written in a compact form with our convention,

$$ 
\|\nabla_x \mathcal{L}_A(x_{k+1}, \lambda_k; \mu_k)\|_\infty \le \omega_k
\quad \text{and} \quad
\|h(x_{k+1})\|_\infty \le \eta_k
$$

implies an accepted multiplier update, while failure of the feasibility test triggers a penalty update such as

$$
\mu_{k+1} = \kappa \mu_k,
\qquad \kappa > 1.
$$

The exact update formulas for $\eta_k$ and $\omega_k$ vary across BCL variants. Conn gives globally convergent rules, and Nocedal presents the same basic globalization idea in the augmented-Lagrangian framework. The implementation here uses a simple rule that is consistent with the present convention:

$$
\eta_{k+1} = \frac{\eta_k}{\mu_k^\alpha},
\qquad
\omega_{k+1} = \frac{\omega_k}{\mu_k},
$$

after an accepted multiplier update, and leaves the thresholds unchanged when feasibility is not yet good enough.

The important point is the role of the tests:

- `omega` says "do not trust the outer update unless the current inner solve is stationary enough."
- `eta` says "do not trust the multiplier update unless the current point is feasible enough."
- increasing `mu` is the fallback globalization move when feasibility is lagging.

That is the BCL strategy, expressed with the same sign and penalty convention used in the current `gtsam` implementation.

## Where This Is Implemented in `gtsam`

The implementation lives in [BoundConstrainedLagrangian.h](../BoundConstrainedLagrangian.h) and [BoundConstrainedLagrangian.cpp](../BoundConstrainedLagrangian.cpp).

### Parameters and State

[BoundConstrainedLagrangian.h](../BoundConstrainedLagrangian.h) defines the parameters:

- `k`: multiplicative factor used when increasing `muEq`
- `alpha`: exponent used when shrinking `eta`
- `eta0`, `omega0`: initial BCL tolerances
- `eta_threshold`, `omega_threshold`: stopping thresholds

It also sets

$$
\texttt{lmParams.maxIterations} = 1,
$$

so the inner solver performs only one Levenberg-Marquardt step per outer iteration. This is a strong design choice: the outer loop is effectively steering a sequence of very short inner solves.

The state extends the generic augmented-Lagrangian state with:

- `muEq`
- `lambdaEq`
- `eta`
- `omega`

### Building the Inner Subproblem

Each outer iteration is implemented by [BoundConstrainedLagrangian::iterate()](../BoundConstrainedLagrangian.cpp), which:

1. calls `augmentedLagrangianFunction(state)` to build the factor-graph form of the current augmented Lagrangian
2. creates a `LevenbergMarquardtOptimizer`
3. runs the optimizer
4. calls `updateMultipliers(previousState, &newState)`

The actual augmented-Lagrangian graph construction is delegated to [AugmentedLagrangianFunction()](../AugmentedLagrangian.h), whose comments explicitly document the Nocedal-style merit function.

### The BCL Decision Logic

The BCL-like logic is concentrated in [BoundConstrainedLagrangian::updateMultipliers()](../BoundConstrainedLagrangian.cpp).

That function does the following:

1. It linearizes the original cost graph `problem_.costs()` at the previous iterate and computes the infinity norm of the corresponding gradient.
2. If that norm is below `previousState.omega`, it computes the equality-constraint violations at the previous iterate and takes their infinity norm.
3. If the violation norm is also below `previousState.eta`, it accepts a multiplier update.
4. Otherwise, it keeps the multipliers fixed and increases the penalty.

In formulas, the implemented logic is

$$ 
\|\nabla \phi(x_k)\|_\infty < \omega_k
$$

followed by

$$ 
\|h(x_k)\|_\infty < \eta_k.
$$

If both are true, the code executes the accepted outer step:

$$
\lambda_{k+1} \leftarrow \lambda_k + \mu_k \, \mathrm{violation}(x_k),
\qquad
\mu_{k+1} \leftarrow \mu_k,
$$

and shrinks the BCL thresholds:

$$
\eta_{k+1} = \frac{\eta_k}{\mu_k^\alpha},
\qquad
\omega_{k+1} = \frac{\omega_k}{\mu_k}.
$$

If stationarity is good enough but feasibility is not, the code performs the penalty step:

$$
\lambda_{k+1} \leftarrow \lambda_k,
\qquad
\mu_{k+1} \leftarrow k \mu_k.
$$

If the stationarity test fails, everything is left unchanged.

### Initialization and Termination

[BoundConstrainedLagrangian::optimize()](../BoundConstrainedLagrangian.cpp) initializes:

- `lambdaEq` from zero multipliers
- `muEq` from `initialMuEq`
- `eta` from `eta0`
- `omega` from `omega0`

and then repeats outer iterations until [checkConvergenceBC()](../BoundConstrainedLagrangian.h) or the inherited constrained-optimizer stopping conditions fire.

The customized stopping rule terminates as soon as either

$$
\eta_k < \texttt{eta_threshold}
\qquad \text{or} \qquad
\omega_k < \texttt{omega_threshold}.
$$

## What the Current Code Is, and Is Not

The current code is a compact BCL-style outer loop wrapped around `LevenbergMarquardtOptimizer`.

It is important to be precise about what is actually implemented:

- The selective BCL update logic is only implemented for equality constraints.
- The acceptance test uses the gradient of the original cost, not the gradient of the full augmented Lagrangian.
- The constraint test uses `whitenedError(previousState.values)` for each equality constraint.
- The inner solver is one LM step, not an inner solve driven to an adaptive tolerance.
- Globalization outside that LM step is limited to the `eta` / `omega` logic and the penalty update.

So this is clearly in the Conn/Nocedal augmented-Lagrangian family, but it is a relatively lightweight specialization of that family.

## Carpentier Implementations

The Carpentier-group implementations are best viewed as later descendants of the same broad BCL-augmented-Lagrangian line, not as the primary reference for the write-up above.

- `~/git/proxsuite` implements `ProxQP`, which explicitly exposes a BCL update rule and cites both Conn and Nocedal. It applies the idea in a proximal-QP setting.
- `~/git/aligator` implements `SolverProxDDP`, which applies related BCL-style outer-loop logic inside a constrained DDP trajectory-optimization solver.

Both are useful comparison points, but both also use a different inner algorithmic stack and, in practice, an inverse-`mu` penalty convention rather than the direct $+\frac{\mu}{2}\|c(x)\|^2$ convention used here.
