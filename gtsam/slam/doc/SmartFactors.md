# Smart Factors

Smart factors in GTSAM provide an efficient way to handle constraints involving landmarks (like 3D points) in Structure from Motion (SfM) or SLAM problems without explicitly including the landmark variables in the optimized state. Instead, the landmark is implicitly represented and marginalized out, leading to factors that directly constrain only the camera-related variables (poses and/or calibrations). This often results in a smaller state space and can significantly speed up optimization, especially when using iterative linear solvers.

The core idea is based on the **Schur complement**. If we consider a factor graph involving cameras $C_i$ and a single landmark $p$, the Hessian matrix of the linearized system has a block structure:

```math
H = \begin{bmatrix} H_{pp} & H_{pc} \\ H_{cp} & H_{cc} \end{bmatrix}
```

where $H_{pp}$ relates to the landmark, $H_{cc}$ relates to the cameras, and $H_{pc}$ ($H_{cp}$) are the off-diagonal blocks. Marginalizing out the landmark variable $\delta_p$ corresponds to solving the system using the Schur complement $S = H_{cc} - H_{cp} H_{pp}^{-1} H_{pc}$. Smart factors effectively represent or compute this Schur complement system directly.

Key Reference:
{cite}`10.1109/ICRA.2014.6907483`.

## Mathematical Details

### 1. Triangulation

The core internal operation of a smart factor is triangulation. Given a set of cameras $C_i$ (which include pose and calibration) and corresponding 2D measurements $z_i$, the factor finds the most likely 3D landmark position $p^*$ by solving:

```math
p^* = \underset{p}{\arg \min} \sum_i \| \Pi(C_i, p) - z_i \|_{\Sigma_i}^2
```

where $\Pi(C_i, p)$ is the projection function of the $i$-th camera, and $\Sigma_i$ is the noise covariance for the $i$-th measurement (though typically a shared noise model $\Sigma$ is used). GTSAM uses the robust `gtsam.triangulateSafe` function internally, which returns a `gtsam.TriangulationResult` containing the point estimate $p^*$ and status information (valid, degenerate, behind camera, etc.).

### 2. Error Function

The nonlinear error minimized by the smart factor is the sum of squared reprojection errors using the *implicitly triangulated* point $p^*$:

```math
\text{error}( \{C_i\} ) = \frac{1}{2} \sum_i \| \Pi(C_i, p^*) - z_i \|_{\Sigma}^2
```

Note that $p^*$ itself is a function of the cameras $\{C_i\}$.

### 3. Linearization

When a smart factor is linearized (e.g., during optimization), it computes a Gaussian factor that only involves the camera variables $\{\delta_{C_i}\}$. This is achieved by implicitly performing the Schur complement.

Let the standard projection factor error for camera $i$ be $e_i(\delta_{C_i}, \delta_p) = F_i \delta_{C_i} + E_i \delta_p - b_i$, where $F_i$ and $E_i$ are the Jacobians with respect to the camera and point, evaluated at the current estimates $C_i^0$ and the triangulated point $p^*$, and $b_i = z_i - \Pi(C_i^0, p^*)$ is the residual. The combined linearized system is:

```math
\sum_i \| F_i \delta_{C_i} + E_i \delta_p - b_i \|_{\Sigma}^2 =
\left\| \begin{bmatrix} F & E \end{bmatrix} \begin{bmatrix} \delta_C \\ \delta_p \end{bmatrix} - b \right\|_{\Sigma}^2
```

where $F = \text{blkdiag}(F_i)$, $E = [E_1^T, ..., E_m^T]^T$, $b = [b_1^T, ..., b_m^T]^T$, and $\delta_C = [\delta_{C_1}^T, ..., \delta_{C_m}^T]^T$. The noise model $\Sigma$ is typically block-diagonal with identical blocks.

Marginalizing $\delta_p$ yields a Gaussian factor on $\delta_C$ with Hessian $H_S$ and information vector $\eta_S$:

```math
H_S = F^T \Sigma^{-1} F - F^T \Sigma^{-1} E (E^T \Sigma^{-1} E)^{-1} E^T \Sigma^{-1} F
```

```math
\eta_S = F^T \Sigma^{-1} b - F^T \Sigma^{-1} E (E^T \Sigma^{-1} E)^{-1} E^T \Sigma^{-1} b
```

Let $P = (E^T \Sigma^{-1} E)^{-1}$ be the point covariance and define the projection matrix $Q = \Sigma^{-1} (I - E P E^T \Sigma^{-1})$. Then the system simplifies to:

```math
H_S = F^T Q F
```

```math
\eta_S = F^T Q b
```

The smart factor's linearization computes or represents this Schur complement system $(H_S, \eta_S)$.

## The Smart Factor Family

GTSAM provides several types of smart factors, primarily for projection measurements. They inherit common functionality from `gtsam.SmartFactorBase`.

### `gtsam.SmartProjectionFactor`

This factor is used when **both camera poses and camera calibration** are unknown and being optimized. It connects multiple `CAMERA` variables, where the `CAMERA` type (e.g., `gtsam.PinholeCameraCal3_S2`) encapsulates both the pose and the intrinsic calibration.

See also: [https://github.com/borglab/gtsam/blob/develop/gtsam/slam/SmartFactorBase.h], [SmartProjectionFactor Notebook](SmartProjectionFactor.ipynb).

### `gtsam.SmartProjectionPoseFactor`

This factor is optimized for the common case where **camera calibration is known and fixed**, and only the camera **poses** (`gtsam.Pose3`) are being optimized. It assumes a single, shared calibration object (`gtsam.Cal3_S2`, `gtsam.Cal3Bundler`, etc.) for all measurements within the factor.

See also: [https://github.com/borglab/gtsam/blob/develop/gtsam/slam/SmartProjectionPoseFactor.h], [SmartProjectionPoseFactor Notebook](SmartProjectionPoseFactor.ipynb).

### `gtsam.SmartProjectionRigFactor`

This factor extends the concept to **multi-camera rigs** where the *relative* poses and calibrations of the cameras within the rig are fixed, but the **rig's body pose** (`gtsam.Pose3`) is being optimized. It allows multiple measurements from different cameras within the rig, potentially associated with the same body pose key.

**Note:** Due to implementation details, the `CAMERA` template parameter used with this factor should typically be `gtsam.PinholePose` rather than `gtsam.PinholeCamera`.

See also: [https://github.com/borglab/gtsam/blob/develop/gtsam/slam/SmartProjectionRigFactor.h], [SmartProjectionRigFactor Notebook](SmartProjectionRigFactor.ipynb).

## Configuration

The behavior of smart factors is controlled by `gtsam.SmartProjectionParams`. Key parameters include:

*   `linearizationMode`: Selects which type of `GaussianFactor` is returned by `linearize`, default is `HESSIAN`, see below.
*   `degeneracyMode`: Defines how to handle cases where triangulation fails or is ill-conditioned (e.g., collinear cameras). Options include ignoring it, returning a zero-information factor, or treating the point as being at infinity.
*   `triangulation`: Parameters passed to `triangulateSafe`, such as rank tolerance for SVD and outlier rejection thresholds.
*   `retriangulationThreshold`: If the camera poses change significantly between linearizations (measured by Frobenius norm), the point is re-triangulated.
*   `throwCheirality`/`verboseCheirality`: Control behavior when the triangulated point ends up behind one or more cameras.

See also: [https://github.com/borglab/gtsam/blob/develop/gtsam/slam/SmartFactorParams.h], [SmartFactorParams Notebook](SmartProjectionParams.ipynb).

## Linear Factor Representations

Depending on the `linearizationMode` specified in `gtsam.SmartProjectionParams`, the `linearize` method of a smart factor returns different types of `gtsam.GaussianFactor` objects, all representing the same underlying Schur complement system but optimized for different solvers:

### 1. `gtsam.RegularHessianFactor` (`HESSIAN` mode, default)

When a smart factor is linearized using the `HESSIAN` mode, it computes and returns a `gtsam.RegularHessianFactor`. In this specific context, this factor explicitly represents the dense **Schur complement** system obtained after marginalizing out the 3D landmark.

*   **Defining Feature:** This factor requires that all involved camera variables have the *same, fixed dimension D* (e.g., $D=6$ for `gtsam.Pose3`), specified via a template parameter. This contrasts with the base `gtsam.HessianFactor` which handles variable dimensions.
*   **Stored Information:** It stores the blocks of the effective Hessian matrix $H_S$ (denoted internally as $G_{ij}$) and the effective information vector $\eta_S$ (denoted internally as $g_i$) that act *only* on the camera variables.
*   **Optimized Operators (Benefit for CG):** Crucially, this regularity allows `RegularHessianFactor` to provide highly optimized implementations for key linear algebra operations, especially the raw-memory (`double*`) version of `multiplyHessianAdd`. This operation computes the Hessian-vector product ($y \leftarrow y + \alpha H_S x$) extremely efficiently by leveraging `Eigen::Map` and fixed-size matrix operations, making it ideal for **iterative solvers like Conjugate Gradient (CG)** which rely heavily on this product. The base `HessianFactor`'s implementations are more general and typically involve more overhead.
*   **Formation Cost:** Creating this factor from a smart factor can still be computationally expensive (forming the Schur complement $H_S = F^T Q F$), especially for landmarks observed by many cameras.
*   **Solver Suitability:** While efficient for CG (using the `double*` methods), it is also suitable for **direct linear solvers** (like Cholesky or QR factorization) as it provides the explicit system matrix $H_S$.

Source: [https://github.com/borglab/gtsam/blob/develop/gtsam/linear/RegularHessianFactor.h].

### 2. `gtsam.RegularImplicitSchurFactor` (`IMPLICIT_SCHUR` mode)

This factor *stores* the ingredients $F_i$, $E_i$, $P$, and $b_i$ (or rather, their whitened versions) instead of the final Hessian $H_S$. Its key feature is an efficient `multiplyHessianAdd` method that computes the Hessian-vector product $y \leftarrow y + \alpha H_S x$ *without ever forming* the potentially large $H_S$ matrix. It computes this via:

```math
v_i = F_i x_i \\
w = E^T \Sigma^{-1} v \\
d = P w \\
u_i = E_i d \\
y_i \leftarrow y_i + \alpha F_i^T \Sigma^{-1} (v_i - u_i)
```
This is highly efficient for iterative linear solvers like Conjugate Gradient (CG), which primarily rely on Hessian-vector products.

Source: [https://github.com/borglab/gtsam/blob/develop/gtsam/slam/RegularImplicitSchurFactor.h].

### 3. `gtsam.JacobianFactorQ` (`JACOBIAN_Q` mode)

This factor represents the Schur complement system as a Jacobian factor. It computes a projection matrix $Q_{proj} = \Sigma^{-1/2} (I - E P E^T \Sigma^{-1}) \Sigma^{-1/2}$ (where $\Sigma = \text{blkdiag}(\Sigma_i)$) and represents the error term $\| Q_{proj}^{1/2} (F \delta_C - b) \|^2$. It stores the projected Jacobians $A_i = Q_{proj, i}^{1/2} F_i$ and the projected right-hand side $b' = Q_{proj}^{1/2} b$.

Source: [https://github.com/borglab/gtsam/blob/develop/gtsam/slam/JacobianFactorQ.h].

### 4. `gtsam.JacobianFactorSVD` (`JACOBIAN_SVD` mode)

This factor uses the "Nullspace Trick". It computes $E_{null}$, a matrix whose columns form an orthonormal basis for the left nullspace of $E$ (i.e., $E_{null}^T E = 0$). Multiplying the original linearized system by $E_{null}^T \Sigma^{-1}$ yields a smaller system involving only $\delta_C$:

```math
\| E_{null}^T \Sigma^{-1} F \delta_C - E_{null}^T \Sigma^{-1} b \|^2_{(E_{null}^T \Sigma^{-1} E_{null})^{-1}}
```

The factor stores the projected Jacobian $A = E_{null}^T \Sigma^{-1} F$, the projected right-hand side $b' = E_{null}^T \Sigma^{-1} b$, and an appropriate noise model derived from $(E_{null}^T \Sigma^{-1} E_{null})^{-1}$. This method is mathematically equivalent to the Schur complement but can offer computational advantages in certain scenarios.

Source: [https://github.com/borglab/gtsam/blob/develop/gtsam/slam/JacobianFactorSVD.h].

## Conclusion

Smart factors provide a powerful mechanism for efficiently handling landmark-based constraints in SLAM and SfM. By implicitly marginalizing landmarks, they reduce the size of the state space and enable the use of specialized linear factor representations like `gtsam.RegularImplicitSchurFactor`, which are highly effective when combined with iterative solvers like Conjugate Gradient. Understanding the underlying mathematical connection to the Schur complement and the different linearization options allows users to choose the most appropriate configuration for their specific problem and solver.
