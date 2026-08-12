# Basis

The `basis` module provides tools for representing continuous functions as
linear combinations of basis functions or as values at interpolation points.
It is useful for smooth function approximation, trajectory modeling, and
building factors that constrain functions (or their derivatives) at specific
points.

At a high level, you choose a basis (Fourier or Chebyshev), decide whether you
want a coefficient-based representation or a pseudo-spectral one (values at
Chebyshev points), and then use the provided factors or fitting utilities to
solve for parameters in GTSAM.

## Getting Oriented

- **Coefficient-based bases**: `Chebyshev1Basis`, `Chebyshev2Basis`, and
  `FourierBasis` treat the parameters as coefficients on basis functions.
- **Pseudo-spectral basis**: `Chebyshev2` treats the parameters as values at
  Chebyshev points and uses barycentric interpolation.
- **Factors**: A family of unary factors enforce function values or derivatives
  at specific points, including vector- and manifold-valued variants.
- **Fitting**: `FitBasis` performs least-squares regression from samples.

## Core Concepts

- [Basis](doc/Basis.ipynb): CRTP base class providing evaluation and derivative
  functors, Jacobians, and common helpers.

## Polynomial Bases

- [Chebyshev1Basis](doc/Chebyshev1Basis.ipynb): First-kind Chebyshev basis
  $T_n(x)$ on $[-1,1]$ (coefficient-based).
- [Chebyshev2Basis](doc/Chebyshev2Basis.ipynb): Second-kind Chebyshev basis
  $U_n(x)$ on $[-1,1]$ (coefficient-based).
- [FourierBasis](doc/FourierBasis.ipynb): Real Fourier series basis for
  periodic functions.

## Pseudo-Spectral Basis

- [Chebyshev2](doc/Chebyshev2.ipynb): Chebyshev points, barycentric
  interpolation, differentiation/integration matrices, and quadrature weights.

## Factors for Basis Evaluation

These factors connect basis parameters to measurements of values or derivatives.

- [EvaluationFactor](doc/EvaluationFactor.ipynb): Scalar value at a point.
- [VectorEvaluationFactor](doc/VectorEvaluationFactor.ipynb): Vector value at a
  point.
- [VectorComponentFactor](doc/VectorComponentFactor.ipynb): Single component of
  a vector value.
- [ManifoldEvaluationFactor](doc/ManifoldEvaluationFactor.ipynb): Manifold-valued
  measurement (e.g., `Rot3`, `Pose3`).

## Factors for Derivative Constraints

- [DerivativeFactor](doc/DerivativeFactor.ipynb): Scalar derivative at a point.
- [VectorDerivativeFactor](doc/VectorDerivativeFactor.ipynb): Vector derivative
  at a point.
- [ComponentDerivativeFactor](doc/ComponentDerivativeFactor.ipynb): Single
  component of a vector derivative.

## Fitting from Data

- [FitBasis](doc/FitBasis.ipynb): Build a least-squares problem from samples and
  solve for basis parameters.
