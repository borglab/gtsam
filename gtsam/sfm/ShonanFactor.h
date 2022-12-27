/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010-2019, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file   ShonanFactor.h
 * @date   March 2019
 * @author Frank Dellaert
 * @brief  Main factor type in Shonan averaging, on SO(n) pairs
 */

#pragma once

#include <gtsam/geometry/Rot2.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/SOn.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

#include <type_traits>

namespace gtsam {

/**
 * ShonanFactor is a BetweenFactor that moves in SO(p), but will
 * land on the SO(d) sub-manifold of SO(p) at the global minimum. It projects
 * the SO(p) matrices down to a Stiefel manifold of p*d matrices.
 */
template <size_t d>
class GTSAM_EXPORT ShonanFactor : public NoiseModelFactorN<SOn, SOn> {
  ADD_NOISE_MODEL_FACTOR_N_DEPRECATED_TYPEDEFS(ShonanFactor, 2);

  Matrix M_;                    ///< measured rotation between R1 and R2
  size_t p_, pp_;               ///< dimensionality constants
  boost::shared_ptr<Matrix> G_; ///< matrix of vectorized generators

  // Select Rot2 or Rot3 interface based template parameter d
  using Rot = typename std::conditional<d == 2, Rot2, Rot3>::type;

public:
  /// @name Constructor
  /// @{

  /// Constructor. Note we convert to d*p-dimensional noise model.
  /// To save memory and mallocs, pass in the vectorized Lie algebra generators:
  ///    G = boost::make_shared<Matrix>(SOn::VectorizedGenerators(p));
  ShonanFactor(Key j1, Key j2, const Rot &R12, size_t p,
               const SharedNoiseModel &model = nullptr,
               const boost::shared_ptr<Matrix> &G = nullptr);

  /// @}
  /// @name Testable
  /// @{

  /// print with optional string
  void
  print(const std::string &s,
        const KeyFormatter &keyFormatter = DefaultKeyFormatter) const override;

  /// assert equality up to a tolerance
  bool equals(const NonlinearFactor &expected,
              double tol = 1e-9) const override;

  /// @}
  /// @name NoiseModelFactorN methods
  /// @{

  /// Error is Frobenius norm between Q1*P*R12 and Q2*P, where P=[I_3x3;0]
  /// projects down from SO(p) to the Stiefel manifold of px3 matrices.
  Vector
  evaluateError(const SOn &Q1, const SOn &Q2,
                boost::optional<Matrix &> H1 = boost::none,
                boost::optional<Matrix &> H2 = boost::none) const override;
  /// @}

private:
  /// Calculate Jacobians if asked, Only implemented for d=2 and 3 in .cpp
  void fillJacobians(const Matrix &M1, const Matrix &M2,
                     boost::optional<Matrix &> H1,
                     boost::optional<Matrix &> H2) const;
};

// Explicit instantiation for d=2 and d=3 in .cpp file:
using ShonanFactor2 = ShonanFactor<2>;
using ShonanFactor3 = ShonanFactor<3>;

} // namespace gtsam
