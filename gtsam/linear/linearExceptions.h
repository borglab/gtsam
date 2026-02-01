/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    linearExceptions.h
 * @brief   Exceptions that may be thrown by linear solver components
 * @author  Richard Roberts
 * @date    Aug 17, 2012
 */
#pragma once

#include <gtsam/base/ThreadsafeException.h>
#include <gtsam/base/types.h>

namespace gtsam {

  /**
  Thrown when a linear system is ill-posed.  The most common cause for this
  error is having underconstrained variables.  Mathematically, the system is
  either underdetermined, or its quadratic error function is concave in some
  directions.

  Examples of situations causing this error are:
   - A landmark observed by two cameras with a very small baseline will have
     high uncertainty in its distance from the cameras but low uncertainty
     in its bearing, creating a poorly-conditioned system.
   - A landmark observed by only a single ProjectionFactor, RangeFactor, or
     BearingFactor (the landmark is not completely constrained).
   - An overall scale or rigid transformation ambiguity, for example missing
     a prior or hard constraint on the first pose, or missing a scale
     constraint between the first two cameras (in structure-from-motion).

  Mathematically, the following conditions cause this problem:
   - Underdetermined system:  This occurs when the variables are not
     completely constrained by factors.  Even if all variables are involved
     in factors, the variables can still be numerically underconstrained,
     (for example, if a landmark is observed by only one ProjectionFactor).
     Mathematically in this case, the rank of the linear Jacobian or Hessian
     is less than the number of scalars in the system.
   - Indefinite system:  This condition occurs when the system Hessian is
     indefinite, i.e. non-positive-semidefinite.  Note that this condition
     can also indicate an underdetermined system, but when solving with
     Cholesky, accumulation of numerical errors can cause the system to
     become indefinite (have some negative eigenvalues) even if it is in
     reality positive semi-definite (has some near-zero positive
     eigenvalues).  Alternatively, if the system contains some
     HessianFactor's with negative eigenvalues, these can create a truly
     indefinite system, whose quadratic error function has negative
     curvature in some directions.
   - Poorly-conditioned system:  A system that is positive-definite (has a
     unique solution) but is numerically ill-conditioned can accumulate
     numerical errors during solving that cause the system to become
     indefinite later during the elimination process.  Note that poorly-
     conditioned systems will usually have inaccurate solutions, even if
     they do not always trigger this error.  Systems with almost-
     unconstrained variables or vastly different measurement uncertainties
     or variable units can be poorly-conditioned.

  Resolving this problem:
   - This exception contains the variable at which the problem was
     discovered (IndeterminantLinearSystemException::nearbyVariable()).
     Note, however, that this is not necessarily the variable where the
     problem originates.  For example, in the case that a prior on the
     first camera was forgotten, it may only be another camera or landmark
     where the problem can be detected.  Where the problem is detected
     depends on the graph structure and variable elimination ordering.
   - MATLAB (or similar software) is a useful tool to diagnose this problem.
     Use GaussianFactorGraph::sparseJacobian_(),
     GaussianFactorGraph::sparseJacobian()
     GaussianFactorGraph::denseJacobian(), and
     GaussianFactorGraph::denseHessian() to output the linear graph in
     matrix format.  If using the MATLAB wrapper, the returned matrices can
     be immediately inspected and analyzed using standard methods.  If not
     using the MATLAB wrapper, the output of these functions may be written
     to text files and then loaded into MATLAB.
   - When outputting linear graphs as Jacobian matrices, rows are ordered in
     the same order as factors in the graph, which each factor occupying the
     number of rows indicated by its JacobianFactor::rows() function.  Each
     column appears in elimination ordering, as in the Ordering class.  Each
     variable occupies the number of columns indicated by the
     JacobianFactor::getDim() function.
   - When outputting linear graphs as Hessian matrices, rows and columns are
     ordered in elimination order and occupy scalars in the same way as
     described for Jacobian columns in the previous bullet.
   */
  class GTSAM_EXPORT IndeterminantLinearSystemException : public ThreadsafeException<IndeterminantLinearSystemException> {
    Key j_;
  public:
    IndeterminantLinearSystemException(Key j) noexcept : j_(j) {}
    ~IndeterminantLinearSystemException() noexcept override {}
    Key nearbyVariable() const { return j_; }
    const char* what() const noexcept override;
  };

  /* ************************************************************************* */
  /** An exception indicating that the noise model dimension passed into a
   * JacobianFactor has a different dimensionality than the factor. */
  class GTSAM_EXPORT InvalidNoiseModel : public ThreadsafeException<InvalidNoiseModel> {
  public:
    const DenseIndex factorDims; ///< The dimensionality of the factor
    const DenseIndex noiseModelDims; ///< The dimensionality of the noise model

    InvalidNoiseModel(DenseIndex factorDims, DenseIndex noiseModelDims) :
      factorDims(factorDims), noiseModelDims(noiseModelDims) {}
    ~InvalidNoiseModel() noexcept override {}

    const char* what() const noexcept override;
  };

  /* ************************************************************************* */
  /** An exception indicating that a matrix block passed into a
   * JacobianFactor has a different dimensionality than the factor. */
  class GTSAM_EXPORT InvalidMatrixBlock : public ThreadsafeException<InvalidMatrixBlock> {
  public:
    const DenseIndex factorRows; ///< The dimensionality of the factor
    const DenseIndex blockRows; ///< The dimensionality of the noise model

    InvalidMatrixBlock(DenseIndex factorRows, DenseIndex blockRows) :
      factorRows(factorRows), blockRows(blockRows) {}
    ~InvalidMatrixBlock() noexcept override {}

    const char* what() const noexcept override;
  };

  /* ************************************************************************* */
  class InvalidDenseElimination : public ThreadsafeException<InvalidDenseElimination> {
  public:
    InvalidDenseElimination(const char *message) : ThreadsafeException<InvalidDenseElimination>(message) {}
  };

 }
