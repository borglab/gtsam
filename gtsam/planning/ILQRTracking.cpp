// Copyright (c) Georgia Tech Research Corporation. See LICENSE for details.
/**
 * @file ILQRTracking.cpp
 * @author Frank Dellaert
 * @author Akash Ratheesh
 */
#include <gtsam/inference/Ordering.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/GaussianConditional.h>
#include <gtsam/linear/HessianFactor.h>
#include <gtsam/linear/linearExceptions.h>
#include <gtsam/planning/ILQRTracking.h>

namespace gtsam {
namespace internal {

using symbol_shorthand::U;
using symbol_shorthand::X;

TrackingSweep backwardILQR(const Matrix& stateErrors,
                           const Matrix& controlErrors,
                           const Vector& terminalError,
                           const ILQRLinearization& lin, const Matrix& q,
                           const Matrix& r, const Matrix& terminal,
                           double damping,
                           const std::vector<Matrix>& errorJacobians) {
  const Eigen::Index m = controlErrors.rows(), h = controlErrors.cols();
  const bool hasChartJac = !errorJacobians.empty();
  TrackingSweep sweep{Matrix::Zero(m, h),
                      std::vector<Matrix>(static_cast<size_t>(h))};

  // Terminal cost-to-go in GTSAM HessianFactor convention (g = -grad V(0)).
  Matrix Vxx;
  Vector vx;
  if (hasChartJac) {
    const Matrix& Ch = errorJacobians[static_cast<size_t>(h)];
    Vxx = Ch.transpose() * terminal * Ch;
    vx = -Ch.transpose() * terminal * terminalError;
  } else {
    Vxx = terminal;
    vx = -terminal * terminalError;
  }

  for (Eigen::Index k = h - 1; k >= 0; --k) {
    const auto idx = static_cast<size_t>(k);
    const Matrix& A = lin.A[idx];
    const Matrix& B = lin.B[idx];

    // Pull cost-to-go back through dx_{k+1} = A dx_k + B du_k and add stage
    // costs.
    const Matrix VxxA = Vxx * A;
    const Matrix VxxB = Vxx * B;
    Matrix Guu = r + B.transpose() * VxxB;
    if (damping > 0.0) Guu.diagonal().array() += damping;
    const Matrix Gux = B.transpose() * VxxA;
    Matrix Gxx;
    Vector gx;
    if (hasChartJac) {
      const Matrix& Ck = errorJacobians[idx];
      Gxx = Ck.transpose() * q * Ck + A.transpose() * VxxA;
      gx = -Ck.transpose() * q * stateErrors.col(k) + A.transpose() * vx;
    } else {
      Gxx = q + A.transpose() * VxxA;
      gx = -q * stateErrors.col(k) + A.transpose() * vx;
    }
    const Vector gu = -r * controlErrors.col(k) + B.transpose() * vx;

    // Eliminate U(k) via GTSAM's HessianFactor::eliminateCholesky to obtain
    // P(du_k | dx_k) and the exact Schur-complement HessianFactor on X(k).
    HessianFactor stageFactor(U(idx), X(idx), Guu, Gux, gu, Gxx, gx, 0.0);
    GaussianConditional::shared_ptr conditional;
    try {
      conditional = stageFactor.eliminateCholesky(Ordering{U(idx)});
    } catch (const IndeterminateSystemException&) {
      return {};
    }

    const auto R = conditional->R().triangularView<Eigen::Upper>();
    sweep.feedforward.col(k) = R.solve(conditional->d());
    sweep.gains[idx] = -R.solve(conditional->S());

    // Remaining Schur-complement factor on X(k) becomes the next cost-to-go.
    Vxx = stageFactor.information();
    vx = stageFactor.linearTerm();
  }
  return sweep;
}

}  // namespace internal
}  // namespace gtsam
