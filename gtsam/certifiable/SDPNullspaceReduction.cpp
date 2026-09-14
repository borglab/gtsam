/* Copyright 2026, Georgia Tech Research Corporation. See LICENSE. */
#include <gtsam/certifiable/SDPNullspaceReduction.h>

#include <cmath>
#include <limits>

namespace gtsam::internal {
namespace {

// Require an explicit h^2=1 equality, not an assumption about a key's type.
KeySet UnitHomogeneousKeys(const QcqpProblem& problem) {
  KeySet keys;
  for (const auto& factor : problem.eConstraints()) {
    const auto* quadratic =
        dynamic_cast<const QuadraticEqualityConstraintFactor*>(factor.get());
    if (!quadratic) continue;
    const auto& constraint = quadratic->quadraticConstraint();
    Matrix expected =
        Matrix::Zero(constraint.A().rows(), constraint.A().cols());
    if (expected.size() == 0 || constraint.b() == 0.0) continue;
    expected(0, 0) = constraint.b();
    if ((constraint.A().array() == expected.array()).all()) {
      keys.insert(constraint.key());
    }
  }
  return keys;
}

// Recognize only a*x_r + a0*h = b: in Gram coordinates x_r=(b-a0)*h/a.
// Coupled linear equations are deliberately left untouched in this first pass.
std::map<Key, Vector> FixedCoordinates(
    const QcqpProblem& problem, const std::map<Key, DenseIndex>& dimensions) {
  std::map<Key, Vector> fixed;
  for (const auto& [key, dimension] : dimensions) {
    fixed[key] =
        Vector::Constant(dimension, std::numeric_limits<double>::quiet_NaN());
  }
  for (const auto& factor : problem.eConstraints()) {
    const auto* linear =
        dynamic_cast<const LinearEqualityConstraintFactor*>(factor.get());
    if (!linear) continue;
    const auto& jacobian = linear->linearConstraint().factor();
    if (jacobian.size() != 1) continue;
    const Matrix A = jacobian.getA(jacobian.begin());
    for (DenseIndex row = 0; row < A.rows(); ++row) {
      DenseIndex coordinate = 0;
      size_t count = 0;
      for (DenseIndex col = 1; col < A.cols(); ++col) {
        if (A(row, col) != 0.0) {
          coordinate = col;
          ++count;
        }
      }
      if (count == 1) {
        fixed.at(*jacobian.begin())(coordinate) =
            (jacobian.getb()(row) - A(row, 0)) / A(row, coordinate);
      }
    }
  }
  return fixed;
}

// A sparse substitution basis preserves the physical coordinate scaling.
Matrix CliqueBasis(const KeyVector& keys, const KeySet& homogeneous,
                   const std::map<Key, Vector>& fixed) {
  DenseIndex dimension = 0, reduced = 1;
  bool supported = true;
  for (Key key : keys) {
    dimension += fixed.at(key).size();
    supported = supported && homogeneous.count(key);
    for (DenseIndex row = 1; row < fixed.at(key).size(); ++row) {
      if (!std::isfinite(fixed.at(key)(row))) ++reduced;
    }
  }
  if (!supported || keys.empty()) return Matrix::Identity(dimension, dimension);
  Matrix basis = Matrix::Zero(dimension, reduced);
  DenseIndex offset = 0, column = 1;
  for (Key key : keys) {
    basis(offset++, 0) = 1.0;
    for (DenseIndex row = 1; row < fixed.at(key).size(); ++row) {
      const double value = fixed.at(key)(row);
      if (std::isfinite(value))
        basis(offset++, 0) = value;
      else
        basis(offset++, column++) = 1.0;
    }
  }
  return basis;
}

}  // namespace

std::map<KeyVector, Matrix> EliminateKnownNullDirections(
    const QcqpProblem& problem, const std::map<Key, DenseIndex>& dimensions,
    const std::vector<KeyVector>& cliques) {
  const KeySet homogeneous = UnitHomogeneousKeys(problem);
  const auto fixed = FixedCoordinates(problem, dimensions);
  std::map<KeyVector, Matrix> bases;
  for (const auto& keys : cliques) {
    bases.emplace(keys, CliqueBasis(keys, homogeneous, fixed));
  }
  return bases;
}

}  // namespace gtsam::internal
