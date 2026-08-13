/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    QpCost.cpp
 * @brief   Affine quadratic cost factor for QP and QCQP problems.
 * @author  Frank Dellaert
 */

#include <gtsam/base/GenericValue.h>
#include <gtsam/constrained/QpCost.h>

#include <stdexcept>
#include <vector>

namespace gtsam {
namespace {

/* ************************************************************************* */
Vector VectorOrMatrixValue(const Values& values, Key key) {
  const Value& value = values.at(key);
  if (const auto* vectorValue =
          dynamic_cast<const GenericValue<Vector>*>(&value)) {
    return vectorValue->value();
  }
  if (const auto* matrixValue =
          dynamic_cast<const GenericValue<Matrix>*>(&value)) {
    const Matrix& matrix = matrixValue->value();
    return Eigen::Map<const Vector>(matrix.data(), matrix.size());
  }
  throw std::invalid_argument(
      "QpCost: only Vector and Matrix Values entries are supported.");
}

/* ************************************************************************* */
VectorValues DirectVectorValues(const Values& values,
                                const HessianFactor& factor) {
  VectorValues result;
  for (auto it = factor.begin(); it != factor.end(); ++it) {
    const Vector vector = VectorOrMatrixValue(values, *it);
    if (factor.getDim(it) != vector.size()) {
      throw std::invalid_argument(
          "QpCost: Hessian block dimension does not match value dimension.");
    }
    result.insert(*it, vector);
  }
  return result;
}

/* ************************************************************************* */
Matrix ExpandedBlock(const Matrix& block, size_t columns) {
  const DenseIndex rows = block.rows();
  const DenseIndex cols = block.cols();
  Matrix expanded = Matrix::Zero(rows * columns, cols * columns);
  for (size_t column = 0; column < columns; ++column) {
    expanded.block(column * rows, column * cols, rows, cols) = block;
  }
  return expanded;
}

}  // namespace

/* ************************************************************************* */
QpCost::QpCost(const KeyVector& keys, const SymmetricBlockMatrix& Q,
               size_t columnDim)
    : Base(keys) {
  if (keys.size() != static_cast<size_t>(Q.nBlocks())) {
    throw std::invalid_argument(
        "QpCost: number of keys must match Q block count.");
  }
  if (columnDim == 0) {
    throw std::invalid_argument("QpCost: columnDim must be positive.");
  }

  std::vector<Matrix> Gs;
  Gs.reserve(keys.size() * (keys.size() + 1) / 2);
  for (size_t i = 0; i < keys.size(); ++i) {
    for (size_t j = i; j < keys.size(); ++j) {
      Gs.push_back(ExpandedBlock(Q.block(i, j), columnDim));
    }
  }

  std::vector<Vector> gs;
  gs.reserve(keys.size());
  for (size_t i = 0; i < keys.size(); ++i) {
    const size_t dim = static_cast<size_t>(Q.getDim(i)) * columnDim;
    gs.push_back(Vector::Zero(dim));
  }

  hessianFactor_ = std::make_shared<HessianFactor>(keys, Gs, gs, 0.0);
}

/* ************************************************************************* */
void QpCost::print(const std::string& s, const KeyFormatter& formatter) const {
  Base::print(s + "QpCost", formatter);
  hessianFactor_->print("  Hessian factor: ", formatter);
}

/* ************************************************************************* */
bool QpCost::equals(const NonlinearFactor& other, double tol) const {
  const auto* expected = dynamic_cast<const QpCost*>(&other);
  return expected != nullptr && Base::equals(other, tol) &&
         hessianFactor_->equals(*expected->hessianFactor_, tol);
}

/* ************************************************************************* */
double QpCost::error(const Values& values) const {
  return hessianFactor_->error(DirectVectorValues(values, *hessianFactor_));
}

/* ************************************************************************* */
GaussianFactor::shared_ptr QpCost::linearize(const Values& values) const {
  auto shifted = std::make_shared<HessianFactor>(*hessianFactor_);
  const VectorValues vectorValues = DirectVectorValues(values, *hessianFactor_);
  const Vector valueVector = vectorValues.vector(hessianFactor_->keys());
  const auto informationView = shifted->informationView();
  const Vector Gx = informationView * valueVector;
  const Vector linearTerm = shifted->linearTerm().col(0);
  shifted->constantTerm() +=
      valueVector.dot(Gx) - 2.0 * valueVector.dot(linearTerm);
  shifted->linearTerm() -= Gx;
  return shifted;
}

}  // namespace gtsam
