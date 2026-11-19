/* ----------------------------------------------------------------------------
 * GTSAM Copyright 2026, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415. All Rights Reserved.
 * See LICENSE for the license information.
 * -------------------------------------------------------------------------- */
#include <gtsam/constrained/internal/DualQpSolver.h>
#include <gtsam/constrained/internal/ActiveSetSolverInternal.h>
#include <gtsam/constrained/internal/SparseDualSolver.h>
#include <gtsam/linear/GaussianISAM2.h>
#include <gtsam/linear/linearAlgorithms-inst.h>
#include <Eigen/Eigenvalues>
#include <algorithm>
#include <cmath>
#include <limits>
#include <set>

namespace gtsam::internal {
namespace {

// All rows are normalized only internally; public tolerances and multipliers
// retain their original units.
double rowDot(const GaussianFactor::shared_ptr& factor, const VectorValues& value) {
  const auto& row = static_cast<const JacobianFactor&>(*factor);
  double result = 0;
  for (auto key = row.begin(); key != row.end(); ++key)
    result += row.getA(key).row(0).dot(value.at(*key));
  return result;
}

double bound(const GaussianFactor::shared_ptr& factor) {
  return static_cast<const JacobianFactor&>(*factor).getb()(0);
}

void addNormal(const GaussianFactor::shared_ptr& factor, double weight,
               VectorValues* value) {
  const auto& row = static_cast<const JacobianFactor&>(*factor);
  for (auto key = row.begin(); key != row.end(); ++key)
    value->at(*key) += weight * row.getA(key).row(0).transpose();
}

}  // namespace

struct DualQpSolver::Impl {
  SparseQpData data;
  DenseQpSolver* dense;
  Ordering ordering;
  std::unique_ptr<GaussianISAM2> tree;
  VectorValues zero, reference;
  size_t dimension = 0;
  Vector scales;
  Statistics statistics;
  std::vector<size_t> rows;
  std::vector<VectorValues> responses;
  Matrix lower, gram;
  Vector diagonal;
  bool nonsingular = true;

  Impl(SparseQpData inputs, DenseQpSolver* denseWorkspace,
       const Ordering& denseOrdering)
      : data(std::move(inputs)), dense(denseWorkspace), ordering(denseOrdering) {
    for (const auto& [key, count] : data.keyDimensions) {
      zero.insert(key, Vector::Zero(count));
      dimension += count;
    }
    if (!dense) {
      GaussianFactorGraph graph = data.objective;
      graph.push_back(data.equalities);
      // Include keys occurring only in inequalities in a complete ordering.
      for (const auto& [key, count] : data.keyDimensions)
        graph.emplace_shared<JacobianFactor>(key, Matrix::Identity(count, count),
                                             Vector::Zero(count));
      ordering = data.ordering ? *data.ordering : Ordering::Colamd(graph);
    }
    scales.resize(data.inequalities.size());
    for (size_t i = 0; i < data.inequalities.size(); ++i) {
      const auto& row = static_cast<const JacobianFactor&>(*data.inequalities[i]);
      scales(i) = row.getA().stableNorm();
      if (scales(i) == 0) scales(i) = 1;
    }
    if (dense) dimension = dense->reducedDimension();
    else if (!data.equalities.empty()) prepareEqualities();
  }

  Vector pack(const VectorValues& values) const {
    return values.vector(ordering);
  }

  VectorValues unpack(const Vector& value) const {
    VectorValues result;
    size_t offset = 0;
    for (Key key : ordering) {
      const size_t count = data.keyDimensions.at(key);
      result.insert(key, value.segment(offset, count));
      offset += count;
    }
    return result;
  }

  void checkEqualities(const VectorValues& value) const {
    for (const auto& factor : data.equalities) {
      const double rhs = bound(factor), actual = rowDot(factor, value);
      const double tolerance = 256 * std::numeric_limits<double>::epsilon() *
          (1 + std::abs(rhs) + std::abs(actual));
      if (!std::isfinite(actual) || std::abs(actual - rhs) > std::max(1e-10, tolerance))
        throw std::runtime_error("Dual QP: inconsistent equality constraints.");
    }
  }

  // An identity cost makes the equality-only problem determinate without
  // altering the actual QP. Hard conditional pivots count independent rows.
  void prepareEqualities() {
    GaussianFactorGraph graph;
    for (const auto& [key, count] : data.keyDimensions)
      graph.emplace_shared<JacobianFactor>(key, Matrix::Identity(count, count),
                                           Vector::Zero(count));
    graph.push_back(data.equalities);
    GaussianISAM2 projection(ordering, EliminateQR);
    projection.update(graph);
    checkEqualities(projection.optimize());
    std::set<const GaussianISAM2Clique*> visited;
    for (const auto& [key, clique] : projection.nodes()) {
      if (!visited.insert(clique.get()).second) continue;
      const auto& model = clique->conditional()->get_model();
      if (model)
        for (double sigma : model->sigmas())
          if (sigma == 0) --dimension;
    }
  }

  bool prepare() {
    if (dense) {
      const bool valid = dense->prepareDual();
      if (valid) ++statistics.objectiveFactorizations;
      return valid;
    }
    GaussianFactorGraph graph = data.objective;
    // Zero-weight rows retain constraint-only keys and expose missing curvature.
    const double weight = std::sqrt(data.regularization);
    for (const auto& [key, count] : data.keyDimensions)
      graph.emplace_shared<JacobianFactor>(key, weight * Matrix::Identity(count, count),
                                           Vector::Zero(count));
    graph.push_back(data.equalities);
    const bool jacobians = std::all_of(data.objective.begin(), data.objective.end(),
        [](const auto& factor) { return dynamic_cast<const JacobianFactor*>(factor.get()); });
    GaussianFactorGraph::Eliminate eliminate = EliminatePreferCholesky;
    if (jacobians) eliminate = EliminateQR;
    tree = std::make_unique<GaussianISAM2>(ordering, eliminate);
    try {
      tree->update(graph);
      reference = tree->optimize();
    } catch (const IndeterminateSystemException&) {
      tree.reset();
      return false;
    }
    ++statistics.objectiveFactorizations;
    checkEqualities(reference);
    return true;
  }

  VectorValues point(const VectorValues& center, const Vector& multipliers) {
    VectorValues forcing = zero;
    for (size_t i = 0; i < data.inequalities.size(); ++i)
      if (multipliers(i) != 0) addNormal(data.inequalities[i], -multipliers(i), &forcing);
    ++statistics.rhsSolves;
    if (dense) return unpack(dense->solveDual(pack(center), pack(forcing)));
    for (const auto& [key, value] : center)
      forcing.at(key) += data.regularization * value;
    VectorValues candidate = tree->optimize(forcing);
    // Correct roundoff using original gradients, preserving the affine manifold.
    VectorValues gradient = zero;
    for (const auto& factor : data.objective)
      for (Key key : factor->keys()) gradient.at(key) += factor->gradient(key, candidate);
    for (const auto& [key, value] : candidate)
      gradient.at(key) += data.regularization * value - forcing.at(key);
    return tree->optimize(forcing - gradient);
  }

  VectorValues response(size_t row) {
    VectorValues forcing = zero;
    addNormal(data.inequalities[row], 1 / scales(row), &forcing);
    ++statistics.responseSolves;
    if (dense) return unpack(dense->dualResponse(pack(forcing)));
    // Homogeneous cached back-substitution avoids cancellation against the
    // affine minimizer, especially when proximal curvature is very small.
    const VectorValues rhs = linearAlgorithms::transposeBayesTree(*tree, std::move(forcing));
    return linearAlgorithms::optimizeBayesTree(*tree, &rhs);
  }

  double threshold() const {
    return 128 * std::numeric_limits<double>::epsilon() *
        std::max<Eigen::Index>(1, gram.rows()) *
        (gram.size() ? gram.diagonal().cwiseAbs().maxCoeff() : 0);
  }

  // Border the factorization with one original inequality. Keep a singular
  // border for the null-direction step rather than silently discarding a row.
  void append(size_t row) {
    const size_t count = rows.size();
    VectorValues column = response(row);
    Vector cross(count);
    for (size_t i = 0; i < count; ++i)
      cross(i) = rowDot(data.inequalities[rows[i]], column) / scales(rows[i]);
    const double pivot = rowDot(data.inequalities[row], column) / scales(row);
    gram.conservativeResize(count + 1, count + 1);
    gram.topRightCorner(count, 1) = cross;
    gram.bottomLeftCorner(1, count) = cross.transpose();
    gram(count, count) = pivot;
    rows.push_back(row);
    responses.push_back(std::move(column));
    ++statistics.rowAdditions;
    if (!nonsingular) return;
    Vector border = lower.triangularView<Eigen::UnitLower>().solve(cross);
    if (count) border.array() /= diagonal.array();
    const double remainder = pivot - border.dot(diagonal.cwiseProduct(border));
    lower.conservativeResize(count + 1, count + 1);
    lower.topRightCorner(count, 1).setZero();
    lower.bottomLeftCorner(1, count) = border.transpose();
    lower(count, count) = 1;
    diagonal.conservativeResize(count + 1);
    diagonal(count) = remainder;
    nonsingular = remainder > threshold();
  }

  void rebuild() {
    ++statistics.workingRebuilds;
    lower = Matrix::Identity(rows.size(), rows.size());
    diagonal.resize(rows.size());
    nonsingular = true;
    for (size_t i = 0; i < rows.size(); ++i) {
      Vector border = lower.topLeftCorner(i, i).triangularView<Eigen::UnitLower>()
          .solve(Vector(gram.col(i).head(i)));
      if (i) border.array() /= diagonal.head(i).array();
      diagonal(i) = gram(i, i) - border.dot(diagonal.head(i).cwiseProduct(border));
      lower.row(i).head(i) = border.transpose();
      if (diagonal(i) <= threshold()) { nonsingular = false; break; }
    }
  }

  // Deleting a working row is a rank-one update of the trailing LDL factors.
  void remove(size_t position) {
    const Eigen::Index count = rows.size(), remaining = count - position - 1;
    Matrix compact(count - 1, count - 1);
    for (Eigen::Index i = 0; i < count - 1; ++i)
      for (Eigen::Index j = 0; j < count - 1; ++j)
        compact(i, j) = gram(i + (i >= Eigen::Index(position)),
                              j + (j >= Eigen::Index(position)));
    gram = std::move(compact);
    rows.erase(rows.begin() + position);
    responses.erase(responses.begin() + position);
    ++statistics.rowRemovals;
    if (!nonsingular) { rebuild(); return; }
    Vector update = lower.col(position).tail(remaining);
    double weight = diagonal(position);
    Matrix next = Matrix::Identity(count - 1, count - 1);
    next.topLeftCorner(position, position) = lower.topLeftCorner(position, position);
    next.bottomLeftCorner(remaining, position) = lower.bottomLeftCorner(remaining, position);
    next.bottomRightCorner(remaining, remaining) = lower.bottomRightCorner(remaining, remaining);
    Vector pivots(count - 1);
    pivots.head(position) = diagonal.head(position);
    pivots.tail(remaining) = diagonal.tail(remaining);
    for (Eigen::Index i = 0; i < remaining; ++i) {
      const Eigen::Index index = position + i, trailing = remaining - i - 1;
      const double previous = pivots(index), component = update(i);
      pivots(index) += weight * component * component;
      const double coefficient = weight * component / pivots(index);
      weight *= previous / pivots(index);
      update.tail(trailing) -= component * next.col(index).tail(trailing);
      next.col(index).tail(trailing) += coefficient * update.tail(trailing);
    }
    lower = std::move(next);
    diagonal = std::move(pivots);
  }

  // Return either a stationary point or a strict null-space descent direction.
  std::pair<Vector, bool> workingSolution(const Vector& linear) const {
    if (rows.empty()) return {Vector(), false};
    if (nonsingular) {
      Vector value = lower.triangularView<Eigen::UnitLower>().solve(Vector(-linear));
      value.array() /= diagonal.array();
      return {lower.transpose().triangularView<Eigen::UnitUpper>().solve(value), false};
    }
    Eigen::SelfAdjointEigenSolver<Matrix> eigen(gram);
    if (eigen.info() != Eigen::Success || eigen.eigenvalues().minCoeff() < -threshold())
      throw std::runtime_error("Dual QP: numerically indefinite working system.");
    const Vector projected = eigen.eigenvectors().transpose() * linear;
    Vector solution = Vector::Zero(linear.size()), direction = solution;
    for (Eigen::Index i = 0; i < linear.size(); ++i) {
      if (eigen.eigenvalues()(i) > threshold())
        solution(i) = -projected(i) / eigen.eigenvalues()(i);
      else direction(i) = -projected(i);
    }
    const double allowance = 256 * std::numeric_limits<double>::epsilon() *
        std::max(1.0, linear.norm());
    if (direction.norm() > allowance)
      return {eigen.eigenvectors() * direction, true};
    return {eigen.eigenvectors() * solution, false};
  }

  Vector recoverEqualities(const VectorValues& gradient, double tolerance) const {
    if (dense) return dense->equalityMultipliers(pack(gradient));
    if (data.equalities.empty()) return Vector();
    GaussianFactorGraph graph;
    for (const auto& [key, value] : gradient) {
      std::vector<std::pair<Key, Matrix>> terms;
      for (size_t row = 0; row < data.equalities.size(); ++row) {
        const auto& factor = static_cast<const JacobianFactor&>(*data.equalities[row]);
        const auto found = factor.find(key);
        if (found != factor.end()) terms.emplace_back(row, factor.getA(found).transpose());
      }
      if (!terms.empty()) graph.emplace_shared<JacobianFactor>(terms, value);
    }
    const auto recovered = solveSparseDualGraph(graph, Ordering::Colamd(graph), tolerance);
    Vector result = Vector::Zero(data.equalities.size());
    for (size_t row = 0; row < data.equalities.size(); ++row)
      if (recovered.multipliers.exists(row)) result(row) = recovered.multipliers.at(row)(0);
    return result;
  }

  // Verify a Farkas certificate in original coordinates, independent of the
  // inverse-curvature operator's rank decision: A^T*p = E^T*v, p >= 0,
  // and b^T*p - e^T*v < 0. Uncertain numerical rays are never called infeasible.
  void checkRay(const Vector& direction) const {
    VectorValues normal = zero;
    double separation = 0, scale = 0;
    const Vector normalized = direction / direction.lpNorm<Eigen::Infinity>();
    for (size_t i = 0; i < rows.size(); ++i) {
      if (normalized(i) < 0)
        throw std::runtime_error("Dual QP: invalid infeasibility direction.");
      const double weight = normalized(i) / scales(rows[i]);
      addNormal(data.inequalities[rows[i]], weight, &normal);
      const double term = weight * bound(data.inequalities[rows[i]]);
      separation += term;
      scale += std::abs(term);
    }
    const Vector equalities = recoverEqualities(normal, 1e-10);
    double normalScale = 0;
    for (const auto& [key, value] : normal) normalScale = std::max(normalScale, value.norm());
    for (size_t i = 0; i < data.equalities.size(); ++i) {
      addNormal(data.equalities[i], -equalities(i), &normal);
      const double term = equalities(i) * bound(data.equalities[i]);
      separation -= term;
      scale += std::abs(term);
    }
    const double roundoff = 1024 * std::numeric_limits<double>::epsilon();
    double residual = 0;
    for (const auto& [key, value] : normal) residual = std::max(residual, value.norm());
    if (!std::isfinite(residual) || !std::isfinite(separation) ||
        residual > roundoff * std::max(1.0, normalScale) ||
        separation >= -roundoff * std::max(1.0, scale))
      throw std::runtime_error("Dual QP: ambiguous singular working system.");
    throw std::runtime_error("Dual QP: infeasible inequality constraints (verified dual ray).");
  }

  // A tiny multiplier correction may round away when the objective forcing
  // nearly cancels A^T*lambda. Apply the corresponding primal correction
  // directly as well, then certify stationarity against the original QP.
  VectorValues refinedCandidate(const VectorValues& center, Vector* multipliers,
                                 double tolerance) {
    VectorValues candidate = point(center, *multipliers);
    if (!nonsingular || rows.empty()) return candidate;
    for (size_t refinement = 0; refinement < 3; ++refinement) {
      Vector residual(rows.size());
      double maximum = 0;
      for (size_t i = 0; i < rows.size(); ++i) {
        const double error = rowDot(data.inequalities[rows[i]], candidate) -
            bound(data.inequalities[rows[i]]);
        maximum = std::max(maximum, std::abs(error));
        residual(i) = error / scales(rows[i]);
      }
      if (maximum <= tolerance * 0.25) break;
      const Vector correction = workingSolution(-residual).first;
      for (size_t i = 0; i < rows.size(); ++i) {
        const double multiplier = (*multipliers)(rows[i]) + correction(i) / scales(rows[i]);
        if (multiplier < 0)
          throw std::runtime_error("Dual QP: refinement changes multiplier feasibility.");
        (*multipliers)(rows[i]) = multiplier;
        for (const auto& [key, response] : responses[i])
          candidate.at(key) -= correction(i) * response;
      }
    }
    return candidate;
  }

  VectorValues solve(const VectorValues& center, Vector* multipliers,
      std::vector<bool>* active, size_t* iterations, size_t maxIterations,
      double feasibilityTolerance, double multiplierTolerance) {
    // The working set persists across proximal centers; initial warm starts
    // are incorporated on the first invocation only.
    if (rows.empty())
      for (size_t row = 0; row < active->size(); ++row)
        if ((*active)[row]) append(row);
    const VectorValues unconstrained = point(center, Vector::Zero(scales.size()));
    Vector linearAll(scales.size());
    for (Eigen::Index row = 0; row < scales.size(); ++row)
      linearAll(row) = (bound(data.inequalities[row]) -
          rowDot(data.inequalities[row], unconstrained)) / scales(row);
    while (*iterations < maxIterations) {
      ++*iterations;
      Vector linear(rows.size()), current(rows.size());
      for (size_t i = 0; i < rows.size(); ++i) {
        linear(i) = linearAll(rows[i]);
        current(i) = (*multipliers)(rows[i]) * scales(rows[i]);
      }
      auto [target, ray] = workingSolution(linear);
      // Refine in original primal coordinates: forming d = b - A*xbar and
      // the Schur product can otherwise leave a boundary roundoff floor.
      if (!ray && nonsingular && !rows.empty()) {
        for (size_t refinement = 0; refinement < 2; ++refinement) {
          Vector trial = Vector::Zero(scales.size());
          for (size_t i = 0; i < rows.size(); ++i) trial(rows[i]) = target(i) / scales(rows[i]);
          const VectorValues value = point(center, trial);
          Vector residual(rows.size());
          for (size_t i = 0; i < rows.size(); ++i)
            residual(i) = (rowDot(data.inequalities[rows[i]], value) -
                bound(data.inequalities[rows[i]])) / scales(rows[i]);
          target += workingSolution(-residual).first;
        }
      }
      const Vector direction = ray ? target : Vector(target - current);
      double step = ray ? std::numeric_limits<double>::infinity() : 1;
      size_t leaving = rows.size();
      for (size_t i = 0; i < rows.size(); ++i) {
        if (direction(i) >= 0 || (!ray && target(i) >= 0)) continue;
        const double candidate = -current(i) / direction(i);
        if (candidate < step || (candidate == step &&
            (leaving == rows.size() || rows[i] < rows[leaving]))) {
          step = candidate;
          leaving = i;
        }
      }
      if (ray && leaving == rows.size()) checkRay(direction);
      const Vector next = current + step * direction;
      for (size_t i = 0; i < rows.size(); ++i)
        (*multipliers)(rows[i]) = std::max(0.0, next(i) / scales(rows[i]));
      if (leaving != rows.size()) {
        (*multipliers)(rows[leaving]) = 0;
        (*active)[rows[leaving]] = false;
        remove(leaving);
        continue;
      }
      const VectorValues candidate = refinedCandidate(center, multipliers, feasibilityTolerance);
      (void)maximumStep(candidate, candidate);
      size_t entering = active->size();
      double largest = feasibilityTolerance;
      for (size_t row = 0; row < active->size(); ++row) {
        const double violation = rowDot(data.inequalities[row], candidate) - bound(data.inequalities[row]);
        if (!std::isfinite(violation))
          throw std::runtime_error("Dual QP: non-finite primal candidate.");
        if (!(*active)[row] && violation > largest) { largest = violation; entering = row; }
        if ((*active)[row] && std::abs(violation) > feasibilityTolerance &&
            (*multipliers)(row) > multiplierTolerance)
          throw std::runtime_error("Dual QP: inaccurate working-system solve.");
      }
      if (entering == active->size()) return candidate;
      (*active)[entering] = true;
      append(entering);
    }
    throw IterationLimitError("ActiveSetSolver: dual QP failed to converge.");
  }
};

DualQpSolver::DualQpSolver(SparseQpData data, DenseQpSolver* dense,
                         const Ordering& denseOrdering)
    : impl_(std::make_unique<Impl>(std::move(data), dense, denseOrdering)) {}
DualQpSolver::~DualQpSolver() = default;
size_t DualQpSolver::reducedDimension() const { return impl_->dimension; }
bool DualQpSolver::prepare() { return impl_->prepare(); }
const DualQpSolver::Statistics& DualQpSolver::statistics() const { return impl_->statistics; }
VectorValues DualQpSolver::solve(const VectorValues& center, Vector* multipliers,
    std::vector<bool>* active, size_t* iterations, size_t maxIterations,
    double feasibilityTolerance, double multiplierTolerance) {
  return impl_->solve(center, multipliers, active, iterations, maxIterations,
                      feasibilityTolerance, multiplierTolerance);
}

Vector DualQpSolver::equalityMultipliers(const VectorValues& gradient, double tolerance) const {
  return impl_->recoverEqualities(gradient, tolerance);
}

}  // namespace gtsam::internal
