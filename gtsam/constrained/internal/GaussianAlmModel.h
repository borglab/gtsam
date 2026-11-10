/* ----------------------------------------------------------------------------
 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */
#pragma once

#include <gtsam/constrained/internal/AugmentedLagrangianEngine.h>
#include <gtsam/linear/HessianFactor.h>

namespace gtsam {
namespace internal {

/// Fixed affine model with owned mutable objective views and shared inputs.
class GaussianAlmModel {
 public:
  using Primal = VectorValues;

  /// Normalize constraint scales and allocate shifted objective storage once.
  GaussianAlmModel(const GaussianFactorGraph& costs,
                   const GaussianFactorGraph& equalities,
                   const GaussianFactorGraph& inequalities,
                   bool profiling = false)
      : costs_(costs) {
    AlmTimer timer(profiling, &preparationSeconds_);
    for (const auto* graph : {&costs, &equalities, &inequalities})
      for (const auto& factor : *graph) {
        if (!factor)
          throw std::invalid_argument(
              "ALM Gaussian inputs contain a null factor");
        for (auto it = factor->begin(); it != factor->end(); ++it) {
          size_t dim = factor->getDim(it);
          auto [entry, inserted] = dimensions_.emplace(*it, dim);
          if (dim == 0 || (!inserted && entry->second != dim))
            throw std::invalid_argument(
                "ALM Gaussian variable dimensions disagree");
        }
      }
    for (const auto& factor : costs) {
      if (!dynamic_cast<const JacobianFactor*>(factor.get()) &&
          !dynamic_cast<const HessianFactor*>(factor.get()))
        throw std::invalid_argument(
            "ALM objectives require Jacobian or Hessian factors");
      shifted_.push_back(factor->clone());
      size_t dim = 0;
      for (auto it = factor->begin(); it != factor->end(); ++it)
        dim += factor->getDim(it);
      vectors_.push_back(Vector::Zero(dim));
    }
    for (const auto& factor : equalities)
      equalityRows_.push_back(normalize(factor));
    for (const auto& factor : inequalities) {
      const auto whitened = normalize(factor);
      for (DenseIndex row = 0; row < whitened->getb().size(); ++row) {
        std::vector<std::pair<Key, Matrix>> terms;
        for (auto it = whitened->begin(); it != whitened->end(); ++it)
          terms.emplace_back(*it, whitened->getA(it).row(row));
        inequalityRows_.push_back(std::make_shared<JacobianFactor>(
            terms, Vector1(whitened->getb()(row))));
      }
    }
  }

  /// Zero vectors covering objective and constraint-only variables.
  VectorValues zeroValues() const {
    VectorValues values;
    for (const auto& [key, dim] : dimensions_)
      values.insert(key, Vector::Zero(dim));
    return values;
  }

  /// Check supplied vector structure and finiteness once at the public
  /// boundary.
  void validateValues(const VectorValues& values) const {
    if (values.size() != dimensions_.size())
      throw std::invalid_argument(
          "ALM initial values must cover exactly the input keys");
    for (const auto& [key, dim] : dimensions_)
      if (!values.exists(key) ||
          values.at(key).size() != static_cast<DenseIndex>(dim) ||
          !values.at(key).allFinite())
        throw std::invalid_argument(
            "ALM initial vector dimensions or entries are invalid");
  }

  /// Equality multiplier blocks follow original factor boundaries.
  std::vector<size_t> equalityDimensions() const {
    std::vector<size_t> result;
    for (const auto& row : equalityRows_) result.push_back(row->getb().size());
    return result;
  }
  /// Scalar inequality rows in original graph and row order.
  size_t inequalityCount() const { return inequalityRows_.size(); }
  /// Native objective factors are copied once, never inside the inner loop.
  size_t objectiveFactorCopies() const { return shifted_.size(); }
  /// Preparation includes normalization and owned coefficient storage.
  double preparationSeconds() const { return preparationSeconds_; }
  /// All constraint derivative matrices are invariant throughout this call.
  bool fixedCoefficients() const { return true; }
  /// Allocate a keyed gradient/scratch layout without converting through
  /// Values.
  VectorValues zeroVectors(const Primal& values) const {
    return VectorValues::Zero(values);
  }
  /// Apply a direct Euclidean step with preserved key blocks.
  Primal retract(const Primal& values, const VectorValues& delta) const {
    return values + delta;
  }

  /// Evaluate original Gaussian costs and affine residuals without linearizing.
  AlmPoint evaluate(const Primal& values, bool derivatives,
                    AugmentedLagrangianStatistics*) const {
    AlmPoint point;
    point.cost = costs_.error(values);
    for (const auto& row : equalityRows_)
      point.equalities.push_back(row->unweighted_error(values));
    for (const auto& row : inequalityRows_)
      point.inequalities.push_back(row->unweighted_error(values)(0));
    if (derivatives) {
      point.equalityRows = equalityRows_;
      point.inequalityRows = inequalityRows_;
    }
    point.derivatives = derivatives;
    return point;
  }

  /// Accumulate the original quadratic gradient directly into retained storage.
  void addCostGradient(const Primal& values, const AlmPoint&,
                       VectorValues* gradient) const {
    for (const auto& factor : costs_)
      for (Key key : factor->keys())
        gradient->at(key) += factor->gradient(key, values);
  }

  /// Shift only RHS/linear/constant terms; objective matrices stay untouched.
  const GaussianFactorGraph& linearizeCosts(const Primal& values,
                                            const AlmPoint&) {
    for (size_t i = 0; i < costs_.size(); ++i) {
      if (const auto* jacobian =
              dynamic_cast<const JacobianFactor*>(costs_[i].get())) {
        static_cast<JacobianFactor&>(*shifted_[i]).getb() =
            -jacobian->unweighted_error(values);
      } else {
        const auto& original = static_cast<const HessianFactor&>(*costs_[i]);
        auto& shifted = static_cast<HessianFactor&>(*shifted_[i]);
        Vector& x = vectors_[i];
        DenseIndex offset = 0;
        for (Key key : original.keys()) {
          const auto& block = values.at(key);
          x.segment(offset, block.size()) = block;
          offset += block.size();
        }
        const Vector product = original.informationView() * x;
        shifted.linearTerm() = original.linearTerm();
        shifted.linearTerm().col(0) -= product;
        shifted.constantTerm() = original.constantTerm() + x.dot(product) -
                                 2 * x.dot(original.linearTerm().col(0));
      }
    }
    return shifted_;
  }

 private:
  double preparationSeconds_ = 0;
  GaussianFactorGraph costs_, shifted_;
  std::vector<Vector> vectors_;
  std::map<Key, size_t> dimensions_;
  std::vector<JacobianFactor::shared_ptr> equalityRows_, inequalityRows_;

  /// Whiten each affine input once, keeping exact constraint semantics
  /// separate.
  static JacobianFactor::shared_ptr normalize(
      const GaussianFactor::shared_ptr& factor) {
    const auto* row = dynamic_cast<const JacobianFactor*>(factor.get());
    if (!row)
      throw std::invalid_argument("ALM constraints require Jacobian factors");
    if (row->get_model()) {
      const Vector sigmas = row->get_model()->sigmas();
      if (!sigmas.allFinite() || (sigmas.array() <= 0).any())
        throw std::invalid_argument(
            "ALM constraint scales must be finite and positive");
    }
    if (!row->augmentedJacobianUnweighted().allFinite())
      throw std::invalid_argument("ALM constraint coefficients must be finite");
    auto normalized = std::make_shared<JacobianFactor>(row->whiten());
    normalized->setModel(false, Vector::Ones(normalized->getb().size()));
    return normalized;
  }
};

}  // namespace internal
}  // namespace gtsam
