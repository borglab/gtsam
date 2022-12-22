/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file   MixtureFactor.h
 * @brief  Nonlinear Mixture factor of continuous and discrete.
 * @author Kevin Doherty, kdoherty@mit.edu
 * @author Varun Agrawal
 * @date   December 2021
 */

#pragma once

#include <gtsam/discrete/DiscreteValues.h>
#include <gtsam/hybrid/GaussianMixtureFactor.h>
#include <gtsam/hybrid/HybridNonlinearFactor.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/nonlinear/Symbol.h>

#include <algorithm>
#include <boost/format.hpp>
#include <cmath>
#include <limits>
#include <vector>

namespace gtsam {

/**
 * @brief Implementation of a discrete conditional mixture factor.
 *
 * Implements a joint discrete-continuous factor where the discrete variable
 * serves to "select" a mixture component corresponding to a NonlinearFactor
 * type of measurement.
 *
 * This class stores all factors as HybridFactors which can then be typecast to
 * one of (NonlinearFactor, GaussianFactor) which can then be checked to perform
 * the correct operation.
 */
class MixtureFactor : public HybridFactor {
 public:
  using Base = HybridFactor;
  using This = MixtureFactor;
  using shared_ptr = boost::shared_ptr<MixtureFactor>;
  using sharedFactor = boost::shared_ptr<NonlinearFactor>;

  /**
   * @brief typedef for DecisionTree which has Keys as node labels and
   * NonlinearFactor as leaf nodes.
   */
  using Factors = DecisionTree<Key, sharedFactor>;

 private:
  /// Decision tree of Gaussian factors indexed by discrete keys.
  Factors factors_;
  bool normalized_;

 public:
  MixtureFactor() = default;

  /**
   * @brief Construct from Decision tree.
   *
   * @param keys Vector of keys for continuous factors.
   * @param discreteKeys Vector of discrete keys.
   * @param factors Decision tree with of shared factors.
   * @param normalized Flag indicating if the factor error is already
   * normalized.
   */
  MixtureFactor(const KeyVector& keys, const DiscreteKeys& discreteKeys,
                const Factors& factors, bool normalized = false)
      : Base(keys, discreteKeys), factors_(factors), normalized_(normalized) {}

  /**
   * @brief Convenience constructor that generates the underlying factor
   * decision tree for us.
   *
   * Here it is important that the vector of factors has the correct number of
   * elements based on the number of discrete keys and the cardinality of the
   * keys, so that the decision tree is constructed appropriately.
   *
   * @tparam FACTOR The type of the factor shared pointers being passed in. Will
   * be typecast to NonlinearFactor shared pointers.
   * @param keys Vector of keys for continuous factors.
   * @param discreteKeys Vector of discrete keys.
   * @param factors Vector of shared pointers to factors.
   * @param normalized Flag indicating if the factor error is already
   * normalized.
   */
  template <typename FACTOR>
  MixtureFactor(const KeyVector& keys, const DiscreteKeys& discreteKeys,
                const std::vector<boost::shared_ptr<FACTOR>>& factors,
                bool normalized = false)
      : Base(keys, discreteKeys), normalized_(normalized) {
    std::vector<NonlinearFactor::shared_ptr> nonlinear_factors;
    KeySet continuous_keys_set(keys.begin(), keys.end());
    KeySet factor_keys_set;
    for (auto&& f : factors) {
      // Insert all factor continuous keys in the continuous keys set.
      std::copy(f->keys().begin(), f->keys().end(),
                std::inserter(factor_keys_set, factor_keys_set.end()));

      if (auto nf = boost::dynamic_pointer_cast<NonlinearFactor>(f)) {
        nonlinear_factors.push_back(nf);
      } else {
        throw std::runtime_error(
            "Factors passed into MixtureFactor need to be nonlinear!");
      }
    }
    factors_ = Factors(discreteKeys, nonlinear_factors);

    if (continuous_keys_set != factor_keys_set) {
      throw std::runtime_error(
          "The specified continuous keys and the keys in the factors don't "
          "match!");
    }
  }

  ~MixtureFactor() = default;

  /**
   * @brief Compute error of the MixtureFactor as a tree.
   *
   * @param continuousValues The continuous values for which to compute the
   * error.
   * @return AlgebraicDecisionTree<Key> A decision tree with the same keys
   * as the factor, and leaf values as the error.
   */
  AlgebraicDecisionTree<Key> error(const Values& continuousValues) const {
    // functor to convert from sharedFactor to double error value.
    auto errorFunc = [continuousValues](const sharedFactor& factor) {
      return factor->error(continuousValues);
    };
    DecisionTree<Key, double> errorTree(factors_, errorFunc);
    return errorTree;
  }

  /**
   * @brief Compute error of factor given both continuous and discrete values.
   *
   * @param continuousValues The continuous Values.
   * @param discreteValues The discrete Values.
   * @return double The error of this factor.
   */
  double error(const Values& continuousValues,
               const DiscreteValues& discreteValues) const {
    // Retrieve the factor corresponding to the assignment in discreteValues.
    auto factor = factors_(discreteValues);
    // Compute the error for the selected factor
    const double factorError = factor->error(continuousValues);
    if (normalized_) return factorError;
    return factorError + this->nonlinearFactorLogNormalizingConstant(
                             factor, continuousValues);
  }

  size_t dim() const {
    // TODO(Varun)
    throw std::runtime_error("MixtureFactor::dim not implemented.");
  }

  /// Testable
  /// @{

  /// print to stdout
  void print(
      const std::string& s = "",
      const KeyFormatter& keyFormatter = DefaultKeyFormatter) const override {
    std::cout << (s.empty() ? "" : s + " ");
    Base::print("", keyFormatter);
    std::cout << "\nMixtureFactor\n";
    auto valueFormatter = [](const sharedFactor& v) {
      if (v) {
        return (boost::format("Nonlinear factor on %d keys") % v->size()).str();
      } else {
        return std::string("nullptr");
      }
    };
    factors_.print("", keyFormatter, valueFormatter);
  }

  /// Check equality
  bool equals(const HybridFactor& other, double tol = 1e-9) const override {
    // We attempt a dynamic cast from HybridFactor to MixtureFactor. If it
    // fails, return false.
    if (!dynamic_cast<const MixtureFactor*>(&other)) return false;

    // If the cast is successful, we'll properly construct a MixtureFactor
    // object from `other`
    const MixtureFactor& f(static_cast<const MixtureFactor&>(other));

    // Ensure that this MixtureFactor and `f` have the same `factors_`.
    auto compare = [tol](const sharedFactor& a, const sharedFactor& b) {
      return traits<NonlinearFactor>::Equals(*a, *b, tol);
    };
    if (!factors_.equals(f.factors_, compare)) return false;

    // If everything above passes, and the keys_, discreteKeys_ and normalized_
    // member variables are identical, return true.
    return (std::equal(keys_.begin(), keys_.end(), f.keys().begin()) &&
            (discreteKeys_ == f.discreteKeys_) &&
            (normalized_ == f.normalized_));
  }

  /// @}

  /// Linearize specific nonlinear factors based on the assignment in
  /// discreteValues.
  GaussianFactor::shared_ptr linearize(
      const Values& continuousValues,
      const DiscreteValues& discreteValues) const {
    auto factor = factors_(discreteValues);
    return factor->linearize(continuousValues);
  }

  /// Linearize all the continuous factors to get a GaussianMixtureFactor.
  boost::shared_ptr<GaussianMixtureFactor> linearize(
      const Values& continuousValues) const {
    // functional to linearize each factor in the decision tree
    auto linearizeDT = [continuousValues](const sharedFactor& factor) {
      return factor->linearize(continuousValues);
    };

    DecisionTree<Key, GaussianFactor::shared_ptr> linearized_factors(
        factors_, linearizeDT);

    return boost::make_shared<GaussianMixtureFactor>(
        continuousKeys_, discreteKeys_, linearized_factors);
  }

  /**
   * If the component factors are not already normalized, we want to compute
   * their normalizing constants so that the resulting joint distribution is
   * appropriately computed. Remember, this is the _negative_ normalizing
   * constant for the measurement likelihood (since we are minimizing the
   * _negative_ log-likelihood).
   */
  double nonlinearFactorLogNormalizingConstant(const sharedFactor& factor,
                                               const Values& values) const {
    // Information matrix (inverse covariance matrix) for the factor.
    Matrix infoMat;

    // If this is a NoiseModelFactor, we'll use its noiseModel to
    // otherwise noiseModelFactor will be nullptr
    if (auto noiseModelFactor =
            boost::dynamic_pointer_cast<NoiseModelFactor>(factor)) {
      // If dynamic cast to NoiseModelFactor succeeded, see if the noise model
      // is Gaussian
      auto noiseModel = noiseModelFactor->noiseModel();

      auto gaussianNoiseModel =
          boost::dynamic_pointer_cast<noiseModel::Gaussian>(noiseModel);
      if (gaussianNoiseModel) {
        // If the noise model is Gaussian, retrieve the information matrix
        infoMat = gaussianNoiseModel->information();
      } else {
        // If the factor is not a Gaussian factor, we'll linearize it to get
        // something with a normalized noise model
        // TODO(kevin): does this make sense to do? I think maybe not in
        // general? Should we just yell at the user?
        auto gaussianFactor = factor->linearize(values);
        infoMat = gaussianFactor->information();
      }
    }

    // Compute the (negative) log of the normalizing constant
    return -(factor->dim() * log(2.0 * M_PI) / 2.0) -
           (log(infoMat.determinant()) / 2.0);
  }
};

}  // namespace gtsam
