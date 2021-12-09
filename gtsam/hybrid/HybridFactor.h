/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file HybridFactor.h
 * @date December, 2021
 * @brief Custom discrete-continuous factor
 * @author Kevin Doherty, Varun Agrawal
 */

#pragma once

#include <gtsam/discrete/DecisionTreeFactor.h>
#include <gtsam/discrete/DiscreteFactor.h>
#include <gtsam/discrete/DiscreteKey.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/nonlinear/Symbol.h>
#include <math.h>

#include <algorithm>
#include <limits>
#include <string>
#include <vector>

namespace gtsam {

// TODO use gtsam::Values
using DiscreteValues = gtsam::DiscreteFactor::Values;

/**
 * @brief Function to normalize a set of log probabilities.
 *
 * Normalizing a set of log probabilities in a numerically stable way is
 * tricky. To avoid overflow/underflow issues, we compute the largest
 * (finite) log probability and subtract it from each log probability before
 * normalizing. This comes from the observation that if:
 *    p_i = exp(L_i) / ( sum_j exp(L_j) ),
 * Then,
 *    p_i = exp(Z) exp(L_i - Z) / (exp(Z) sum_j exp(L_j - Z))
 *        = exp(L_i - Z) / ( sum_j exp(L_j - Z) )
 *
 * Setting Z = max_j L_j, we can avoid numerical issues that arise when all
 * of the (unnormalized) log probabilities are either very large or very
 * small.
 */
std::vector<double> expNormalize(const std::vector<double>& log_probs) {
  double max_log_prob = -std::numeric_limits<double>::infinity();
  for (size_t i = 0; i < log_probs.size(); i++) {
    double log_prob = log_probs[i];
    if ((log_prob != std::numeric_limits<double>::infinity()) &&
        log_prob > max_log_prob) {
      max_log_prob = log_prob;
    }
  }

  // After computing the max = "Z" of the log probabilities L_i, we compute
  // the log of the normalizing constant, log S, where S = sum_j exp(L_j - Z).
  double total = 0.0;
  for (size_t i = 0; i < log_probs.size(); i++) {
    double prob_prime = exp(log_probs[i] - max_log_prob);
    total += prob_prime;
  }
  double log_total = log(total);

  // Now we compute the (normalized) probability (for each i):
  // p_i = exp(L_i - Z - log S)
  double check_normalization = 0.0;
  std::vector<double> probs;
  for (size_t i = 0; i < log_probs.size(); i++) {
    double prob = exp(log_probs[i] - max_log_prob - log_total);
    probs.push_back(prob);
    check_normalization += prob;
  }

  // Numerical tolerance for floating point comparisons
  double tol = 1e-9;

  if (!gtsam::fpEqual(check_normalization, 1.0, tol)) {
    std::string errMsg =
        std::string("expNormalize failed to normalize probabilities. ") +
        std::string("Expected normalization constant = 1.0. Got value: ") +
        std::to_string(check_normalization) +
        std::string(
            "\n This could have resulted from numerical overflow/underflow.");
    throw std::logic_error(errMsg);
  }
  return probs;
}

/**
 * @brief Abstract class implementing a discrete-continuous factor.
 *
 * `keys_` member variable stores keys for *continuous* variables.
 * `discrete_keys_` contains the keys (plus cardinalities) for *discrete*
 * variables.
 */
class GTSAM_EXPORT HybridFactor : public gtsam::Factor {
 protected:
  // Set of DiscreteKeys for this factor.
  gtsam::DiscreteKeys discrete_keys_;

 public:
  using Base = gtsam::Factor;

  HybridFactor() = default;

  /**
   * Base constructor for a HybridFactor from a set of discrete keys and
   * continuous keys
   *
   * @param continuous_keys - the keys for *continuous* variables
   * @param discrete_keys - the keys for *discrete* variables
   */
  HybridFactor(const gtsam::KeyVector& continuous_keys,
               const gtsam::DiscreteKeys& discrete_keys)
      : Base(continuous_keys), discrete_keys_(discrete_keys) {}

  HybridFactor& operator=(const HybridFactor& rhs) {
    Base::operator=(rhs);
    discrete_keys_ = rhs.discrete_keys_;
    return *this;
  }

  virtual ~HybridFactor() = default;

  /**
   * Return the error for the HybridFactor given an assignment to the variables
   * referenced by the discrete keys and the continuous keys (`keys_`).
   *
   * This will be specific to the model being implemented, so it is a pure
   * virtual function and must be overridden.
   *
   * NOTE: Depending on the application, it may be important that the error
   * function is *normalized*, i.e. it corresponds to a proper negative
   * log-likelihood, rather than the negative log-likelihood with normalization
   * constants discarded.
   *
   * @param continuous_values - the values assigned to the continuous variables,
   * must contain keys in keys_ member variable
   * @param discrete_values - likewise, the values assigned to discrete
   * variables, must contain keys in discrete_keys_
   * @return error (usually the negative log-likelihood) for the measurement
   * model as a double.
   */
  virtual double error(
      const gtsam::Values& continuous_values,
      const gtsam::DiscreteFactor::Values& discrete_values) const = 0;

  /**
   * Linearize the error function with respect to the continuous
   * variables (given in `keys_`) at the point specified by `continuous_values`.
   * Since this Jacobian can be dependent on the assignment to discrete
   * variables as well, they are required as a parameter.
   *
   * This will be specific to the model being implemented, so it is a pure
   * virtual function and must be overridden.
   *
   * @param continuous_values - Assignment to the continuous variables in
   * `keys_`
   * @param discrete_values - Likewise, assignment to the discrete variables in
   * `discrete_keys__`.
   */
  virtual boost::shared_ptr<gtsam::GaussianFactor> linearize(
      const gtsam::Values& continuous_values,
      const DiscreteValues& discrete_values) const = 0;

  /**
   * Returns true when the HybridFactor is equal to `other`
   *
   * This will be specific to the model being implemented, so it is a pure
   * virtual function and must be overridden.
   *
   * @param other - the HybridFactor for which to test equality.
   * @param tol = 1e-9 - numerical tolerance for any floating point comparisons.
   */
  virtual bool equals(const HybridFactor& other, double tol = 1e-9) const = 0;

  // TODO(kevin): not sure if needed???
  /**
   * Returns the number of rows in the Jacobian with respect to the continuous
   * variables for this factor. Internally this is used in the conversion to a
   * gtsam::NonlinearFactor
   *
   * This will be specific to the model being implemented, so it is a pure
   * virtual function and must be overridden
   *
   * @return the dimension (number of rows) in the Jacobian of the `error`
   * function with respect to continuous variables for
   */
  virtual size_t dim() const = 0;

  /*
   * Return the discrete keys for this factor.
   */
  gtsam::DiscreteKeys discreteKeys() const { return discrete_keys_; }

  /**
   * Converts the HybridFactor to a gtsam::DecisionTreeFactor. Internally, this
   * will be used to generate a gtsam::DiscreteFactor type, which itself
   * requires a conversion function to gtsam::DecisionTreeFactor for inference
   * using GTSAM.
   *
   * Performing this conversion can be problem specific, so we allow for the
   * option to override, but try to implement a sensible default: we assume the
   * error function can be called setting each discrete key's variable
   * individually, and the overall HybridFactor can itself be factored as a
   * product of unary gtsam::DecisionTreeFactors.
   *
   * Alternative implementations might consider something like the
   * gtsam::AllDiff approach here:
   * https://github.com/borglab/gtsam/blob/43e8f1e5aeaf11890262722c1e5e04a11dbf9d75/gtsam_unstable/discrete/AllDiff.cpp#L43
   *
   * @param continuous_values - an assignment to the continuous variables.
   * @param discrete_values - an assignment to the discrete variables.
   * @return a DecisionTreeFactor implementing this HybridFactor.
   */
  virtual DecisionTreeFactor toDecisionTreeFactor(
      const Values& continuous_values,
      const DiscreteValues& discrete_values) const {
    DecisionTreeFactor converted;
    for (const DiscreteKey& dkey : discrete_keys_) {
      std::vector<double> probs = evalProbs(dkey, continuous_values);
      // Cardinality of gtsam::DiscreteKey is located at `second`
      assert(probs.size() == dkey.second);
      DecisionTreeFactor unary(dkey, probs);
      converted = converted * unary;
    }
    return converted;
  }

  // TODO(Kurran): is this the cleanest way to do this? Seems necessary for the
  // DCMaxMixtureFactor implementations etc...
  /**
   * Calculate a normalizing constant for this HybridFactor. Most
   * implementations will be able to use the helper function
   * nonlinearFactorLogNormalizingConstant provided below for most of the
   * calculation.
   */
  virtual double logNormalizingConstant(const Values& values) const {
    throw std::logic_error(
        "Normalizing constant not implemented."
        "One or more of the factors in use requires access to the normalization"
        "constant for a child class of HybridFactor, "
        "but`logNormalizingConstant` "
        "has not been overridden.");
  }

  /**
   * Default for computing the _negative_ normalizing constant for the
   * measurement likelihood (since we are minimizing the _negative_
   * log-likelihood), to be used as a utility for computing the
   * HybridFactorLogNormalizingConstant.
   */
  template <typename NonlinearFactorType>
  double nonlinearFactorLogNormalizingConstant(
      const NonlinearFactorType& factor, const Values& values) const {
    // Information matrix (inverse covariance matrix) for the factor.
    Matrix info_matrix;

    auto factor_ptr = boost::make_shared<NonlinearFactorType>(factor);

    // If this is a NoiseModelFactor, we'll use its noiseModel to get the
    // information matrix otherwise noise_model_factor will be nullptr
    auto noise_model_factor =
        boost::dynamic_pointer_cast<NoiseModelFactor>(factor_ptr);

    if (noise_model_factor) {
      // If dynamic cast to NoiseModelFactor succeeded, see if the noise model
      // is Gaussian
      noiseModel::Base::shared_ptr noise_model =
          noise_model_factor->noiseModel();

      boost::shared_ptr<noiseModel::Gaussian> gaussian_noise_model =
          boost::dynamic_pointer_cast<noiseModel::Gaussian>(noise_model);

      if (gaussian_noise_model) {
        // If the noise model is Gaussian, retrieve the information matrix
        info_matrix = gaussian_noise_model->information();
      } else {
        // If the factor is not a Gaussian factor, we'll linearize it to get
        // something with a normalized noise model
        // TODO(kevin): does this make sense to do? I think maybe not in
        // general? Should we just yell at the user?
        boost::shared_ptr<gtsam::GaussianFactor> gaussian_factor =
            factor.linearize(values);
        info_matrix = gaussian_factor->information();
      }
    }

    // Compute the (negative) log of the normalizing constant
    return (factor.dim() * log(2.0 * M_PI) / 2.0) -
           (log(info_matrix.determinant()) / 2.0);
  }

  /**
   * Obtain the likelihood function for a single discrete variable conditioned
   * on continuous values. Since the `error` function typically represents the
   * negative log-likelihood, we compute `exp(-error)`.
   *
   * NOTE: Depending on the application, it may be important that the error
   * function is *normalized*, i.e. it corresponds to a proper negative
   * log-likelihood, rather than the negative log-likelihood with normalization
   * constants discarded.
   *
   * For this, we assume that the `error` function can be called with only a
   * single discrete value set, though this may be dependent on the particular
   * implementation behavior desired. At present, this is only used internally
   * to specify the default behavior for `toDecisionTreeFactor`.
   *
   * @param dk - the discrete key to evaluate the likelihood for
   * @param continuous_values - an assignment to the continuous valued variables
   * @return a vector of length == cardinality of dk specifying the probability
   * of each possible assignment to dk.
   */
  std::vector<double> evalProbs(const gtsam::DiscreteKey& discrete_key,
                                const gtsam::Values& continuous_values) const {
    /*
     * Normalizing a set of log probabilities in a numerically stable way is
     * tricky. To avoid overflow/underflow issues, we compute the largest
     * (finite) log probability and subtract it from each log probability before
     * normalizing. This comes from the observation that if:
     *    p_i = exp(L_i) / ( sum_j exp(L_j) ),
     * Then,
     *    p_i = exp(Z) exp(L_i - Z) / (exp(Z) sum_j exp(L_j - Z)),
     *        = exp(L_i - Z) / ( sum_j exp(L_j - Z) )
     *
     * Setting Z = max_j L_j, we can avoid numerical issues that arise when all
     * of the (unnormalized) log probabilities are either very large or very
     * small.
     */
    std::vector<double> log_probs;
    for (size_t i = 0; i < discrete_key.second; i++) {
      DiscreteValues discrete_vals;
      discrete_vals[discrete_key.first] = i;
      // Recall: `error` returns -log(prob), so we compute exp(-error) to
      // recover probability
      double log_prob = -error(continuous_values, discrete_vals);
      log_probs.push_back(log_prob);
    }
    return expNormalize(log_probs);
  }

  /**
   * Take the product of this HybridFactor (as a gtsam::DecisionTreeFactor)
   * conditioned on an assignment to the continous variables,
   * `continuous_values` with another gtsam::DecisionTreeFactor `f`. Used
   * internally by GTSAM to solve discrete factor graphs.
   *
   * @param f - the gtsam::DecisionTreeFactor to be multiplied by this
   * HybridFactor
   * @param continuous_values - an assignment to the continuous variables
   * (specified by keys_).
   * @return a gtsam::DecisionTreeFactor representing the product of this factor
   * with `f`.
   */
  gtsam::DecisionTreeFactor conditionalTimes(
      const gtsam::DecisionTreeFactor& f,
      const gtsam::Values& continuous_values,
      const DiscreteValues& discrete_values) const {
    return toDecisionTreeFactor(continuous_values, discrete_values) * f;
  }
};
}  // namespace gtsam