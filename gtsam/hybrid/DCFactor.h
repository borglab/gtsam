/* ----------------------------------------------------------------------------
 * Copyright 2020 The Ambitious Folks of the MRG
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   DCFactor.h
 * @brief  Custom discrete-continuous factor
 * @author Kevin Doherty, kdoherty@mit.edu
 * @date   December 2021
 */

#pragma once

#include <gtsam/discrete/DecisionTreeFactor.h>
#include <gtsam/discrete/DiscreteKey.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/nonlinear/Symbol.h>
#include <math.h>

#include <algorithm>
#include <limits>
#include <string>
#include <vector>

namespace gtsam {

// Forward declaration of subclass.
class DCGaussianMixtureFactor;

/**
 * @brief Abstract class implementing a discrete-continuous factor.
 *
 * keys_ member variable stores keys for *continuous* variables.
 * discreteKeys_ contains the keys (plus cardinalities) for *discrete*
 * variables.
 */
class DCFactor : public Factor {
 protected:
  // Set of DiscreteKeys for this factor.
  DiscreteKeys discreteKeys_;
  // For bookkeeping
  KeyVector continuousKeys_;

 public:
  using Base = Factor;
  using shared_ptr = boost::shared_ptr<DCFactor>;

  DCFactor() = default;

  /// Used in constructor to gather all keys, continuous and discrete.
  static KeyVector AllKeys(const KeyVector& continuousKeys,
                           const DiscreteKeys& discreteKeys) {
    auto result = continuousKeys;
    for (auto& dk : discreteKeys) {
      result.push_back(dk.first);
    }
    return result;
  }

  /**
   * Base constructor for a DCFactor from a set of discrete keys and continuous
   * keys
   *
   * @param continuousKeys - the keys for *continuous* variables
   * @param discreteKeys - the keys for *discrete* variables
   */
  DCFactor(const KeyVector& continuousKeys, const DiscreteKeys& discreteKeys)
      : Base(AllKeys(continuousKeys, discreteKeys)),
        discreteKeys_(discreteKeys),
        continuousKeys_(continuousKeys) {}

  DCFactor& operator=(const DCFactor& rhs) {
    Base::operator=(rhs);
    discreteKeys_ = rhs.discreteKeys_;
    return *this;
  }

  virtual ~DCFactor() = default;

  /**
   * Return the error for the DCFactor given an assignment to the variables
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
   * @param continuousVals - the values assigned to the continuous variables,
   * must contain keys in keys_ member variable
   * @param discreteVals - likewise, the values assigned to discrete variables,
   * must contain keys in discreteKeys_
   * @return error (usually the negative log-likelihood) for the measurement
   * model as a double.
   */
  virtual double error(const Values& continuousVals,
                       const DiscreteValues& discreteVals) const = 0;

  /**
   * Linearize the error function with respect to the continuous
   * variables (given in `keys_`) at the point specified by `continuousVals`.
   * Since this Jacobian can be dependent on the assignment to discrete
   * variables as well, they are required as a parameter.
   *
   * This will be specific to the model being implemented, so it is a pure
   * virtual function and must be overridden.
   *
   * @param continuousVals - Assignment to the continuous variables in `keys_`
   * @param discreteVals - Likewise, assignment to the discrete variables in
   * `discreteKeys__`.
   */
  virtual GaussianFactor::shared_ptr linearize(
      const Values& continuousVals,
      const DiscreteValues& discreteVals) const = 0;

  /**
   * @brief Linearize all the continuous factors only with respect to the
   * continuous variables (as given in `keys_`).
   *
   * This `linearize` is different from the `linearize(continuous, discrete)` in
   * that it assumes no assignment of discrete keys and linearizes all
   * continuous factors associated with this factor.
   *
   * @param continuousVals The continuous variables referenced by `keys_`.
   * @return DCGaussianMixtureFactor::shared_ptr
   */
  virtual boost::shared_ptr<DCGaussianMixtureFactor> linearize(
      const Values& continuousVals) const = 0;

  /**
   * Returns true when the DCFactor is equal to `other`
   *
   * This will be specific to the model being implemented, so it is a pure
   * virtual function and must be overridden.
   *
   * @param other - the DCFactor for which to test equality.
   * @param tol = 1e-9 - numerical tolerance for any floating point comparisons.
   */
  virtual bool equals(const DCFactor& other, double tol = 1e-9) const = 0;

  /**
   * Returns the number of rows in the Jacobian with respect to the continuous
   * variables for this factor. Internally this is used in the conversion to a
   * NonlinearFactor
   *
   * TODO(kevin): not sure if needed???
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
  const DiscreteKeys& discreteKeys() const { return discreteKeys_; }

  /*
   * Return only the continuous keys for this factor.
   */
  const KeyVector& continuousKeys() const { return continuousKeys_; }

  /**
   * Converts the DCFactor to a DecisionTreeFactor. Internally, this will
   * be used to generate a DiscreteFactor type, which itself requires a
   * conversion function to DecisionTreeFactor for inference using GTSAM.
   *
   * Performing this conversion can be problem specific, so we allow for the
   * option to override, but try to implement a sensible default: we assume the
   * error function can be called setting each discrete key's variable
   * individually, and the overall DCFactor can itself be factored as a product
   * of unary DecisionTreeFactors.
   *
   * Alternative implementations might consider something like the
   * AllDiff approach here:
   * https://github.com/borglab/gtsam/blob/43e8f1e5aeaf11890262722c1e5e04a11dbf9d75/gtsam_unstable/discrete/AllDiff.cpp#L43
   *
   * @param continuousVals - an assignment to the continuous variables
   * @param discreteVals -
   * @return a DecisionTreeFactor implementing this DCFactor.
   */
  virtual DecisionTreeFactor toDecisionTreeFactor(
      const Values& continuousVals, const DiscreteValues& discreteVals) const;

  /**
   * Calculate a normalizing constant for this DCFactor. Most implementations
   * will be able to use the helper function
   * nonlinearFactorLogNormalizingConstant provided below for most of the
   * calculation.
   * TODO(Kurran) is this the cleanest way to do this? Seems necessary for the
   * DCMaxMixtureFactor implementations etc...
   */
  virtual double logNormalizingConstant(const Values& values) const;

  /**
   * Default for computing the _negative_ normalizing constant for the
   * measurement likelihood (since we are minimizing the _negative_
   * log-likelihood), to be used as a utility for computing the
   * DCFactorLogNormalizingConstant.
   */
  template <typename NonlinearFactorType>
  double nonlinearFactorLogNormalizingConstant(
      const NonlinearFactorType& factor, const Values& values) const {
    // Information matrix (inverse covariance matrix) for the factor.
    Matrix infoMat;

    // NOTE: This is sloppy, is there a cleaner way?
    boost::shared_ptr<NonlinearFactorType> fPtr =
        boost::make_shared<NonlinearFactorType>(factor);
    boost::shared_ptr<NonlinearFactorType> factorPtr(fPtr);

    // If this is a NoiseModelFactor, we'll use its noiseModel to
    // otherwise noiseModelFactor will be nullptr
    boost::shared_ptr<NoiseModelFactor> noiseModelFactor =
        boost::dynamic_pointer_cast<NoiseModelFactor>(factorPtr);
    if (noiseModelFactor) {
      // If dynamic cast to NoiseModelFactor succeeded, see if the noise model
      // is Gaussian
      noiseModel::Base::shared_ptr noiseModel = noiseModelFactor->noiseModel();

      boost::shared_ptr<noiseModel::Gaussian> gaussianNoiseModel =
          boost::dynamic_pointer_cast<noiseModel::Gaussian>(noiseModel);
      if (gaussianNoiseModel) {
        // If the noise model is Gaussian, retrieve the information matrix
        infoMat = gaussianNoiseModel->information();
      } else {
        // If the factor is not a Gaussian factor, we'll linearize it to get
        // something with a normalized noise model
        // TODO(kevin): does this make sense to do? I think maybe not in
        // general? Should we just yell at the user?
        GaussianFactor::shared_ptr gaussianFactor = factor.linearize(values);
        infoMat = gaussianFactor->information();
      }
    }

    // Compute the (negative) log of the normalizing constant
    return (factor.dim() * log(2.0 * M_PI) / 2.0) -
           (log(infoMat.determinant()) / 2.0);
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
   * @param continuousVals - an assignment to the continuous valued variables
   * @return a vector of length == cardinality of dk specifying the probability
   * of each possible assignment to dk.
   */
  std::vector<double> evalProbs(const DiscreteKey& dk,
                                const Values& continuousVals) const;

  /**
   * Take the product of this DCFactor (as a DecisionTreeFactor)
   * conditioned on an assignment to the continous variables, `continuousVals`
   * with another DecisionTreeFactor `f`. Used internally by GTSAM to
   * solve discrete factor graphs.
   *
   * @param f - the DecisionTreeFactor to be multiplied by this DCFactor
   * @param continuousVals - an assignment to the continuous variables
   * (specified by keys_).
   * @return a DecisionTreeFactor representing the product of this factor
   * with `f`.
   */
  DecisionTreeFactor conditionalTimes(const DecisionTreeFactor& f,
                                      const Values& continuousVals,
                                      const DiscreteValues& discreteVals) const;
};
}  // namespace gtsam
