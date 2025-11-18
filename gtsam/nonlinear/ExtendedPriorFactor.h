/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 *  @file  ExtendedPriorFactor.h
 *  @author Frank Dellaert
 **/
#pragma once

#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

#include <optional>

namespace gtsam {

/**
 * A class for a soft prior on any Value type, but with a non-zero mean
 * in the tangent space.
 * The error is `e(x) = -Local(x, origin) - mean`.
 * The loss is `0.5 * ||e(x)||^2_Sigma` for a Gaussian noise model,
 * and `rho(e(x))` for a robust noise model `rho`.
 * The likelihood is `exp(-loss)`.
 *
 * Note: The "extended concentrated Gaussian" on Lie groups is defined as
 * `~ exp{-0.5*|log(origin_^{-1} x) - mean|^2_Sigma}`. Our implementation,
 * with a Gaussian noise model, is identical (up to a constant) to this
 * for all Lie Groups.
 *
 * @ingroup nonlinear
 */
template <class VALUE>
class ExtendedPriorFactor : public NoiseModelFactorN<VALUE> {
 public:
  typedef VALUE T;

  /// Provide access to the Matrix& version of evaluateError:
  using NoiseModelFactor1<VALUE>::evaluateError;

 protected:
  typedef NoiseModelFactorN<VALUE> Base;

  VALUE origin_; /** The point in manifold at which tangent space is rooted. */
  std::optional<Vector>
      mean_; /** Mean in the tangent space, default nullopt. */

  /// concept check by type
  GTSAM_CONCEPT_TESTABLE_TYPE(T)

 public:
  /// Typedef to this class
  typedef ExtendedPriorFactor<VALUE> This;

  /// @name Standard Constructors
  /// @{

  /// default constructor - only use for serialization
  ExtendedPriorFactor() {}

  /// Constructor with noise model and optional mean in tangent space
  ExtendedPriorFactor(Key key, const T& origin, const SharedNoiseModel& model)
      : Base(model, key), origin_(origin) {}

  /// Constructor with noise model and optional mean in tangent space
  ExtendedPriorFactor(Key key, const T& origin, const Vector& mean,
                      const SharedNoiseModel& model)
      : Base(model, key), origin_(origin), mean_(mean) {
    if (mean.size() != static_cast<Eigen::Index>(model->dim()))
      throw std::invalid_argument(
          "ConcentratedGaussian: mean dimension does not match noise model");
  }

  /// Constructor with covariance matrix (zero mean in tangent space)
  ExtendedPriorFactor(Key key, const T& origin, const Matrix& covariance)
      : Base(noiseModel::Gaussian::Covariance(covariance), key),
        origin_(origin) {}

  /// Constructor with mean (in tangent space) and covariance matrix
  ExtendedPriorFactor(Key key, const T& origin, const Vector& mean,
                      const Matrix& covariance)
      : Base(noiseModel::Gaussian::Covariance(covariance), key),
        origin_(origin),
        mean_(mean) {
    if (mean.size() != covariance.rows() ||
        covariance.rows() != covariance.cols())
      throw std::invalid_argument(
          "ConcentratedGaussian: mean and covariance dimensions do not match");
  }

  /// @}
  /// @name Standard Destructor
  /// @{

  ~ExtendedPriorFactor() override {}

  /// @}
  /// @name Testable
  /// @{

  /// print
  void print(const std::string& s, const KeyFormatter& keyFormatter =
                                       DefaultKeyFormatter) const override {
    std::cout << s << "ExtendedPriorFactor on " << keyFormatter(this->key())
              << "\n";
    traits<T>::Print(origin_, "  origin: ");
    if (mean_) {
      gtsam::print(*mean_, "  tangent space mean: ");
    }
    if (this->noiseModel_)
      this->noiseModel_->print("  noise model: ");
    else
      std::cout << "no noise model" << std::endl;
  }

  /// equals
  bool equals(const NonlinearFactor& expected,
              double tol = 1e-9) const override {
    const This* e = dynamic_cast<const This*>(&expected);
    if (!e) return false;
    bool mean_equals =
        (!mean_ && !e->mean_) ||
        (mean_ && e->mean_ && equal_with_abs_tol(*mean_, *e->mean_, tol));
    return Base::equals(*e, tol) &&
           traits<T>::Equals(origin_, e->origin_, tol) && mean_equals;
  }

  /// @}
  /// @name Factor
  /// @{

  /// @return a deep copy of this factor
  gtsam::NonlinearFactor::shared_ptr clone() const override {
    return std::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new This(*this)));
  }

  /// vector of errors
  Vector evaluateError(const T& x, OptionalMatrixType H) const override {
    if (H) {
      (*H) = Matrix::Identity(traits<T>::GetDimension(x),
                              traits<T>::GetDimension(x));
    }
    // manifold equivalent of z-x -> Local(x,z)
    Vector error = -traits<T>::Local(x, origin_);
    if (mean_) {
      return error - *mean_;
    }
    return error;
  }

  /// Bring the error(const Values&) method into scope.
  using Base::error;

  /// Compute the negative log-likelihood of a given value
  double error(const T& x) const {
    Vector e = evaluateError(x);
    double squared_mahalanobis_distance =
        this->noiseModel_->squaredMahalanobisDistance(e);
    return this->noiseModel_->loss(squared_mahalanobis_distance);
  }

  /// Compute the likelihood of a given value
  double likelihood(const T& x) const { return exp(-error(x)); }

  /// @}
  /// @name Access
  /// @{

  const VALUE& origin() const { return origin_; }
  const std::optional<Vector>& mean() const { return mean_; }

  /// Get the Gaussian noise model, or return nullopt/throw if not Gaussian.
  noiseModel::Gaussian::shared_ptr gaussianModel(
      const std::string& method = "<unknown>",
      bool throwOnFailure = false) const {
    using noiseModel::Gaussian;
    const auto& model = this->noiseModel();
    if (!model) {
      if (throwOnFailure) {
        throw std::runtime_error(method + " requires a noise model");
      }
      return nullptr;
    }
    auto g = std::dynamic_pointer_cast<Gaussian>(model);
    if (!g) {
      if (throwOnFailure) {
        throw std::runtime_error(method +
                                 " is only implemented for Gaussian noise "
                                 "models. The noise model used is of type " +
                                 std::string(typeid(*model).name()));
      }
      return nullptr;
    }
    return g;
  }

  /// Get Gaussian covariance, or return nullopt/throw if not Gaussian.
  std::optional<Matrix> covariance(const std::string& method = "<unknown>",
                                   bool throwOnFailure = false) const {
    auto gaussian = gaussianModel(method, throwOnFailure);
    if (!gaussian) return std::nullopt;
    return gaussian->covariance();
  }

  /// Simple, non-templated Gaussian fusion in a common tangent space
  using Gaussian = std::pair<Vector, Matrix>;  // mean, covariance

  /// Get a Gaussian, or return nullopt if not Gaussian and throw is false
  std::optional<Gaussian> gaussian(const std::string& method = "<unknown>",
                                   bool throwOnFailure = false) const {
    auto cov = covariance(method, throwOnFailure);
    if (!cov) return std::nullopt;
    return Gaussian{this->mean_.value_or(Vector::Zero(this->dim())), *cov};
  }

  /// @}

 private:
#if GTSAM_ENABLE_BOOST_SERIALIZATION
  /// Serialization function
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE& ar, const unsigned int /*version*/) {
    // NoiseModelFactor1 instead of NoiseModelFactorN for backward compatibility
    ar& boost::serialization::make_nvp(
        "NoiseModelFactor1", boost::serialization::base_object<Base>(*this));
    ar& BOOST_SERIALIZATION_NVP(origin_);
    ar& BOOST_SERIALIZATION_NVP(mean_);
  }
#endif

  /// Alignment, see
  /// https://eigen.tuxfamily.org/dox/group__TopicStructHavingEigenMembers.html
  inline constexpr static auto NeedsToAlign = (sizeof(T) % 16) == 0;

 public:
  GTSAM_MAKE_ALIGNED_OPERATOR_NEW_IF(NeedsToAlign)
};

/// traits
template <class VALUE>
struct traits<ExtendedPriorFactor<VALUE> >
    : public Testable<ExtendedPriorFactor<VALUE> > {};

}  // namespace gtsam
