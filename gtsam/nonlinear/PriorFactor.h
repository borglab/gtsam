/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 *  @file  PriorFactor.h
 *  @author Frank Dellaert
 **/
#pragma once

#include <gtsam/nonlinear/ExtendedPriorFactor.h>

namespace gtsam {

/**
 * A class for a soft prior on any Value type.
 * @ingroup nonlinear
 **/
template <class VALUE>
class PriorFactor : public ExtendedPriorFactor<VALUE> {
 public:
  typedef VALUE T;
  typedef ExtendedPriorFactor<VALUE> Base;

 private:
  /// concept check by type
  GTSAM_CONCEPT_TESTABLE_TYPE(T)

 public:
  /// @name Standard Constructors
  /// @{

  /// shorthand for a smart pointer to a factor
  typedef typename std::shared_ptr<PriorFactor<VALUE>> shared_ptr;

  /// Typedef to this class
  typedef PriorFactor<VALUE> This;

  /// default constructor - only use for serialization
  PriorFactor() {}

  /// Constructor
  PriorFactor(Key key, const VALUE& prior,
              const SharedNoiseModel& model = nullptr)
      : Base(key, prior, model) {}

  /// Convenience constructor that takes a full covariance argument
  PriorFactor(Key key, const VALUE& prior, const Matrix& covariance)
      : Base(key, prior, noiseModel::Gaussian::Covariance(covariance)) {}

  /// @}
  /// @name Standard Destructor
  /// @{

  ~PriorFactor() override {}

  /// @}
  /// @name Testable
  /// @{

  /// @return a deep copy of this factor
  gtsam::NonlinearFactor::shared_ptr clone() const override {
    return std::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new This(*this)));
  }

  /// print
  void print(const std::string& s, const KeyFormatter& keyFormatter =
                                       DefaultKeyFormatter) const override {
    std::cout << s << "PriorFactor on " << keyFormatter(this->key()) << "\n";
    traits<T>::Print(this->origin(), "  prior mean: ");
    if (this->noiseModel_)
      this->noiseModel_->print("  noise model: ");
    else
      std::cout << "no noise model" << std::endl;
  }

  /// equals
  bool equals(const NonlinearFactor& expected,
              double tol = 1e-9) const override {
    const auto* e = dynamic_cast<const This*>(&expected);
    return e && Base::equals(*e, tol);
  }

  /// @}
  /// @name Access
  /// @{

  const VALUE& prior() const { return this->origin(); }

  /// @}

 private:
#if GTSAM_ENABLE_BOOST_SERIALIZATION
  /// Serialization function
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE& ar, const unsigned int /*version*/) {
    // For backwards compatibility, we manually serialize the base class members
    // as if we were a PriorFactor from before the refactor.
    ar& boost::serialization::make_nvp(
        "NoiseModelFactor1",
        boost::serialization::base_object<NoiseModelFactorN<VALUE>>(*this));
    ar& boost::serialization::make_nvp("prior_", this->origin_);
  }
#endif

  // Alignment, see
  // https://eigen.tuxfamily.org/dox/group__TopicStructHavingEigenMembers.html
  inline constexpr static auto NeedsToAlign = (sizeof(T) % 16) == 0;

 public:
  GTSAM_MAKE_ALIGNED_OPERATOR_NEW_IF(NeedsToAlign)
};

/// traits
template <class VALUE>
struct traits<PriorFactor<VALUE>> : public Testable<PriorFactor<VALUE>> {};

}  // namespace gtsam
