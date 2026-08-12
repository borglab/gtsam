/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file PartialPriorFactor.h
 * @brief A deprecated prior factor on selected tangent-space coordinates.
 * @author Alex Cunningham
 */

#pragma once

#include <gtsam/config.h>

#ifdef GTSAM_ALLOW_DEPRECATED_SINCE_V43

#include <gtsam/base/Lie.h>
#include <gtsam/nonlinear/FunctorizedFactor.h>

#include <cassert>

namespace gtsam {

/**
 * A soft prior on selected coordinates of an identity-centered tangent
 * representation. The prior is stored in compressed form, in the same order
 * as the requested indices into `VALUE`'s Logmap coordinates.
 *
 * This factor does not select physical components in the world frame. In
 * particular, the translation entries of a Pose3 tangent vector are not the
 * same operation as `pose.translation()`. It is also not a coordinate-invariant
 * manifold prior and does not directly express a known world-frame attitude.
 *
 * Prefer FunctorizedFactor with a projection that names the quantity being
 * measured. For example, a world-frame Pose3 translation prior can use:
 * @code
 * auto factor = MakeFunctorizedFactor<Pose3>(
 *     key, measuredWorldPoint, model,
 *     [](const Pose3& pose, OptionalJacobian<3, 6> H) {
 *       return pose.translation(H);
 *     });
 * @endcode
 * A world-frame attitude prior can use:
 * @code
 * auto factor = MakeFunctorizedFactor<Pose3>(
 *     key, measured_wRb, model,
 *     [](const Pose3& pose, OptionalJacobian<3, 6> H) {
 *       return pose.rotation(H);
 *     });
 * @endcode
 * The same pattern applies to NavState and Gal3 accessors with
 * `OptionalJacobian<3, 9>` and `OptionalJacobian<3, 10>`, respectively.
 *
 * @tparam VALUE Type of variable affected by the prior.
 */
template <class VALUE>
class PartialPriorFactor : public NoiseModelFactorN<VALUE> {
  public:
    typedef VALUE T;

  protected:
    // Concept checks on the variable type - currently requires Lie
    GTSAM_CONCEPT_LIE_TYPE(VALUE)

    typedef NoiseModelFactorN<VALUE> Base;
    typedef PartialPriorFactor<VALUE> This;
    typedef FunctorizedFactor<Vector, VALUE> InternalFactor;

    Vector prior_;                 ///< Measurement on tangent space parameters, in compressed form.
    std::vector<size_t> indices_;  ///< Indices of the measured tangent space parameters.
    InternalFactor internalFactor_;  ///< Implements the projected prior.

    /** Create the index projection used by the internal functorized factor. */
    static InternalFactor MakeInternalFactor(
        Key key, const Vector& prior, const SharedNoiseModel& model,
        const std::vector<size_t>& indices) {
      auto projection =
          [indices](const T& value,
                    OptionalJacobian<Eigen::Dynamic, Eigen::Dynamic> H = {}) {
            constexpr int Dim = traits<T>::dimension;
            Eigen::Matrix<double, Dim, Dim> Hlocal;

// Rot3's Cayley chart cannot provide the Logmap Jacobian.
#ifdef GTSAM_ROT3_EXPMAP
            const Vector fullTangent = traits<T>::Local(
                traits<T>::Identity(), value, {}, H ? &Hlocal : nullptr);
#else
            const Vector fullTangent =
                traits<T>::Logmap(value, H ? &Hlocal : nullptr);
#endif

            if (H) {
              H->setZero(indices.size(), Dim);
              for (size_t i = 0; i < indices.size(); ++i) {
                H->row(i) = Hlocal.row(indices.at(i));
              }
            }

            Vector partialTangent(indices.size());
            for (size_t i = 0; i < indices.size(); ++i) {
              partialTangent(i) = fullTangent(indices.at(i));
            }
            return partialTangent;
          };
      return MakeFunctorizedFactor<T>(key, prior, model, projection);
    }

    /// Recreate the non-serializable projection lambda after loading.
    void initializeInternalFactor() {
      internalFactor_ =
          MakeInternalFactor(this->key1(), prior_, this->noiseModel(), indices_);
    }

    /**
     * constructor with just minimum requirements for a factor - allows more
     * computation in the constructor.  This should only be used by subclasses
     */
    PartialPriorFactor(Key key, const SharedNoiseModel& model)
        : Base(model, key),
          internalFactor_(MakeInternalFactor(key, prior_, model, indices_)) {}

  public:

    // Provide access to the Matrix& version of evaluateError:
    using Base::evaluateError;

    /** default constructor - only use for serialization */
    PartialPriorFactor() {}

    /** Single Element Constructor: Prior on a single parameter at index 'idx' in the tangent vector.*/
    PartialPriorFactor(Key key, size_t idx, double prior,
                       const SharedNoiseModel& model)
        : Base(model, key),
          prior_(Vector{{prior}}),
          indices_(1, idx),
          internalFactor_(MakeInternalFactor(key, prior_, model, indices_)) {
      assert(model->dim() == 1);
    }

    /** Indices Constructor: Specify the relevant measured indices in the tangent vector.*/
    PartialPriorFactor(Key key, const std::vector<size_t>& indices, const Vector& prior,
        const SharedNoiseModel& model) :
        Base(model, key),
        prior_(prior),
        indices_(indices),
        internalFactor_(MakeInternalFactor(key, prior_, model, indices_)) {
      assert((size_t)prior_.size() == indices_.size());
      assert(model->dim() == (size_t)prior.size());
    }

    ~PartialPriorFactor() override {}

    /// @return a deep copy of this factor
    gtsam::NonlinearFactor::shared_ptr clone() const override {
      return std::static_pointer_cast<gtsam::NonlinearFactor>(
          gtsam::NonlinearFactor::shared_ptr(new This(*this))); }

    /** implement functions needed for Testable */

    /** print */
    void print(const std::string& s, const KeyFormatter& keyFormatter = DefaultKeyFormatter) const override {
      Base::print(s, keyFormatter);
      gtsam::print(prior_, "Prior: ");
      std::cout << "Indices: ";
      for (const int i : indices_) {
        std::cout << i << " ";
      }
      std::cout << std::endl;
    }

    /** equals */
    bool equals(const NonlinearFactor& expected, double tol=1e-9) const override {
      const This *e = dynamic_cast<const This*> (&expected);
      return e != nullptr && Base::equals(*e, tol) &&
          gtsam::equal_with_abs_tol(this->prior_, e->prior_, tol) &&
          this->indices_ == e->indices_;
    }

    /** implement functions needed to derive from Factor */

    /** Returns errors for the selected tangent coordinates. */
    Vector evaluateError(const T& p, OptionalMatrixType H) const override {
      return internalFactor_.evaluateError(p, H);
    }

    // access
    const Vector& prior() const { return prior_; }
    const std::vector<size_t>& indices() const { return indices_; }

  private:
#if GTSAM_ENABLE_BOOST_SERIALIZATION
    /** Serialization function */
    friend class boost::serialization::access;
    template <class ARCHIVE>
    void save(ARCHIVE& ar, const unsigned int /*version*/) const {
      // NoiseModelFactor1 instead of NoiseModelFactorN for backward compatibility
      ar & boost::serialization::make_nvp("NoiseModelFactor1",
          boost::serialization::base_object<Base>(*this));
      ar & BOOST_SERIALIZATION_NVP(prior_);
      ar & BOOST_SERIALIZATION_NVP(indices_);
    }

    template <class ARCHIVE>
    void load(ARCHIVE& ar, const unsigned int /*version*/) {
      // NoiseModelFactor1 instead of NoiseModelFactorN for backward compatibility
      ar & boost::serialization::make_nvp("NoiseModelFactor1",
          boost::serialization::base_object<Base>(*this));
      ar & BOOST_SERIALIZATION_NVP(prior_);
      ar & BOOST_SERIALIZATION_NVP(indices_);
      initializeInternalFactor();
    }

    BOOST_SERIALIZATION_SPLIT_MEMBER()
#endif
  }; // \class PartialPriorFactor

} /// namespace gtsam

#endif  // GTSAM_ALLOW_DEPRECATED_SINCE_V43
