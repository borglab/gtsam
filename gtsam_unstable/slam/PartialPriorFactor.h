/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file PartialPriorFactor.h
 * @brief A simple prior factor that allows for setting a prior only on a part of linear parameters
 * @author Alex Cunningham
 */

#pragma once

#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/base/Lie.h>

namespace gtsam {

  /**
   * A class for a soft partial prior on any Lie type, with a mask over Expmap
   * parameters. Note that this will use Logmap() to find a tangent space parameterization
   * for the variable attached, so this may fail for highly nonlinear manifolds.
   *
   * The prior vector used in this factor is stored in compressed form, such that
   * it only contains values for measurements that are to be compared, and they are in
   * the same order as VALUE::Logmap(). The provided indices will determine which components to
   * extract in the error function.
   *
   * @tparam VALUE is the type of variable the prior effects
   */
  template<class VALUE>
  class PartialPriorFactor: public NoiseModelFactorN<VALUE> {

  public:
    typedef VALUE T;

  protected:
    // Concept checks on the variable type - currently requires Lie
    GTSAM_CONCEPT_LIE_TYPE(VALUE)

    typedef NoiseModelFactorN<VALUE> Base;
    typedef PartialPriorFactor<VALUE> This;

    Vector prior_;                 ///< Measurement on tangent space parameters, in compressed form.
    std::vector<size_t> indices_;  ///< Indices of the measured tangent space parameters.

    /** default constructor - only use for serialization */
    PartialPriorFactor() {}

    /**
     * constructor with just minimum requirements for a factor - allows more
     * computation in the constructor.  This should only be used by subclasses
     */
    PartialPriorFactor(Key key, const SharedNoiseModel& model)
      : Base(model, key) {}

  public:

    ~PartialPriorFactor() override {}

    /** Single Element Constructor: Prior on a single parameter at index 'idx' in the tangent vector.*/
    PartialPriorFactor(Key key, size_t idx, double prior, const SharedNoiseModel& model) :
      Base(model, key),
      prior_((Vector(1) << prior).finished()),
      indices_(1, idx) {
      assert(model->dim() == 1);
    }

    /** Indices Constructor: Specify the relevant measured indices in the tangent vector.*/
    PartialPriorFactor(Key key, const std::vector<size_t>& indices, const Vector& prior,
        const SharedNoiseModel& model) :
        Base(model, key),
        prior_(prior),
        indices_(indices) {
      assert((size_t)prior_.size() == indices_.size());
      assert(model->dim() == (size_t)prior.size());
    }

    /// @return a deep copy of this factor
    gtsam::NonlinearFactor::shared_ptr clone() const override {
      return boost::static_pointer_cast<gtsam::NonlinearFactor>(
          gtsam::NonlinearFactor::shared_ptr(new This(*this))); }

    /** implement functions needed for Testable */

    /** print */
    void print(const std::string& s, const KeyFormatter& keyFormatter = DefaultKeyFormatter) const override {
      Base::print(s, keyFormatter);
      gtsam::print(prior_, "prior");
    }

    /** equals */
    bool equals(const NonlinearFactor& expected, double tol=1e-9) const override {
      const This *e = dynamic_cast<const This*> (&expected);
      return e != nullptr && Base::equals(*e, tol) &&
          gtsam::equal_with_abs_tol(this->prior_, e->prior_, tol) &&
          this->indices_ == e->indices_;
    }

    /** implement functions needed to derive from Factor */

    /** Returns a vector of errors for the measured tangent parameters.  */
    Vector evaluateError(const T& p, boost::optional<Matrix&> H = boost::none) const override {
      Eigen::Matrix<double, T::dimension, T::dimension> H_local;

      // If the Rot3 Cayley map is used, Rot3::LocalCoordinates will throw a runtime error
      // when asked to compute the Jacobian matrix (see Rot3M.cpp).
      #ifdef GTSAM_ROT3_EXPMAP
      const Vector full_tangent = T::LocalCoordinates(p, H ? &H_local : nullptr);
      #else
      const Vector full_tangent = T::Logmap(p, H ? &H_local : nullptr);
      #endif

      if (H) {
        (*H) = Matrix::Zero(indices_.size(), T::dimension);
        for (size_t i = 0; i < indices_.size(); ++i) {
          (*H).row(i) = H_local.row(indices_.at(i));
        }
      }
      // Select relevant parameters from the tangent vector.
      Vector partial_tangent = Vector::Zero(indices_.size());
      for (size_t i = 0; i < indices_.size(); ++i) {
        partial_tangent(i) = full_tangent(indices_.at(i));
      }

      return partial_tangent - prior_;
    }

    // access
    const Vector& prior() const { return prior_; }
    const std::vector<size_t>& indices() const { return indices_; }

  private:
    /** Serialization function */
    friend class boost::serialization::access;
    template<class ARCHIVE>
    void serialize(ARCHIVE & ar, const unsigned int /*version*/) {
      // NoiseModelFactor1 instead of NoiseModelFactorN for backward compatibility
      ar & boost::serialization::make_nvp("NoiseModelFactor1",
          boost::serialization::base_object<Base>(*this));
      ar & BOOST_SERIALIZATION_NVP(prior_);
      ar & BOOST_SERIALIZATION_NVP(indices_);
      // ar & BOOST_SERIALIZATION_NVP(H_);
    }
  }; // \class PartialPriorFactor

} /// namespace gtsam
