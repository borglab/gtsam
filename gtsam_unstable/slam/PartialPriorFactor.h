/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file PartialPriorFactor.h
 * @brief A factor for setting a prior on a subset of a variable's parameters.
 * @author Alex Cunningham
 */
#pragma once

#include <gtsam/base/Lie.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

namespace gtsam {

/**
 * A class for putting a partial prior on any manifold type by specifying the
 * indices of measured parameter. Note that the prior is over a customizable
 * *parameter* vector, and not over the tangent vector (e.g given by
 * T::Local(x)). This is due to the fact that the tangent vector entries may not
 * be directly useful for common pose constraints; for example, the translation
 * component of Pose3::Local(x) != x.translation() for non-identity rotations,
 * which makes it hard to use the tangent vector for translation constraints.
 *
 * Instead, the prior and indices are given with respect to a "parameter vector"
 * whose values we can measure and put a prior on. Derived classes implement
 * the desired parameterization by overriding the Parameterize(x) function.
 *
 * The prior parameter vector used in this factor is stored in compressed form,
 * such that it only contains values for measurements that are to be compared.
 * The provided indices will determine which parameters to extract in the error
 * function.
 *
 * @tparam VALUE is the type of variable that the prior effects.
 */
template <class VALUE>
class PartialPriorFactor : public NoiseModelFactorN<VALUE> {
  public:
    typedef VALUE T;

  protected:
    typedef NoiseModelFactorN<VALUE> Base;
    typedef PartialPriorFactor<VALUE> This;

    Vector prior_;                 ///< Prior on measured parameters.
    std::vector<size_t> indices_;  ///< Indices of the measured parameters.

    /**
     * constructor with just minimum requirements for a factor - allows more
     * computation in the constructor.  This should only be used by subclasses
     */
    PartialPriorFactor(Key key, const SharedNoiseModel& model)
      : Base(model, key) {}

  public:

    // Provide access to the Matrix& version of evaluateError:
    using Base::evaluateError;

    /** default constructor - only use for serialization */
    PartialPriorFactor() {}

    /** Single Index Constructor: Prior on a single entry at index 'idx' in the parameter vector.*/
    PartialPriorFactor(Key key, size_t idx, double prior, const SharedNoiseModel& model)
        : Base(model, key),
        prior_(Vector1(prior)),
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

    ~PartialPriorFactor() override {}

    /** Implement functions needed for Testable */

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
    bool equals(const NonlinearFactor& expected, double tol = 1e-9) const override {
      const This* e = dynamic_cast<const This*>(&expected);
      return e != nullptr && Base::equals(*e, tol) &&
          gtsam::equal_with_abs_tol(this->prior_, e->prior_, tol) &&
          this->indices_ == e->indices_;
    }

    /** implement functions needed to derive from Factor */

    /**
     * Evaluate the error h(x) - prior, where h(x) returns a parameter vector
     * representation of x that is consistent with the prior. Note that this
     * function expects any derived factor to have overridden the Parameterize()
     * member function.
     */
    Vector evaluateError(const T& x, OptionalMatrixType H) const override {
      // Extract the relevant subset of the parameter vector and Jacobian.
      Matrix H_full;
      const Vector hx_full = Parameterize(x, H ? &H_full : nullptr);

      Vector hx(indices_.size());
      if (H) (*H) = Matrix::Zero(indices_.size(), T::dimension);

      for (size_t i = 0; i < indices_.size(); ++i) {
        hx(i) = hx_full(indices_.at(i));
        if (H) (*H).row(i) = H_full.row(indices_.at(i));
      }

      return hx - prior_;
    }

    /**
     * Map an input x on the manifold to a parameter vector h(x).
     * Note that this function may not be the same as VALUE::Local() if the
     * parameter vector is defined differently from the tangent vector.
     *
     * This function should be implemented by derived classes based on VALUE.
     */
    virtual Vector Parameterize(const T& x, Matrix* H = nullptr) const = 0;

    // access
    const Vector& prior() const { return prior_; }
    const std::vector<size_t>& indices() const { return indices_; }

  private:
#ifdef GTSAM_ENABLE_BOOST_SERIALIZATION
    /** Serialization function */
    friend class boost::serialization::access;
    template<class ARCHIVE>
    void serialize(ARCHIVE & ar, const unsigned int /*version*/) {
      // NoiseModelFactor1 instead of NoiseModelFactorN for backward compatibility
      ar & boost::serialization::make_nvp("NoiseModelFactor1", boost::serialization::base_object<Base>(*this));
      ar & BOOST_SERIALIZATION_NVP(prior_);
      ar & BOOST_SERIALIZATION_NVP(indices_);
    }
#endif
};  // \class PartialPriorFactor

} /// namespace gtsam
