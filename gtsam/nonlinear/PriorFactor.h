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

#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/base/Testable.h>

#include <string>

namespace gtsam {

  /**
   * A class for a soft prior on any Value type
   * @ingroup nonlinear
   */
  template<class VALUE>
  class PriorFactor: public NoiseModelFactorN<VALUE> {

  public:
    typedef VALUE T;
    // Provide access to the Matrix& version of evaluateError:
    using NoiseModelFactor1<VALUE>::evaluateError;


  private:

    typedef NoiseModelFactorN<VALUE> Base;

    VALUE prior_; /** The measurement */

    /** concept check by type */
    GTSAM_CONCEPT_TESTABLE_TYPE(T)

  public:

    /// shorthand for a smart pointer to a factor
    typedef typename boost::shared_ptr<PriorFactor<VALUE> > shared_ptr;

    /// Typedef to this class
    typedef PriorFactor<VALUE> This;

    /** default constructor - only use for serialization */
    PriorFactor() {}

    ~PriorFactor() override {}

    /** Constructor */
    PriorFactor(Key key, const VALUE& prior, const SharedNoiseModel& model = nullptr) :
      Base(model, key), prior_(prior) {
    }

    /** Convenience constructor that takes a full covariance argument */
    PriorFactor(Key key, const VALUE& prior, const Matrix& covariance) :
      Base(noiseModel::Gaussian::Covariance(covariance), key), prior_(prior) {
    }

    /// @return a deep copy of this factor
    gtsam::NonlinearFactor::shared_ptr clone() const override {
      return boost::static_pointer_cast<gtsam::NonlinearFactor>(
          gtsam::NonlinearFactor::shared_ptr(new This(*this))); }

    /** implement functions needed for Testable */

    /** print */
    void print(const std::string& s,
       const KeyFormatter& keyFormatter = DefaultKeyFormatter) const override {
      std::cout << s << "PriorFactor on " << keyFormatter(this->key()) << "\n";
      traits<T>::Print(prior_, "  prior mean: ");
      if (this->noiseModel_)
        this->noiseModel_->print("  noise model: ");
      else
        std::cout << "no noise model" << std::endl;
    }

    /** equals */
    bool equals(const NonlinearFactor& expected, double tol=1e-9) const override {
      const This* e = dynamic_cast<const This*> (&expected);
      return e != nullptr && Base::equals(*e, tol) && traits<T>::Equals(prior_, e->prior_, tol);
    }

    /** implement functions needed to derive from Factor */

    /** vector of errors */
    Vector evaluateError(const T& x, OptionalMatrixType H) const override {
      if (H) (*H) = Matrix::Identity(traits<T>::GetDimension(x),traits<T>::GetDimension(x));
      // manifold equivalent of z-x -> Local(x,z)
      return -traits<T>::Local(x, prior_);
    }

    const VALUE & prior() const { return prior_; }

  private:

    /** Serialization function */
    friend class boost::serialization::access;
    template<class ARCHIVE>
    void serialize(ARCHIVE & ar, const unsigned int /*version*/) {
      // NoiseModelFactor1 instead of NoiseModelFactorN for backward compatibility
      ar & boost::serialization::make_nvp("NoiseModelFactor1",
          boost::serialization::base_object<Base>(*this));
      ar & BOOST_SERIALIZATION_NVP(prior_);
    }

  // Alignment, see https://eigen.tuxfamily.org/dox/group__TopicStructHavingEigenMembers.html
  enum { NeedsToAlign = (sizeof(T) % 16) == 0 };
  public:
  GTSAM_MAKE_ALIGNED_OPERATOR_NEW_IF(NeedsToAlign)
  };

  /// traits
  template<class VALUE>
  struct traits<PriorFactor<VALUE> > : public Testable<PriorFactor<VALUE> > {};


} /// namespace gtsam
