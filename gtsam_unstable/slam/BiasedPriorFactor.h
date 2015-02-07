/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 *  @file  BiasedPriorFactor.h
 *  @author Siddharth Choudhary
 **/
#pragma once

#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/base/Testable.h>

namespace gtsam {

  /**
   * A class for a soft prior along with a bias on any Value type
   * @addtogroup SLAM
   */
  template<class VALUE, class BIAS>
  class BiasedPriorFactor: public NoiseModelFactor1<VALUE> {

  public:
    typedef VALUE T;

  private:

    typedef NoiseModelFactor1<VALUE> Base;

    VALUE prior_; /** The measurement */
    BIAS bias_; /** The bias */

    /** concept check by type */
    GTSAM_CONCEPT_TESTABLE_TYPE(T)

  public:

    /// shorthand for a smart pointer to a factor
    typedef typename boost::shared_ptr<BiasedPriorFactor<VALUE, BIAS> > shared_ptr;

    /// Typedef to this class
    typedef BiasedPriorFactor<VALUE, BIAS> This;

    /** default constructor - only use for serialization */
    BiasedPriorFactor() {}

    virtual ~BiasedPriorFactor() {}

    /** Constructor */
    BiasedPriorFactor(Key key, const VALUE& prior, const BIAS& bias, const SharedNoiseModel& model) :
      Base(model, key), prior_(prior), bias_(bias) {
    }

    /// @return a deep copy of this factor
    virtual gtsam::NonlinearFactor::shared_ptr clone() const {
      return boost::static_pointer_cast<gtsam::NonlinearFactor>(
          gtsam::NonlinearFactor::shared_ptr(new This(*this))); }

    /** implement functions needed for Testable */

    /** print */
    virtual void print(const std::string& s, const KeyFormatter& keyFormatter = DefaultKeyFormatter) const {
      std::cout << s << "BiasedPriorFactor on " << keyFormatter(this->key()) << "\n";
      traits<T>::Print(prior_, "  prior mean: ");
      this->noiseModel_->print("  noise model: ");
    }

    /** equals */
    virtual bool equals(const NonlinearFactor& expected, double tol=1e-9) const {
      const This* e = dynamic_cast<const This*> (&expected);
      return e != NULL && Base::equals(*e, tol) && traits<T>::Equals(prior_, e->prior_, tol);
    }

    /** implement functions needed to derive from Factor */

    /** vector of errors */
    Vector evaluateError(const T& p, boost::optional<Matrix&> H = boost::none) const {
      if (H) (*H) = eye(traits<T>::GetDimension(p));
      // manifold equivalent of h(x)-z -> log(z,h(x))
      // TODO(ASL) Add Jacobians.
      Vector rval = traits<T>::Local(prior_,p);
      rval = rval + bias_;
      return rval;
    }

    const VALUE & prior() const { return prior_; }

    const BIAS & bias() const { return bias_; }

  private:

    /** Serialization function */
    friend class boost::serialization::access;
    template<class ARCHIVE>
    void serialize(ARCHIVE & ar, const unsigned int version) {
      ar & boost::serialization::make_nvp("NoiseModelFactor1",
          boost::serialization::base_object<Base>(*this));
      ar & BOOST_SERIALIZATION_NVP(prior_);

    }
  };

} /// namespace gtsam

