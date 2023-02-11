/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 *  @file  BetweenFactor.h
 *  @author Frank Dellaert, Viorela Ila
 **/
#pragma once

#include <ostream>

#include <gtsam/base/Testable.h>
#include <gtsam/base/Lie.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

#ifdef _WIN32
#define BETWEENFACTOR_VISIBILITY
#else
// This will trigger a LNKxxxx on MSVC, so disable for MSVC build
// Please refer to https://github.com/borglab/gtsam/blob/develop/Using-GTSAM-EXPORT.md
#define BETWEENFACTOR_VISIBILITY GTSAM_EXPORT
#endif

namespace gtsam {

  /**
   * A class for a measurement predicted by "between(config[key1],config[key2])"
   * @tparam VALUE the Value type
   * @ingroup slam
   */
  template<class VALUE>
  class BetweenFactor: public NoiseModelFactorN<VALUE, VALUE> {

    // Check that VALUE type is a testable Lie group
    GTSAM_CONCEPT_ASSERT(IsTestable<VALUE>)
    GTSAM_CONCEPT_ASSERT(IsLieGroup<VALUE>)

  public:

    typedef VALUE T;

  private:

    typedef BetweenFactor<VALUE> This;
    typedef NoiseModelFactorN<VALUE, VALUE> Base;

    VALUE measured_; /** The measurement */

  public:

    // Provide access to the Matrix& version of evaluateError:
    using Base::evaluateError;

    // shorthand for a smart pointer to a factor
    typedef typename std::shared_ptr<BetweenFactor> shared_ptr;

    /// @name Standard Constructors
    /// @{

    /** default constructor - only use for serialization */
    BetweenFactor() {}

    /** Constructor */
    BetweenFactor(Key key1, Key key2, const VALUE& measured,
        const SharedNoiseModel& model = nullptr) :
      Base(model, key1, key2), measured_(measured) {
    }

    /// @}

    ~BetweenFactor() override {}

    /// @return a deep copy of this factor
    gtsam::NonlinearFactor::shared_ptr clone() const override {
      return std::static_pointer_cast<gtsam::NonlinearFactor>(
          gtsam::NonlinearFactor::shared_ptr(new This(*this))); }

    /// @name Testable
    /// @{

    /// print with optional string
    void print(
        const std::string& s = "",
        const KeyFormatter& keyFormatter = DefaultKeyFormatter) const override {
      std::cout << s << "BetweenFactor("
          << keyFormatter(this->key1()) << ","
          << keyFormatter(this->key2()) << ")\n";
      traits<T>::Print(measured_, "  measured: ");
      this->noiseModel_->print("  noise model: ");
    }

    /// assert equality up to a tolerance
    bool equals(const NonlinearFactor& expected, double tol=1e-9) const override {
      const This *e =  dynamic_cast<const This*> (&expected);
      return e != nullptr && Base::equals(*e, tol) && traits<T>::Equals(this->measured_, e->measured_, tol);
    }

    /// @}
    /// @name NoiseModelFactorN methods 
    /// @{

    /// evaluate error, returns vector of errors size of tangent space
    Vector evaluateError(const T& p1, const T& p2,
			OptionalMatrixType H1, OptionalMatrixType H2) const override {
      T hx = traits<T>::Between(p1, p2, H1, H2); // h(x)
      // manifold equivalent of h(x)-z -> log(z,h(x))
#ifdef GTSAM_SLOW_BUT_CORRECT_BETWEENFACTOR
      typename traits<T>::ChartJacobian::Jacobian Hlocal;
      Vector rval = traits<T>::Local(measured_, hx, OptionalNone, (H1 || H2) ? &Hlocal : 0);
      if (H1) *H1 = Hlocal * (*H1);
      if (H2) *H2 = Hlocal * (*H2);
      return rval;
#else
      return traits<T>::Local(measured_, hx);
#endif
    }

    /// @}
    /// @name Standard interface 
    /// @{

    /// return the measurement
    const VALUE& measured() const {
      return measured_;
    }
    /// @}

  private:

#ifdef GTSAM_ENABLE_BOOST_SERIALIZATION
    /** Serialization function */
    friend class boost::serialization::access;
    template<class ARCHIVE>
    void serialize(ARCHIVE & ar, const unsigned int /*version*/) {
      // NoiseModelFactor2 instead of NoiseModelFactorN for backward compatibility
      ar & boost::serialization::make_nvp("NoiseModelFactor2",
          boost::serialization::base_object<Base>(*this));
      ar & BOOST_SERIALIZATION_NVP(measured_);
    }
#endif

	  // Alignment, see https://eigen.tuxfamily.org/dox/group__TopicStructHavingEigenMembers.html
	  enum { NeedsToAlign = (sizeof(VALUE) % 16) == 0 };
    public:
      GTSAM_MAKE_ALIGNED_OPERATOR_NEW_IF(NeedsToAlign)
  }; // \class BetweenFactor

  /// traits
  template<class VALUE>
  struct traits<BetweenFactor<VALUE> > : public Testable<BetweenFactor<VALUE> > {};

  /**
   * Binary between constraint - forces between to a given value
   * This constraint requires the underlying type to a Lie type
   *
   */
  template<class VALUE>
  class BetweenConstraint : public BetweenFactor<VALUE> {
  public:
    typedef std::shared_ptr<BetweenConstraint<VALUE> > shared_ptr;

    /** Syntactic sugar for constrained version */
    BetweenConstraint(const VALUE& measured, Key key1, Key key2, double mu = 1000.0) :
      BetweenFactor<VALUE>(key1, key2, measured,
                           noiseModel::Constrained::All(traits<VALUE>::GetDimension(measured), std::abs(mu)))
    {}

  private:

    /** Serialization function */
#ifdef GTSAM_ENABLE_BOOST_SERIALIZATION
    friend class boost::serialization::access;
    template<class ARCHIVE>
    void serialize(ARCHIVE & ar, const unsigned int /*version*/) {
      ar & boost::serialization::make_nvp("BetweenFactor",
          boost::serialization::base_object<BetweenFactor<VALUE> >(*this));
    }
#endif
  }; // \class BetweenConstraint

  /// traits
  template<class VALUE>
  struct traits<BetweenConstraint<VALUE> > : public Testable<BetweenConstraint<VALUE> > {};

} /// namespace gtsam
