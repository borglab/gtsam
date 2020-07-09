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

namespace gtsam {

  /**
   * A class for a measurement predicted by "between(config[key1],config[key2])"
   * @tparam VALUE the Value type
   * @addtogroup SLAM
   */
  template<class VALUE>
  class BetweenFactor: public NoiseModelFactor2<VALUE, VALUE> {

    // Check that VALUE type is a testable Lie group
    BOOST_CONCEPT_ASSERT((IsTestable<VALUE>));
    BOOST_CONCEPT_ASSERT((IsLieGroup<VALUE>));

  public:

    typedef VALUE T;

  private:

    typedef BetweenFactor<VALUE> This;
    typedef NoiseModelFactor2<VALUE, VALUE> Base;

    VALUE measured_; /** The measurement */

  public:

    // shorthand for a smart pointer to a factor
    typedef typename boost::shared_ptr<BetweenFactor> shared_ptr;

    /** default constructor - only use for serialization */
    BetweenFactor() {}

    /** Constructor */
    BetweenFactor(Key key1, Key key2, const VALUE& measured,
        const SharedNoiseModel& model = nullptr) :
      Base(model, key1, key2), measured_(measured) {
    }

    virtual ~BetweenFactor() {}

    /// @return a deep copy of this factor
    virtual gtsam::NonlinearFactor::shared_ptr clone() const {
      return boost::static_pointer_cast<gtsam::NonlinearFactor>(
          gtsam::NonlinearFactor::shared_ptr(new This(*this))); }

    /** implement functions needed for Testable */

    /** print */
    virtual void print(const std::string& s, const KeyFormatter& keyFormatter = DefaultKeyFormatter) const {
      std::cout << s << "BetweenFactor("
          << keyFormatter(this->key1()) << ","
          << keyFormatter(this->key2()) << ")\n";
      traits<T>::Print(measured_, "  measured: ");
      this->noiseModel_->print("  noise model: ");
    }

    /** equals */
    virtual bool equals(const NonlinearFactor& expected, double tol=1e-9) const {
      const This *e =  dynamic_cast<const This*> (&expected);
      return e != nullptr && Base::equals(*e, tol) && traits<T>::Equals(this->measured_, e->measured_, tol);
    }

    /** implement functions needed to derive from Factor */

    /** vector of errors */
  Vector evaluateError(const T& p1, const T& p2, boost::optional<Matrix&> H1 =
      boost::none, boost::optional<Matrix&> H2 = boost::none) const {
      T hx = traits<T>::Between(p1, p2, H1, H2); // h(x)
      // manifold equivalent of h(x)-z -> log(z,h(x))
#ifdef SLOW_BUT_CORRECT_BETWEENFACTOR
      typename traits<T>::ChartJacobian::Jacobian Hlocal;
      Vector rval = traits<T>::Local(measured_, hx, boost::none, (H1 || H2) ? &Hlocal : 0);
      if (H1) *H1 = Hlocal * (*H1);
      if (H2) *H2 = Hlocal * (*H2);
      return rval;
#else
      return traits<T>::Local(measured_, hx);
#endif
    }

    /** return the measured */
    const VALUE& measured() const {
      return measured_;
    }

    /** number of variables attached to this factor */
    std::size_t size() const {
      return 2;
    }

  private:

    /** Serialization function */
    friend class boost::serialization::access;
    template<class ARCHIVE>
    void serialize(ARCHIVE & ar, const unsigned int /*version*/) {
      ar & boost::serialization::make_nvp("NoiseModelFactor2",
          boost::serialization::base_object<Base>(*this));
      ar & BOOST_SERIALIZATION_NVP(measured_);
    }

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
    typedef boost::shared_ptr<BetweenConstraint<VALUE> > shared_ptr;

    /** Syntactic sugar for constrained version */
    BetweenConstraint(const VALUE& measured, Key key1, Key key2, double mu = 1000.0) :
      BetweenFactor<VALUE>(key1, key2, measured,
                           noiseModel::Constrained::All(traits<VALUE>::GetDimension(measured), std::abs(mu)))
    {}

  private:

    /** Serialization function */
    friend class boost::serialization::access;
    template<class ARCHIVE>
    void serialize(ARCHIVE & ar, const unsigned int /*version*/) {
      ar & boost::serialization::make_nvp("BetweenFactor",
          boost::serialization::base_object<BetweenFactor<VALUE> >(*this));
    }
  }; // \class BetweenConstraint

  /// traits
  template<class VALUE>
  struct traits<BetweenConstraint<VALUE> > : public Testable<BetweenConstraint<VALUE> > {};

} /// namespace gtsam
