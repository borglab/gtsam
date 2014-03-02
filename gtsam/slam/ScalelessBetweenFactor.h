/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 *  @file  ScalelessBetweenFactor.h
 *  @author Luca Carlone
 **/
#pragma once

#include <ostream>

#include <gtsam/base/Testable.h>
#include <gtsam/base/Lie.h>
#include <gtsam/geometry/EssentialMatrix.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

namespace gtsam {

  /**
   * A class for a measurement predicted by "between(config[key1],config[key2])", assuming that the translation is up to scale
   * @tparam VALUE the Value type
   * @addtogroup SLAM
   */
  template<class VALUE>
  class ScalelessBetweenFactor: public NoiseModelFactor2<VALUE, VALUE> {

  public:

    typedef VALUE T;

  private:

    typedef ScalelessBetweenFactor<VALUE> This;
    typedef NoiseModelFactor2<VALUE, VALUE> Base;

    EssentialMatrix measured_; /** The measurement */

  public:

    // shorthand for a smart pointer to a factor
    typedef typename boost::shared_ptr<ScalelessBetweenFactor> shared_ptr;

    /** default constructor - only use for serialization */
    ScalelessBetweenFactor() {}

    /** Constructor */
    ScalelessBetweenFactor(Key key1, Key key2, const EssentialMatrix& measured,
        const SharedNoiseModel& model) :
      Base(model, key1, key2), measured_(measured) {
    }

    virtual ~ScalelessBetweenFactor() {}

    /// @return a deep copy of this factor
    virtual gtsam::NonlinearFactor::shared_ptr clone() const {
      return boost::static_pointer_cast<gtsam::NonlinearFactor>(
          gtsam::NonlinearFactor::shared_ptr(new This(*this))); }

    /** implement functions needed for Testable */

    /** print */
    virtual void print(const std::string& s, const KeyFormatter& keyFormatter = DefaultKeyFormatter) const {
      std::cout << s << "ScalelessBetweenFactor("
          << keyFormatter(this->key1()) << ","
          << keyFormatter(this->key2()) << ")\n";
      measured_.print("  measured: ");
      this->noiseModel_->print("  noise model: ");
    }

    /** equals */
    virtual bool equals(const NonlinearFactor& expected, double tol=1e-9) const {
      const This *e =  dynamic_cast<const This*> (&expected);
      return e != NULL && Base::equals(*e, tol) && this->measured_.equals(e->measured_, tol);
    }

    /** implement functions needed to derive from Factor */

    /** vector of errors */
    Vector evaluateError(const Pose3& p1, const Pose3& p2,
        boost::optional<Matrix&> H1 = boost::none, boost::optional<Matrix&> H2 =
            boost::none) const {
      Pose3 hx = p1.between(p2, H1, H2); // h(x)
      Matrix He;
      EssentialMatrix he = EssentialMatrix::FromPose3(hx,He);

      //std::cout<< "He \n" << He << std::endl;
      Vector3 hx_rot = Rot3::Logmap(this->measured_.rotation().inverse() * hx.rotation());
      Matrix3 J = Rot3::rightJacobianExpMapSO3inverse(hx_rot);
      if(H1){
        //std::cout<< "(*H1) \n" << (*H1) << std::endl;
        (*H1) = He * (*H1);
        (*H1).block(0,0,3,3) = J * (*H1).block(0,0,3,3);
      }
      if(H2){
        //std::cout<< "(*H2) \n" << (*H2) << std::endl;
        (*H2) = He * (*H2);
        (*H2).block(0,0,3,3) = J * (*H2).block(0,0,3,3);
      }
      return measured_.localCoordinates(he);
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
    void serialize(ARCHIVE & ar, const unsigned int version) {
      ar & boost::serialization::make_nvp("NoiseModelFactor2",
          boost::serialization::base_object<Base>(*this));
      ar & BOOST_SERIALIZATION_NVP(measured_);
    }
  }; // \class ScalelessBetweenFactor

  /**
   * Binary between constraint - forces between to a given value
   * This constraint requires the underlying type to a Lie type
   *
   */
  template<class VALUE>
  class BetweenConstraint : public ScalelessBetweenFactor<VALUE> {
  public:
    typedef boost::shared_ptr<BetweenConstraint<VALUE> > shared_ptr;

    /** Syntactic sugar for constrained version */
    BetweenConstraint(const VALUE& measured, Key key1, Key key2, double mu = 1000.0) :
      ScalelessBetweenFactor<VALUE>(key1, key2, measured, noiseModel::Constrained::All(VALUE::Dim(), fabs(mu))) {}

  private:

    /** Serialization function */
    friend class boost::serialization::access;
    template<class ARCHIVE>
    void serialize(ARCHIVE & ar, const unsigned int version) {
      ar & boost::serialization::make_nvp("ScalelessBetweenFactor",
          boost::serialization::base_object<ScalelessBetweenFactor<VALUE> >(*this));
    }
  }; // \class BetweenConstraint

} /// namespace gtsam
