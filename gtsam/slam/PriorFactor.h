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

#include <gtsam/base/Lie.h>
#include <gtsam/geometry/Pose3.h>

namespace gtsam {

  /**
   * A class for a soft prior on any Value type
   * @addtogroup SLAM
   */
  template<class VALUE>
  class PriorFactor: public NoiseModelFactor1<VALUE> {

  public:
    typedef VALUE T;

  private:

    typedef NoiseModelFactor1<VALUE> Base;

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

    virtual ~PriorFactor() {}

    /** Constructor */
    PriorFactor(Key key, const VALUE& prior, const SharedNoiseModel& model) :
      Base(model, key), prior_(prior) {
    }

    /** Convenience constructor that takes a full covariance argument */
    PriorFactor(Key key, const VALUE& prior, const Matrix& covariance) :
      Base(noiseModel::Gaussian::Covariance(covariance), key), prior_(prior) {
    }

    /// @return a deep copy of this factor
    virtual gtsam::NonlinearFactor::shared_ptr clone() const {
      return boost::static_pointer_cast<gtsam::NonlinearFactor>(
          gtsam::NonlinearFactor::shared_ptr(new This(*this))); }

    /** implement functions needed for Testable */

    /** print */
    virtual void print(const std::string& s, const KeyFormatter& keyFormatter = DefaultKeyFormatter) const {
      std::cout << s << "PriorFactor on " << keyFormatter(this->key()) << "\n";
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
    Vector evaluateError(const T& x, boost::optional<Matrix&> H = boost::none) const {
      if (H) (*H) = Matrix::Identity(traits<T>::GetDimension(x),traits<T>::GetDimension(x));
      // manifold equivalent of z-x -> Local(x,z)
      // TODO(ASL) Add Jacobians.
      return -traits<T>::Local(x, prior_);
    }

    const VALUE & prior() const { return prior_; }

  private:

    /** Serialization function */
    friend class boost::serialization::access;
    template<class ARCHIVE>
    void serialize(ARCHIVE & ar, const unsigned int /*version*/) {
      ar & boost::serialization::make_nvp("NoiseModelFactor1",
          boost::serialization::base_object<Base>(*this));
      ar & BOOST_SERIALIZATION_NVP(prior_);
    }
  };

//========================== MHPriorFactor ==========================
  //TODO:
  template<class VALUE>
  class MHPriorFactor: public MHNoiseModelFactor1<VALUE> {

  public:
    typedef VALUE T;

  private:

    typedef MHNoiseModelFactor1<VALUE> Base;

    std::vector<VALUE> prior_arr_; /** The measurement */

    /** concept check by type */
    GTSAM_CONCEPT_TESTABLE_TYPE(T)

  public:

    /// shorthand for a smart pointer to a factor
    typedef typename boost::shared_ptr<MHPriorFactor<VALUE> > shared_ptr;

    /// Typedef to this class
    typedef MHPriorFactor<VALUE> This;

    /** default constructor - only use for serialization */
    MHPriorFactor() {}

    virtual ~MHPriorFactor() {}

    /** Constructor */
    MHPriorFactor(Key key, const std::vector<VALUE>& prior_arr, const SharedNoiseModel& model) :
      Base(model, key), prior_arr_(prior_arr) {
    }

    /** Convenience constructor that takes a full covariance argument */
    MHPriorFactor(Key key, const std::vector<VALUE>& prior_arr, const Matrix& covariance) :
      Base(noiseModel::Gaussian::Covariance(covariance), key), prior_arr_(prior_arr) {
    }

    /// @return a deep copy of this factor
    virtual gtsam::NonlinearFactor::shared_ptr clone() const {
      return boost::static_pointer_cast<gtsam::NonlinearFactor>(
          gtsam::NonlinearFactor::shared_ptr(new This(*this))); }

    /** implement functions needed for Testable */

    /** print */
    virtual void print(const std::string& s, const KeyFormatter& keyFormatter = DefaultKeyFormatter) const {
      /*
      std::cout << s << "PriorFactor on " << keyFormatter(this->key()) << "\n";
      traits<T>::Print(prior_, "  prior mean: ");
      this->noiseModel_->print("  noise model: ");
      // */
      std::cout << "MHPF::print() NOT implemented yet" << std::endl;
    }

    /** equals */
    virtual bool equals(const NonlinearFactor& expected, double tol=1e-9) const {
      /*
      const This* e = dynamic_cast<const This*> (&expected);
      return e != NULL && Base::equals(*e, tol) && traits<T>::Equals(prior_, e->prior_, tol);
      // */
      std::cout << "MHPF::equals() NOT implemented yet" << std::endl;
      return false;
    }

    /** implement functions needed to derive from Factor */

    /** vector of errors */
    //[MH-A]:
    Vector evaluateSingleError(const T& x, const size_t& mode_id, boost::optional<Matrix&> H = boost::none) const {
      if (H) (*H) = Matrix::Identity(traits<T>::GetDimension(x),traits<T>::GetDimension(x));
      // manifold equivalent of z-x -> Local(x,z)
      // TODO(ASL) Add Jacobians.
      return -traits<T>::Local(x, prior_arr_[mode_id]);
    }

    const std::vector<VALUE>& priorAll() const { 
        return prior_arr_;
    }
  
  private:

    /** Serialization function */
    friend class boost::serialization::access;
    template<class ARCHIVE>
    void serialize(ARCHIVE & ar, const unsigned int /*version*/) {
      ar & boost::serialization::make_nvp("MHNoiseModelFactor1",
          boost::serialization::base_object<Base>(*this));
      ar & BOOST_SERIALIZATION_NVP(prior_arr_);
    }
  };

//========================== END MHPriorFactor ==========================

//========================== MH_Pose3_Gravity_PriorFactor ==========================
  //TODO:
  class MH_Pose3_Gravity_PriorFactor: public MHNoiseModelFactor1<Pose3> {
  private:

    typedef MHNoiseModelFactor1<Pose3> Base;

    std::vector<Pose3> prior_arr_; /** The measurement */

    /** concept check by type */
    GTSAM_CONCEPT_TESTABLE_TYPE(Pose3)

  public:

    /// shorthand for a smart pointer to a factor
    typedef typename boost::shared_ptr<MH_Pose3_Gravity_PriorFactor> shared_ptr;

    /// Typedef to this class
    typedef MH_Pose3_Gravity_PriorFactor This;

    /** default constructor - only use for serialization */
    MH_Pose3_Gravity_PriorFactor() {}

    virtual ~MH_Pose3_Gravity_PriorFactor() {}

    /** Constructor */
    MH_Pose3_Gravity_PriorFactor(Key key, const std::vector<Pose3>& prior_arr, const SharedNoiseModel& model) :
      Base(model, key), prior_arr_(prior_arr) {
    }

    /** Convenience constructor that takes a full covariance argument */
    MH_Pose3_Gravity_PriorFactor(Key key, const std::vector<Pose3>& prior_arr, const Matrix& covariance) :
      Base(noiseModel::Gaussian::Covariance(covariance), key), prior_arr_(prior_arr) {
    }

    /// @return a deep copy of this factor
    virtual gtsam::NonlinearFactor::shared_ptr clone() const {
      return boost::static_pointer_cast<gtsam::NonlinearFactor>(
          gtsam::NonlinearFactor::shared_ptr(new This(*this))); }

    /** implement functions needed for Testable */

    /** print */
    virtual void print(const std::string& s, const KeyFormatter& keyFormatter = DefaultKeyFormatter) const {
      /*
      std::cout << s << "PriorFactor on " << keyFormatter(this->key()) << "\n";
      traits<T>::Print(prior_, "  prior mean: ");
      this->noiseModel_->print("  noise model: ");
      // */
      std::cout << "MH_Pose3_Gravity_PF::print() NOT implemented yet" << std::endl;
    }

    /** equals */
    virtual bool equals(const NonlinearFactor& expected, double tol=1e-9) const {
      /*
      const This* e = dynamic_cast<const This*> (&expected);
      return e != NULL && Base::equals(*e, tol) && traits<T>::Equals(prior_, e->prior_, tol);
      // */
      std::cout << "MH_Pose3_Gravity_PF::equals() NOT implemented yet" << std::endl;
      return false;
    }

    /** implement functions needed to derive from Factor */

    /** vector of errors */
    //TODO:
    Vector evaluateSingleError(const Pose3& x, const size_t& mode_id, boost::optional<Matrix&> H = boost::none) const {
      if (H) (*H) = Matrix::Identity(2, 2);
      // manifold equivalent of z-x -> Local(x,z)
      // TODO(ASL) Add Jacobians.
      return -traits<Pose3>::Local(x, prior_arr_[mode_id]).head(2);
    }

    const std::vector<Pose3>& priorAll() const {
        return prior_arr_;
    }

  private:

    /** Serialization function */
    friend class boost::serialization::access;
    template<class ARCHIVE>
    void serialize(ARCHIVE & ar, const unsigned int /*version*/) {
      ar & boost::serialization::make_nvp("MHNoiseModelFactor1",
          boost::serialization::base_object<Base>(*this));
      ar & BOOST_SERIALIZATION_NVP(prior_arr_);
    }
  };

//========================== END MH_Pose3_Gravity_PriorFactor ==========================

} /// namespace gtsam
