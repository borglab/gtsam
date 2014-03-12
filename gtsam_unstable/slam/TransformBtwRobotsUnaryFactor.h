/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 *  @file  TransformBtwRobotsUnaryFactor.h
 *  @brief Unary factor for determining transformation between given trajectories of two robots
 *  @author Vadim Indelman
 **/
#pragma once

#include <ostream>

#include <gtsam/base/Testable.h>
#include <gtsam/base/Lie.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/linear/GaussianFactor.h>
#include <gtsam/slam/BetweenFactor.h>

namespace gtsam {

  /**
   * A class for a measurement predicted by "between(config[key1],config[key2])"
   * @tparam VALUE the Value type
   * @addtogroup SLAM
   */
  template<class VALUE>
  class TransformBtwRobotsUnaryFactor: public NonlinearFactor {

  public:

    typedef VALUE T;

  private:

    typedef TransformBtwRobotsUnaryFactor<VALUE> This;
    typedef gtsam::NonlinearFactor Base;

    gtsam::Key key_;

    VALUE measured_; /** The measurement */

    gtsam::Values valA_; // given values for robot A map\trajectory
    gtsam::Values valB_; // given values for robot B map\trajectory
    gtsam::Key keyA_;    // key of robot A to which the measurement refers
    gtsam::Key keyB_;    // key of robot B to which the measurement refers

    SharedGaussian model_;

    /** concept check by type */
    GTSAM_CONCEPT_LIE_TYPE(T)
    GTSAM_CONCEPT_TESTABLE_TYPE(T)

  public:

    // shorthand for a smart pointer to a factor
    typedef typename boost::shared_ptr<TransformBtwRobotsUnaryFactor> shared_ptr;

    /** default constructor - only use for serialization */
    TransformBtwRobotsUnaryFactor() {}

    /** Constructor */
    TransformBtwRobotsUnaryFactor(Key key, const VALUE& measured, Key keyA, Key keyB,
        const gtsam::Values valA, const gtsam::Values valB,
        const SharedGaussian& model) :
          Base(cref_list_of<1>(key)), key_(key), measured_(measured), keyA_(keyA), keyB_(keyB),
          model_(model){

      setValAValB(valA, valB);

    }

    virtual ~TransformBtwRobotsUnaryFactor() {}


    /** Clone */
    virtual gtsam::NonlinearFactor::shared_ptr clone() const { return boost::make_shared<This>(*this); }


    /** implement functions needed for Testable */

    /** print */
    virtual void print(const std::string& s, const KeyFormatter& keyFormatter = DefaultKeyFormatter) const {
      std::cout << s << "TransformBtwRobotsUnaryFactor("
          << keyFormatter(key_) << ")\n";
      std::cout << "MR between factor keys: "
          << keyFormatter(keyA_) << ","
          << keyFormatter(keyB_) << "\n";
      measured_.print("  measured: ");
      model_->print("  noise model: ");
      //      Base::print(s, keyFormatter);
    }

    /** equals */
    virtual bool equals(const NonlinearFactor& f, double tol=1e-9) const {
      const This *t =  dynamic_cast<const This*> (&f);

      if(t && Base::equals(f))
        return key_ == t->key_ && measured_.equals(t->measured_);
      else
        return false;
    }

    /** implement functions needed to derive from Factor */

    /* ************************************************************************* */
    void setValAValB(const gtsam::Values valA, const gtsam::Values valB){
      if ( (!valA.exists(keyA_)) && (!valB.exists(keyA_)) && (!valA.exists(keyB_)) && (!valB.exists(keyB_)) )
        throw("something is wrong!");

      // TODO: make sure the two keys belong to different robots

      if (valA.exists(keyA_)){
        valA_ = valA;
        valB_ = valB;
      }
      else {
        valA_ = valB;
        valB_ = valA;
      }
    }

    /* ************************************************************************* */
    virtual double error(const gtsam::Values& x) const {
      return whitenedError(x).squaredNorm();
    }

    /* ************************************************************************* */
    /**
     * Linearize a non-linearFactorN to get a gtsam::GaussianFactor,
     * \f$ Ax-b \approx h(x+\delta x)-z = h(x) + A \delta x - z \f$
     * Hence \f$ b = z - h(x) = - \mathtt{error\_vector}(x) \f$
     */
    /* This version of linearize recalculates the noise model each time */
    virtual boost::shared_ptr<gtsam::GaussianFactor> linearize(const gtsam::Values& x) const {
      // Only linearize if the factor is active
      if (!this->active(x))
        return boost::shared_ptr<gtsam::JacobianFactor>();

      //std::cout<<"About to linearize"<<std::endl;
      gtsam::Matrix A1;
      std::vector<gtsam::Matrix> A(this->size());
      gtsam::Vector b = -whitenedError(x, A);
      A1 = A[0];

      return gtsam::GaussianFactor::shared_ptr(
          new gtsam::JacobianFactor(key_, A1, b, gtsam::noiseModel::Unit::Create(b.size())));
    }


    /* ************************************************************************* */
    gtsam::Vector whitenedError(const gtsam::Values& x,
        boost::optional<std::vector<gtsam::Matrix>&> H = boost::none) const {

      bool debug = true;

      Matrix H_compose, H_between1, H_dummy;

      T orgA_T_currA = valA_.at<T>(keyA_);
      T orgB_T_currB = valB_.at<T>(keyB_);

      T orgA_T_orgB = x.at<T>(key_);

      T orgA_T_currB = orgA_T_orgB.compose(orgB_T_currB, H_compose, H_dummy);

      T currA_T_currB_pred = orgA_T_currA.between(orgA_T_currB, H_dummy, H_between1);

      T currA_T_currB_msr  = measured_;

      Vector err_unw = currA_T_currB_msr.localCoordinates(currA_T_currB_pred);

      Vector err_wh = err_unw;
      if (H) {
        (*H)[0] = H_compose * H_between1;
        model_->WhitenSystem(*H, err_wh);
      }
      else {
        model_->whitenInPlace(err_wh);
      }

      Vector err_wh2 = model_->whiten(err_wh);

      if (debug){
        //        std::cout<<"err_wh: "<<err_wh[0]<<err_wh[1]<<err_wh[2]<<std::endl;
        //        std::cout<<"err_wh2: "<<err_wh2[0]<<err_wh2[1]<<err_wh2[2]<<std::endl;
        //        std::cout<<"H_compose - rows, cols, : "<<H_compose.rows()<<", "<< H_compose.cols()<<std::endl;
        //        std::cout<<"H_between1 - rows, cols, : "<<H_between1.rows()<<", "<< H_between1.cols()<<std::endl;
        //        std::cout<<"H_unwh - rows, cols, : "<<H_unwh.rows()<<", "<< H_unwh.cols()<<std::endl;
        //        std::cout<<"H_unwh: "<<std:endl<<H_unwh[0]

      }


      return err_wh;
    }


    /* ************************************************************************* */
    gtsam::Vector unwhitenedError(const gtsam::Values& x) const {

      T orgA_T_currA = valA_.at<T>(keyA_);
      T orgB_T_currB = valB_.at<T>(keyB_);

      T orgA_T_orgB = x.at<T>(key_);

      T orgA_T_currB = orgA_T_orgB.compose(orgB_T_currB);

      T currA_T_currB_pred = orgA_T_currA.between(orgA_T_currB);

      T currA_T_currB_msr  = measured_;

      return currA_T_currB_msr.localCoordinates(currA_T_currB_pred);
    }

    /* ************************************************************************* */

    /** number of variables attached to this factor */
    std::size_t size() const {
      return 1;
    }

    virtual size_t dim() const {
      return model_->R().rows() + model_->R().cols();
    }

  private:

    /** Serialization function */
    friend class boost::serialization::access;
    template<class ARCHIVE>
    void serialize(ARCHIVE & ar, const unsigned int version) {
      ar & boost::serialization::make_nvp("NonlinearFactor",
          boost::serialization::base_object<Base>(*this));
      //ar & BOOST_SERIALIZATION_NVP(measured_);
    }
  }; // \class TransformBtwRobotsUnaryFactor

} /// namespace gtsam
