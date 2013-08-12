/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 *  @file  BetweenFactorEM.h
 *  @author Vadim Indelman
 **/
#pragma once

#include <ostream>

#include <gtsam/base/Testable.h>
#include <gtsam/base/Lie.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/linear/GaussianFactor.h>

namespace gtsam {

  /**
   * A class for a measurement predicted by "between(config[key1],config[key2])"
   * @tparam VALUE the Value type
   * @addtogroup SLAM
   */
  template<class VALUE>
  class BetweenFactorEM: public NonlinearFactor {

  public:

    typedef VALUE T;

  private:

    typedef BetweenFactorEM<VALUE> This;
    typedef gtsam::NonlinearFactor Base;

    gtsam::Key key1_;
    gtsam::Key key2_;

    VALUE measured_; /** The measurement */

    SharedGaussian model_inlier_;
    SharedGaussian model_outlier_;

    double prior_inlier_;
    double prior_outlier_;

    /** concept check by type */
    GTSAM_CONCEPT_LIE_TYPE(T)
    GTSAM_CONCEPT_TESTABLE_TYPE(T)

  public:

    // shorthand for a smart pointer to a factor
    typedef typename boost::shared_ptr<BetweenFactorEM> shared_ptr;

    /** default constructor - only use for serialization */
    BetweenFactorEM() {}

    /** Constructor */
    BetweenFactorEM(Key key1, Key key2, const VALUE& measured,
        const SharedGaussian& model_inlier, const SharedGaussian& model_outlier,
        const double prior_inlier, const double prior_outlier) :
          Base(key1, key2), key1_(key1), key2_(key2), measured_(measured),
          model_inlier_(model_inlier), model_outlier_(model_outlier),
          prior_inlier_(prior_inlier), prior_outlier_(prior_outlier){
    }

    virtual ~BetweenFactorEM() {}


    /** implement functions needed for Testable */

    /** print */
    virtual void print(const std::string& s, const KeyFormatter& keyFormatter = DefaultKeyFormatter) const {
      std::cout << s << "BetweenFactorEM("
          << keyFormatter(key1_) << ","
          << keyFormatter(key2_) << ")\n";
      measured_.print("  measured: ");
      model_inlier_->print("  noise model inlier: ");
      model_outlier_->print("  noise model outlier: ");
      std::cout << "(prior_inlier, prior_outlier_) = ("
                << prior_inlier_ << ","
                << prior_outlier_ << ")\n";
      //      Base::print(s, keyFormatter);
    }

    /** equals */
    virtual bool equals(const NonlinearFactor& f, double tol=1e-9) const {
      const This *t =  dynamic_cast<const This*> (&f);

      if(t && Base::equals(f))
        return key1_ == t->key1_ && key2_ == t->key2_ &&
            //            model_inlier_->equals(t->model_inlier_ ) && // TODO: fix here
            //            model_outlier_->equals(t->model_outlier_ ) &&
            prior_outlier_ == t->prior_outlier_ && prior_inlier_ == t->prior_inlier_ && measured_.equals(t->measured_);
      else
        return false;
    }

    /** implement functions needed to derive from Factor */

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
    virtual boost::shared_ptr<gtsam::GaussianFactor> linearize(const gtsam::Values& x, const gtsam::Ordering& ordering) const {
      // Only linearize if the factor is active
      if (!this->active(x))
        return boost::shared_ptr<gtsam::JacobianFactor>();

      //std::cout<<"About to linearize"<<std::endl;
      gtsam::Matrix A1, A2;
      std::vector<gtsam::Matrix> A(this->size());
      gtsam::Vector b = -whitenedError(x, A);
      A1 = A[0];
      A2 = A[1];

      return gtsam::GaussianFactor::shared_ptr(
          new gtsam::JacobianFactor(ordering[key1_], A1, ordering[key2_], A2, b, gtsam::noiseModel::Unit::Create(b.size())));
    }


    /* ************************************************************************* */
    gtsam::Vector whitenedError(const gtsam::Values& x,
        boost::optional<std::vector<gtsam::Matrix>&> H = boost::none) const {

      bool debug = true;

      const T& p1 = x.at<T>(key1_);
      const T& p2 = x.at<T>(key2_);

      Matrix H1, H2;

      T hx = p1.between(p2, H1, H2); // h(x)
      // manifold equivalent of h(x)-z -> log(z,h(x))

      Vector err = measured_.localCoordinates(hx);

      // Calculate indicator probabilities (inlier and outlier)
      Vector p_inlier_outlier = calcIndicatorProb(x);
      double p_inlier  = p_inlier_outlier[0];
      double p_outlier = p_inlier_outlier[1];

      Vector err_wh_inlier  = model_inlier_->whiten(err);
      Vector err_wh_outlier = model_outlier_->whiten(err);

      Matrix invCov_inlier  = model_inlier_->R().transpose() * model_inlier_->R();
      Matrix invCov_outlier = model_outlier_->R().transpose() * model_outlier_->R();

      Vector err_wh_eq;
      err_wh_eq.resize(err_wh_inlier.rows()*2);
      err_wh_eq << std::sqrt(p_inlier) * err_wh_inlier.array() , std::sqrt(p_outlier) * err_wh_outlier.array();

      if (H){
        // stack Jacobians for the two indicators for each of the key

        Matrix H1_inlier  = std::sqrt(p_inlier)*model_inlier_->Whiten(H1);
        Matrix H1_outlier = std::sqrt(p_outlier)*model_outlier_->Whiten(H1);
        Matrix H1_aug = gtsam::stack(2, &H1_inlier, &H1_outlier);

        Matrix H2_inlier  = std::sqrt(p_inlier)*model_inlier_->Whiten(H2);
        Matrix H2_outlier = std::sqrt(p_outlier)*model_outlier_->Whiten(H2);
        Matrix H2_aug = gtsam::stack(2, &H2_inlier, &H2_outlier);

        (*H)[0].resize(H1_aug.rows(),H1_aug.cols());
        (*H)[1].resize(H2_aug.rows(),H2_aug.cols());

        (*H)[0] = H1_aug;
        (*H)[1] = H2_aug;
      }

      if (debug){
        //        std::cout<<"unwhitened error: "<<err[0]<<" "<<err[1]<<" "<<err[2]<<std::endl;
        //        std::cout<<"err_wh_inlier: "<<err_wh_inlier[0]<<" "<<err_wh_inlier[1]<<" "<<err_wh_inlier[2]<<std::endl;
        //        std::cout<<"err_wh_outlier: "<<err_wh_outlier[0]<<" "<<err_wh_outlier[1]<<" "<<err_wh_outlier[2]<<std::endl;
        //
        //        std::cout<<"p_inlier, p_outlier, sumP: "<<p_inlier<<" "<<p_outlier<<" " << sumP << std::endl;
        //
        //        std::cout<<"prior_inlier_, prior_outlier_: "<<prior_inlier_<<" "<<prior_outlier_<< std::endl;
        //
        //        double s_inl  = -0.5 * err_wh_inlier.dot(err_wh_inlier);
        //        double s_outl = -0.5 * err_wh_outlier.dot(err_wh_outlier);
        //        std::cout<<"s_inl, s_outl: "<<s_inl<<" "<<s_outl<<std::endl;
        //
        //        std::cout<<"norm of invCov_inlier, invCov_outlier: "<<invCov_inlier.norm()<<" "<<invCov_outlier.norm()<<std::endl;
        //        double q_inl  = invCov_inlier.norm() * exp( -0.5 * err_wh_inlier.dot(err_wh_inlier) );
        //        double q_outl = invCov_outlier.norm() * exp( -0.5 * err_wh_outlier.dot(err_wh_outlier) );
        //        std::cout<<"q_inl, q_outl: "<<q_inl<<" "<<q_outl<<std::endl;

        //        Matrix Cov_inlier  = invCov_inlier.inverse();
        //        Matrix Cov_outlier = invCov_outlier.inverse();
        //        std::cout<<"Cov_inlier: "<<std::endl<<
        //            Cov_inlier(0,0) << " " << Cov_inlier(0,1) << " " << Cov_inlier(0,2) <<std::endl<<
        //            Cov_inlier(1,0) << " " << Cov_inlier(1,1) << " " << Cov_inlier(1,2) <<std::endl<<
        //            Cov_inlier(2,0) << " " << Cov_inlier(2,1) << " " << Cov_inlier(2,2) <<std::endl;
        //        std::cout<<"Cov_outlier: "<<std::endl<<
        //                    Cov_outlier(0,0) << " " << Cov_outlier(0,1) << " " << Cov_outlier(0,2) <<std::endl<<
        //                    Cov_outlier(1,0) << " " << Cov_outlier(1,1) << " " << Cov_outlier(1,2) <<std::endl<<
        //                    Cov_outlier(2,0) << " " << Cov_outlier(2,1) << " " << Cov_outlier(2,2) <<std::endl;
        //        std::cout<<"===="<<std::endl;
      }


      return err_wh_eq;
    }

    /* ************************************************************************* */
    gtsam::Vector calcIndicatorProb(const gtsam::Values& x) const {

      Vector err =  unwhitenedError(x);

      // Calculate indicator probabilities (inlier and outlier)
      Vector err_wh_inlier  = model_inlier_->whiten(err);
      Vector err_wh_outlier = model_outlier_->whiten(err);

      Matrix invCov_inlier  = model_inlier_->R().transpose() * model_inlier_->R();
      Matrix invCov_outlier = model_outlier_->R().transpose() * model_outlier_->R();

      double p_inlier  = prior_inlier_ * sqrt(invCov_inlier.norm()) * exp( -0.5 * err_wh_inlier.dot(err_wh_inlier) );
      double p_outlier = prior_outlier_ * sqrt(invCov_outlier.norm()) * exp( -0.5 * err_wh_outlier.dot(err_wh_outlier) );

      double sumP = p_inlier + p_outlier;
      p_inlier  /= sumP;
      p_outlier /= sumP;

      // Bump up near-zero probabilities (as in linerFlow.h)
      double minP = 0.05; // == 0.1 / 2 indicator variables
      if (p_inlier < minP || p_outlier < minP){
        if (p_inlier < minP)
          p_inlier = minP;
        if (p_outlier < minP)
          p_outlier = minP;
        sumP = p_inlier + p_outlier;
        p_inlier  /= sumP;
        p_outlier /= sumP;
      }

      return Vector_(2, p_inlier, p_outlier);
    }

    /* ************************************************************************* */
    gtsam::Vector unwhitenedError(const gtsam::Values& x) const {

      bool debug = true;

      const T& p1 = x.at<T>(key1_);
      const T& p2 = x.at<T>(key2_);

      Matrix H1, H2;

      T hx = p1.between(p2, H1, H2); // h(x)

      return measured_.localCoordinates(hx);
    }

    /* ************************************************************************* */
    /** return the measured */
    const VALUE& measured() const {
      return measured_;
    }

    /** number of variables attached to this factor */
    std::size_t size() const {
      return 2;
    }

    virtual size_t dim() const {
      return model_inlier_->R().rows() + model_inlier_->R().cols();
    }

  private:

    /** Serialization function */
    friend class boost::serialization::access;
    template<class ARCHIVE>
    void serialize(ARCHIVE & ar, const unsigned int version) {
      ar & boost::serialization::make_nvp("NonlinearFactor",
          boost::serialization::base_object<Base>(*this));
      ar & BOOST_SERIALIZATION_NVP(measured_);
    }
  }; // \class BetweenFactorEM

} /// namespace gtsam
