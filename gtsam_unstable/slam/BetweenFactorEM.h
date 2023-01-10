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
#include <gtsam/nonlinear/Marginals.h>

namespace gtsam {

/**
 * A class for a measurement predicted by "between(config[key1],config[key2])"
 * @tparam VALUE the Value type
 * @ingroup slam
 */
template<class VALUE>
class BetweenFactorEM: public NonlinearFactor {

public:

  typedef VALUE T;

private:

  typedef BetweenFactorEM<VALUE> This;
  typedef NonlinearFactor Base;

  Key key1_;
  Key key2_;

  VALUE measured_; /** The measurement */

  SharedGaussian model_inlier_;
  SharedGaussian model_outlier_;

  double prior_inlier_;
  double prior_outlier_;

  bool flag_bump_up_near_zero_probs_;

  /** concept check by type */
  GTSAM_CONCEPT_LIE_TYPE(T)
  GTSAM_CONCEPT_TESTABLE_TYPE(T)

public:

  // shorthand for a smart pointer to a factor
  typedef typename boost::shared_ptr<BetweenFactorEM> shared_ptr;

  /** default constructor - only use for serialization */
  BetweenFactorEM() {
  }

  /** Constructor */
  BetweenFactorEM(Key key1, Key key2, const VALUE& measured,
      const SharedGaussian& model_inlier, const SharedGaussian& model_outlier,
      const double prior_inlier, const double prior_outlier,
      const bool flag_bump_up_near_zero_probs = false) :
      Base(KeyVector{key1, key2}), key1_(key1), key2_(key2), measured_(
          measured), model_inlier_(model_inlier), model_outlier_(model_outlier), prior_inlier_(
          prior_inlier), prior_outlier_(prior_outlier), flag_bump_up_near_zero_probs_(
          flag_bump_up_near_zero_probs) {
  }

  ~BetweenFactorEM() override {
  }

  /** implement functions needed for Testable */

  /** print */
  void print(const std::string& s, const KeyFormatter& keyFormatter =
      DefaultKeyFormatter) const override {
    std::cout << s << "BetweenFactorEM(" << keyFormatter(key1_) << ","
        << keyFormatter(key2_) << ")\n";
    measured_.print("  measured: ");
    model_inlier_->print("  noise model inlier: ");
    model_outlier_->print("  noise model outlier: ");
    std::cout << "(prior_inlier, prior_outlier_) = (" << prior_inlier_ << ","
        << prior_outlier_ << ")\n";
    //      Base::print(s, keyFormatter);
  }

  /** equals */
  bool equals(const NonlinearFactor& f, double tol = 1e-9) const override {
    const This *t = dynamic_cast<const This*>(&f);

    if (t && Base::equals(f))
      return key1_ == t->key1_ && key2_ == t->key2_
          &&
          //            model_inlier_->equals(t->model_inlier_ ) && // TODO: fix here
          //            model_outlier_->equals(t->model_outlier_ ) &&
          prior_outlier_ == t->prior_outlier_
          && prior_inlier_ == t->prior_inlier_ && measured_.equals(t->measured_);
    else
      return false;
  }

  /** implement functions needed to derive from Factor */

  /* ************************************************************************* */
  double error(const Values &x) const override {
    return whitenedError(x).squaredNorm();
  }

  /* ************************************************************************* */
  /**
   * Linearize a non-linearFactorN to get a GaussianFactor,
   * \f$ Ax-b \approx h(x+\delta x)-z = h(x) + A \delta x - z \f$
   * Hence \f$ b = z - h(x) = - \mathtt{error\_vector}(x) \f$
   */
  /* This version of linearize recalculates the noise model each time */
  boost::shared_ptr<GaussianFactor> linearize(const Values &x) const override {
    // Only linearize if the factor is active
    if (!this->active(x))
      return boost::shared_ptr<JacobianFactor>();

    //std::cout<<"About to linearize"<<std::endl;
    Matrix A1, A2;
    std::vector<Matrix> A(this->size());
    Vector b = -whitenedError(x, A);
    A1 = A[0];
    A2 = A[1];

    return GaussianFactor::shared_ptr(
        new JacobianFactor(key1_, A1, key2_, A2, b,
            noiseModel::Unit::Create(b.size())));
  }

  /* ************************************************************************* */
  Vector whitenedError(const Values& x,
      OptionalMatrixVecType H = OptionalMatrixVecNone) const {

    bool debug = true;

    const T& p1 = x.at<T>(key1_);
    const T& p2 = x.at<T>(key2_);

    Matrix H1, H2;

    T hx = p1.between(p2, H1, H2); // h(x)
    // manifold equivalent of h(x)-z -> log(z,h(x))

    Vector err = measured_.localCoordinates(hx);

    // Calculate indicator probabilities (inlier and outlier)
    Vector p_inlier_outlier = calcIndicatorProb(x);
    double p_inlier = p_inlier_outlier[0];
    double p_outlier = p_inlier_outlier[1];

    Vector err_wh_inlier = model_inlier_->whiten(err);
    Vector err_wh_outlier = model_outlier_->whiten(err);

    Matrix invCov_inlier = model_inlier_->R().transpose() * model_inlier_->R();
    Matrix invCov_outlier = model_outlier_->R().transpose()
        * model_outlier_->R();

    Vector err_wh_eq;
    err_wh_eq.resize(err_wh_inlier.rows() * 2);
    err_wh_eq << sqrt(p_inlier) * err_wh_inlier.array(), sqrt(p_outlier)
        * err_wh_outlier.array();

    if (H) {
      // stack Jacobians for the two indicators for each of the key

      Matrix H1_inlier = sqrt(p_inlier) * model_inlier_->Whiten(H1);
      Matrix H1_outlier = sqrt(p_outlier) * model_outlier_->Whiten(H1);
      Matrix H1_aug = stack(2, &H1_inlier, &H1_outlier);

      Matrix H2_inlier = sqrt(p_inlier) * model_inlier_->Whiten(H2);
      Matrix H2_outlier = sqrt(p_outlier) * model_outlier_->Whiten(H2);
      Matrix H2_aug = stack(2, &H2_inlier, &H2_outlier);

      (*H)[0].resize(H1_aug.rows(), H1_aug.cols());
      (*H)[1].resize(H2_aug.rows(), H2_aug.cols());

      (*H)[0] = H1_aug;
      (*H)[1] = H2_aug;
    }

    if (debug) {
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

  Vector whitenedError(const Values& x, std::vector<Matrix>& H) const {
	  return whitenedError(x, &H);
  }

  /* ************************************************************************* */
  Vector calcIndicatorProb(const Values& x) const {

    bool debug = false;

    Vector err = unwhitenedError(x);

    // Calculate indicator probabilities (inlier and outlier)
    Vector err_wh_inlier = model_inlier_->whiten(err);
    Vector err_wh_outlier = model_outlier_->whiten(err);

    Matrix invCov_inlier = model_inlier_->R().transpose() * model_inlier_->R();
    Matrix invCov_outlier = model_outlier_->R().transpose()
        * model_outlier_->R();

    double p_inlier = prior_inlier_ * std::sqrt(invCov_inlier.determinant())
        * exp(-0.5 * err_wh_inlier.dot(err_wh_inlier));
    double p_outlier = prior_outlier_ * std::sqrt(invCov_outlier.determinant())
        * exp(-0.5 * err_wh_outlier.dot(err_wh_outlier));

    if (debug) {
      std::cout << "in calcIndicatorProb. err_unwh: " << err[0] << ", "
          << err[1] << ", " << err[2] << std::endl;
      std::cout << "in calcIndicatorProb. err_wh_inlier: " << err_wh_inlier[0]
          << ", " << err_wh_inlier[1] << ", " << err_wh_inlier[2] << std::endl;
      std::cout << "in calcIndicatorProb. err_wh_inlier.dot(err_wh_inlier): "
          << err_wh_inlier.dot(err_wh_inlier) << std::endl;
      std::cout << "in calcIndicatorProb. err_wh_outlier.dot(err_wh_outlier): "
          << err_wh_outlier.dot(err_wh_outlier) << std::endl;

      std::cout
          << "in calcIndicatorProb. p_inlier, p_outlier before normalization: "
          << p_inlier << ", " << p_outlier << std::endl;
    }

    double sumP = p_inlier + p_outlier;
    p_inlier /= sumP;
    p_outlier /= sumP;

    if (flag_bump_up_near_zero_probs_) {
      // Bump up near-zero probabilities (as in linerFlow.h)
      double minP = 0.05; // == 0.1 / 2 indicator variables
      if (p_inlier < minP || p_outlier < minP) {
        if (p_inlier < minP)
          p_inlier = minP;
        if (p_outlier < minP)
          p_outlier = minP;
        sumP = p_inlier + p_outlier;
        p_inlier /= sumP;
        p_outlier /= sumP;
      }
    }

    return (Vector(2) << p_inlier, p_outlier).finished();
  }

  /* ************************************************************************* */
  Vector unwhitenedError(const Values& x) const {

    const T& p1 = x.at<T>(key1_);
    const T& p2 = x.at<T>(key2_);

    Matrix H1, H2;

    T hx = p1.between(p2, H1, H2); // h(x)

    return measured_.localCoordinates(hx);
  }

  /* ************************************************************************* */
  void set_flag_bump_up_near_zero_probs(bool flag) {
    flag_bump_up_near_zero_probs_ = flag;
  }

  /* ************************************************************************* */
  bool get_flag_bump_up_near_zero_probs() const {
    return flag_bump_up_near_zero_probs_;
  }

  /* ************************************************************************* */
  SharedGaussian get_model_inlier() const {
    return model_inlier_;
  }

  /* ************************************************************************* */
  SharedGaussian get_model_outlier() const {
    return model_outlier_;
  }

  /* ************************************************************************* */
  Matrix get_model_inlier_cov() const {
    return (model_inlier_->R().transpose() * model_inlier_->R()).inverse();
  }

  /* ************************************************************************* */
  Matrix get_model_outlier_cov() const {
    return (model_outlier_->R().transpose() * model_outlier_->R()).inverse();
  }

  /* ************************************************************************* */
  void updateNoiseModels(const Values& values,
      const NonlinearFactorGraph& graph) {
    /* Update model_inlier_ and model_outlier_ to account for uncertainty in robot trajectories
     * (note these are given in the E step, where indicator probabilities are calculated).
     *
     * Principle: R += [H1 H2] * joint_cov12 * [H1 H2]', where H1, H2 are Jacobians of the
     * unwhitened error w.r.t. states, and R is the measurement covariance (inlier or outlier modes).
     *
     * TODO: improve efficiency (info form)
     */

    // get joint covariance of the involved states
    KeyVector Keys;
    Keys.push_back(key1_);
    Keys.push_back(key2_);
    Marginals marginals(graph, values, Marginals::QR);
    JointMarginal joint_marginal12 = marginals.jointMarginalCovariance(Keys);
    Matrix cov1 = joint_marginal12(key1_, key1_);
    Matrix cov2 = joint_marginal12(key2_, key2_);
    Matrix cov12 = joint_marginal12(key1_, key2_);

    updateNoiseModels_givenCovs(values, cov1, cov2, cov12);
  }

  /* ************************************************************************* */
  void updateNoiseModels_givenCovs(const Values& values,
      const Matrix& cov1, const Matrix& cov2, const Matrix& cov12) {
    /* Update model_inlier_ and model_outlier_ to account for uncertainty in robot trajectories
     * (note these are given in the E step, where indicator probabilities are calculated).
     *
     * Principle: R += [H1 H2] * joint_cov12 * [H1 H2]', where H1, H2 are Jacobians of the
     * unwhitened error w.r.t. states, and R is the measurement covariance (inlier or outlier modes).
     *
     * TODO: improve efficiency (info form)
     */

    const T& p1 = values.at<T>(key1_);
    const T& p2 = values.at<T>(key2_);

    Matrix H1, H2;
    p1.between(p2, H1, H2); // h(x)

    Matrix H;
    H.resize(H1.rows(), H1.rows() + H2.rows());
    H << H1, H2; // H = [H1 H2]

    Matrix joint_cov;
    joint_cov.resize(cov1.rows() + cov2.rows(), cov1.cols() + cov2.cols());
    joint_cov << cov1, cov12, cov12.transpose(), cov2;

    Matrix cov_state = H * joint_cov * H.transpose();

    //       model_inlier_->print("before:");

    // update inlier and outlier noise models
    Matrix covRinlier =
        (model_inlier_->R().transpose() * model_inlier_->R()).inverse();
    model_inlier_ = noiseModel::Gaussian::Covariance(
        covRinlier + cov_state);

    Matrix covRoutlier =
        (model_outlier_->R().transpose() * model_outlier_->R()).inverse();
    model_outlier_ = noiseModel::Gaussian::Covariance(
        covRoutlier + cov_state);

    //       model_inlier_->print("after:");
    //       std::cout<<"covRinlier + cov_state: "<<covRinlier + cov_state<<std::endl;
  }

  /* ************************************************************************* */
  /** return the measured */
  const VALUE& measured() const {
    return measured_;
  }

  size_t dim() const override {
    return model_inlier_->R().rows() + model_inlier_->R().cols();
  }

private:

  /** Serialization function */
  friend class boost::serialization::access;
  template<class ARCHIVE>
  void serialize(ARCHIVE & ar, const unsigned int /*version*/) {
    ar
        & boost::serialization::make_nvp("NonlinearFactor",
            boost::serialization::base_object<Base>(*this));
    ar & BOOST_SERIALIZATION_NVP(measured_);
  }
};
// \class BetweenFactorEM

/// traits
template<class VALUE>
struct traits<BetweenFactorEM<VALUE> > : public Testable<BetweenFactorEM<VALUE> > {};

}  // namespace gtsam
