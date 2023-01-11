/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 *  @file   GaussMarkov1stOrderFactor.h
 *  @author Vadim Indelman, Stephen Williams, Luca Carlone
 *  @date   Jan 17, 2012
 **/
#pragma once

#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/linear/GaussianFactor.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/Lie.h>

#include <ostream>

namespace gtsam {

/*
 * - The 1st order GaussMarkov factor relates two keys of the same type. This relation is given via
 *          key_2 = exp(-1/tau*delta_t) * key1 + w_d
 *     where tau is the time constant and delta_t is the time difference between the two keys.
 *     w_d is the equivalent discrete noise, whose covariance is calculated from the continuous noise model and delta_t.
 * - w_d is approximated as a Gaussian noise.
 * - In the multi-dimensional case, tau is a vector, and the above equation is applied on each element
 *      in the state (represented by keys), using the appropriate time constant in the vector tau.
 */

/*
 * A class for a measurement predicted by "GaussMarkov1stOrderFactor(config[key1],config[key2])"
 * KEY1::Value is the Lie Group type
 * T is the measurement type, by default the same
 */
template<class VALUE>
class GaussMarkov1stOrderFactor: public NoiseModelFactorN<VALUE, VALUE> {

private:

  typedef GaussMarkov1stOrderFactor<VALUE> This;
  typedef NoiseModelFactorN<VALUE, VALUE> Base;

  double dt_;
  Vector tau_;

public:
  // Provide access to the Matrix& version of evaluateError:
  using Base::evaluateError;


  // shorthand for a smart pointer to a factor
  typedef typename boost::shared_ptr<GaussMarkov1stOrderFactor> shared_ptr;

  /** default constructor - only use for serialization */
  GaussMarkov1stOrderFactor() {}

  /** Constructor */
  GaussMarkov1stOrderFactor(const Key& key1, const Key& key2, double delta_t, Vector tau,
      const SharedGaussian& model) :
        Base(calcDiscreteNoiseModel(model, delta_t), key1, key2), dt_(delta_t), tau_(tau) {
  }

  ~GaussMarkov1stOrderFactor() override {}

  /** implement functions needed for Testable */

  /** print */
  void print(const std::string& s, const KeyFormatter& keyFormatter = DefaultKeyFormatter) const override {
    std::cout << s << "GaussMarkov1stOrderFactor("
        << keyFormatter(this->key1()) << ","
        << keyFormatter(this->key2()) << ")\n";
    this->noiseModel_->print("  noise model");
  }

  /** equals */
  bool equals(const NonlinearFactor& expected, double tol=1e-9) const override {
    const This *e =  dynamic_cast<const This*> (&expected);
    return e != nullptr && Base::equals(*e, tol);
  }

  /** implement functions needed to derive from Factor */

  /** vector of errors */
  Vector evaluateError(const VALUE& p1, const VALUE& p2,
      OptionalMatrixType H1, OptionalMatrixType H2) const override {

    Vector v1( traits<VALUE>::Logmap(p1) );
    Vector v2( traits<VALUE>::Logmap(p2) );

    Vector alpha(tau_.size());
    Vector alpha_v1(tau_.size());
    for(int i=0; i<tau_.size(); i++){
      alpha(i) = exp(- 1/tau_(i)*dt_ );
      alpha_v1(i) = alpha(i) * v1(i);
    }

    Vector hx(v2 - alpha_v1);

    if(H1) *H1 = -1 * alpha.asDiagonal();
    if(H2) *H2 = Matrix::Identity(v2.size(),v2.size());

    return hx;
  }

private:

  /** Serialization function */
  friend class boost::serialization::access;
  template<class ARCHIVE>
  void serialize(ARCHIVE & ar, const unsigned int /*version*/) {
    ar & BOOST_SERIALIZATION_BASE_OBJECT_NVP(Base);
    ar & BOOST_SERIALIZATION_NVP(dt_);
    ar & BOOST_SERIALIZATION_NVP(tau_);
  }

  SharedGaussian calcDiscreteNoiseModel(const SharedGaussian& model, double delta_t){
    /* Q_d (approx)= Q * delta_t */
    /* In practice, square root of the information matrix is represented, so that:
     *  R_d (approx)= R / sqrt(delta_t)
     * */
    noiseModel::Gaussian::shared_ptr gaussian_model = boost::dynamic_pointer_cast<noiseModel::Gaussian>(model);
    SharedGaussian model_d(noiseModel::Gaussian::SqrtInformation(gaussian_model->R()/sqrt(delta_t)));
    return model_d;
  }

}; // \class GaussMarkov1stOrderFactor

/// traits
template<class VALUE> struct traits<GaussMarkov1stOrderFactor<VALUE> > :
    public Testable<GaussMarkov1stOrderFactor<VALUE> > {
};

} /// namespace gtsam
