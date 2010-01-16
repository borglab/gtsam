/*
 * NoiseModel.h
 *
 *  Created on: Jan 13, 2010
 *      Author: richard
 */

#pragma once

#include <boost/shared_ptr.hpp>
#include <boost/numeric/ublas/traits.hpp>
//#include <iostream>
//using namespace std;

#include "Vector.h"
#include "Matrix.h"

namespace gtsam {

  // Forward declaration
  class NoiseModel;

  /*****************************************************************************
   * NoiseModelBase is the abstract base class for all noise models.  NoiseModels
   * must implement a 'whiten' function to normalize an error vector, and an
   * 'unwhiten' function to unnormalize an error vector.
   */
  class NoiseModelBase {
  public:
    /**
     * Whiten an error vector.
     */
    virtual Vector whiten(const Vector& v) const = 0;

    /**
     * Unwhiten an error vector.
     */
    virtual Vector unwhiten(const Vector& v) const = 0;

    friend class NoiseModel;

  private:
    /**
     * Used internally to duplicate the object while retaining the type.
     */
    virtual boost::shared_ptr<NoiseModelBase> clone() const = 0;
  };



  /*****************************************************************************
   * NoiseModel is a container for NoiseModelBase, which internally stores
   * a shared_ptr to a NoiseModelBase as to support fast and compact storage and
   * copies.  Copying this class simply assigns the internal shared_ptr.
   */
  class NoiseModel {
  private:
    const boost::shared_ptr<const NoiseModelBase> base_;

    /**
     * Fast constructor, simply assigns shared_ptr.
     */
    NoiseModel(boost::shared_ptr<NoiseModelBase> noiseModel): base_(noiseModel) {
      /*std::cout << "Assigning pointer" << std::endl;*/
    }

  public:
    /**
     * Fast copy constructor, simply assigns shared_ptr.
     */
    NoiseModel(const NoiseModel& noiseModel): base_(noiseModel.base_) { /*std::cout << "Assigning pointer" << std::endl;*/ }

    /**
     * Constructor that creates a fast-copyable NoiseModel class by cloning
     * a non-pointer NoiseModelBase.  The type is retained and can be retrieved
     * using a dynamic_cast.
     */
    template<class T>
    NoiseModel(const T& noiseModel): base_(noiseModel.clone()) {}

    /**
     * Cast to boost::shared_ptr<NoiseModelBase> to retrieve a pointer to the
     * NoiseModelBase type.  Can be used with dynamic_pointer_cast to retrieve
     * the type at runtime.
     * E.g.:  dynamic_pointer_cast<const Isotropic>(noiseModel).
     */
    operator const boost::shared_ptr<const NoiseModelBase> () const {
      return base_; }

    /**
     * Call the NoiseModelBase virtual whiten function
     */
    Vector whiten(const Vector& v) const { return base_->whiten(v); }

    /**
     * Call the NoiseModelBase virtual unwhiten function
     */
    Vector unwhiten(const Vector& v) const { return base_->unwhiten(v); }

    template<class T> friend boost::shared_ptr<const T> dynamic_pointer_cast(const NoiseModel& p);
  };

  template<class T>
  boost::shared_ptr<const T> dynamic_pointer_cast(const NoiseModel& p) {
    return boost::dynamic_pointer_cast<const T>(p.base_); }



  /*****************************************************************************
   * An isotropic noise model assigns the same sigma to each vector element.
   * This class has no public constructors.  Instead, use either either the
   * Sigma or Variance class.
   */
  class Isotropic : public NoiseModelBase {
  protected:
    double sigma_;
    double invsigma_;

    Isotropic(double sigma): sigma_(sigma), invsigma_(1.0/sigma) {}
    Isotropic(const Isotropic& isotropic):
      sigma_(isotropic.sigma_), invsigma_(isotropic.invsigma_) {}

  public:
    /**
     * Whiten error vector by dividing by sigma
     */
    virtual Vector whiten(const Vector& v) const { return v * invsigma_; }

    /**
     * Unwhiten error vector by multiplying by sigma
     */
    virtual Vector unwhiten(const Vector& v) const { return v * sigma_; }

    /**
     * Clone is used to duplicate object while retaining type
     */
    boost::shared_ptr<NoiseModelBase> clone() const {
      /*cout << "Cloning Isotropic" << endl;*/
      return boost::shared_ptr<NoiseModelBase>(new Isotropic(*this)); }
  };


  /*****************************************************************************
   * A diagonal noise model implements a diagonal covariance matrix, with the
   * elements of the diagonal specified in a Vector.  This class has no public
   * constructors, instead, use either the Sigmas or Variances class.
   */
  class Diagonal : public NoiseModelBase {
  protected:
    Vector sigmas_;
    Vector invsigmas_;

    Diagonal() {}
    Diagonal(const Vector& sigmas): sigmas_(sigmas), invsigmas_(1.0 / sigmas) {}
    Diagonal(const Diagonal& d): sigmas_(d.sigmas_), invsigmas_(d.invsigmas_) {}

  public:
    /**
     * Whiten error vector by dividing by sigmas
     */
    virtual Vector whiten(const Vector& v) const { return emul(v, invsigmas_); }

    /**
     * Unwhiten error vector by multiplying by sigmas
     */
    virtual Vector unwhiten(const Vector& v) const { return emul(v, sigmas_); }

    /**
     * Clone is used to duplicate object while retaining type
     */
    boost::shared_ptr<NoiseModelBase> clone() const {
      /*cout << "Cloning Isotropic" << endl;*/
      return boost::shared_ptr<NoiseModelBase>(new Diagonal(*this)); }
  };


  /*****************************************************************************
   * A full covariance noise model.
   */
  class FullCovariance : public NoiseModelBase {
  protected:
    Matrix sqrt_covariance_;
    Matrix sqrt_inv_covariance_;

  public:
    FullCovariance(const Matrix& covariance):
        sqrt_covariance_(square_root_positive(covariance)),
        sqrt_inv_covariance_(inverse_square_root(covariance)) {}

    FullCovariance(const FullCovariance& c):
        sqrt_covariance_(c.sqrt_covariance_), sqrt_inv_covariance_(c.sqrt_inv_covariance_) {}

    /**
     * Whiten error vector by dividing by sigmas
     */
    virtual Vector whiten(const Vector& v) const { return sqrt_inv_covariance_ * v; }

    /**
     * Unwhiten error vector by multiplying by sigmas
     */
    virtual Vector unwhiten(const Vector& v) const { return sqrt_covariance_ * v; }

    /**
     * Clone is used to duplicate object while retaining type
     */
    boost::shared_ptr<NoiseModelBase> clone() const {
      /*cout << "Cloning Isotropic" << endl;*/
      return boost::shared_ptr<NoiseModelBase>(new FullCovariance(*this)); }
  };



  /*****************************************************************************
   * An isotropic noise model using sigma, the noise standard
   * deviation.
   */
  class Sigma : public Isotropic {
  public:
    Sigma(const Sigma& isotropic): Isotropic(isotropic) { /*cout << "Constructing Sigma from Sigma" << endl;*/ }

    Sigma(double sigma): Isotropic(sigma) { /*cout << "Constructing Sigma from double" << endl;*/ }

    boost::shared_ptr<NoiseModelBase> clone() const {
      return boost::shared_ptr<NoiseModelBase>(new Sigma(*this)); }
  };


  /*****************************************************************************
   * An isotropic noise model using the noise variance = sigma^2.
   */
  class Variance : public Isotropic {
  public:
    Variance(const Variance& v): Isotropic(v) {}

    Variance(double variance): Isotropic(sqrt(variance)) {}

    boost::shared_ptr<NoiseModelBase> clone() const {
      return boost::shared_ptr<NoiseModelBase>(new Variance(*this)); }
  };


  /*****************************************************************************
   * A diagonal noise model created by specifying a Vector of sigmas, i.e.
   * standard devations, i.e. the diagonal of the square root covariance
   * matrix.
   */
  class Sigmas : public Diagonal {
  public:
    Sigmas(const Sigmas& s): Diagonal(s) {}

    Sigmas(const Vector& sigmas): Diagonal(sigmas) {}

    boost::shared_ptr<NoiseModelBase> clone() const {
      return boost::shared_ptr<NoiseModelBase>(new Sigmas(*this)); }
  };


  /*****************************************************************************
   * A diagonal noise model created by specifying a Vector of variances, i.e.
   * i.e. the diagonal of the covariance matrix.
   */
  class Variances : public Diagonal {
  public:
    Variances(const Variances& s): Diagonal(s) {}

    Variances(const Vector& variances) {
      std::transform(variances.begin(), variances.end(), sigmas_.begin(), sqrt);
      invsigmas_ = 1.0 / sigmas_;
    }

    boost::shared_ptr<NoiseModelBase> clone() const {
      return boost::shared_ptr<NoiseModelBase>(new Variances(*this)); }
  };

}
