/*
 * NoiseModel.h
 *
 *  Created on: Jan 13, 2010
 *      Author: Richard Roberts
 *      Author: Frank Dellaert
 */

#pragma once

#include <boost/shared_ptr.hpp>
//#include "Testable.h" TODO
#include "Vector.h"
#include "Matrix.h"

namespace gtsam {

  /**
   * NoiseModel is the abstract base class for all noise models.
   *
   * It must implement a 'whiten' function to normalize an error vector, and an
   * 'unwhiten' function to unnormalize an error vector.
   */
  class NoiseModel /* TODO : public Testable<NoiseModel> */ {

  protected:

  	size_t dim_;

  public:

  	NoiseModel(size_t dim):dim_(dim) {}

    /**
     * Whiten an error vector.
     */
    virtual Vector whiten(const Vector& v) const = 0;

    /**
     * Unwhiten an error vector.
     */
     virtual Vector unwhiten(const Vector& v) const = 0;
  };

  /**
	 * GaussianNoiseModel implements the mathematical model
	 *  |R*x|^2 = |y|^2 with R'*R=inv(Sigma)
	 * where
	 *   y = whiten(x) = R*x
	 *   x = unwhiten(x) = inv(R)*y
	 * as indeed
	 *   |y|^2 = y'*y = x'*R'*R*x
	 * Various simplified models are available that implement this efficiently.
	 */
	struct GaussianNoiseModel : public NoiseModel {

		GaussianNoiseModel(size_t dim):NoiseModel(dim) {}

    /**
	   * Return R itself, but note that Whiten(H) is cheaper than R*H
	   */
	  virtual Matrix R() const = 0;

	  /**
	   * Multiply a derivative with R (derivative of whiten)
	   * Equivalent to whitening each column of the input matrix.
	   */
	  Matrix Whiten(const Matrix& H) const;

	  /**
	   * In-place version
	   */
	  void WhitenInPlace(Matrix& H) const;
	};

	/**
	 * We identify the Gaussian noise model with R
	 */
	// FD: does not work, ambiguous overload :-(
  // inline Vector operator*(const GaussianNoiseModel& R, const Vector& v) {return R.whiten(v);}

  /**
   * UnitCovariance: i.i.d. noise on all m dimensions.
   */
  class UnitCovariance : public GaussianNoiseModel {
  protected:
    double sigma_;
    double invsigma_;

    UnitCovariance(size_t dim): GaussianNoiseModel(dim) {}

  public:
    Vector whiten(const Vector& v) const { return v; }
    Vector unwhiten(const Vector& v) const { return v; }
    Matrix R() const { return eye(dim_); }
    Matrix Whiten(const Matrix& H) const { return H; }
	  void WhitenInPlace(Matrix& H) const {}
  };

  /**
   * An isotropic noise model corresponds to a scaled diagonal covariance
   * This class has no public constructors.  Instead, use either either the
   * Sigma or Variance class.
   */
  class Isotropic : public GaussianNoiseModel {
  protected:
    double sigma_;
    double invsigma_;

    Isotropic(size_t dim, double sigma) :
			GaussianNoiseModel(dim), sigma_(sigma), invsigma_(1.0 / sigma) {}

  public:
    Vector whiten(const Vector& v) const;
    Vector unwhiten(const Vector& v) const;
    Matrix R() const { return diag(repeat(dim_,invsigma_)); }
  };

  /**
   * An isotropic noise model using sigma, the standard deviation.
   */
  class Sigma : public Isotropic {
  public:
    Sigma(size_t n, double sigma): Isotropic(n, sigma) {}
  };

  /**
   * An isotropic noise model using the noise variance = sigma^2.
   */
  class Variance : public Isotropic {
  public:
    Variance(size_t n, double variance): Isotropic(n, sqrt(variance)) {}
  };

  /**
   * A diagonal noise model implements a diagonal covariance matrix, with the
   * elements of the diagonal specified in a Vector.  This class has no public
   * constructors, instead, use either the Sigmas or Variances class.
   */
  class Diagonal : public GaussianNoiseModel {
  protected:
    Vector sigmas_;
    Vector invsigmas_;

    Diagonal(const Vector& sigmas);

  public:
    Vector whiten(const Vector& v) const;
    Vector unwhiten(const Vector& v) const;
    Matrix R() const { return diag(invsigmas_); }
  };

  /**
   * A diagonal noise model created by specifying a Vector of sigmas, i.e.
   * standard devations, i.e. the diagonal of the square root covariance
   * matrix.
   */
  class Sigmas : public Diagonal {
  public:
    Sigmas(const Vector& sigmas): Diagonal(sigmas) {}
  };

  /**
   * A diagonal noise model created by specifying a Vector of variances, i.e.
   * i.e. the diagonal of the covariance matrix.
   */
  class Variances : public Diagonal {
  public:
    Variances(const Vector& variances);
  };

  /**
   * A full covariance noise model.
   */
  class FullCovariance : public GaussianNoiseModel {
  protected:
    Matrix sqrt_covariance_;
    Matrix sqrt_inv_covariance_;

  public:
    FullCovariance(const Matrix& covariance);
    FullCovariance(const FullCovariance& c);
    Vector whiten(const Vector& v) const;
    Vector unwhiten(const Vector& v) const;
    Matrix R() const { return sqrt_inv_covariance_; }
  };

}
