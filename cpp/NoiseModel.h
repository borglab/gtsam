/*
 * NoiseModel.h
 *
 *  Created on: Jan 13, 2010
 *      Author: Richard Roberts
 *      Author: Frank Dellaert
 */

#pragma once

#include "Vector.h"
#include "Matrix.h"

namespace gtsam {

  /**
   * NoiseModel is the abstract base class for all noise models.
   *
   * It must implement a 'whiten' function to normalize an error vector, and an
   * 'unwhiten' function to unnormalize an error vector.
   */
  struct NoiseModel {

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

	  /**
	   * Multiply a derivative with R (derivative of whiten)
	   * Equivalent to whitening each column of the input matrix.
	   */
	  Matrix whiten(const Matrix& H);

	};

	/**
	 * We identify the Gaussian noise model with R
	 */
	// FD: does not work, ambiguous overload :-(
  // inline Vector operator*(const GaussianNoiseModel& R, const Vector& v) {return R.whiten(v);}

  /**
   * An isotropic noise model corresponds to a scaled diagonal covariance
   * This class has no public constructors.  Instead, use either either the
   * Sigma or Variance class.
   */
  class Isotropic : public GaussianNoiseModel {
  protected:
    double sigma_;
    double invsigma_;

    Isotropic(double sigma): sigma_(sigma), invsigma_(1.0/sigma) {}
    Isotropic(const Isotropic& isotropic):
      sigma_(isotropic.sigma_), invsigma_(isotropic.invsigma_) {}

  public:
    Vector whiten(const Vector& v) const;
    Vector unwhiten(const Vector& v) const;
  };

  /**
   * An isotropic noise model using sigma, the standard deviation.
   */
  class Sigma : public Isotropic {
  public:
    Sigma(const Sigma& isotropic): Isotropic(isotropic) {}
    Sigma(double sigma): Isotropic(sigma) {}
  };

  /**
   * An isotropic noise model using the noise variance = sigma^2.
   */
  class Variance : public Isotropic {
  public:
    Variance(const Variance& v): Isotropic(v) {}
    Variance(double variance): Isotropic(sqrt(variance)) {}
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

    Diagonal() {}
    Diagonal(const Vector& sigmas);
    Diagonal(const Diagonal& d);

  public:
    Vector whiten(const Vector& v) const;
    Vector unwhiten(const Vector& v) const;
  };

  /**
   * A diagonal noise model created by specifying a Vector of sigmas, i.e.
   * standard devations, i.e. the diagonal of the square root covariance
   * matrix.
   */
  class Sigmas : public Diagonal {
  public:
    Sigmas(const Sigmas& s): Diagonal(s) {}
    Sigmas(const Vector& sigmas): Diagonal(sigmas) {}
  };

  /**
   * A diagonal noise model created by specifying a Vector of variances, i.e.
   * i.e. the diagonal of the covariance matrix.
   */
  class Variances : public Diagonal {
  public:
    Variances(const Variances& s): Diagonal(s) {}
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
  };

}
