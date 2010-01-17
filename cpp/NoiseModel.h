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
  	virtual ~NoiseModel() {}

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
	 * Various derived classes are available that are more efficient.
	 */
	struct GaussianNoiseModel: public NoiseModel {

	protected:

		// TODO: store as boost upper-triangular or whatever is passed from above
		/* Matrix square root of information matrix (R) */
		Matrix sqrt_information_;

		/** protected constructor takes square root information matrix */
		GaussianNoiseModel(const Matrix& sqrt_information) :
			NoiseModel(sqrt_information.size1()), sqrt_information_(sqrt_information) {
		}

	public:

		typedef boost::shared_ptr<GaussianNoiseModel> shared_ptr;

    /**
     * A Gaussian noise model created by specifying a square root information matrix.
     */
    static shared_ptr SqrtInformation(const Matrix& R) {
    	return shared_ptr(new GaussianNoiseModel(R));
    }

    /**
     * A Gaussian noise model created by specifying a covariance matrix.
     */
    static shared_ptr Covariance(const Matrix& Sigma) {
    	return shared_ptr(new GaussianNoiseModel(inverse_square_root(Sigma)));
    }

    /**
     * A Gaussian noise model created by specifying an information matrix.
     */
    static shared_ptr Information(const Matrix& Q)  {
    	return shared_ptr(new GaussianNoiseModel(square_root_positive(Q)));
    }

    virtual Vector whiten(const Vector& v) const;
		virtual Vector unwhiten(const Vector& v) const;

		/**
		 * Multiply a derivative with R (derivative of whiten)
		 * Equivalent to whitening each column of the input matrix.
		 */
		virtual Matrix Whiten(const Matrix& H) const;

		/**
		 * In-place version
		 */
		virtual void WhitenInPlace(Matrix& H) const;

		/**
		 * Return R itself, but note that Whiten(H) is cheaper than R*H
		 */
		const Matrix& R() const {
			return sqrt_information_;
		}

	}; // GaussianNoiseModel

	// FD: does not work, ambiguous overload :-(
  // inline Vector operator*(const GaussianNoiseModel& R, const Vector& v) {return R.whiten(v);}

  /**
   * A diagonal noise model implements a diagonal covariance matrix, with the
   * elements of the diagonal specified in a Vector.  This class has no public
   * constructors, instead, use the static constructor functions Sigmas etc...
   */
  class Diagonal : public GaussianNoiseModel {
  protected:

  	/** sigmas and reciprocal */
    Vector sigmas_, invsigmas_;

    /** protected constructor takes sigmas */
    Diagonal(const Vector& sigmas);

  public:

		typedef boost::shared_ptr<Diagonal> shared_ptr;

    /**
     * A diagonal noise model created by specifying a Vector of sigmas, i.e.
     * standard devations, the diagonal of the square root covariance matrix.
     */
    static shared_ptr Sigmas(const Vector& sigmas) {
    	return shared_ptr(new Diagonal(sigmas));
    }

    /**
     * A diagonal noise model created by specifying a Vector of variances, i.e.
     * i.e. the diagonal of the covariance matrix.
     */
    static shared_ptr Variances(const Vector& variances) {
    	return shared_ptr(new Diagonal(esqrt(variances)));
    }

    /**
     * A diagonal noise model created by specifying a Vector of precisions, i.e.
     * i.e. the diagonal of the information matrix, i.e., weights
     */
    static shared_ptr Precisions(const Vector& precisions) {
    	return Variances(reciprocal(precisions));
    }

    virtual Vector whiten(const Vector& v) const;
    virtual Vector unwhiten(const Vector& v) const;
  };

  /**
   * An isotropic noise model corresponds to a scaled diagonal covariance
   * To construct, use one of the static methods
   */
  class Isotropic : public Diagonal {
  protected:
    double sigma_, invsigma_;

    /** protected constructor takes sigma */
    Isotropic(size_t dim, double sigma) :
			Diagonal(repeat(dim, sigma)),sigma_(sigma),invsigma_(1.0/sigma) {}

  public:

		typedef boost::shared_ptr<Isotropic> shared_ptr;

    /**
     * An isotropic noise model created by specifying a standard devation sigma
     */
    static shared_ptr Sigma(size_t dim, double sigma) {
    	return shared_ptr(new Isotropic(dim, sigma));
    }

    /**
     * An isotropic noise model created by specifying a variance = sigma^2.
     */
    static shared_ptr Variance(size_t dim, double variance)  {
    	return shared_ptr(new Isotropic(dim, sqrt(variance)));
    }

    /**
     * An isotropic noise model created by specifying a precision
     */
    static shared_ptr Precision(size_t dim, double precision)  {
    	return Variance(dim, 1.0/precision);
    }

    virtual Vector whiten(const Vector& v) const;
    virtual Vector unwhiten(const Vector& v) const;
  };

  /**
   * UnitCovariance: i.i.d. unit-variance noise on all m dimensions.
   */
  class UnitCovariance : public Isotropic {
  protected:

    UnitCovariance(size_t dim): Isotropic(dim,1.0) {}

  public:

    typedef boost::shared_ptr<UnitCovariance> shared_ptr;

    /**
     * An isotropic noise model created by specifying a standard devation sigma
     */
    static shared_ptr Create(size_t dim);

    virtual Vector whiten(const Vector& v) const { return v; }
    virtual Vector unwhiten(const Vector& v) const { return v; }
    virtual Matrix Whiten(const Matrix& H) const { return H; }
	  virtual void WhitenInPlace(Matrix& H) const {}
  };

}
