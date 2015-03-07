/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file   WhiteNoiseFactor.h
 * @brief  Binary white noise factor
 * @author Chris Beall
 * @author Frank Dellaert
 * @date   September 2011
 */

#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/linear/HessianFactor.h>
#include <gtsam/base/LieScalar.h>
#include <cmath>

namespace gtsam {

	const double logSqrt2PI = log(sqrt(2.0 * M_PI)); ///< constant needed below

	/**
	 * @brief Binary factor to estimate parameters of zero-mean Gaussian white noise
	 *
	 * This factor uses the mean-precision parameterization.
	 *
	 * Takes three template arguments:
	 * MKEY: key type to use for mean
	 * PKEY: key type to use for precision
	 * VALUES: Values type for optimization
	 * \nosubgrouping
	 */
	template<class MKEY, class PKEY, class VALUES>
	class WhiteNoiseFactor: public NonlinearFactor<VALUES> {

	private:

		double z_; ///< Measurement

		MKEY meanKey_; ///< key by which to access mean variable
		PKEY precisionKey_; ///< key by which to access precision variable

		typedef NonlinearFactor<VALUES> Base;

	public:

		/** @brief negative log likelihood as a function of mean \f$ \mu \f$ and precision \f$ \tau \f$
		 * @f[
		 * f(z, \tau, \mu)
		 * = -\log \left( \frac{\sqrt{\tau}}{\sqrt{2\pi}} \exp(-0.5\tau(z-\mu)^2) \right)
		 * = \log(\sqrt{2\pi}) - 0.5* \log(\tau) + 0.5\tau(z-\mu)^2
		 * @f]
		 */
		static double f(double z, double u, double p) {
			return logSqrt2PI - 0.5 * log(p) + 0.5 * (z - u) * (z - u) * p;
		}

		/**
		 * @brief linearize returns a Hessianfactor that approximates error
		 * Hessian is @f[
		 * 0.5f - x^T g + 0.5*x^T G x
		 * @f]
		 * Taylor expansion is @f[
		 * f(x+dx) = f(x) + df(x) dx + 0.5 ddf(x) dx^2
		 * @f]
		 * So f = 2 f(x),  g = -df(x), G = ddf(x)
		 */
		static HessianFactor::shared_ptr linearize(double z, double u, double p,
				Index j1, Index j2) {
			double e = u - z, e2 = e * e;
			double c = 2 * logSqrt2PI - log(p) + e2 * p;
			Vector g1 = Vector_(1, -e * p);
			Vector g2 = Vector_(1, 0.5 / p - 0.5 * e2);
			Matrix G11 = Matrix_(1, 1, p);
			Matrix G12 = Matrix_(1, 1, e);
			Matrix G22 = Matrix_(1, 1, 0.5 / (p * p));
			return HessianFactor::shared_ptr(
					new HessianFactor(j1, j2, G11, G12, g1, G22, g2, c));
		}

		/// @name Standard Constructors
		/// @{

		/** Construct from measurement
		 * @param z Measurment value
		 * @param meanKey Key for mean variable
		 * @param precisionKey Key for precision variable
		 */
		WhiteNoiseFactor(double z, const MKEY& meanKey, const PKEY& precisionKey) :
				Base(), z_(z), meanKey_(meanKey), precisionKey_(precisionKey) {
		}

		/// @}
		/// @name Advanced Constructors
		/// @{

		/// Destructor
		virtual ~WhiteNoiseFactor() {
		}

		/// @}
		/// @name Testable
		/// @{

		/// Print
		void print(const std::string& p = "WhiteNoiseFactor") const {
			Base::print(p);
			std::cout << p + ".z: " << z_ << std::endl;
		}

		/// @}
		/// @name Standard Interface
		/// @{

		/// get the dimension of the factor (number of rows on linearization)
		virtual size_t dim() const {
			return 2;
		}

		/// Calculate the error of the factor, typically equal to log-likelihood
		inline double error(const VALUES& x) const {
			return f(z_, x[meanKey_].value(), x[precisionKey_].value());
		}

		/**
		 * Vector of errors
		 * "unwhitened" does not make sense for this factor
		 * What is meant typically is only "e" above
		 * Here we shoehorn sqrt(2*error(p))
		 * TODO: Where is this used? should disappear.
		 */
		virtual Vector unwhitenedError(const VALUES& x) const {
			return Vector_(1, sqrt(2 * error(x)));
		}

		/**
		 * Create a symbolic factor using the given ordering to determine the
		 * variable indices.
		 */
		virtual IndexFactor::shared_ptr symbolic(const Ordering& ordering) const {
			const Index j1 = ordering[meanKey_], j2 = ordering[precisionKey_];
			return IndexFactor::shared_ptr(new IndexFactor(j1, j2));
		}

		/// @}
		/// @name Advanced Interface
		/// @{

		/// linearize returns a Hessianfactor that is an approximation of error(p)
		virtual boost::shared_ptr<GaussianFactor> linearize(const VALUES& x,
				const Ordering& ordering) const {
			double u = x[meanKey_].value();
			double p = x[precisionKey_].value();
			Index j1 = ordering[meanKey_];
			Index j2 = ordering[precisionKey_];
			return linearize(z_, u, p, j1, j2);
		}

		/// @return a deep copy of this factor
    virtual gtsam::NonlinearFactor::shared_ptr clone() const {
		  return boost::static_pointer_cast<gtsam::NonlinearFactor>(
		      gtsam::NonlinearFactor::shared_ptr(new This(*this))); }

		/// @}

	};
// WhiteNoiseFactor

}// namespace gtsam

