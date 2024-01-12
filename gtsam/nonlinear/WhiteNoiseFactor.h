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

#pragma once

#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/linear/HessianFactor.h>
#include <cmath>

namespace gtsam {

  const double logSqrt2PI = log(std::sqrt(2.0 * M_PI)); ///< constant needed below

  /**
   * @brief Binary factor to estimate parameters of zero-mean Gaussian white noise
   *
   * This factor uses the mean-precision parameterization.
   *
   * Takes three template arguments:
   * Key: key type to use for mean
   * Key: key type to use for precision
   * Values: Values type for optimization
   * \nosubgrouping
   */
  class WhiteNoiseFactor: public NonlinearFactor {

  private:

    double z_; ///< Measurement

    Key meanKey_; ///< key by which to access mean variable
    Key precisionKey_; ///< key by which to access precision variable

    typedef NonlinearFactor Base;

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
        Key j1, Key j2) {
      double e = u - z, e2 = e * e;
      double c = 2 * logSqrt2PI - log(p) + e2 * p;
      Vector g1 = (Vector(1) << -e * p).finished();
      Vector g2 = (Vector(1) <<  0.5 / p - 0.5 * e2).finished();
      Matrix G11 = (Matrix(1, 1) << p).finished();
      Matrix G12 = (Matrix(1, 1) << e).finished();
      Matrix G22 = (Matrix(1, 1) << 0.5 / (p * p)).finished();
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
    WhiteNoiseFactor(double z, Key meanKey, Key precisionKey) :
        Base(), z_(z), meanKey_(meanKey), precisionKey_(precisionKey) {
    }

    /// @}
    /// @name Advanced Constructors
    /// @{

    /// Destructor
    ~WhiteNoiseFactor() override {
    }

    /// @}
    /// @name Testable
    /// @{

    /// Print
    void print(const std::string& p = "WhiteNoiseFactor",
        const KeyFormatter& keyFormatter = DefaultKeyFormatter) const override {
      Base::print(p, keyFormatter);
      std::cout << p + ".z: " << z_ << std::endl;
    }

    /// @}
    /// @name Standard Interface
    /// @{

    /// get the dimension of the factor (number of rows on linearization)
    size_t dim() const override {
      return 2;
    }

    /// Calculate the error of the factor, typically equal to log-likelihood
    double error(const Values& x) const override {
      return f(z_, x.at<double>(meanKey_), x.at<double>(precisionKey_));
    }

    /**
     * Vector of errors
     * "unwhitened" does not make sense for this factor
     * What is meant typically is only "e" above
     * Here we shoehorn sqrt(2*error(p))
     * TODO: Where is this used? should disappear.
     */
    virtual Vector unwhitenedError(const Values& x) const {
      return (Vector(1) << std::sqrt(2 * error(x))).finished();
    }

    /**
     * Create a symbolic factor using the given ordering to determine the
     * variable indices.
     */
//    virtual IndexFactor::shared_ptr symbolic(const Ordering& ordering) const {
//      const Key j1 = ordering[meanKey_], j2 = ordering[precisionKey_];
//      return IndexFactor::shared_ptr(new IndexFactor(j1, j2));
//    }

    /// @}
    /// @name Advanced Interface
    /// @{

    /// linearize returns a Hessianfactor that is an approximation of error(p)
    std::shared_ptr<GaussianFactor> linearize(const Values& x) const override {
      double u = x.at<double>(meanKey_);
      double p = x.at<double>(precisionKey_);
      Key j1 = meanKey_;
      Key j2 = precisionKey_;
      return linearize(z_, u, p, j1, j2);
    }

    // TODO: Frank commented this out for now, can it go?
    //    /// @return a deep copy of this factor
    //    gtsam::NonlinearFactor::shared_ptr clone() const override {
    //      return std::static_pointer_cast<gtsam::NonlinearFactor>(
    //          gtsam::NonlinearFactor::shared_ptr(new This(*this))); }

    /// @}

  };
// WhiteNoiseFactor

}// namespace gtsam

