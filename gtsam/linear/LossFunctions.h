
/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file NoiseModel.h
 * @date  Jan 13, 2010
 * @author Richard Roberts
 * @author Frank Dellaert
 * @author Fan Jiang
 */

#pragma once

#include <optional>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Testable.h>
#include <gtsam/dllexport.h>

#if GTSAM_ENABLE_BOOST_SERIALIZATION
#include <boost/serialization/extended_type_info.hpp>
#include <boost/serialization/nvp.hpp>
#include <boost/serialization/version.hpp>
#include <boost/serialization/optional.hpp>
#include <boost/serialization/shared_ptr.hpp>
#include <boost/serialization/singleton.hpp>
#endif

namespace gtsam {
namespace noiseModel {
// clang-format off
/**
 * The mEstimator name space contains all robust error functions.
 * It mirrors the exposition at
 *  https://members.loria.fr/MOBerger/Enseignement/Master2/Documents/ZhangIVC-97-01.pdf
 * which talks about minimizing \sum \rho(r_i), where \rho is a loss function of choice.
 *
 * To illustrate, let's consider the least-squares (L2), L1, and Huber estimators as examples:
 *
 * Name        Symbol          Least-Squares   L1-norm    Huber
 * Loss        \rho(x)         0.5*x^2         |x|        0.5*x^2 if |x|<k, 0.5*k^2 + k|x-k| otherwise
 * Derivative  \phi(x)         x               sgn(x)     x       if |x|<k, k sgn(x)         otherwise
 * Weight      w(x)=\phi(x)/x  1               1/|x|      1       if |x|<k, k/|x|            otherwise
 *
 * With these definitions, D(\rho(x), p) = \phi(x) D(x,p) = w(x) x D(x,p) = w(x) D(L2(x), p),
 * and hence we can solve the equivalent weighted least squares problem \sum w(r_i) \rho(r_i)
 *
 * Each M-estimator in the mEstimator name space simply implements the above functions.
 *
 * Each M-estimator additionally implements "graduated" versions of these functions.
 * Name                Symbol
 * Graduated Loss      \rho(x,\mu)
 * Graduated Weight    \w(x,\mu)
 * The control parameter \mu in [0, 1] transitions the loss from convex (\mu=0) to its original robust form (\mu=1).
 * This is used by continuation-style algorithms (GNC, riSAM) to modify the underlying problem structure.
 *
 * GTSAM convention for graduated robust losses:
 *   \mu in [0, 1]
 *   \mu = 0: most convex / least-squares-like initialization
 *   \mu = 1: final target robust loss
 * Increasing \mu always increases non-convexity, and both endpoints are exact
 * rather than approached asymptotically.
 */
// clang-format on
namespace mEstimator {

/**
 * Pure virtual class for all robust error function classes.
 *
 * It provides the machinery for block vs scalar reweighting strategies, in
 * addition to defining the interface of derived classes.
 */
class GTSAM_EXPORT Base {
 public:
  /** the rows can be weighted independently according to the error
   * or uniformly with the norm of the right hand side */
  enum ReweightScheme { Scalar, Block };
  typedef std::shared_ptr<Base> shared_ptr;

 protected:
  /// Strategy for reweighting \sa ReweightScheme
  ReweightScheme reweight_;

 public:
  Base(const ReweightScheme reweight = Block) : reweight_(reweight) {}
  virtual ~Base() {}

  /// Returns the reweight scheme, as explained in ReweightScheme
  ReweightScheme reweightScheme() const { return reweight_; }

  /**
   * This method is responsible for returning the total penalty for a given
   * amount of error. For example, this method is responsible for implementing
   * the quadratic function for an L2 penalty, the absolute value function for
   * an L1 penalty, etc.
   *
   * TODO(mikebosse): When the loss function has as input the norm of the
   * error vector, then it prevents implementations of asymmeric loss
   * functions. It would be better for this function to accept the vector and
   * internally call the norm if necessary.
   *
   * This returns \f$\rho(x)\f$ in \ref mEstimator
   */
  virtual double loss(double distance) const { return 0; }

  /**
   * This method is responsible for returning the weight function for a given
   * amount of error. The weight function is related to the analytic derivative
   * of the loss function. See
   *  https://members.loria.fr/MOBerger/Enseignement/Master2/Documents/ZhangIVC-97-01.pdf
   * for details. This method is required when optimizing cost functions with
   * robust penalties using iteratively re-weighted least squares.
   *
   * This returns w(x) in \ref mEstimator
   */
  virtual double weight(double distance) const = 0;

  /**
   * This method is responsible for returning the total penalty for a given
   * amount of error and the current control parameter \f$\mu\f$.
   *
   * This returns \f$\rho(x, \mu)\f$ in \ref mEstimator
   */
  virtual double graduatedLoss(double distance, double mu) const = 0;

  /**
   * This method is responsible for returning the weight for a given
   * amount of error and the current control parameter \f$\mu\f$.
   *
   * This returns \f$w(x, \mu)\f$ in \ref mEstimator
   */
  virtual double graduatedWeight(double distance, double mu) const = 0;

  virtual void print(const std::string &s) const = 0;
  virtual bool equals(const Base &expected, double tol = 1e-8) const = 0;

  double sqrtWeight(double distance) const { return std::sqrt(weight(distance)); }

  /** produce a weight vector according to an error vector and the implemented
   * robust function */
  Vector weight(const Vector &error) const;

  /** square root version of the weight function */
  Vector sqrtWeight(const Vector &error) const;

  /** reweight block matrices and a vector according to their weight
   * implementation */
  void reweight(Vector &error) const;
  void reweight(std::vector<Matrix> &A, Vector &error) const;
  void reweight(Matrix &A, Vector &error) const;
  void reweight(Matrix &A1, Matrix &A2, Vector &error) const;
  void reweight(Matrix &A1, Matrix &A2, Matrix &A3, Vector &error) const;

 private:
#if GTSAM_ENABLE_BOOST_SERIALIZATION
  /** Serialization function */
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE &ar, const unsigned int /*version*/) {
    ar &BOOST_SERIALIZATION_NVP(reweight_);
  }
#endif
};

/** "Null" robust loss function, equivalent to a Gaussian pdf noise model, or
 *  plain least-squares (non-robust).
 *
 *  This model has no additional parameters.
 *
 * - Loss       \f$ \rho(x)          = 0.5 x² \f$
 * - Derivative \f$ \phi(x)          = x \f$
 * - Weight     \f$ w(x) = \phi(x)/x = 1 \f$
 */
class GTSAM_EXPORT Null : public Base {
 public:
  typedef std::shared_ptr<Null> shared_ptr;

  Null(const ReweightScheme reweight = Block) : Base(reweight) {}
  ~Null() override {}
  double weight(double /*error*/) const override { return 1.0; }
  double loss(double distance) const override { return 0.5 * distance * distance; }
  double graduatedWeight(double distance, double /*error*/) const override { return weight(distance); }
  double graduatedLoss(double distance, double /*error*/) const override { return loss(distance); }
  void print(const std::string &s) const override;
  bool equals(const Base & /*expected*/, double /*tol*/) const override { return true; }
  static shared_ptr Create();

 private:
#if GTSAM_ENABLE_BOOST_SERIALIZATION
  /** Serialization function */
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE &ar, const unsigned int /*version*/) {
    ar &BOOST_SERIALIZATION_BASE_OBJECT_NVP(Base);
  }
#endif
};

/** Implementation of the "Fair" robust error model (Zhang97ivc)
 *
 *  This model has a scalar parameter "c".
 *
 * - Loss       \rho(x) = c² (|x|/c - log(1+|x|/c))
 * - Derivative \phi(x) = x/(1+|x|/c)
 * - Weight     w(x) = \phi(x)/x = 1/(1+|x|/c)
 *
 *  Fair loss has no graduated form.
 */
class GTSAM_EXPORT Fair : public Base {
 protected:
  double c_;

 public:
  typedef std::shared_ptr<Fair> shared_ptr;

  Fair(double c = 1.3998, const ReweightScheme reweight = Block);
  double weight(double distance) const override;
  double loss(double distance) const override;
  double graduatedWeight(double /*error*/, double /*error*/) const override;
  double graduatedLoss(double /*error*/, double /*error*/) const override;
  void print(const std::string &s) const override;
  bool equals(const Base &expected, double tol = 1e-8) const override;
  static shared_ptr Create(double c, const ReweightScheme reweight = Block);
  double modelParameter() const { return c_; }

 private:
#if GTSAM_ENABLE_BOOST_SERIALIZATION
  /** Serialization function */
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE &ar, const unsigned int /*version*/) {
    ar &BOOST_SERIALIZATION_BASE_OBJECT_NVP(Base);
    ar &BOOST_SERIALIZATION_NVP(c_);
  }
#endif
};

/** The "Huber" robust error model (Zhang97ivc).
 *
 *  This model has a scalar parameter "k".
 *
 * - Loss       \rho(x)          = 0.5 x²  if |x|<k, 0.5 k² + k|x-k|  otherwise
 * - Derivative \phi(x)          = x       if |x|<k, k sgn(x)         otherwise
 * - Weight     w(x) = \phi(x)/x = 1       if |x|<k, k/|x|            otherwise
 *
 *  Huber loss is graduated by scaling k with \lambda = 1 / \mu, i.e. by
 *  replacing k with k / sqrt(\mu). \mu = 0 is least squares.
 */
class GTSAM_EXPORT Huber : public Base {
 protected:
  double k_;

 public:
  typedef std::shared_ptr<Huber> shared_ptr;

  Huber(double k = 1.345, const ReweightScheme reweight = Block);
  double weight(double distance) const override;
  double loss(double distance) const override;
  double graduatedWeight(double distance, double mu) const override;
  double graduatedLoss(double distance, double mu) const override;
  void print(const std::string &s) const override;
  bool equals(const Base &expected, double tol = 1e-8) const override;
  static shared_ptr Create(double k, const ReweightScheme reweight = Block);
  double modelParameter() const { return k_; }

  /// @brief Static implementation of Huber Weight
  static double Weight(double distance, double k);
  /// @brief Static implementation of Huber Loss
  static double Loss(double distance, double k);

 private:
#if GTSAM_ENABLE_BOOST_SERIALIZATION
  /** Serialization function */
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE &ar, const unsigned int /*version*/) {
    ar &BOOST_SERIALIZATION_BASE_OBJECT_NVP(Base);
    ar &BOOST_SERIALIZATION_NVP(k_);
  }
#endif
};

/** Implementation of the "Cauchy" robust error model (Lee2013IROS).
 * Contributed by:
 *  Dipl.-Inform. Jan Oberlaender (M.Sc.), FZI Research Center for
 *  Information Technology, Karlsruhe, Germany.
 *  oberlaender@fzi.de
 *  Thanks Jan!
 *
 *  This model has a scalar parameter "k".
 *
 * - Loss       \rho(x) = 0.5 k² log(1+x²/k²)
 * - Derivative \phi(x) = (k²x)/(x²+k²)
 * - Weight     w(x) = \phi(x)/x = k²/(x²+k²)
 *
 *  Cauchy loss is graduated by scaling k with \lambda = 1 / \mu, i.e. by
 *  replacing k² with k²/\mu. \mu = 0 is least squares.
 */
class GTSAM_EXPORT Cauchy : public Base {
 protected:
  double k_, ksquared_;

 public:
  typedef std::shared_ptr<Cauchy> shared_ptr;

  Cauchy(double k = 0.1, const ReweightScheme reweight = Block);
  double weight(double distance) const override;
  double loss(double distance) const override;
  double graduatedWeight(double distance, double mu) const override;
  double graduatedLoss(double distance, double mu) const override;
  void print(const std::string &s) const override;
  bool equals(const Base &expected, double tol = 1e-8) const override;
  static shared_ptr Create(double k, const ReweightScheme reweight = Block);
  double modelParameter() const { return k_; }

  /// @brief Static implementation of Cauchy Weight
  static double Weight(double distance, double ksquared);
  /// @brief Static implementation of Cauchy Loss
  static double Loss(double distance, double ksquared);

 private:
#if GTSAM_ENABLE_BOOST_SERIALIZATION
  /** Serialization function */
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE &ar, const unsigned int /*version*/) {
    ar &BOOST_SERIALIZATION_BASE_OBJECT_NVP(Base);
    ar &BOOST_SERIALIZATION_NVP(k_);
    ar &BOOST_SERIALIZATION_NVP(ksquared_);
  }
#endif
};

/** Implementation of the "Tukey" robust error model (Zhang97ivc).
 *
 *  This model has a scalar parameter "c".
 *
 * - Loss       \f$ \rho(x) = c² (1 - (1-x²/c²)³)/6 \f$  if |x|<c,  c²/6   otherwise
 * - Derivative \f$ \phi(x) = x(1-x²/c²)² if |x|<c \f$,  0   otherwise
 * - Weight     \f$ w(x) = \phi(x)/x = (1-x²/c²)² \f$ if |x|<c,  0   otherwise
 *
 *  Tukey loss is graduated by scaling c with \lambda = 1 / \mu, i.e. by
 *  replacing c² with c²/\mu. \mu = 0 is least squares.
 */
class GTSAM_EXPORT Tukey : public Base {
 protected:
  double c_, csquared_;

 public:
  typedef std::shared_ptr<Tukey> shared_ptr;

  Tukey(double c = 4.6851, const ReweightScheme reweight = Block);
  double weight(double distance) const override;
  double loss(double distance) const override;
  double graduatedWeight(double distance, double mu) const override;
  double graduatedLoss(double distance, double mu) const override;
  void print(const std::string &s) const override;
  bool equals(const Base &expected, double tol = 1e-8) const override;
  static shared_ptr Create(double k, const ReweightScheme reweight = Block);
  double modelParameter() const { return c_; }

  /// @brief Static implementation of Tukey Weight
  static double Weight(double distance, double c, double csquared);
  /// @brief Static implementation of Tukey Loss
  static double Loss(double distance, double c, double csquared);


 private:
#if GTSAM_ENABLE_BOOST_SERIALIZATION
  /** Serialization function */
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE &ar, const unsigned int /*version*/) {
    ar &BOOST_SERIALIZATION_BASE_OBJECT_NVP(Base);
    ar &BOOST_SERIALIZATION_NVP(c_);
  }
#endif
};

/** Implementation of the "Welsch" robust error model (Zhang97ivc).
 *
 *  This model has a scalar parameter "c".
 *
 * - Loss       \f$ \rho(x) = -0.5 c² (exp(-x²/c²) - 1) \f$
 * - Derivative \f$ \phi(x) = x exp(-x²/c²) \f$
 * - Weight     \f$ w(x) = \phi(x)/x = exp(-x²/c²) \f$
 *
 *  Welsch loss is graduated by scaling c with \lambda = 1 / \mu, i.e. by
 *  replacing c² with c²/\mu. \mu = 0 is least squares.
 */
class GTSAM_EXPORT Welsch : public Base {
 protected:
  double c_, csquared_;


 public:
  typedef std::shared_ptr<Welsch> shared_ptr;

  Welsch(double c = 2.9846, const ReweightScheme reweight = Block);
  double weight(double distance) const override;
  double loss(double distance) const override;
  double graduatedWeight(double distance, double mu) const override;
  double graduatedLoss(double distance, double mu) const override;
  void print(const std::string &s) const override;
  bool equals(const Base &expected, double tol = 1e-8) const override;
  static shared_ptr Create(double k, const ReweightScheme reweight = Block);
  double modelParameter() const { return c_; }

  /// @brief Static implementation of Welsch Weight
  static double Weight(double distance, double csquared);
  /// @brief Static implementation of Welsch Loss
  static double Loss(double distance, double csquared);

 private:
#if GTSAM_ENABLE_BOOST_SERIALIZATION
  /** Serialization function */
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE &ar, const unsigned int /*version*/) {
    ar &BOOST_SERIALIZATION_BASE_OBJECT_NVP(Base);
    ar &BOOST_SERIALIZATION_NVP(c_);
    ar &BOOST_SERIALIZATION_NVP(csquared_);
  }
#endif
};

/** Implementation of the "Geman-McClure" robust error model (Zhang97ivc).
 *
 * Note that Geman-McClure weight function uses the parameter c == 1.0,
 * but here it's allowed to use different values, so we actually have
 * the generalized Geman-McClure from (Agarwal15phd).
 *
 * - Loss       \rho(x) = 0.5 (c²x²)/(c²+x²)
 * - Derivative \phi(x) = xc⁴/(c²+x²)²
 * - Weight     w(x) = \phi(x)/x = c⁴/(c²+x²)²
 *
 * Geman-McClure loss has two graduated forms
 *
 * STANDARD [1] is the normalized form of the Yang GNC-GM surrogate. It relaxes
 * the shape parameter by the historical \lambda = 1 / \mu, i.e. it evaluates
 * the Geman-McClure loss above with c² replaced by c²/\mu.
 * - Loss       \rho(x,\mu) = 0.5 ((c²/\mu)x²)/((c²/\mu)+x²)
 * - Derivative \phi(x,\mu) = x(c²/\mu)²/((c²/\mu)+x²)²
 * - Weight     w(x,\mu) = \phi(x)/x = (c²/\mu)²/((c²/\mu)+x²)²
 * Both endpoints are exact: \mu = 1 is Geman-McClure, and \mu = 0 is the
 * least-squares limit (\rho = 0.5x², w = 1), evaluated directly rather than by
 * letting c²/\mu diverge.
 *
 * SCALE_INVARIANT [2] is graduated according to the following form.
 * - Loss       \rho(x) = 0.5 (c²x²)/(c²+(x²)^\mu)
 * - Derivative \phi(x) = x(c²(c²+(x²)^\mu * (1-\mu)))/(c²+(x²)^\mu)²
 * - Weight     w(x) = \phi(x)/x = (c²(c²+(x²)^\mu * (1-\mu)))/(c²+(x²)^\mu)²
 */
class GTSAM_EXPORT GemanMcClure : public Base {
 public:
  typedef std::shared_ptr<GemanMcClure> shared_ptr;
  enum GradScheme { STANDARD, SCALE_INVARIANT };

  /// @brief Construct standard GemanMcClure loss
  GemanMcClure(double c = 1.0, const ReweightScheme reweight = Block);
  /// @brief Constructor for a loss with an explicit graduation scheme.
  GemanMcClure(double c, const GradScheme graduation,
               const ReweightScheme reweight = Block);
  ~GemanMcClure() override {}
  double weight(double distance) const override;
  double loss(double distance) const override;
  double graduatedWeight(double distance, double mu) const override;
  double graduatedLoss(double distance, double mu) const override;
  void print(const std::string &s) const override;
  bool equals(const Base &expected, double tol = 1e-8) const override;
  static shared_ptr Create(double k, const ReweightScheme reweight = Block);
  /// @brief Factory for a loss with an explicit graduation scheme.
  static shared_ptr Create(double k, const GradScheme graduation,
                           const ReweightScheme reweight = Block);
  double modelParameter() const { return c_; }
  /// @brief Returns the graduation scheme used by this loss
  GradScheme gradScheme() const { return graduation_; }

  /// @brief A static helper function to compute the Geman-McClure robust weight
  static double Weight(double distance2, double c2);
  /// @brief Static implementation of GemanMcClure Loss
  static double Loss(double distance2, double c2);
  /// @brief Static implementation of GemanMcClure Graduated Weight.
  static double GraduatedWeight(double distance2, double c2, double mu,
                                GradScheme graduation);
  /// @brief Static implementation of GemanMcClure Graduated Loss
  static double GraduatedLoss(double distance2, double c2, double mu,
                              GradScheme graduation);

  /** @brief Static helper to compute shape param (c) using outlier influence.
   * Computes a shape param such that an outlier will have:
   *    d/dx(\rho(x)) <= influenceThreshold
   * @param influenceThreshold - The max influence permited by an outlier.
   *                           Must be in (0, sqrt(chi2 quantile))
   * @param dof - The degrees of freedom of the corresponding measurement
   * @param chiSquaredOutlierThreshold - The threshold for outlier (i.e. 0.95).
   *                              Must be in [0,1]
   * @throws std::invalid_argument if influenceThreshold is outside that range.
   * @returns The shape param
   */
  static double ShapeParameterFromInfluenceThreshold(
      double influenceThreshold, size_t dof, double chiSquaredOutlierThreshold);

 protected:
  double c_;
  double csquared_;
  GradScheme graduation_;

 private:
#if GTSAM_ENABLE_BOOST_SERIALIZATION
  /** Serialization function */
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE &ar, const unsigned int /*version*/) {
    ar &BOOST_SERIALIZATION_BASE_OBJECT_NVP(Base);
    ar &BOOST_SERIALIZATION_NVP(c_);
    ar &BOOST_SERIALIZATION_NVP(csquared_);
    ar &BOOST_SERIALIZATION_NVP(graduation_);
  }
#endif
};

/** Truncated Least Squares (TLS) robust error model.
 *
 *  This model has a scalar parameter "c" (threshold).
 *
 * - Loss       \rho(x) = 0.5 x^2  if |x|<=c, 0.5 c^2 otherwise
 * - Derivative \phi(x) = x        if |x|<=c, 0 otherwise
 * - Weight     w(x) = \phi(x)/x = 1 if |x|<=c, 0 otherwise
 *
 *  TLS has three graduated forms. All are stated in terms of the normalized
 *  \mu; the historical Yang/Peng parameter is \theta = \mu / (1 - \mu), which
 *  GncOptimizer schedules under the loss-independent name \lambda.
 *
 *  STANDARD TLS loss is graduated by relaxing the threshold with
 *  \lambda = 1 / \mu, i.e. by replacing c² with c²/\mu.
 * - Loss  \rho(x,\mu) = 0.5 x^2 if x^2 <= c^2/\mu, 0.5 c^2/\mu otherwise
 * - Weight w(x,\mu)   = 1 if x^2 <= c^2/\mu, 0 otherwise
 *
 *  GNC_LINEAR is the normalized Yang GNC-TLS surrogate
 *      LB = \mu c^2
 *      UB = c^2 / \mu
 * - Loss  \rho(x,\mu) = 0.5 x^2 if x^2 < LB
 *                     = 0.5 (2\sqrt(\mu c^2 x^2) - \mu(c^2 + x^2))/(1-\mu)
 *                       if LB < x^2 < UB
 *                     = 0.5 c^2 if UB < x^2
 * - Weight w(x,\mu)   = 1 if x^2 < LB
 *                     = (\sqrt(\mu c^2 / x^2) - \mu)/(1-\mu) if LB < x^2 < UB
 *                     = 0 if UB < x^2
 *  Yang's \theta -> 0 surrogate is degenerate (every weight vanishes), so the
 *  \mu = 0 endpoint is defined to be the all-inlier least-squares
 *  initialization step: w = 1 and \rho = 0.5 x^2. \mu = 1 is exact TLS.
 *
 * GNC_SUPERLINEAR is the normalized Peng MS-GNC-TLS majorizer
 *      LB = c^2
 *      UB = c^2 / \mu^2
 * - Loss  \rho(x, \mu) = (not formally defined)
 * - Weight w(x, \mu)   = 1 if x^2 < LB
 *                      = (\sqrt(c^2 / x^2) - \mu)/(1-\mu) if LB < x^2 < UB
 *                      = 0 if UB < x^2
 *  Its \mu = 0 endpoint is the non-degenerate convex weight min(1, c/|x|), so
 *  unlike GNC_LINEAR it needs no separate initialization convention.
 *  \mu = 1 is exact TLS.
 */
class GTSAM_EXPORT TruncatedLeastSquares : public Base {
 public:
  typedef std::shared_ptr<TruncatedLeastSquares> shared_ptr;
  enum GradScheme { STANDARD, GNC_LINEAR, GNC_SUPERLINEAR };

  /// @brief Construct standard TLS loss
  TruncatedLeastSquares(double c = 1.0, const ReweightScheme reweight = Block);
  /// @brief Construct with an explicit graduation scheme.
  TruncatedLeastSquares(double c, GradScheme graduation,
                        const ReweightScheme reweight = Block);
  double weight(double distance) const override;
  double loss(double distance) const override;
  double graduatedWeight(double distance, double mu) const override;
  double graduatedLoss(double distance, double mu) const override;
  void print(const std::string& s) const override;
  bool equals(const Base& expected, double tol = 1e-8) const override;
  static shared_ptr Create(double c, const ReweightScheme reweight = Block);
  /// @brief Factory for a loss with an explicit graduation scheme.
  static shared_ptr Create(double c, GradScheme graduation,
                           const ReweightScheme reweight = Block);
  double modelParameter() const { return c_; }
  /// @brief Returns the graduation scheme used by this loss
  GradScheme gradScheme() const { return graduation_; }

  /// @brief Static implementation of TLS Weight
  static double Weight(double distance2, double c2);
  /// @brief Static implementation of TLS Loss.
  static double Loss(double distance2, double c2);

  /// @brief Static implementation of TLS Graduated Weight.
  static double GraduatedWeight(double distance2, double c2, double mu,
                                GradScheme graduation);
  /// @brief Static implementation of TLS Graduated Loss
  static double GraduatedLoss(double distance2, double c2, double mu,
                              GradScheme graduation);

 protected:
  double c_;
  double csquared_;
  GradScheme graduation_;

 private:
#if GTSAM_ENABLE_BOOST_SERIALIZATION
  /** Serialization function */
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE &ar, const unsigned int /*version*/) {
    ar &BOOST_SERIALIZATION_BASE_OBJECT_NVP(Base);
    ar &BOOST_SERIALIZATION_NVP(c_);
    ar &BOOST_SERIALIZATION_NVP(csquared_);
    ar &BOOST_SERIALIZATION_NVP(graduation_);
  }
#endif
};

/** DCS implements the Dynamic Covariance Scaling robust error model
 *  from the paper Robust Map Optimization (Agarwal13icra).
 *
 *  Under the special condition of the parameter c == 1.0 and not
 *  forcing the output weight s <= 1.0, DCS is similar to Geman-McClure.
 *
 *  This model has a scalar parameter "c" (with "units" of squared error).
 *
 * - Loss       \rho(x) = (c²x² + cx⁴)/(x²+c)²   (for any "x")
 * - Derivative \phi(x) = 2c²x/(x²+c)²
 * - Weight     w(x) = \phi(x)/x = 2c²/(x²+c)²  if x²>c,   1  otherwise
 *
 *  DCS loss is graduated by scaling c with \lambda = 1 / \mu, i.e. by
 *  replacing c with c/\mu (c already has units of squared error).
 *  \mu = 0 is least squares.
 */
class GTSAM_EXPORT DCS : public Base {
 public:
  typedef std::shared_ptr<DCS> shared_ptr;

  DCS(double c = 1.0, const ReweightScheme reweight = Block);
  ~DCS() override {}
  double weight(double distance) const override;
  double loss(double distance) const override;
  double graduatedWeight(double distance, double mu) const override;
  double graduatedLoss(double distance, double mu) const override;
  void print(const std::string &s) const override;
  bool equals(const Base &expected, double tol = 1e-8) const override;
  static shared_ptr Create(double k, const ReweightScheme reweight = Block);
  double modelParameter() const { return c_; }

  /// @brief Static implementation of DCS Weight
  static double Weight(double distance, double c);
  /// @brief Static implementation of DCS Loss
  static double Loss(double distance, double c);

 protected:
  double c_;

 private:
#if GTSAM_ENABLE_BOOST_SERIALIZATION
  /** Serialization function */
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE &ar, const unsigned int /*version*/) {
    ar &BOOST_SERIALIZATION_BASE_OBJECT_NVP(Base);
    ar &BOOST_SERIALIZATION_NVP(c_);
  }
#endif
};

/** L2WithDeadZone implements a standard L2 penalty, but with a dead zone of
 *  width 2*k, centered at the origin. The resulting penalty within the dead
 *  zone is always zero, and grows quadratically outside the dead zone. In this
 *  sense, the L2WithDeadZone penalty is "robust to inliers", rather than being
 *  robust to outliers. This penalty can be used to create barrier functions in
 *  a general way.
 *
 *  This model has a scalar parameter "k".
 *
 * - Loss       \f$ \rho(x) = 0 \f$ if |x|<k,    0.5(k-|x|)² otherwise
 * - Derivative \f$ \phi(x) = 0 \f$ if |x|<k, (-k+x) if x>k,  (k+x) if x<-k
 * - Weight     \f$ w(x) = \phi(x)/x = 0 \f$ if |x|<k, (-k+x)/x if x>k,  (k+x)/x if x<-k
 *
 *  L2WithDeadZone loss has no graduated form.
 */
class GTSAM_EXPORT L2WithDeadZone : public Base {
 protected:
  double k_;

 public:
  typedef std::shared_ptr<L2WithDeadZone> shared_ptr;

  L2WithDeadZone(double k = 1.0, const ReweightScheme reweight = Block);
  double weight(double distance) const override;
  double loss(double distance) const override;
  double graduatedWeight(double /*error*/, double /*error*/) const override;
  double graduatedLoss(double /*error*/, double /*error*/) const override;
  void print(const std::string &s) const override;
  bool equals(const Base &expected, double tol = 1e-8) const override;
  static shared_ptr Create(double k, const ReweightScheme reweight = Block);
  double modelParameter() const { return k_; }

 private:
#if GTSAM_ENABLE_BOOST_SERIALIZATION
  /** Serialization function */
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE &ar, const unsigned int /*version*/) {
    ar &BOOST_SERIALIZATION_BASE_OBJECT_NVP(Base);
    ar &BOOST_SERIALIZATION_NVP(k_);
  }
#endif
};

/** Implementation of the "AsymmetricTukey" robust error model.
 *
 *  This model has a scalar parameter "c".
 *
 * - Following are all for one side, the other is standard L2
 * - Loss       \rho(x) = c² (1 - (1-x²/c²)³)/6  if |x|<c,  c²/6   otherwise
 * - Derivative \phi(x) = x(1-x²/c²)² if |x|<c,  0   otherwise
 * - Weight     w(x) = \phi(x)/x = (1-x²/c²)² if |x|<c,  0   otherwise
 *
 *  AsymmetricTukey loss has no graduated form.
 */
class GTSAM_EXPORT AsymmetricTukey : public Base {
 protected:
  double c_, csquared_;

 public:
  typedef std::shared_ptr<AsymmetricTukey> shared_ptr;

  AsymmetricTukey(double c = 4.6851, const ReweightScheme reweight = Block);
  double weight(double distance) const override;
  double loss(double distance) const override;
  double graduatedWeight(double /*error*/, double /*error*/) const override;
  double graduatedLoss(double /*error*/, double /*error*/) const override;
  void print(const std::string &s) const override;
  bool equals(const Base &expected, double tol = 1e-8) const override;
  static shared_ptr Create(double k, const ReweightScheme reweight = Block);
  double modelParameter() const { return c_; }

 private:
#if GTSAM_ENABLE_BOOST_SERIALIZATION
  /** Serialization function */
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE &ar, const unsigned int /*version*/) {
    ar &BOOST_SERIALIZATION_BASE_OBJECT_NVP(Base);
    ar &BOOST_SERIALIZATION_NVP(c_);
  }
#endif
};

/** Implementation of the "AsymmetricCauchy" robust error model.
 *
 *  This model has a scalar parameter "k".
 *
 * - Following are all for one side, the other is standard L2
 * - Loss       \rho(x) = 0.5 k² log(1+x²/k²)
 * - Derivative \phi(x) = (k²x)/(x²+k²)
 * - Weight     w(x) = \phi(x)/x = k²/(x²+k²)
 *
 *  AsymmetricCauchy loss has no graduated form.
 */
class GTSAM_EXPORT AsymmetricCauchy : public Base {
 protected:
  double k_, ksquared_;

 public:
  typedef std::shared_ptr<AsymmetricCauchy> shared_ptr;

  AsymmetricCauchy(double k = 0.1, const ReweightScheme reweight = Block);
  double weight(double distance) const override;
  double loss(double distance) const override;
  double graduatedWeight(double /*error*/, double /*error*/) const override;
  double graduatedLoss(double /*error*/, double /*error*/) const override;
  void print(const std::string &s) const override;
  bool equals(const Base &expected, double tol = 1e-8) const override;
  static shared_ptr Create(double k, const ReweightScheme reweight = Block);
  double modelParameter() const { return k_; }

 private:
#if GTSAM_ENABLE_BOOST_SERIALIZATION
  /** Serialization function */
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE &ar, const unsigned int /*version*/) {
    ar &BOOST_SERIALIZATION_BASE_OBJECT_NVP(Base);
    ar &BOOST_SERIALIZATION_NVP(k_);
    ar &BOOST_SERIALIZATION_NVP(ksquared_);
  }
#endif
};

// Type alias for the custom loss and weight functions
using CustomLossFunction = std::function<double(double)>;
using CustomWeightFunction = std::function<double(double)>;
using CustomGraduatedLossFunction =
    std::optional<std::function<double(double, double)>>;
using CustomGraduatedWeightFunction =
    std::optional<std::function<double(double, double)>>;

/** Implementation of the "Custom" robust error model.
 *
 *  This model just takes two functions as input, one for the loss and one for the weight.
 *
 *  Optionally this model can also define graduated loss functions
 */
class GTSAM_EXPORT Custom : public Base {
 protected:
  std::function<double(double)> weight_, loss_;
  std::optional<std::function<double(double, double)>> gradWeight_, gradLoss_;
  std::string name_;

 public:
  typedef std::shared_ptr<Custom> shared_ptr;

  Custom(CustomWeightFunction weight, CustomLossFunction loss,
         CustomGraduatedWeightFunction gradWeight = std::nullopt,
         CustomGraduatedLossFunction gradLoss = std::nullopt,
         const ReweightScheme reweight = Block, std::string name = "Custom");
  Custom(CustomWeightFunction weight, CustomLossFunction loss,
         const ReweightScheme reweight, std::string name = "Custom");
  double weight(double distance) const override;
  double loss(double distance) const override;
  double graduatedWeight(double distance, double mu) const override;
  double graduatedLoss(double distance, double mu) const override;
  void print(const std::string& s) const override;
  bool equals(const Base& expected, double tol = 1e-8) const override;
  static shared_ptr Create(
      std::function<double(double)> weight, std::function<double(double)> loss,
      CustomGraduatedWeightFunction gradWeight = std::nullopt,
      CustomGraduatedLossFunction gradLoss = std::nullopt,
      const ReweightScheme reweight = Block,
      const std::string &name = "Custom");
  static shared_ptr Create(std::function<double(double)> weight,
                           std::function<double(double)> loss,
                           const ReweightScheme reweight,
                           const std::string& name = "Custom");
  inline std::string& name() { return name_; }

  inline std::function<double(double)>& weightFunction() { return weight_; }
  inline std::function<double(double)>& lossFunction() { return loss_; }

  // Default constructor for serialization
  inline Custom() = default;

 private:
#if GTSAM_ENABLE_BOOST_SERIALIZATION
  /** Serialization function */
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE &ar, const unsigned int /*version*/) {
    ar &BOOST_SERIALIZATION_BASE_OBJECT_NVP(Base);
    ar &BOOST_SERIALIZATION_NVP(name_);
  }
#endif
};

}  // namespace mEstimator
}  // namespace noiseModel
}  // namespace gtsam
