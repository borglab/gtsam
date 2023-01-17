
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
 */

#pragma once

#include <gtsam/base/Matrix.h>
#include <gtsam/base/Testable.h>
#include <gtsam/dllexport.h>

#include <boost/serialization/extended_type_info.hpp>
#include <boost/serialization/nvp.hpp>
#include <boost/serialization/version.hpp>
#include <boost/serialization/optional.hpp>
#include <boost/serialization/shared_ptr.hpp>
#include <boost/serialization/singleton.hpp>

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
   * This returns \rho(x) in \ref mEstimator
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

  virtual void print(const std::string &s) const = 0;
  virtual bool equals(const Base &expected, double tol = 1e-8) const = 0;

  double sqrtWeight(double distance) const { return std::sqrt(weight(distance)); }

  /** produce a weight vector according to an error vector and the implemented
   * robust function */
  Vector weight(const Vector &error) const;

  /** square root version of the weight function */
  Vector sqrtWeight(const Vector &error) const {
    return weight(error).cwiseSqrt();
  }

  /** reweight block matrices and a vector according to their weight
   * implementation */
  void reweight(Vector &error) const;
  void reweight(std::vector<Matrix> &A, Vector &error) const;
  void reweight(Matrix &A, Vector &error) const;
  void reweight(Matrix &A1, Matrix &A2, Vector &error) const;
  void reweight(Matrix &A1, Matrix &A2, Matrix &A3, Vector &error) const;

 private:
  /** Serialization function */
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE &ar, const unsigned int /*version*/) {
    ar &BOOST_SERIALIZATION_NVP(reweight_);
  }
};

/** "Null" robust loss function, equivalent to a Gaussian pdf noise model, or
 *  plain least-squares (non-robust).
 *
 *  This model has no additional parameters.
 *
 * - Loss       \rho(x)          = 0.5 x²
 * - Derivative \phi(x)          = x
 * - Weight     w(x) = \phi(x)/x = 1
 */
class GTSAM_EXPORT Null : public Base {
 public:
  typedef std::shared_ptr<Null> shared_ptr;

  Null(const ReweightScheme reweight = Block) : Base(reweight) {}
  ~Null() override {}
  double weight(double /*error*/) const override { return 1.0; }
  double loss(double distance) const override { return 0.5 * distance * distance; }
  void print(const std::string &s) const override;
  bool equals(const Base & /*expected*/, double /*tol*/) const override { return true; }
  static shared_ptr Create();

 private:
  /** Serialization function */
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE &ar, const unsigned int /*version*/) {
    ar &BOOST_SERIALIZATION_BASE_OBJECT_NVP(Base);
  }
};

/** Implementation of the "Fair" robust error model (Zhang97ivc)
 *
 *  This model has a scalar parameter "c".
 *
 * - Loss       \rho(x) = c² (|x|/c - log(1+|x|/c))
 * - Derivative \phi(x) = x/(1+|x|/c)
 * - Weight     w(x) = \phi(x)/x = 1/(1+|x|/c)
 */
class GTSAM_EXPORT Fair : public Base {
 protected:
  double c_;

 public:
  typedef std::shared_ptr<Fair> shared_ptr;

  Fair(double c = 1.3998, const ReweightScheme reweight = Block);
  double weight(double distance) const override;
  double loss(double distance) const override;
  void print(const std::string &s) const override;
  bool equals(const Base &expected, double tol = 1e-8) const override;
  static shared_ptr Create(double c, const ReweightScheme reweight = Block);
  double modelParameter() const { return c_; }

 private:
  /** Serialization function */
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE &ar, const unsigned int /*version*/) {
    ar &BOOST_SERIALIZATION_BASE_OBJECT_NVP(Base);
    ar &BOOST_SERIALIZATION_NVP(c_);
  }
};

/** The "Huber" robust error model (Zhang97ivc).
 *
 *  This model has a scalar parameter "k".
 *
 * - Loss       \rho(x)          = 0.5 x²  if |x|<k, 0.5 k² + k|x-k|  otherwise
 * - Derivative \phi(x)          = x       if |x|<k, k sgn(x)         otherwise
 * - Weight     w(x) = \phi(x)/x = 1       if |x|<k, k/|x|            otherwise
 */
class GTSAM_EXPORT Huber : public Base {
 protected:
  double k_;

 public:
  typedef std::shared_ptr<Huber> shared_ptr;

  Huber(double k = 1.345, const ReweightScheme reweight = Block);
  double weight(double distance) const override;
  double loss(double distance) const override;
  void print(const std::string &s) const override;
  bool equals(const Base &expected, double tol = 1e-8) const override;
  static shared_ptr Create(double k, const ReweightScheme reweight = Block);
  double modelParameter() const { return k_; }

 private:
  /** Serialization function */
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE &ar, const unsigned int /*version*/) {
    ar &BOOST_SERIALIZATION_BASE_OBJECT_NVP(Base);
    ar &BOOST_SERIALIZATION_NVP(k_);
  }
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
 */
class GTSAM_EXPORT Cauchy : public Base {
 protected:
  double k_, ksquared_;

 public:
  typedef std::shared_ptr<Cauchy> shared_ptr;

  Cauchy(double k = 0.1, const ReweightScheme reweight = Block);
  double weight(double distance) const override;
  double loss(double distance) const override;
  void print(const std::string &s) const override;
  bool equals(const Base &expected, double tol = 1e-8) const override;
  static shared_ptr Create(double k, const ReweightScheme reweight = Block);
  double modelParameter() const { return k_; }

 private:
  /** Serialization function */
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE &ar, const unsigned int /*version*/) {
    ar &BOOST_SERIALIZATION_BASE_OBJECT_NVP(Base);
    ar &BOOST_SERIALIZATION_NVP(k_);
    ar &BOOST_SERIALIZATION_NVP(ksquared_);
  }
};

/** Implementation of the "Tukey" robust error model (Zhang97ivc).
 *
 *  This model has a scalar parameter "c".
 *
 * - Loss       \rho(x) = c² (1 - (1-x²/c²)³)/6  if |x|<c,  c²/6   otherwise
 * - Derivative \phi(x) = x(1-x²/c²)² if |x|<c,  0   otherwise
 * - Weight     w(x) = \phi(x)/x = (1-x²/c²)² if |x|<c,  0   otherwise
 */
class GTSAM_EXPORT Tukey : public Base {
 protected:
  double c_, csquared_;

 public:
  typedef std::shared_ptr<Tukey> shared_ptr;

  Tukey(double c = 4.6851, const ReweightScheme reweight = Block);
  double weight(double distance) const override;
  double loss(double distance) const override;
  void print(const std::string &s) const override;
  bool equals(const Base &expected, double tol = 1e-8) const override;
  static shared_ptr Create(double k, const ReweightScheme reweight = Block);
  double modelParameter() const { return c_; }

 private:
  /** Serialization function */
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE &ar, const unsigned int /*version*/) {
    ar &BOOST_SERIALIZATION_BASE_OBJECT_NVP(Base);
    ar &BOOST_SERIALIZATION_NVP(c_);
  }
};

/** Implementation of the "Welsch" robust error model (Zhang97ivc).
 *
 *  This model has a scalar parameter "c".
 *
 * - Loss       \rho(x) = -0.5 c² (exp(-x²/c²) - 1)
 * - Derivative \phi(x) = x exp(-x²/c²)
 * - Weight     w(x) = \phi(x)/x = exp(-x²/c²)
 */
class GTSAM_EXPORT Welsch : public Base {
 protected:
  double c_, csquared_;

 public:
  typedef std::shared_ptr<Welsch> shared_ptr;

  Welsch(double c = 2.9846, const ReweightScheme reweight = Block);
  double weight(double distance) const override;
  double loss(double distance) const override;
  void print(const std::string &s) const override;
  bool equals(const Base &expected, double tol = 1e-8) const override;
  static shared_ptr Create(double k, const ReweightScheme reweight = Block);
  double modelParameter() const { return c_; }

 private:
  /** Serialization function */
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE &ar, const unsigned int /*version*/) {
    ar &BOOST_SERIALIZATION_BASE_OBJECT_NVP(Base);
    ar &BOOST_SERIALIZATION_NVP(c_);
    ar &BOOST_SERIALIZATION_NVP(csquared_);
  }
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
 */
class GTSAM_EXPORT GemanMcClure : public Base {
 public:
  typedef std::shared_ptr<GemanMcClure> shared_ptr;

  GemanMcClure(double c = 1.0, const ReweightScheme reweight = Block);
  ~GemanMcClure() override {}
  double weight(double distance) const override;
  double loss(double distance) const override;
  void print(const std::string &s) const override;
  bool equals(const Base &expected, double tol = 1e-8) const override;
  static shared_ptr Create(double k, const ReweightScheme reweight = Block);
  double modelParameter() const { return c_; }

 protected:
  double c_;

 private:
  /** Serialization function */
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE &ar, const unsigned int /*version*/) {
    ar &BOOST_SERIALIZATION_BASE_OBJECT_NVP(Base);
    ar &BOOST_SERIALIZATION_NVP(c_);
  }
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
 */
class GTSAM_EXPORT DCS : public Base {
 public:
  typedef std::shared_ptr<DCS> shared_ptr;

  DCS(double c = 1.0, const ReweightScheme reweight = Block);
  ~DCS() override {}
  double weight(double distance) const override;
  double loss(double distance) const override;
  void print(const std::string &s) const override;
  bool equals(const Base &expected, double tol = 1e-8) const override;
  static shared_ptr Create(double k, const ReweightScheme reweight = Block);
  double modelParameter() const { return c_; }

 protected:
  double c_;

 private:
  /** Serialization function */
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE &ar, const unsigned int /*version*/) {
    ar &BOOST_SERIALIZATION_BASE_OBJECT_NVP(Base);
    ar &BOOST_SERIALIZATION_NVP(c_);
  }
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
 * - Loss       \rho(x) = 0 if |x|<k,    0.5(k-|x|)² otherwise
 * - Derivative \phi(x) = 0 if |x|<k, (-k+x) if x>k,  (k+x) if x<-k
 * - Weight     w(x) = \phi(x)/x = 0 if |x|<k, (-k+x)/x if x>k,  (k+x)/x if x<-k
 */
class GTSAM_EXPORT L2WithDeadZone : public Base {
 protected:
  double k_;

 public:
  typedef std::shared_ptr<L2WithDeadZone> shared_ptr;

  L2WithDeadZone(double k = 1.0, const ReweightScheme reweight = Block);
  double weight(double distance) const override;
  double loss(double distance) const override;
  void print(const std::string &s) const override;
  bool equals(const Base &expected, double tol = 1e-8) const override;
  static shared_ptr Create(double k, const ReweightScheme reweight = Block);
  double modelParameter() const { return k_; }

 private:
  /** Serialization function */
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE &ar, const unsigned int /*version*/) {
    ar &BOOST_SERIALIZATION_BASE_OBJECT_NVP(Base);
    ar &BOOST_SERIALIZATION_NVP(k_);
  }
};

}  // namespace mEstimator
}  // namespace noiseModel
}  // namespace gtsam
