/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file   SmartProjectionCameraFactor.h
 * @brief  Produces an Hessian factors on CAMERAS (Pose3+CALIBRATION) from monocular measurements of a landmark
 * @author Luca Carlone
 * @author Chris Beall
 * @author Zsolt Kira
 */

#pragma once
#include <gtsam/slam/SmartProjectionFactor.h>

namespace gtsam {

/**
 * @addtogroup SLAM
 */
template<class CALIBRATION, size_t D>
class SmartProjectionCameraFactor: public SmartProjectionFactor<CALIBRATION, D> {
protected:

  bool isImplicit_;

public:

  /// shorthand for base class type
  typedef SmartProjectionFactor<CALIBRATION, D> Base;

  /// shorthand for this class
  typedef SmartProjectionCameraFactor<CALIBRATION, D> This;

  /// shorthand for a smart pointer to a factor
  typedef boost::shared_ptr<This> shared_ptr;

  /**
   * Constructor
   * @param rankTol tolerance used to check if point triangulation is degenerate
   * @param linThreshold threshold on relative pose changes used to decide whether to relinearize (selective relinearization)
   * @param manageDegeneracy is true, in presence of degenerate triangulation, the factor is converted to a rotation-only constraint,
   * otherwise the factor is simply neglected
   * @param enableEPI if set to true linear triangulation is refined with embedded LM iterations
   * @param isImplicit if set to true linearize the factor to an implicit Schur factor
   * @param body_P_sensor is the transform from body to sensor frame (default identity)
   */
  SmartProjectionCameraFactor(const double rankTol = 1,
      const double linThreshold = -1, const bool manageDegeneracy = false,
      const bool enableEPI = false, const bool isImplicit = false,
      boost::optional<Pose3> body_P_sensor = boost::none) :
      Base(rankTol, linThreshold, manageDegeneracy, enableEPI, body_P_sensor), isImplicit_(
          isImplicit) {
    if (linThreshold != -1) {
      std::cout << "SmartProjectionCameraFactor:  linThreshold "
          << linThreshold << std::endl;
    }
  }

  /** Virtual destructor */
  virtual ~SmartProjectionCameraFactor() {
  }

  /**
   * print
   * @param s optional string naming the factor
   * @param keyFormatter optional formatter useful for printing Symbols
   */
  void print(const std::string& s = "", const KeyFormatter& keyFormatter =
      DefaultKeyFormatter) const {
    std::cout << s << "SmartProjectionCameraFactor, z = \n ";
    Base::print("", keyFormatter);
  }

  /// equals
  virtual bool equals(const NonlinearFactor& p, double tol = 1e-9) const {
    const This *e = dynamic_cast<const This*>(&p);

    return e && Base::equals(p, tol);
  }

  /// get the dimension of the factor (number of rows on linearization)
  virtual size_t dim() const {
    return D * this->keys_.size(); // 6 for the pose and 3 for the calibration
  }

  /// Collect all cameras: important that in key order
  typename Base::Cameras cameras(const Values& values) const {
    typename Base::Cameras cameras;
    BOOST_FOREACH(const Key& k, this->keys_)
      cameras.push_back(values.at<typename Base::Camera>(k));
    return cameras;
  }

  /// linearize and adds damping on the points
  boost::shared_ptr<GaussianFactor> linearizeDamped(const Values& values,
      const double lambda=0.0) const {
    if (!isImplicit_)
      return Base::createHessianFactor(cameras(values), lambda);
    else
      return Base::createRegularImplicitSchurFactor(cameras(values));
  }

  /// linearize returns a Hessianfactor that is an approximation of error(p)
  virtual boost::shared_ptr<RegularHessianFactor<D> > linearizeToHessian(
      const Values& values, double lambda=0.0) const {
    return Base::createHessianFactor(cameras(values),lambda);
  }

  /// linearize returns a Hessianfactor that is an approximation of error(p)
  virtual boost::shared_ptr<RegularImplicitSchurFactor<D> > linearizeToImplicit(
      const Values& values, double lambda=0.0) const {
    return Base::createRegularImplicitSchurFactor(cameras(values),lambda);
  }

  /// linearize returns a Jacobianfactor that is an approximation of error(p)
  virtual boost::shared_ptr<JacobianFactorQ<D, 2> > linearizeToJacobian(
      const Values& values, double lambda=0.0) const {
    return Base::createJacobianQFactor(cameras(values),lambda);
  }

  /// linearize returns a Hessianfactor that is an approximation of error(p)
  virtual boost::shared_ptr<GaussianFactor> linearizeWithLambda(
                                                      const Values& values, double lambda) const {
    if (isImplicit_)
      return linearizeToImplicit(values,lambda);
    else
      return linearizeToHessian(values,lambda);
  }
  
  /// linearize returns a Hessianfactor that is an approximation of error(p)
  virtual boost::shared_ptr<GaussianFactor> linearize(
                                                      const Values& values) const {
    return linearizeWithLambda(values,0.0);
  }
  
  /// Calculare total reprojection error
  virtual double error(const Values& values) const {
    if (this->active(values)) {
      return Base::totalReprojectionError(cameras(values));
    } else { // else of active flag
      return 0.0;
    }
  }

private:

  /// Serialization function
  friend class boost::serialization::access;
  template<class ARCHIVE>
  void serialize(ARCHIVE & ar, const unsigned int version) {
    ar & BOOST_SERIALIZATION_BASE_OBJECT_NVP(Base);
  }

};

} // \ namespace gtsam
