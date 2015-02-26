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
template<class CALIBRATION>
class SmartProjectionCameraFactor: public SmartProjectionFactor<
    PinholeCamera<CALIBRATION> > {

private:
  typedef PinholeCamera<CALIBRATION> Camera;
  typedef SmartProjectionFactor<Camera> Base;
  typedef SmartProjectionCameraFactor<CALIBRATION> This;

protected:

  static const int Dim = traits<Camera>::dimension; ///< CAMERA dimension

  bool isImplicit_;

public:

  /// shorthand for a smart pointer to a factor
  typedef boost::shared_ptr<This> shared_ptr;

  // A set of cameras
  typedef CameraSet<Camera> Cameras;

  /**
   * Constructor
   * @param rankTol tolerance used to check if point triangulation is degenerate
   * @param linThreshold threshold on relative pose changes used to decide whether to relinearize (selective relinearization)
   * @param manageDegeneracy is true, in presence of degenerate triangulation, the factor is converted to a rotation-only constraint,
   * otherwise the factor is simply neglected
   * @param enableEPI if set to true linear triangulation is refined with embedded LM iterations
   * @param isImplicit if set to true linearize the factor to an implicit Schur factor
   */
  SmartProjectionCameraFactor(const double rankTol = 1,
      const double linThreshold = -1, const bool manageDegeneracy = false,
      const bool enableEPI = false, const bool isImplicit = false) :
      Base(rankTol, linThreshold, manageDegeneracy, enableEPI), isImplicit_(
          isImplicit) {
    if (linThreshold != -1) {
      std::cout << "SmartProjectionCameraFactor:  linThreshold " << linThreshold
          << std::endl;
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
    return Dim * this->keys_.size(); // 6 for the pose and 3 for the calibration
  }

  /// linearize and adds damping on the points
  boost::shared_ptr<GaussianFactor> linearizeDamped(const Values& values,
      const double lambda=0.0) const {
    if (!isImplicit_)
      return Base::createHessianFactor(Base::cameras(values), lambda);
    else
      return Base::createRegularImplicitSchurFactor(Base::cameras(values));
  }

  /// linearize returns a Hessianfactor that is an approximation of error(p)
  virtual boost::shared_ptr<RegularHessianFactor<Dim> > linearizeToHessian(
      const Values& values, double lambda=0.0) const {
    return Base::createHessianFactor(Base::cameras(values),lambda);
  }

  /// linearize returns a Hessianfactor that is an approximation of error(p)
  virtual boost::shared_ptr<RegularImplicitSchurFactor<Dim> > linearizeToImplicit(
      const Values& values, double lambda=0.0) const {
    return Base::createRegularImplicitSchurFactor(Base::cameras(values),lambda);
  }

  /// linearize returns a Jacobianfactor that is an approximation of error(p)
  virtual boost::shared_ptr<JacobianFactorQ<Dim, 2> > linearizeToJacobian(
      const Values& values, double lambda=0.0) const {
    return Base::createJacobianQFactor(Base::cameras(values),lambda);
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
      return Base::totalReprojectionError(Base::cameras(values));
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
