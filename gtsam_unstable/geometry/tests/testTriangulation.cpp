/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * testTriangulation.cpp
 *
 *  Created on: July 30th, 2013
 *      Author: cbeall3
 */

#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/geometry/SimpleCamera.h>
#include <gtsam/base/numericalDerivative.h>
#include <boost/optional.hpp>

namespace gtsam {

/**
 * Non-linear factor for a constraint derived from a 2D measurement.
 * The calibration and pose are assumed known.
 * i.e. the main building block for visual SLAM.
 * TODO: refactor to avoid large copy/paste
 * TODO: even better, make GTSAM designate certain variables as constant
 * @addtogroup SLAM
 */
template<class POSE, class LANDMARK, class CALIBRATION = Cal3_S2>
class TriangulationFactor: public NoiseModelFactor1<LANDMARK> {
protected:

  // Keep a copy of measurement and calibration for I/O
  const Pose3 pose_; ///< Pose where this landmark was seen
  const Point2 measured_; ///< 2D measurement
  boost::shared_ptr<CALIBRATION> K_; ///< shared pointer to calibration object
  boost::optional<POSE> body_P_sensor_; ///< The pose of the sensor in the body frame

  // verbosity handling for Cheirality Exceptions
  const bool throwCheirality_; ///< If true, rethrows Cheirality exceptions (default: false)
  const bool verboseCheirality_; ///< If true, prints text for Cheirality exceptions (default: false)

public:

  /// shorthand for base class type
  typedef NoiseModelFactor1<LANDMARK> Base;

  /// shorthand for this class
  typedef TriangulationFactor<POSE, LANDMARK, CALIBRATION> This;

  /// shorthand for a smart pointer to a factor
  typedef boost::shared_ptr<This> shared_ptr;

  /// Default constructor
  TriangulationFactor() :
      throwCheirality_(false), verboseCheirality_(false) {
  }

  /**
   * Constructor
   * TODO: Mark argument order standard (keys, measurement, parameters)
   * @param measured is the 2 dimensional location of point in image (the measurement)
   * @param model is the standard deviation
   * @param poseKey is the index of the camera
   * @param pointKey is the index of the landmark
   * @param K shared pointer to the constant calibration
   * @param body_P_sensor is the transform from body to sensor frame (default identity)
   */
  TriangulationFactor(const Pose3& pose, const Point2& measured,
      const SharedNoiseModel& model, Key pointKey,
      const boost::shared_ptr<CALIBRATION>& K,
      boost::optional<POSE> body_P_sensor = boost::none) :
      Base(model, pointKey), pose_(pose), measured_(measured), K_(K), body_P_sensor_(
          body_P_sensor), throwCheirality_(false), verboseCheirality_(false) {
  }

  /**
   * Constructor with exception-handling flags
   * TODO: Mark argument order standard (keys, measurement, parameters)
   * @param measured is the 2 dimensional location of point in image (the measurement)
   * @param model is the standard deviation
   * @param poseKey is the index of the camera
   * @param pointKey is the index of the landmark
   * @param K shared pointer to the constant calibration
   * @param throwCheirality determines whether Cheirality exceptions are rethrown
   * @param verboseCheirality determines whether exceptions are printed for Cheirality
   * @param body_P_sensor is the transform from body to sensor frame  (default identity)
   */
  TriangulationFactor(const Pose3& pose, const Point2& measured,
      const SharedNoiseModel& model, Key poseKey, Key pointKey,
      const boost::shared_ptr<CALIBRATION>& K, bool throwCheirality,
      bool verboseCheirality, boost::optional<POSE> body_P_sensor = boost::none) :
      Base(model, pointKey), pose_(pose), measured_(measured), K_(K), body_P_sensor_(
          body_P_sensor), throwCheirality_(throwCheirality), verboseCheirality_(
          verboseCheirality) {
  }

  /** Virtual destructor */
  virtual ~TriangulationFactor() {
  }

  /// @return a deep copy of this factor
  virtual gtsam::NonlinearFactor::shared_ptr clone() const {
    return boost::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new This(*this)));
  }

  /**
   * print
   * @param s optional string naming the factor
   * @param keyFormatter optional formatter useful for printing Symbols
   */
  void print(const std::string& s = "", const KeyFormatter& keyFormatter =
      DefaultKeyFormatter) const {
    std::cout << s << "TriangulationFactor, z = ";
    measured_.print();
    if (this->body_P_sensor_)
      this->body_P_sensor_->print("  sensor pose in body frame: ");
    Base::print("", keyFormatter);
  }

  /// equals
  virtual bool equals(const NonlinearFactor& p, double tol = 1e-9) const {
    const This *e = dynamic_cast<const This*>(&p);
    return e && Base::equals(p, tol)
        && this->measured_.equals(e->measured_, tol)
        && this->K_->equals(*e->K_, tol)
        && ((!body_P_sensor_ && !e->body_P_sensor_)
            || (body_P_sensor_ && e->body_P_sensor_
                && body_P_sensor_->equals(*e->body_P_sensor_)));
  }

  /// Evaluate error h(x)-z and optionally derivatives
  Vector evaluateError(const Point3& point, boost::optional<Matrix&> H2 =
      boost::none) const {
    try {
      if (body_P_sensor_) {
        PinholeCamera<CALIBRATION> camera(pose_.compose(*body_P_sensor_), *K_);
        Point2 reprojectionError(
            camera.project(point, boost::none, H2) - measured_);
        return reprojectionError.vector();
      } else {
        PinholeCamera<CALIBRATION> camera(pose_, *K_);
        Point2 reprojectionError(
            camera.project(point, boost::none, H2) - measured_);
        return reprojectionError.vector();
      }
    } catch (CheiralityException& e) {
      if (H2)
        *H2 = zeros(2, 3);
      if (verboseCheirality_)
        std::cout << e.what() << ": Landmark "
            << DefaultKeyFormatter(this->key()) << " moved behind camera"
            << std::endl;
      if (throwCheirality_)
        throw e;
    }
    return ones(2) * 2.0 * K_->fx();
  }

  /** return the measurement */
  const Point2& measured() const {
    return measured_;
  }

  /** return the calibration object */
  inline const boost::shared_ptr<CALIBRATION> calibration() const {
    return K_;
  }

  /** return verbosity */
  inline bool verboseCheirality() const {
    return verboseCheirality_;
  }

  /** return flag for throwing cheirality exceptions */
  inline bool throwCheirality() const {
    return throwCheirality_;
  }

private:

  /// Serialization function
  friend class boost::serialization::access;
  template<class ARCHIVE>
  void serialize(ARCHIVE & ar, const unsigned int version) {
    ar & BOOST_SERIALIZATION_BASE_OBJECT_NVP(Base);
    ar & BOOST_SERIALIZATION_NVP(measured_);
    ar & BOOST_SERIALIZATION_NVP(K_);
    ar & BOOST_SERIALIZATION_NVP(body_P_sensor_);
    ar & BOOST_SERIALIZATION_NVP(throwCheirality_);
    ar & BOOST_SERIALIZATION_NVP(verboseCheirality_);
  }
};
} // \ namespace gtsam

#include <gtsam_unstable/geometry/triangulation.h>
#include <gtsam/geometry/Cal3Bundler.h>
#include <CppUnitLite/TestHarness.h>

#include <boost/assign.hpp>
#include <boost/assign/std/vector.hpp>

using namespace std;
using namespace gtsam;
using namespace boost::assign;

// Some common constants

static const boost::shared_ptr<Cal3_S2> sharedCal = //
    boost::make_shared<Cal3_S2>(1500, 1200, 0, 640, 480);

// Looking along X-axis, 1 meter above ground plane (x-y)
static const Rot3 upright = Rot3::ypr(-M_PI / 2, 0., -M_PI / 2);
static const Pose3 pose1 = Pose3(upright, gtsam::Point3(0, 0, 1));
PinholeCamera<Cal3_S2> camera1(pose1, *sharedCal);

// create second camera 1 meter to the right of first camera
static const Pose3 pose2 = pose1 * Pose3(Rot3(), Point3(1, 0, 0));
PinholeCamera<Cal3_S2> camera2(pose2, *sharedCal);

// landmark ~5 meters infront of camera
static const Point3 landmark(5, 0.5, 1.2);

// 1. Project two landmarks into two cameras and triangulate
Point2 z1 = camera1.project(landmark);
Point2 z2 = camera2.project(landmark);

//******************************************************************************
TEST( triangulation, twoPoses) {

  vector<Pose3> poses;
  vector<Point2> measurements;

  poses += pose1, pose2;
  measurements += z1, z2;

  bool optimize = true;
  double rank_tol = 1e-9;

  boost::optional<Point3> triangulated_landmark = triangulatePoint3(poses,
      sharedCal, measurements, rank_tol, optimize);
  EXPECT(assert_equal(landmark, *triangulated_landmark, 1e-2));

  // 2. Add some noise and try again: result should be ~ (4.995, 0.499167, 1.19814)
  measurements.at(0) += Point2(0.1, 0.5);
  measurements.at(1) += Point2(-0.2, 0.3);

  boost::optional<Point3> triangulated_landmark_noise = triangulatePoint3(poses,
      sharedCal, measurements, rank_tol, optimize);
  EXPECT(assert_equal(landmark, *triangulated_landmark_noise, 1e-2));
}

//******************************************************************************

TEST( triangulation, twoPosesBundler) {

  boost::shared_ptr<Cal3Bundler> bundlerCal = //
      boost::make_shared<Cal3Bundler>(1500, 0, 0, 640, 480);
  PinholeCamera<Cal3Bundler> camera1(pose1, *bundlerCal);
  PinholeCamera<Cal3Bundler> camera2(pose2, *bundlerCal);

  // 1. Project two landmarks into two cameras and triangulate
  Point2 z1 = camera1.project(landmark);
  Point2 z2 = camera2.project(landmark);

  vector<Pose3> poses;
  vector<Point2> measurements;

  poses += pose1, pose2;
  measurements += z1, z2;

  bool optimize = true;
  double rank_tol = 1e-9;

  boost::optional<Point3> triangulated_landmark = triangulatePoint3(poses,
      bundlerCal, measurements, rank_tol, optimize);
  EXPECT(assert_equal(landmark, *triangulated_landmark, 1e-2));

  // 2. Add some noise and try again: result should be ~ (4.995, 0.499167, 1.19814)
  measurements.at(0) += Point2(0.1, 0.5);
  measurements.at(1) += Point2(-0.2, 0.3);

  boost::optional<Point3> triangulated_landmark_noise = triangulatePoint3(poses,
      bundlerCal, measurements, rank_tol, optimize);
  EXPECT(assert_equal(landmark, *triangulated_landmark_noise, 1e-2));
}

//******************************************************************************
TEST( triangulation, fourPoses) {
  vector<Pose3> poses;
  vector<Point2> measurements;

  poses += pose1, pose2;
  measurements += z1, z2;

  boost::optional<Point3> triangulated_landmark = triangulatePoint3(poses,
      sharedCal, measurements);
  EXPECT(assert_equal(landmark, *triangulated_landmark, 1e-2));

  // 2. Add some noise and try again: result should be ~ (4.995, 0.499167, 1.19814)
  measurements.at(0) += Point2(0.1, 0.5);
  measurements.at(1) += Point2(-0.2, 0.3);

  boost::optional<Point3> triangulated_landmark_noise = //
      triangulatePoint3(poses, sharedCal, measurements);
  EXPECT(assert_equal(landmark, *triangulated_landmark_noise, 1e-2));

  // 3. Add a slightly rotated third camera above, again with measurement noise
  Pose3 pose3 = pose1 * Pose3(Rot3::ypr(0.1, 0.2, 0.1), Point3(0.1, -2, -.1));
  SimpleCamera camera3(pose3, *sharedCal);
  Point2 z3 = camera3.project(landmark);

  poses += pose3;
  measurements += z3 + Point2(0.1, -0.1);

  boost::optional<Point3> triangulated_3cameras = //
      triangulatePoint3(poses, sharedCal, measurements);
  EXPECT(assert_equal(landmark, *triangulated_3cameras, 1e-2));

  // Again with nonlinear optimization
  boost::optional<Point3> triangulated_3cameras_opt = triangulatePoint3(poses,
      sharedCal, measurements, 1e-9, true);
  EXPECT(assert_equal(landmark, *triangulated_3cameras_opt, 1e-2));

  // 4. Test failure: Add a 4th camera facing the wrong way
  Pose3 pose4 = Pose3(Rot3::ypr(M_PI / 2, 0., -M_PI / 2), Point3(0, 0, 1));
  SimpleCamera camera4(pose4, *sharedCal);

#ifdef GTSAM_THROW_CHEIRALITY_EXCEPTION
  CHECK_EXCEPTION(camera4.project(landmark);, CheiralityException);

  poses += pose4;
  measurements += Point2(400, 400);

  CHECK_EXCEPTION(triangulatePoint3(poses, sharedCal, measurements),
      TriangulationCheiralityException);
#endif
}

//******************************************************************************
TEST( triangulation, fourPoses_distinct_Ks) {
  Cal3_S2 K1(1500, 1200, 0, 640, 480);
  // create first camera. Looking along X-axis, 1 meter above ground plane (x-y)
  SimpleCamera camera1(pose1, K1);

  // create second camera 1 meter to the right of first camera
  Cal3_S2 K2(1600, 1300, 0, 650, 440);
  SimpleCamera camera2(pose2, K2);

  // 1. Project two landmarks into two cameras and triangulate
  Point2 z1 = camera1.project(landmark);
  Point2 z2 = camera2.project(landmark);

  vector<SimpleCamera> cameras;
  vector<Point2> measurements;

  cameras += camera1, camera2;
  measurements += z1, z2;

  boost::optional<Point3> triangulated_landmark = //
      triangulatePoint3(cameras, measurements);
  EXPECT(assert_equal(landmark, *triangulated_landmark, 1e-2));

  // 2. Add some noise and try again: result should be ~ (4.995, 0.499167, 1.19814)
  measurements.at(0) += Point2(0.1, 0.5);
  measurements.at(1) += Point2(-0.2, 0.3);

  boost::optional<Point3> triangulated_landmark_noise = //
      triangulatePoint3(cameras, measurements);
  EXPECT(assert_equal(landmark, *triangulated_landmark_noise, 1e-2));

  // 3. Add a slightly rotated third camera above, again with measurement noise
  Pose3 pose3 = pose1 * Pose3(Rot3::ypr(0.1, 0.2, 0.1), Point3(0.1, -2, -.1));
  Cal3_S2 K3(700, 500, 0, 640, 480);
  SimpleCamera camera3(pose3, K3);
  Point2 z3 = camera3.project(landmark);

  cameras += camera3;
  measurements += z3 + Point2(0.1, -0.1);

  boost::optional<Point3> triangulated_3cameras = //
      triangulatePoint3(cameras, measurements);
  EXPECT(assert_equal(landmark, *triangulated_3cameras, 1e-2));

  // Again with nonlinear optimization
  boost::optional<Point3> triangulated_3cameras_opt = triangulatePoint3(cameras,
      measurements, 1e-9, true);
  EXPECT(assert_equal(landmark, *triangulated_3cameras_opt, 1e-2));

  // 4. Test failure: Add a 4th camera facing the wrong way
  Pose3 pose4 = Pose3(Rot3::ypr(M_PI / 2, 0., -M_PI / 2), Point3(0, 0, 1));
  Cal3_S2 K4(700, 500, 0, 640, 480);
  SimpleCamera camera4(pose4, K4);

#ifdef GTSAM_THROW_CHEIRALITY_EXCEPTION
  CHECK_EXCEPTION(camera4.project(landmark);, CheiralityException);

  cameras += camera4;
  measurements += Point2(400, 400);
  CHECK_EXCEPTION(triangulatePoint3(cameras, measurements),
      TriangulationCheiralityException);
#endif
}

//******************************************************************************
TEST( triangulation, twoIdenticalPoses) {
  // create first camera. Looking along X-axis, 1 meter above ground plane (x-y)
  SimpleCamera camera1(pose1, *sharedCal);

  // 1. Project two landmarks into two cameras and triangulate
  Point2 z1 = camera1.project(landmark);

  vector<Pose3> poses;
  vector<Point2> measurements;

  poses += pose1, pose1;
  measurements += z1, z1;

  CHECK_EXCEPTION(triangulatePoint3(poses, sharedCal, measurements),
      TriangulationUnderconstrainedException);
}

//******************************************************************************
/*
 TEST( triangulation, onePose) {
 // we expect this test to fail with a TriangulationUnderconstrainedException
 // because there's only one camera observation

 Cal3_S2 *sharedCal(1500, 1200, 0, 640, 480);

 vector<Pose3> poses;
 vector<Point2> measurements;

 poses += Pose3();
 measurements += Point2();

 CHECK_EXCEPTION(triangulatePoint3(poses, measurements, *sharedCal),
 TriangulationUnderconstrainedException);
 }
 */

//******************************************************************************
TEST( triangulation, TriangulationFactor ) {
  // Create the factor with a measurement that is 3 pixels off in x
  Key pointKey(1);
  SharedNoiseModel model;
  typedef TriangulationFactor<Pose3, Point3> Factor;
  Factor factor(pose1, z1, model, pointKey, sharedCal);

  // Use the factor to calculate the Jacobians
  Matrix HActual;
  factor.evaluateError(landmark, HActual);

//  Matrix expectedH1 = numericalDerivative11<Pose3>(
//      boost::bind(&EssentialMatrixConstraint::evaluateError, &factor, _1, pose2,
//          boost::none, boost::none), pose1);
  // The expected Jacobian
  Matrix HExpected = numericalDerivative11<Point3>(
      boost::bind(&Factor::evaluateError, &factor, _1, boost::none), landmark);

  // Verify the Jacobians are correct
  CHECK(assert_equal(HExpected, HActual, 1e-3));
}

//******************************************************************************
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
//******************************************************************************
