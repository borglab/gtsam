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
template<class CALIBRATION = Cal3_S2>
class TriangulationFactor: public NoiseModelFactor1<Point3> {

public:

  /// Camera type
  typedef PinholeCamera<CALIBRATION> Camera;

protected:

  // Keep a copy of measurement and calibration for I/O
  const Camera camera_; ///< Camera in which this landmark was seen
  const Point2 measured_; ///< 2D measurement

  // verbosity handling for Cheirality Exceptions
  const bool throwCheirality_; ///< If true, rethrows Cheirality exceptions (default: false)
  const bool verboseCheirality_; ///< If true, prints text for Cheirality exceptions (default: false)

public:

  /// shorthand for base class type
  typedef NoiseModelFactor1<Point3> Base;

  /// shorthand for this class
  typedef TriangulationFactor<CALIBRATION> This;

  /// shorthand for a smart pointer to a factor
  typedef boost::shared_ptr<This> shared_ptr;

  /// Default constructor
  TriangulationFactor() :
      throwCheirality_(false), verboseCheirality_(false) {
  }

  /**
   * Constructor with exception-handling flags
   * @param camera is the camera in which unknown landmark is seen
   * @param measured is the 2 dimensional location of point in image (the measurement)
   * @param model is the standard deviation
   * @param pointKey is the index of the landmark
   * @param throwCheirality determines whether Cheirality exceptions are rethrown
   * @param verboseCheirality determines whether exceptions are printed for Cheirality
   */
  TriangulationFactor(const Camera& camera, const Point2& measured,
      const SharedNoiseModel& model, Key pointKey, bool throwCheirality = false,
      bool verboseCheirality = false) :
      Base(model, pointKey), camera_(camera), measured_(measured), throwCheirality_(
          throwCheirality), verboseCheirality_(verboseCheirality) {
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
    std::cout << s << "TriangulationFactor,";
    camera_.print("camera");
    measured_.print("z");
    Base::print("", keyFormatter);
  }

  /// equals
  virtual bool equals(const NonlinearFactor& p, double tol = 1e-9) const {
    const This *e = dynamic_cast<const This*>(&p);
    return e && Base::equals(p, tol) && this->camera_.equals(e->camera_, tol)
        && this->measured_.equals(e->measured_, tol);
  }

  /// Evaluate error h(x)-z and optionally derivatives
  Vector evaluateError(const Point3& point, boost::optional<Matrix&> H2 =
      boost::none) const {
    try {
      Point2 error(camera_.project(point, boost::none, H2) - measured_);
      return error.vector();
    } catch (CheiralityException& e) {
      if (H2)
        *H2 = zeros(2, 3);
      if (verboseCheirality_)
        std::cout << e.what() << ": Landmark "
            << DefaultKeyFormatter(this->key()) << " moved behind camera"
            << std::endl;
      if (throwCheirality_)
        throw e;
      return ones(2) * 2.0 * camera_.calibration().fx();
    }
  }

  /** return the measurement */
  const Point2& measured() const {
    return measured_;
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
    ar & BOOST_SERIALIZATION_NVP(camera_);
    ar & BOOST_SERIALIZATION_NVP(measured_);
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
  typedef TriangulationFactor<> Factor;
  Factor factor(camera1, z1, model, pointKey, sharedCal);

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
