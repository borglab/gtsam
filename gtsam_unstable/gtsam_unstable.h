/**
 * Matlab toolbox interface definition for gtsam_unstable
 */

// specify the classes from gtsam we are using
virtual class gtsam::Value;
virtual class gtsam::LieScalar;
virtual class gtsam::Point2;
virtual class gtsam::Rot2;
virtual class gtsam::Pose2;
virtual class gtsam::Point3;
virtual class gtsam::Rot3;
virtual class gtsam::Pose3;
virtual class gtsam::noiseModel::Base;
virtual class gtsam::NonlinearFactor;
virtual class gtsam::GaussianFactor;
virtual class gtsam::HessianFactor;
virtual class gtsam::JacobianFactor;
class gtsam::GaussianFactorGraph;
class gtsam::NonlinearFactorGraph;
class gtsam::Ordering;
class gtsam::Values;
class gtsam::Key;
class gtsam::VectorValues;
class gtsam::KeyList;
class gtsam::KeySet;
class gtsam::KeyVector;
class gtsam::LevenbergMarquardtParams;
class gtsam::ISAM2Params;

namespace gtsam {

#include <gtsam_unstable/base/Dummy.h>
class Dummy {
  Dummy();
  void print(string s) const;
  unsigned char dummyTwoVar(unsigned char a) const;
};

#include <gtsam_unstable/dynamics/PoseRTV.h>
virtual class PoseRTV : gtsam::Value {
  PoseRTV();
  PoseRTV(Vector rtv);
  PoseRTV(const gtsam::Point3& pt, const gtsam::Rot3& rot, const gtsam::Point3& vel);
  PoseRTV(const gtsam::Rot3& rot, const gtsam::Point3& pt, const gtsam::Point3& vel);
  PoseRTV(const gtsam::Pose3& pose, const gtsam::Point3& vel);
  PoseRTV(const gtsam::Pose3& pose);
  PoseRTV(double roll, double pitch, double yaw, double x, double y, double z, double vx, double vy, double vz);

  // testable
  bool equals(const gtsam::PoseRTV& other, double tol) const;
  void print(string s) const;

  // access
  gtsam::Point3 translation() const;
  gtsam::Rot3 rotation() const;
  gtsam::Point3 velocity() const;
  gtsam::Pose3 pose() const;

  // Vector interfaces
  Vector vector() const;
  Vector translationVec() const;
  Vector velocityVec() const;

  // manifold/Lie
  static size_t Dim();
  size_t dim() const;
  gtsam::PoseRTV retract(Vector v) const;
  Vector localCoordinates(const gtsam::PoseRTV& p) const;
  static gtsam::PoseRTV Expmap(Vector v);
  static Vector Logmap(const gtsam::PoseRTV& p);
  gtsam::PoseRTV inverse() const;
  gtsam::PoseRTV compose(const gtsam::PoseRTV& p) const;
  gtsam::PoseRTV between(const gtsam::PoseRTV& p) const;

  // measurement
  double range(const gtsam::PoseRTV& other) const;
  gtsam::PoseRTV transformed_from(const gtsam::Pose3& trans) const;

  // IMU/dynamics
  gtsam::PoseRTV planarDynamics(double vel_rate, double heading_rate, double max_accel, double dt) const;
  gtsam::PoseRTV flyingDynamics(double pitch_rate, double heading_rate, double lift_control, double dt) const;
  gtsam::PoseRTV generalDynamics(Vector accel, Vector gyro, double dt) const;
  Vector imuPrediction(const gtsam::PoseRTV& x2, double dt) const;
  gtsam::Point3 translationIntegration(const gtsam::PoseRTV& x2, double dt) const;
  Vector translationIntegrationVec(const gtsam::PoseRTV& x2, double dt) const;
};

#include <gtsam_unstable/geometry/Pose3Upright.h>
virtual class Pose3Upright : gtsam::Value {
  Pose3Upright();
  Pose3Upright(const gtsam::Pose3Upright& x);
  Pose3Upright(const gtsam::Rot2& bearing, const gtsam::Point3& t);
  Pose3Upright(double x, double y, double z, double theta);
  Pose3Upright(const gtsam::Pose2& pose, double z);

  void print(string s) const;
  bool equals(const gtsam::Pose3Upright& pose, double tol) const;

  double x() const;
  double y() const;
  double z() const;
  double theta() const;

  gtsam::Point2 translation2() const;
  gtsam::Point3 translation() const;
  gtsam::Rot2 rotation2() const;
  gtsam::Rot3 rotation() const;
  gtsam::Pose2 pose2() const;
  gtsam::Pose3 pose() const;

  size_t dim() const;
  gtsam::Pose3Upright retract(Vector v) const;
  Vector localCoordinates(const gtsam::Pose3Upright& p2) const;

  static gtsam::Pose3Upright identity();
  gtsam::Pose3Upright inverse() const;
  gtsam::Pose3Upright compose(const gtsam::Pose3Upright& p2) const;
  gtsam::Pose3Upright between(const gtsam::Pose3Upright& p2) const;

  static gtsam::Pose3Upright Expmap(Vector xi);
  static Vector Logmap(const gtsam::Pose3Upright& p);
}; // \class Pose3Upright

#include <gtsam_unstable/geometry/BearingS2.h>
virtual class BearingS2 : gtsam::Value {
  BearingS2();
  BearingS2(double azimuth, double elevation);
  BearingS2(const gtsam::Rot2& azimuth, const gtsam::Rot2& elevation);

  gtsam::Rot2 azimuth() const;
  gtsam::Rot2 elevation() const;

  static gtsam::BearingS2 fromDownwardsObservation(const gtsam::Pose3& A, const gtsam::Point3& B);
  static gtsam::BearingS2 fromForwardObservation(const gtsam::Pose3& A, const gtsam::Point3& B);

  void print(string s) const;
  bool equals(const gtsam::BearingS2& x, double tol) const;

  size_t dim() const;
  gtsam::BearingS2 retract(Vector v) const;
  Vector localCoordinates(const gtsam::BearingS2& p2) const;
};

// std::vector<gtsam::Point2>
class Point2Vector
{
  //Capacity
  size_t size() const;
  size_t max_size() const;
  void resize(size_t sz);
  size_t capacity() const;
  bool empty() const;
  void reserve(size_t n);

  //Element access
  gtsam::Point2 at(size_t n) const;
  gtsam::Point2 front() const;
  gtsam::Point2 back() const;

  //Modifiers
  void assign(size_t n, const gtsam::Point2& u);
  void push_back(const gtsam::Point2& x);
  void pop_back();
};

#include <gtsam_unstable/geometry/SimWall2D.h>
class SimWall2D {
  SimWall2D();
  SimWall2D(const gtsam::Point2& a, const gtsam::Point2& b);
  SimWall2D(double ax, double ay, double bx, double by);

  void print(string s) const;
  bool equals(const gtsam::SimWall2D& other, double tol) const;

  gtsam::Point2 a() const;
  gtsam::Point2 b() const;

  gtsam::SimWall2D scale(double s) const;
  double length() const;
  gtsam::Point2 midpoint() const;

  bool intersects(const gtsam::SimWall2D& wall) const;
  //   bool intersects(const gtsam::SimWall2D& wall, boost::optional<gtsam::Point2&> pt=boost::none) const;

  gtsam::Point2 norm() const;
  gtsam::Rot2 reflection(const gtsam::Point2& init, const gtsam::Point2& intersection) const;
};

#include <gtsam_unstable/geometry/SimPolygon2D.h>
class SimPolygon2D {
   static void seedGenerator(size_t seed);
   static gtsam::SimPolygon2D createTriangle(const gtsam::Point2& pA, const gtsam::Point2& pB, const gtsam::Point2& pC);
   static gtsam::SimPolygon2D createRectangle(const gtsam::Point2& p, double height, double width);

   static gtsam::SimPolygon2D randomTriangle(double side_len, double mean_side_len, double sigma_side_len,
       double min_vertex_dist, double min_side_len, const gtsam::SimPolygon2DVector& existing_polys);
   static gtsam::SimPolygon2D randomRectangle(double side_len, double mean_side_len, double sigma_side_len,
       double min_vertex_dist, double min_side_len, const gtsam::SimPolygon2DVector& existing_polys);

   gtsam::Point2 landmark(size_t i) const;
   size_t size() const;
   gtsam::Point2Vector vertices() const;

   bool equals(const gtsam::SimPolygon2D& p, double tol) const;
   void print(string s) const;

   gtsam::SimWall2DVector walls() const;
   bool contains(const gtsam::Point2& p) const;
   bool overlaps(const gtsam::SimPolygon2D& p) const;

   static bool anyContains(const gtsam::Point2& p, const gtsam::SimPolygon2DVector& obstacles);
   static bool anyOverlaps(const gtsam::SimPolygon2D& p, const gtsam::SimPolygon2DVector& obstacles);
   static bool insideBox(double s, const gtsam::Point2& p);
   static bool nearExisting(const gtsam::Point2Vector& S,
       const gtsam::Point2& p, double threshold);

   static gtsam::Point2 randomPoint2(double s);
   static gtsam::Rot2 randomAngle();
   static double randomDistance(double mu, double sigma);
   static double randomDistance(double mu, double sigma, double min_dist);
   static gtsam::Point2 randomBoundedPoint2(double boundary_size,
       const gtsam::Point2Vector& landmarks, double min_landmark_dist);
   static gtsam::Point2 randomBoundedPoint2(double boundary_size,
       const gtsam::Point2Vector& landmarks,
       const gtsam::SimPolygon2DVector& obstacles, double min_landmark_dist);
   static gtsam::Point2 randomBoundedPoint2(double boundary_size,
       const gtsam::SimPolygon2DVector& obstacles);
   static gtsam::Point2 randomBoundedPoint2(
       const gtsam::Point2& LL_corner, const gtsam::Point2& UR_corner,
       const gtsam::Point2Vector& landmarks,
       const gtsam::SimPolygon2DVector& obstacles, double min_landmark_dist);
   static gtsam::Pose2 randomFreePose(double boundary_size, const gtsam::SimPolygon2DVector& obstacles);
 };

 // std::vector<gtsam::SimWall2D>
 class SimWall2DVector
 {
   //Capacity
   size_t size() const;
   size_t max_size() const;
   void resize(size_t sz);
   size_t capacity() const;
   bool empty() const;
   void reserve(size_t n);

   //Element access
   gtsam::SimWall2D at(size_t n) const;
   gtsam::SimWall2D front() const;
   gtsam::SimWall2D back() const;

   //Modifiers
   void assign(size_t n, const gtsam::SimWall2D& u);
   void push_back(const gtsam::SimWall2D& x);
   void pop_back();
 };

 // std::vector<gtsam::SimPolygon2D>
 class SimPolygon2DVector
 {
   //Capacity
   size_t size() const;
   size_t max_size() const;
   void resize(size_t sz);
   size_t capacity() const;
   bool empty() const;
   void reserve(size_t n);

   //Element access
   gtsam::SimPolygon2D at(size_t n) const;
   gtsam::SimPolygon2D front() const;
   gtsam::SimPolygon2D back() const;

   //Modifiers
   void assign(size_t n, const gtsam::SimPolygon2D& u);
   void push_back(const gtsam::SimPolygon2D& x);
   void pop_back();
 };

// Nonlinear factors from gtsam, for our Value types
#include <gtsam/slam/PriorFactor.h>
template<T = {gtsam::PoseRTV}>
virtual class PriorFactor : gtsam::NonlinearFactor {
  PriorFactor(size_t key, const T& prior, const gtsam::noiseModel::Base* noiseModel);
};


#include <gtsam/slam/BetweenFactor.h>
template<T = {gtsam::PoseRTV}>
virtual class BetweenFactor : gtsam::NonlinearFactor {
  BetweenFactor(size_t key1, size_t key2, const T& relativePose, const gtsam::noiseModel::Base* noiseModel);
};


#include <gtsam/slam/RangeFactor.h>
template<POSE, POINT>
virtual class RangeFactor : gtsam::NonlinearFactor {
  RangeFactor(size_t key1, size_t key2, double measured, const gtsam::noiseModel::Base* noiseModel);
};

typedef gtsam::RangeFactor<gtsam::PoseRTV, gtsam::PoseRTV> RangeFactorRTV;


#include <gtsam/nonlinear/NonlinearEquality.h>
template<T = {gtsam::PoseRTV}>
virtual class NonlinearEquality : gtsam::NonlinearFactor {
  // Constructor - forces exact evaluation
  NonlinearEquality(size_t j, const T& feasible);
  // Constructor - allows inexact evaluation
  NonlinearEquality(size_t j, const T& feasible, double error_gain);
};

#include <gtsam_unstable/dynamics/IMUFactor.h>
template<POSE = {gtsam::PoseRTV}>
virtual class IMUFactor : gtsam::NonlinearFactor {
  /** Standard constructor */
  IMUFactor(Vector accel, Vector gyro,
    double dt, size_t key1, size_t key2, const gtsam::noiseModel::Base* model);

  /** Full IMU vector specification */
  IMUFactor(Vector imu_vector,
    double dt, size_t key1, size_t key2, const gtsam::noiseModel::Base* model);

  Vector gyro() const;
  Vector accel() const;
  Vector z() const;
  size_t key1() const;
  size_t key2() const;
};

#include <gtsam_unstable/dynamics/FullIMUFactor.h>
template<POSE = {gtsam::PoseRTV}>
virtual class FullIMUFactor : gtsam::NonlinearFactor {
  /** Standard constructor */
  FullIMUFactor(Vector accel, Vector gyro,
    double dt, size_t key1, size_t key2, const gtsam::noiseModel::Base* model);

  /** Single IMU vector - imu = [accel, gyro] */
  FullIMUFactor(Vector imu,
    double dt, size_t key1, size_t key2, const gtsam::noiseModel::Base* model);

  Vector gyro() const;
  Vector accel() const;
  Vector z() const;
  size_t key1() const;
  size_t key2() const;
};


#include <gtsam_unstable/dynamics/DynamicsPriors.h>
virtual class DHeightPrior : gtsam::NonlinearFactor {
  DHeightPrior(size_t key, double height, const gtsam::noiseModel::Base* model);
};


virtual class DRollPrior : gtsam::NonlinearFactor {
  /** allows for explicit roll parameterization - uses canonical coordinate */
  DRollPrior(size_t key, double wx, const gtsam::noiseModel::Base* model);
  /** Forces roll to zero */
  DRollPrior(size_t key, const gtsam::noiseModel::Base* model);
};


virtual class VelocityPrior : gtsam::NonlinearFactor {
  VelocityPrior(size_t key, Vector vel, const gtsam::noiseModel::Base* model);
};


virtual class DGroundConstraint : gtsam::NonlinearFactor {
  // Primary constructor allows for variable height of the "floor"
  DGroundConstraint(size_t key, double height, const gtsam::noiseModel::Base* model);
  // Fully specify vector - use only for debugging
  DGroundConstraint(size_t key, Vector constraint, const gtsam::noiseModel::Base* model);
};

#include <gtsam/base/LieScalar.h>

#include <gtsam_unstable/dynamics/VelocityConstraint3.h>
virtual class VelocityConstraint3 : gtsam::NonlinearFactor {
  /** Standard constructor */
  VelocityConstraint3(size_t key1, size_t key2, size_t velKey, double dt);

  Vector evaluateError(const gtsam::LieScalar& x1, const gtsam::LieScalar& x2, const gtsam::LieScalar& v) const;
};

//*************************************************************************
// nonlinear
//*************************************************************************
#include <gtsam_unstable/nonlinear/summarization.h>
gtsam::GaussianFactorGraph* summarizeGraphSequential(
    const gtsam::GaussianFactorGraph& full_graph, const gtsam::KeyVector& indices);
gtsam::GaussianFactorGraph* summarizeGraphSequential(
    const gtsam::GaussianFactorGraph& full_graph, const gtsam::Ordering& ordering, const gtsam::KeySet& saved_keys);

pair<gtsam::GaussianFactorGraph,gtsam::Ordering>
partialCholeskySummarization(const gtsam::NonlinearFactorGraph& graph, const gtsam::Values& values,
    const gtsam::KeySet& overlap_keys);

#include <gtsam_unstable/nonlinear/FixedLagSmoother.h>
class FixedLagSmootherKeyTimestampMapValue {
  FixedLagSmootherKeyTimestampMapValue(const gtsam::Key& key, double timestamp);
  FixedLagSmootherKeyTimestampMapValue(const gtsam::FixedLagSmootherKeyTimestampMapValue& other);
};

class FixedLagSmootherKeyTimestampMap {
  FixedLagSmootherKeyTimestampMap();
  FixedLagSmootherKeyTimestampMap(const gtsam::FixedLagSmootherKeyTimestampMap& other);

  // Note: no print function

  // common STL methods
  size_t size() const;
  bool empty() const;
  void clear();

  double at(const gtsam::Key& key) const;
  void insert(const gtsam::FixedLagSmootherKeyTimestampMapValue& value);
};

class FixedLagSmootherResult {
  size_t getIterations() const;
  size_t getNonlinearVariables() const;
  size_t getLinearVariables() const;
  double getError() const;
};

#include <gtsam_unstable/nonlinear/FixedLagSmoother.h>
virtual class FixedLagSmoother {
  void print(string s) const;
  bool equals(const gtsam::FixedLagSmoother& rhs, double tol) const;

  gtsam::FixedLagSmootherKeyTimestampMap timestamps() const;
  double smootherLag() const;

  gtsam::FixedLagSmootherResult update(const gtsam::NonlinearFactorGraph& newFactors, const gtsam::Values& newTheta, const gtsam::FixedLagSmootherKeyTimestampMap& timestamps);
  gtsam::Values calculateEstimate() const;
};

#include <gtsam_unstable/nonlinear/BatchFixedLagSmoother.h>
virtual class BatchFixedLagSmoother : gtsam::FixedLagSmoother {
  BatchFixedLagSmoother();
  BatchFixedLagSmoother(double smootherLag);
  BatchFixedLagSmoother(double smootherLag, const gtsam::LevenbergMarquardtParams& params);

  gtsam::LevenbergMarquardtParams params() const;
};

#include <gtsam_unstable/nonlinear/IncrementalFixedLagSmoother.h>
virtual class IncrementalFixedLagSmoother : gtsam::FixedLagSmoother {
  IncrementalFixedLagSmoother();
  IncrementalFixedLagSmoother(double smootherLag);
  IncrementalFixedLagSmoother(double smootherLag, const gtsam::ISAM2Params& params);

  gtsam::ISAM2Params params() const;
};

#include <gtsam_unstable/nonlinear/ConcurrentFilteringAndSmoothing.h>
virtual class ConcurrentFilter {
  void print(string s) const;
  bool equals(const gtsam::ConcurrentFilter& rhs, double tol) const;
};

virtual class ConcurrentSmoother {
  void print(string s) const;
  bool equals(const gtsam::ConcurrentSmoother& rhs, double tol) const;
};

// Synchronize function
void synchronize(gtsam::ConcurrentFilter& filter, gtsam::ConcurrentSmoother& smoother);

#include <gtsam_unstable/nonlinear/ConcurrentBatchFilter.h>
class ConcurrentBatchFilterResult {
  size_t getIterations() const;
  size_t getLambdas() const;
  size_t getNonlinearVariables() const;
  size_t getLinearVariables() const;
  double getError() const;
};

virtual class ConcurrentBatchFilter : gtsam::ConcurrentFilter {
  ConcurrentBatchFilter();
  ConcurrentBatchFilter(const gtsam::LevenbergMarquardtParams& parameters);

  gtsam::NonlinearFactorGraph getFactors() const;
  gtsam::Values getLinearizationPoint() const;
  gtsam::Ordering getOrdering() const;
  gtsam::VectorValues getDelta() const;

  gtsam::ConcurrentBatchFilterResult update();
  gtsam::ConcurrentBatchFilterResult update(const gtsam::NonlinearFactorGraph& newFactors);
  gtsam::ConcurrentBatchFilterResult update(const gtsam::NonlinearFactorGraph& newFactors, const gtsam::Values& newTheta);
  gtsam::ConcurrentBatchFilterResult update(const gtsam::NonlinearFactorGraph& newFactors, const gtsam::Values& newTheta, const gtsam::KeyList& keysToMove);
  gtsam::Values calculateEstimate() const;
};

#include <gtsam_unstable/nonlinear/ConcurrentBatchSmoother.h>
class ConcurrentBatchSmootherResult {
  size_t getIterations() const;
  size_t getLambdas() const;
  size_t getNonlinearVariables() const;
  size_t getLinearVariables() const;
  double getError() const;
};

virtual class ConcurrentBatchSmoother : gtsam::ConcurrentSmoother {
  ConcurrentBatchSmoother();
  ConcurrentBatchSmoother(const gtsam::LevenbergMarquardtParams& parameters);

  gtsam::NonlinearFactorGraph getFactors() const;
  gtsam::Values getLinearizationPoint() const;
  gtsam::Ordering getOrdering() const;
  gtsam::VectorValues getDelta() const;

  gtsam::ConcurrentBatchSmootherResult update();
  gtsam::ConcurrentBatchSmootherResult update(const gtsam::NonlinearFactorGraph& newFactors);
  gtsam::ConcurrentBatchSmootherResult update(const gtsam::NonlinearFactorGraph& newFactors, const gtsam::Values& newTheta);
  gtsam::Values calculateEstimate() const;
};

//*************************************************************************
// slam
//*************************************************************************
#include <gtsam/geometry/Pose2.h>

#include <gtsam_unstable/slam/PoseTranslationPrior.h>
template<POSE>
virtual class PoseTranslationPrior : gtsam::NonlinearFactor {
  PoseTranslationPrior(size_t key, const POSE& pose_z, const gtsam::noiseModel::Base* noiseModel);
};

typedef gtsam::PoseTranslationPrior<gtsam::Pose2> PoseTranslationPrior2D;
typedef gtsam::PoseTranslationPrior<gtsam::Pose3> PoseTranslationPrior3D;

#include <gtsam_unstable/slam/PoseRotationPrior.h>
template<POSE>
virtual class PoseRotationPrior : gtsam::NonlinearFactor {
  PoseRotationPrior(size_t key, const POSE& pose_z, const gtsam::noiseModel::Base* noiseModel);
};

typedef gtsam::PoseRotationPrior<gtsam::Pose2> PoseRotationPrior2D;
typedef gtsam::PoseRotationPrior<gtsam::Pose3> PoseRotationPrior3D;

#include <gtsam_unstable/slam/RelativeElevationFactor.h>
virtual class RelativeElevationFactor: gtsam::NonlinearFactor {
  RelativeElevationFactor();
  RelativeElevationFactor(size_t poseKey, size_t pointKey, double measured,
      const gtsam::noiseModel::Base* model);

  double measured() const;
  void print(string s) const;
};

#include <gtsam_unstable/slam/DummyFactor.h>
virtual class DummyFactor : gtsam::NonlinearFactor {
  DummyFactor(size_t key1, size_t dim1, size_t key2, size_t dim2);
};

} //\namespace gtsam
