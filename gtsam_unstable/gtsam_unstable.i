/**
 * Matlab toolbox interface definition for gtsam_unstable
 */

// specify the classes from gtsam we are using
virtual class gtsam::Value;
class gtsam::Vector6;
class gtsam::Point2;
class gtsam::Point2Vector;
class gtsam::Rot2;
class gtsam::Pose2;
class gtsam::Point3;
class gtsam::SO3;
class gtsam::SO4;
class gtsam::SOn;
class gtsam::Rot3;
class gtsam::Pose3;
virtual class gtsam::noiseModel::Base;
virtual class gtsam::noiseModel::Gaussian;
virtual class gtsam::noiseModel::Isotropic;
virtual class gtsam::imuBias::ConstantBias;
virtual class gtsam::NonlinearFactor;
virtual class gtsam::NoiseModelFactor;
virtual class gtsam::NoiseModelFactorN;
virtual class gtsam::GaussianFactor;
virtual class gtsam::HessianFactor;
virtual class gtsam::JacobianFactor;
class gtsam::Cal3_S2;
class gtsam::Cal3DS2;
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
class gtsam::GaussianDensity;
class gtsam::LevenbergMarquardtOptimizer;

namespace gtsam {

#include <gtsam_unstable/base/Dummy.h>
class Dummy {
  Dummy();
  void print(string s) const;
  unsigned char dummyTwoVar(unsigned char a) const;
};

#include <gtsam_unstable/dynamics/PoseRTV.h>
class PoseRTV {
  PoseRTV();
  PoseRTV(Vector rtv);
  PoseRTV(const gtsam::Point3& pt, const gtsam::Rot3& rot, const Vector& vel);
  PoseRTV(const gtsam::Rot3& rot, const gtsam::Point3& pt, const Vector& vel);
  PoseRTV(const gtsam::Pose3& pose, const Vector& vel);
  PoseRTV(const gtsam::Pose3& pose);
  PoseRTV(double roll, double pitch, double yaw, double x, double y, double z, double vx, double vy, double vz);

  // testable
  bool equals(const gtsam::PoseRTV& other, double tol) const;
  void print(string s) const;

  // access
  gtsam::Point3 translation() const;
  gtsam::Rot3 rotation() const;
  Vector velocity() const;
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

  void serializable() const; // enabling serialization functionality
};

#include <gtsam_unstable/geometry/Pose3Upright.h>
class Pose3Upright {
  Pose3Upright();
  Pose3Upright(const gtsam::Pose3Upright& other);
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

  static gtsam::Pose3Upright Identity();
  gtsam::Pose3Upright inverse() const;
  gtsam::Pose3Upright compose(const gtsam::Pose3Upright& p2) const;
  gtsam::Pose3Upright between(const gtsam::Pose3Upright& p2) const;

  static gtsam::Pose3Upright Expmap(Vector xi);
  static Vector Logmap(const gtsam::Pose3Upright& p);

  void serializable() const; // enabling serialization functionality
}; // \class Pose3Upright

#include <gtsam_unstable/geometry/BearingS2.h>
class BearingS2 {
  BearingS2();
  BearingS2(double azimuth_double, double elevation_double);
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

  void serializable() const; // enabling serialization functionality
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
  //   bool intersects(const gtsam::SimWall2D& wall, gtsam::Point2* pt = nullptr) const;

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
#include <gtsam/nonlinear/PriorFactor.h>
template<T = {gtsam::PoseRTV}>
virtual class PriorFactor : gtsam::NoiseModelFactor {
  PriorFactor(size_t key, const T& prior, const gtsam::noiseModel::Base* noiseModel);

  void serializable() const; // enabling serialization functionality
};

#include <gtsam/slam/BetweenFactor.h>
template<T = {gtsam::PoseRTV}>
virtual class BetweenFactor : gtsam::NoiseModelFactor {
  BetweenFactor(size_t key1, size_t key2, const T& relativePose, const gtsam::noiseModel::Base* noiseModel);

  void serializable() const; // enabling serialization functionality
};

#include <gtsam_unstable/slam/BetweenFactorEM.h>
template<T = {gtsam::Pose2}>
virtual class BetweenFactorEM : gtsam::NonlinearFactor {
  BetweenFactorEM(size_t key1, size_t key2, const T& relativePose,
      const gtsam::noiseModel::Gaussian* model_inlier, const gtsam::noiseModel::Gaussian* model_outlier,
      double prior_inlier, double prior_outlier);

  BetweenFactorEM(size_t key1, size_t key2, const T& relativePose,
        const gtsam::noiseModel::Gaussian* model_inlier, const gtsam::noiseModel::Gaussian* model_outlier,
        double prior_inlier, double prior_outlier,  bool flag_bump_up_near_zero_probs);

  Vector whitenedError(const gtsam::Values& x);
  Vector unwhitenedError(const gtsam::Values& x);
  Vector calcIndicatorProb(const gtsam::Values& x);
  gtsam::Pose2 measured(); // TODO: figure out how to output a template instead
  void set_flag_bump_up_near_zero_probs(bool flag);
  bool get_flag_bump_up_near_zero_probs() const;

  void updateNoiseModels(const gtsam::Values& values, const gtsam::NonlinearFactorGraph& graph);
  void updateNoiseModels_givenCovs(const gtsam::Values& values, Matrix cov1, Matrix cov2, Matrix cov12);
  Matrix get_model_inlier_cov();
  Matrix get_model_outlier_cov();

  void serializable() const; // enabling serialization functionality
};

#include <gtsam_unstable/slam/TransformBtwRobotsUnaryFactorEM.h>
template<T = {gtsam::Pose2}>
virtual class TransformBtwRobotsUnaryFactorEM : gtsam::NonlinearFactor {
  TransformBtwRobotsUnaryFactorEM(size_t key, const T& relativePose, size_t keyA, size_t keyB,
      const gtsam::Values& valA, const gtsam::Values& valB,
      const gtsam::noiseModel::Gaussian* model_inlier, const gtsam::noiseModel::Gaussian* model_outlier,
      double prior_inlier, double prior_outlier);

  TransformBtwRobotsUnaryFactorEM(size_t key, const T& relativePose, size_t keyA, size_t keyB,
        const gtsam::Values& valA, const gtsam::Values& valB,
        const gtsam::noiseModel::Gaussian* model_inlier, const gtsam::noiseModel::Gaussian* model_outlier,
        double prior_inlier, double prior_outlier, bool flag_bump_up_near_zero_probs, bool start_with_M_step);

  Vector whitenedError(const gtsam::Values& x);
  Vector unwhitenedError(const gtsam::Values& x);
  Vector calcIndicatorProb(const gtsam::Values& x);
  void setValAValB(const gtsam::Values valA, const gtsam::Values valB);

  void updateNoiseModels(const gtsam::Values& values, const gtsam::NonlinearFactorGraph& graph);
  void updateNoiseModels_givenCovs(const gtsam::Values& values, Matrix cov1, Matrix cov2, Matrix cov12);
  Matrix get_model_inlier_cov();
  Matrix get_model_outlier_cov();

  void serializable() const; // enabling serialization functionality
};

#include <gtsam_unstable/slam/TransformBtwRobotsUnaryFactor.h>
template<T = {gtsam::Pose2}>
virtual class TransformBtwRobotsUnaryFactor : gtsam::NonlinearFactor {
  TransformBtwRobotsUnaryFactor(size_t key, const T& relativePose, size_t keyA, size_t keyB,
      const gtsam::Values& valA, const gtsam::Values& valB,
      const gtsam::noiseModel::Gaussian* model);

  Vector whitenedError(const gtsam::Values& x);
  Vector unwhitenedError(const gtsam::Values& x);
  void setValAValB(const gtsam::Values valA, const gtsam::Values valB);

  void serializable() const; // enabling serialization functionality
};

#include <gtsam_unstable/slam/SmartRangeFactor.h>
virtual class SmartRangeFactor : gtsam::NoiseModelFactor {
  SmartRangeFactor(double s);

  void addRange(size_t key, double measuredRange);
  gtsam::Point2 triangulate(const gtsam::Values& x) const;
  //void print(string s) const;

};

#include <gtsam/sam/RangeFactor.h>
template<POSE, POINT>
virtual class RangeFactor : gtsam::NoiseModelFactor {
  RangeFactor(size_t key1, size_t key2, double measured, const gtsam::noiseModel::Base* noiseModel);

  void serializable() const; // enabling serialization functionality
};

typedef gtsam::RangeFactor<gtsam::PoseRTV, gtsam::PoseRTV> RangeFactorRTV;

#include <gtsam_unstable/geometry/Event.h>
class Event {
  Event();
  Event(double t, const gtsam::Point3& p);
  Event(double t, double x, double y, double z);
  double time() const;
  gtsam::Point3 location() const;
  double height() const;
  void print(string s) const;
};

class TimeOfArrival {
  TimeOfArrival();
  TimeOfArrival(double speed);
  double measure(const gtsam::Event& event, const gtsam::Point3& sensor) const;
};

#include <gtsam_unstable/slam/TOAFactor.h>
virtual class TOAFactor : gtsam::NonlinearFactor {
  // For now, because of overload issues, we only expose constructor with known sensor coordinates:
  TOAFactor(size_t key1, gtsam::Point3 sensor, double measured,
            const gtsam::noiseModel::Base* noiseModel);
  static void InsertEvent(size_t key, const gtsam::Event& event, gtsam::Values* values);
};

#include <gtsam/nonlinear/NonlinearEquality.h>
template<T = {gtsam::PoseRTV}>
virtual class NonlinearEquality : gtsam::NoiseModelFactor {
  // Constructor - forces exact evaluation
  NonlinearEquality(size_t j, const T& feasible);
  // Constructor - allows inexact evaluation
  NonlinearEquality(size_t j, const T& feasible, double error_gain);

  void serializable() const; // enabling serialization functionality
};

#include <gtsam_unstable/dynamics/IMUFactor.h>
template<POSE = {gtsam::PoseRTV}>
virtual class IMUFactor : gtsam::NoiseModelFactor {
  /** Standard constructor */
  IMUFactor(Vector accel, Vector gyro,
    double dt, size_t key1, size_t key2, const gtsam::noiseModel::Base* model);

  /** Full IMU vector specification */
  IMUFactor(Vector imu_vector,
    double dt, size_t key1, size_t key2, const gtsam::noiseModel::Base* model);

  Vector gyro() const;
  Vector accel() const;
  Vector z() const;

  template <I = {1, 2}>
  size_t key() const;
};

#include <gtsam_unstable/dynamics/FullIMUFactor.h>
template<POSE = {gtsam::PoseRTV}>
virtual class FullIMUFactor : gtsam::NoiseModelFactor {
  /** Standard constructor */
  FullIMUFactor(Vector accel, Vector gyro,
    double dt, size_t key1, size_t key2, const gtsam::noiseModel::Base* model);

  /** Single IMU vector - imu = [accel, gyro] */
  FullIMUFactor(Vector imu,
    double dt, size_t key1, size_t key2, const gtsam::noiseModel::Base* model);

  Vector gyro() const;
  Vector accel() const;
  Vector z() const;

  template <I = {1, 2}>
  size_t key() const;
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

#include <gtsam_unstable/dynamics/VelocityConstraint3.h>
virtual class VelocityConstraint3 : gtsam::NonlinearFactor {
  /** Standard constructor */
  VelocityConstraint3(size_t key1, size_t key2, size_t velKey, double dt);

  Vector evaluateError(const double& x1, const double& x2, const double& v) const;
};

#include <gtsam_unstable/dynamics/Pendulum.h>
virtual class PendulumFactor1 : gtsam::NonlinearFactor {
  /** Standard constructor */
  PendulumFactor1(size_t k1, size_t k, size_t velKey, double dt);

  Vector evaluateError(const double& qk1, const double& qk, const double& v) const;
};

#include <gtsam_unstable/dynamics/Pendulum.h>
virtual class PendulumFactor2 : gtsam::NonlinearFactor {
  /** Standard constructor */
  PendulumFactor2(size_t vk1, size_t vk, size_t qKey, double dt, double L, double g);

  Vector evaluateError(const double& vk1, const double& vk, const double& q) const;
};

virtual class PendulumFactorPk : gtsam::NonlinearFactor {
  /** Standard constructor */
  PendulumFactorPk(size_t pk, size_t qk, size_t qk1, double h, double m, double r, double g, double alpha);

  Vector evaluateError(const double& pk, const double& qk, const double& qk1) const;
};

virtual class PendulumFactorPk1 : gtsam::NonlinearFactor {
  /** Standard constructor */
  PendulumFactorPk1(size_t pk1, size_t qk, size_t qk1, double h, double m, double r, double g, double alpha);

  Vector evaluateError(const double& pk1, const double& qk, const double& qk1) const;
};

#include <gtsam_unstable/dynamics/SimpleHelicopter.h>
virtual class Reconstruction : gtsam::NoiseModelFactor {
  Reconstruction(size_t gKey1, size_t gKey, size_t xiKey, double h);

  Vector evaluateError(const gtsam::Pose3& gK1, const gtsam::Pose3& gK, Vector xiK) const;
};

virtual class DiscreteEulerPoincareHelicopter : gtsam::NoiseModelFactor {
  DiscreteEulerPoincareHelicopter(size_t xiKey, size_t xiKey_1, size_t gKey,
      double h, Matrix Inertia, Vector Fu, double m);

  Vector evaluateError(Vector xiK, Vector xiK_1, const gtsam::Pose3& gK) const;
};

//*************************************************************************
// nonlinear
//*************************************************************************
#include <gtsam_unstable/nonlinear/FixedLagSmoother.h>
class FixedLagSmootherKeyTimestampMapValue {
  FixedLagSmootherKeyTimestampMapValue(size_t key, double timestamp);
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

  double at(const size_t key) const;
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

  gtsam::FixedLagSmootherResult update(const gtsam::NonlinearFactorGraph &newFactors,
                                       const gtsam::Values &newTheta,
                                       const gtsam::FixedLagSmootherKeyTimestampMap &timestamps);
  gtsam::FixedLagSmootherResult update(const gtsam::NonlinearFactorGraph &newFactors,
                                       const gtsam::Values &newTheta,
                                       const gtsam::FixedLagSmootherKeyTimestampMap &timestamps,
                                       const gtsam::FactorIndices &factorsToRemove);
  gtsam::Values calculateEstimate() const;
};

#include <gtsam_unstable/nonlinear/BatchFixedLagSmoother.h>
virtual class BatchFixedLagSmoother : gtsam::FixedLagSmoother {
  BatchFixedLagSmoother();
  BatchFixedLagSmoother(double smootherLag);
  BatchFixedLagSmoother(double smootherLag, const gtsam::LevenbergMarquardtParams& params);

  void print(string s = "BatchFixedLagSmoother:\n") const;

  gtsam::LevenbergMarquardtParams params() const;
  template <VALUE = {gtsam::Point2, gtsam::Rot2, gtsam::Pose2, gtsam::Point3,
                     gtsam::Rot3, gtsam::Pose3, gtsam::Cal3_S2, gtsam::Cal3DS2,
                     Vector, Matrix}>
  VALUE calculateEstimate(size_t key) const;
};

#include <gtsam_unstable/nonlinear/IncrementalFixedLagSmoother.h>
virtual class IncrementalFixedLagSmoother : gtsam::FixedLagSmoother {
  IncrementalFixedLagSmoother();
  IncrementalFixedLagSmoother(double smootherLag);
  IncrementalFixedLagSmoother(double smootherLag, const gtsam::ISAM2Params& params);

  void print(string s = "IncrementalFixedLagSmoother:\n") const;

  gtsam::ISAM2Params params() const;

  gtsam::NonlinearFactorGraph getFactors() const;
  gtsam::ISAM2 getISAM2() const;
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
#include <gtsam_unstable/slam/RelativeElevationFactor.h>
virtual class RelativeElevationFactor: gtsam::NoiseModelFactor {
  RelativeElevationFactor();
  RelativeElevationFactor(size_t poseKey, size_t pointKey, double measured,
      const gtsam::noiseModel::Base* model);

  double measured() const;
  //void print(string s) const;
};

#include <gtsam_unstable/slam/DummyFactor.h>
virtual class DummyFactor : gtsam::NonlinearFactor {
  DummyFactor(size_t key1, size_t dim1, size_t key2, size_t dim2);
};

#include <gtsam_unstable/slam/InvDepthFactorVariant1.h>
virtual class InvDepthFactorVariant1 : gtsam::NoiseModelFactor {
  InvDepthFactorVariant1(size_t poseKey, size_t landmarkKey, const gtsam::Point2& measured, const gtsam::Cal3_S2* K, const gtsam::noiseModel::Base* model);
};

#include <gtsam_unstable/slam/InvDepthFactorVariant2.h>
virtual class InvDepthFactorVariant2 : gtsam::NoiseModelFactor {
  InvDepthFactorVariant2(size_t poseKey, size_t landmarkKey, const gtsam::Point2& measured, const gtsam::Cal3_S2* K, const gtsam::Point3& referencePoint, const gtsam::noiseModel::Base* model);
};

#include <gtsam_unstable/slam/InvDepthFactorVariant3.h>
virtual class InvDepthFactorVariant3a : gtsam::NoiseModelFactor {
  InvDepthFactorVariant3a(size_t poseKey, size_t landmarkKey, const gtsam::Point2& measured, const gtsam::Cal3_S2* K, const gtsam::noiseModel::Base* model);
};
virtual class InvDepthFactorVariant3b : gtsam::NoiseModelFactor {
  InvDepthFactorVariant3b(size_t poseKey1, size_t poseKey2, size_t landmarkKey, const gtsam::Point2& measured, const gtsam::Cal3_S2* K, const gtsam::noiseModel::Base* model);
};


#include <gtsam_unstable/slam/Mechanization_bRn2.h>
class Mechanization_bRn2 {
  Mechanization_bRn2();
  Mechanization_bRn2(gtsam::Rot3& initial_bRn, Vector initial_x_g,
      Vector initial_x_a);
  Vector b_g(double g_e);
  gtsam::Rot3 bRn();
  Vector x_g();
  Vector x_a();
  static gtsam::Mechanization_bRn2 initialize(Matrix U, Matrix F, double g_e);
  gtsam::Mechanization_bRn2 correct(Vector dx) const;
  gtsam::Mechanization_bRn2 integrate(Vector u, double dt) const;
  //void print(string s) const;
};

#include <gtsam_unstable/slam/AHRS.h>
class AHRS {
  AHRS(Matrix U, Matrix F, double g_e);
  pair<gtsam::Mechanization_bRn2, gtsam::GaussianDensity*> initialize(double g_e);
  pair<gtsam::Mechanization_bRn2, gtsam::GaussianDensity*> integrate(const gtsam::Mechanization_bRn2& mech, gtsam::GaussianDensity* state, Vector u, double dt);
  pair<gtsam::Mechanization_bRn2, gtsam::GaussianDensity*> aid(const gtsam::Mechanization_bRn2& mech, gtsam::GaussianDensity* state, Vector f, bool Farrel);
  pair<gtsam::Mechanization_bRn2, gtsam::GaussianDensity*> aidGeneral(const gtsam::Mechanization_bRn2& mech, gtsam::GaussianDensity* state, Vector f, Vector f_expected, const gtsam::Rot3& increment);
  //void print(string s) const;
};

// Tectonic SAM Factors

#include <gtsam_unstable/slam/TSAMFactors.h>
//typedef gtsam::NoiseModelFactorN<gtsam::Pose2, gtsam::Point2> NLPosePose;
virtual class DeltaFactor : gtsam::NoiseModelFactor {
  DeltaFactor(size_t i, size_t j, const gtsam::Point2& measured,
      const gtsam::noiseModel::Base* noiseModel);
  //void print(string s) const;
};

//typedef gtsam::NoiseModelFactorN<gtsam::Pose2, gtsam::Pose2, gtsam::Pose2,
//    gtsam::Point2> NLPosePosePosePoint;
virtual class DeltaFactorBase : gtsam::NoiseModelFactor {
  DeltaFactorBase(size_t b1, size_t i, size_t b2, size_t j,
      const gtsam::Point2& measured, const gtsam::noiseModel::Base* noiseModel);
  //void print(string s) const;
};

//typedef gtsam::NoiseModelFactorN<gtsam::Pose2, gtsam::Pose2, gtsam::Pose2,
//    gtsam::Pose2> NLPosePosePosePose;
virtual class OdometryFactorBase : gtsam::NoiseModelFactor {
  OdometryFactorBase(size_t b1, size_t i, size_t b2, size_t j,
      const gtsam::Pose2& measured, const gtsam::noiseModel::Base* noiseModel);
  //void print(string s) const;
};

#include <gtsam/geometry/Cal3DS2.h>
#include <gtsam_unstable/slam/ProjectionFactorPPP.h>
template<POSE, LANDMARK, CALIBRATION>
virtual class ProjectionFactorPPP : gtsam::NoiseModelFactor {
  ProjectionFactorPPP(const gtsam::Point2& measured, const gtsam::noiseModel::Base* noiseModel,
    size_t poseKey, size_t transformKey, size_t pointKey, const CALIBRATION* k);

  ProjectionFactorPPP(const gtsam::Point2& measured, const gtsam::noiseModel::Base* noiseModel,
      size_t poseKey, size_t transformKey, size_t pointKey, const CALIBRATION* k, bool throwCheirality, bool verboseCheirality);

  gtsam::Point2 measured() const;
  CALIBRATION* calibration() const;
  bool verboseCheirality() const;
  bool throwCheirality() const;

  // enabling serialization functionality
  void serialize() const;
};
typedef gtsam::ProjectionFactorPPP<gtsam::Pose3, gtsam::Point3, gtsam::Cal3_S2> ProjectionFactorPPPCal3_S2;
typedef gtsam::ProjectionFactorPPP<gtsam::Pose3, gtsam::Point3, gtsam::Cal3DS2> ProjectionFactorPPPCal3DS2;


#include <gtsam_unstable/slam/ProjectionFactorPPPC.h>
template<POSE, LANDMARK, CALIBRATION>
virtual class ProjectionFactorPPPC : gtsam::NoiseModelFactor {
  ProjectionFactorPPPC(const gtsam::Point2& measured, const gtsam::noiseModel::Base* noiseModel,
    size_t poseKey, size_t transformKey, size_t pointKey, size_t calibKey);

  ProjectionFactorPPPC(const gtsam::Point2& measured, const gtsam::noiseModel::Base* noiseModel,
      size_t poseKey, size_t transformKey, size_t pointKey, size_t calibKey, bool throwCheirality, bool verboseCheirality);

  gtsam::Point2 measured() const;
  bool verboseCheirality() const;
  bool throwCheirality() const;

  // enabling serialization functionality
  void serialize() const;
};
typedef gtsam::ProjectionFactorPPPC<gtsam::Pose3, gtsam::Point3, gtsam::Cal3_S2> ProjectionFactorPPPCCal3_S2;
typedef gtsam::ProjectionFactorPPPC<gtsam::Pose3, gtsam::Point3, gtsam::Cal3DS2> ProjectionFactorPPPCCal3DS2;

#include <gtsam_unstable/slam/ProjectionFactorRollingShutter.h>
virtual class ProjectionFactorRollingShutter : gtsam::NoiseModelFactor {
  ProjectionFactorRollingShutter(const gtsam::Point2& measured, double alpha, const gtsam::noiseModel::Base* noiseModel,
      size_t poseKey_a, size_t poseKey_b, size_t pointKey, const gtsam::Cal3_S2* K);

  ProjectionFactorRollingShutter(const gtsam::Point2& measured, double alpha, const gtsam::noiseModel::Base* noiseModel,
    size_t poseKey_a, size_t poseKey_b, size_t pointKey, const gtsam::Cal3_S2* K, gtsam::Pose3& body_P_sensor);

  ProjectionFactorRollingShutter(const gtsam::Point2& measured, double alpha, const gtsam::noiseModel::Base* noiseModel,
        size_t poseKey_a, size_t poseKey_b, size_t pointKey, const gtsam::Cal3_S2* K, bool throwCheirality,
        bool verboseCheirality);

  ProjectionFactorRollingShutter(const gtsam::Point2& measured, double alpha, const gtsam::noiseModel::Base* noiseModel,
      size_t poseKey_a, size_t poseKey_b, size_t pointKey, const gtsam::Cal3_S2* K, bool throwCheirality,
      bool verboseCheirality, gtsam::Pose3& body_P_sensor);

  gtsam::Point2 measured() const;
  double alpha() const;
  gtsam::Cal3_S2* calibration() const;
  bool verboseCheirality() const;
  bool throwCheirality() const;

  // enabling serialization functionality
  void serialize() const;
};

} //\namespace gtsam
