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
class gtsam::FixedLagSmoother;

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
  PoseRTV(gtsam::Vector rtv);
  PoseRTV(const gtsam::Point3& pt, const gtsam::Rot3& rot, const gtsam::Vector& vel);
  PoseRTV(const gtsam::Rot3& rot, const gtsam::Point3& pt, const gtsam::Vector& vel);
  PoseRTV(const gtsam::Pose3& pose, const gtsam::Vector& vel);
  PoseRTV(const gtsam::Pose3& pose);
  PoseRTV(double roll, double pitch, double yaw, double x, double y, double z, double vx, double vy, double vz);

  // testable
  bool equals(const gtsam::PoseRTV& other, double tol) const;
  void print(string s) const;

  // access
  gtsam::Point3 translation() const;
  gtsam::Rot3 rotation() const;
  gtsam::Vector velocity() const;
  gtsam::Pose3 pose() const;

  // gtsam::Vector interfaces
  gtsam::Vector vector() const;
  gtsam::Vector translationVec() const;
  gtsam::Vector velocityVec() const;

  // manifold/Lie
  static size_t Dim();
  size_t dim() const;
  gtsam::PoseRTV retract(gtsam::Vector v) const;
  gtsam::Vector localCoordinates(const gtsam::PoseRTV& p) const;
  static gtsam::PoseRTV Expmap(gtsam::Vector v);
  static gtsam::Vector Logmap(const gtsam::PoseRTV& p);
  gtsam::PoseRTV inverse() const;
  gtsam::PoseRTV compose(const gtsam::PoseRTV& p) const;
  gtsam::PoseRTV between(const gtsam::PoseRTV& p) const;

  // measurement
  double range(const gtsam::PoseRTV& other) const;
  gtsam::PoseRTV transformed_from(const gtsam::Pose3& trans) const;

  // IMU/dynamics
  gtsam::PoseRTV planarDynamics(double vel_rate, double heading_rate, double max_accel, double dt) const;
  gtsam::PoseRTV flyingDynamics(double pitch_rate, double heading_rate, double lift_control, double dt) const;
  gtsam::PoseRTV generalDynamics(gtsam::Vector accel, gtsam::Vector gyro, double dt) const;
  gtsam::Vector imuPrediction(const gtsam::PoseRTV& x2, double dt) const;
  gtsam::Point3 translationIntegration(const gtsam::PoseRTV& x2, double dt) const;
  gtsam::Vector translationIntegrationVec(const gtsam::PoseRTV& x2, double dt) const;

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
  gtsam::Pose3Upright retract(gtsam::Vector v) const;
  gtsam::Vector localCoordinates(const gtsam::Pose3Upright& p2) const;

  static gtsam::Pose3Upright Identity();
  gtsam::Pose3Upright inverse() const;
  gtsam::Pose3Upright compose(const gtsam::Pose3Upright& p2) const;
  gtsam::Pose3Upright between(const gtsam::Pose3Upright& p2) const;

  static gtsam::Pose3Upright Expmap(gtsam::Vector xi);
  static gtsam::Vector Logmap(const gtsam::Pose3Upright& p);

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
  gtsam::BearingS2 retract(gtsam::Vector v) const;
  gtsam::Vector localCoordinates(const gtsam::BearingS2& p2) const;

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
  PriorFactor(gtsam::Key key, const T& prior, const gtsam::noiseModel::Base* noiseModel);

  void serializable() const; // enabling serialization functionality
};

#include <gtsam/slam/BetweenFactor.h>
template<T = {gtsam::PoseRTV}>
virtual class BetweenFactor : gtsam::NoiseModelFactor {
  BetweenFactor(gtsam::Key key1, gtsam::Key key2, const T& relativePose, const gtsam::noiseModel::Base* noiseModel);

  void serializable() const; // enabling serialization functionality
};

#include <gtsam_unstable/slam/BetweenFactorEM.h>
template<T = {gtsam::Pose2}>
virtual class BetweenFactorEM : gtsam::NonlinearFactor {
  BetweenFactorEM(gtsam::Key key1, gtsam::Key key2, const T& relativePose,
      const gtsam::noiseModel::Gaussian* model_inlier, const gtsam::noiseModel::Gaussian* model_outlier,
      double prior_inlier, double prior_outlier);

  BetweenFactorEM(gtsam::Key key1, gtsam::Key key2, const T& relativePose,
        const gtsam::noiseModel::Gaussian* model_inlier, const gtsam::noiseModel::Gaussian* model_outlier,
        double prior_inlier, double prior_outlier,  bool flag_bump_up_near_zero_probs);

  gtsam::Vector whitenedError(const gtsam::Values& x);
  gtsam::Vector unwhitenedError(const gtsam::Values& x);
  gtsam::Vector calcIndicatorProb(const gtsam::Values& x);
  gtsam::Pose2 measured(); // TODO: figure out how to output a template instead
  void set_flag_bump_up_near_zero_probs(bool flag);
  bool get_flag_bump_up_near_zero_probs() const;

  void updateNoiseModels(const gtsam::Values& values, const gtsam::NonlinearFactorGraph& graph);
  void updateNoiseModels_givenCovs(const gtsam::Values& values, gtsam::Matrix cov1, gtsam::Matrix cov2, gtsam::Matrix cov12);
  gtsam::Matrix get_model_inlier_cov();
  gtsam::Matrix get_model_outlier_cov();

  void serializable() const; // enabling serialization functionality
};

#include <gtsam_unstable/slam/TransformBtwRobotsUnaryFactorEM.h>
template<T = {gtsam::Pose2}>
virtual class TransformBtwRobotsUnaryFactorEM : gtsam::NonlinearFactor {
  TransformBtwRobotsUnaryFactorEM(gtsam::Key key, const T& relativePose, gtsam::Key keyA, gtsam::Key keyB,
      const gtsam::Values& valA, const gtsam::Values& valB,
      const gtsam::noiseModel::Gaussian* model_inlier, const gtsam::noiseModel::Gaussian* model_outlier,
      double prior_inlier, double prior_outlier);

  TransformBtwRobotsUnaryFactorEM(gtsam::Key key, const T& relativePose, gtsam::Key keyA, gtsam::Key keyB,
        const gtsam::Values& valA, const gtsam::Values& valB,
        const gtsam::noiseModel::Gaussian* model_inlier, const gtsam::noiseModel::Gaussian* model_outlier,
        double prior_inlier, double prior_outlier, bool flag_bump_up_near_zero_probs, bool start_with_M_step);

  gtsam::Vector whitenedError(const gtsam::Values& x);
  gtsam::Vector unwhitenedError(const gtsam::Values& x);
  gtsam::Vector calcIndicatorProb(const gtsam::Values& x);
  void setValAValB(const gtsam::Values valA, const gtsam::Values valB);

  void updateNoiseModels(const gtsam::Values& values, const gtsam::NonlinearFactorGraph& graph);
  void updateNoiseModels_givenCovs(const gtsam::Values& values, gtsam::Matrix cov1, gtsam::Matrix cov2, gtsam::Matrix cov12);
  gtsam::Matrix get_model_inlier_cov();
  gtsam::Matrix get_model_outlier_cov();

  void serializable() const; // enabling serialization functionality
};

#include <gtsam_unstable/slam/TransformBtwRobotsUnaryFactor.h>
template<T = {gtsam::Pose2}>
virtual class TransformBtwRobotsUnaryFactor : gtsam::NonlinearFactor {
  TransformBtwRobotsUnaryFactor(gtsam::Key key, const T& relativePose, gtsam::Key keyA, gtsam::Key keyB,
      const gtsam::Values& valA, const gtsam::Values& valB,
      const gtsam::noiseModel::Gaussian* model);

  gtsam::Vector whitenedError(const gtsam::Values& x);
  gtsam::Vector unwhitenedError(const gtsam::Values& x);
  void setValAValB(const gtsam::Values valA, const gtsam::Values valB);

  void serializable() const; // enabling serialization functionality
};

#include <gtsam_unstable/slam/SmartRangeFactor.h>
virtual class SmartRangeFactor : gtsam::NoiseModelFactor {
  SmartRangeFactor(double s);

  void addRange(gtsam::Key key, double measuredRange);
  gtsam::Point2 triangulate(const gtsam::Values& x) const;
  //void print(string s) const;

};

#include <gtsam/sam/RangeFactor.h>
template<POSE, POINT>
virtual class RangeFactor : gtsam::NoiseModelFactor {
  RangeFactor(gtsam::Key key1, gtsam::Key key2, double measured, const gtsam::noiseModel::Base* noiseModel);

  void serializable() const; // enabling serialization functionality
};

typedef gtsam::RangeFactor<gtsam::PoseRTV, gtsam::PoseRTV> RangeFactorRTV;

class TimeOfArrival {
  TimeOfArrival();
  TimeOfArrival(double speed);
  double measure(const gtsam::Event& event, const gtsam::Point3& sensor) const;
};

#include <gtsam_unstable/slam/TOAFactor.h>
virtual class TOAFactor : gtsam::NonlinearFactor {
  // For now, because of overload issues, we only expose constructor with known sensor coordinates:
  TOAFactor(gtsam::Key key1, gtsam::Point3 sensor, double measured,
            const gtsam::noiseModel::Base* noiseModel);
  static void InsertEvent(gtsam::Key key, const gtsam::Event& event, gtsam::Values* values);
};

#include <gtsam/nonlinear/NonlinearEquality.h>
template<T = {gtsam::PoseRTV}>
virtual class NonlinearEquality : gtsam::NoiseModelFactor {
  // Constructor - forces exact evaluation
  NonlinearEquality(gtsam::Key j, const T& feasible);
  // Constructor - allows inexact evaluation
  NonlinearEquality(gtsam::Key j, const T& feasible, double error_gain);

  void serializable() const; // enabling serialization functionality
};

#include <gtsam_unstable/dynamics/IMUFactor.h>
template<POSE = {gtsam::PoseRTV}>
virtual class IMUFactor : gtsam::NoiseModelFactor {
  /** Standard constructor */
  IMUFactor(gtsam::Vector accel, gtsam::Vector gyro,
    double dt, gtsam::Key key1, gtsam::Key key2, const gtsam::noiseModel::Base* model);

  /** Full IMU vector specification */
  IMUFactor(gtsam::Vector imu_vector,
    double dt, gtsam::Key key1, gtsam::Key key2, const gtsam::noiseModel::Base* model);

  gtsam::Vector gyro() const;
  gtsam::Vector accel() const;
  gtsam::Vector z() const;

  template <I = {1, 2}>
  gtsam::Key key() const;
};

#include <gtsam_unstable/dynamics/FullIMUFactor.h>
template<POSE = {gtsam::PoseRTV}>
virtual class FullIMUFactor : gtsam::NoiseModelFactor {
  /** Standard constructor */
  FullIMUFactor(gtsam::Vector accel, gtsam::Vector gyro,
    double dt, gtsam::Key key1, gtsam::Key key2, const gtsam::noiseModel::Base* model);

  /** Single IMU vector - imu = [accel, gyro] */
  FullIMUFactor(gtsam::Vector imu,
    double dt, gtsam::Key key1, gtsam::Key key2, const gtsam::noiseModel::Base* model);

  gtsam::Vector gyro() const;
  gtsam::Vector accel() const;
  gtsam::Vector z() const;

  template <I = {1, 2}>
  gtsam::Key key() const;
};

#include <gtsam_unstable/dynamics/DynamicsPriors.h>
virtual class DHeightPrior : gtsam::NonlinearFactor {
  DHeightPrior(gtsam::Key key, double height, const gtsam::noiseModel::Base* model);
};

virtual class DRollPrior : gtsam::NonlinearFactor {
  /** allows for explicit roll parameterization - uses canonical coordinate */
  DRollPrior(gtsam::Key key, double wx, const gtsam::noiseModel::Base* model);
  /** Forces roll to zero */
  DRollPrior(gtsam::Key key, const gtsam::noiseModel::Base* model);
};

virtual class VelocityPrior : gtsam::NonlinearFactor {
  VelocityPrior(gtsam::Key key, gtsam::Vector vel, const gtsam::noiseModel::Base* model);
};

virtual class DGroundConstraint : gtsam::NonlinearFactor {
  // Primary constructor allows for variable height of the "floor"
  DGroundConstraint(gtsam::Key key, double height, const gtsam::noiseModel::Base* model);
  // Fully specify vector - use only for debugging
  DGroundConstraint(gtsam::Key key, gtsam::Vector constraint, const gtsam::noiseModel::Base* model);
};

#include <gtsam_unstable/dynamics/VelocityConstraint3.h>
virtual class VelocityConstraint3 : gtsam::NonlinearFactor {
  /** Standard constructor */
  VelocityConstraint3(gtsam::Key key1, gtsam::Key key2, gtsam::Key velKey, double dt);

  gtsam::Vector evaluateError(const double& x1, const double& x2, const double& v) const;
};

#include <gtsam_unstable/dynamics/Pendulum.h>
virtual class PendulumFactor1 : gtsam::NonlinearFactor {
  /** Standard constructor */
  PendulumFactor1(gtsam::Key k1, gtsam::Key k, gtsam::Key velKey, double dt);

  gtsam::Vector evaluateError(const double& qk1, const double& qk, const double& v) const;
};

#include <gtsam_unstable/dynamics/Pendulum.h>
virtual class PendulumFactor2 : gtsam::NonlinearFactor {
  /** Standard constructor */
  PendulumFactor2(gtsam::Key vk1, gtsam::Key vk, gtsam::Key qKey, double dt, double L, double g);

  gtsam::Vector evaluateError(const double& vk1, const double& vk, const double& q) const;
};

virtual class PendulumFactorPk : gtsam::NonlinearFactor {
  /** Standard constructor */
  PendulumFactorPk(gtsam::Key pk, gtsam::Key qk, gtsam::Key qk1, double h, double m, double r, double g, double alpha);

  gtsam::Vector evaluateError(const double& pk, const double& qk, const double& qk1) const;
};

virtual class PendulumFactorPk1 : gtsam::NonlinearFactor {
  /** Standard constructor */
  PendulumFactorPk1(gtsam::Key pk1, gtsam::Key qk, gtsam::Key qk1, double h, double m, double r, double g, double alpha);

  gtsam::Vector evaluateError(const double& pk1, const double& qk, const double& qk1) const;
};

#include <gtsam_unstable/dynamics/SimpleHelicopter.h>
virtual class Reconstruction : gtsam::NoiseModelFactor {
  Reconstruction(gtsam::Key gKey1, gtsam::Key gKey, gtsam::Key xiKey, double h);

  gtsam::Vector evaluateError(const gtsam::Pose3& gK1, const gtsam::Pose3& gK, gtsam::Vector xiK) const;
};

virtual class DiscreteEulerPoincareHelicopter : gtsam::NoiseModelFactor {
  DiscreteEulerPoincareHelicopter(gtsam::Key xiKey, gtsam::Key xiKey_1, gtsam::Key gKey,
      double h, gtsam::Matrix Inertia, gtsam::Vector Fu, double m);

  gtsam::Vector evaluateError(gtsam::Vector xiK, gtsam::Vector xiK_1, const gtsam::Pose3& gK) const;
};

//*************************************************************************
// nonlinear
//*************************************************************************
#include <gtsam/nonlinear/FixedLagSmoother.h>

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
  RelativeElevationFactor(gtsam::Key poseKey, gtsam::Key pointKey, double measured,
      const gtsam::noiseModel::Base* model);

  double measured() const;
  //void print(string s) const;
};

#include <gtsam_unstable/slam/DummyFactor.h>
virtual class DummyFactor : gtsam::NonlinearFactor {
  DummyFactor(gtsam::Key key1, size_t dim1, gtsam::Key key2, size_t dim2);
};

#include <gtsam_unstable/slam/InvDepthFactorVariant1.h>
virtual class InvDepthFactorVariant1 : gtsam::NoiseModelFactor {
  InvDepthFactorVariant1(gtsam::Key poseKey, gtsam::Key landmarkKey, const gtsam::Point2& measured, const gtsam::Cal3_S2* K, const gtsam::noiseModel::Base* model);
};

#include <gtsam_unstable/slam/InvDepthFactorVariant2.h>
virtual class InvDepthFactorVariant2 : gtsam::NoiseModelFactor {
  InvDepthFactorVariant2(gtsam::Key poseKey, gtsam::Key landmarkKey, const gtsam::Point2& measured, const gtsam::Cal3_S2* K, const gtsam::Point3& referencePoint, const gtsam::noiseModel::Base* model);
};

#include <gtsam_unstable/slam/InvDepthFactorVariant3.h>
virtual class InvDepthFactorVariant3a : gtsam::NoiseModelFactor {
  InvDepthFactorVariant3a(gtsam::Key poseKey, gtsam::Key landmarkKey, const gtsam::Point2& measured, const gtsam::Cal3_S2* K, const gtsam::noiseModel::Base* model);
};
virtual class InvDepthFactorVariant3b : gtsam::NoiseModelFactor {
  InvDepthFactorVariant3b(gtsam::Key poseKey1, gtsam::Key poseKey2, gtsam::Key landmarkKey, const gtsam::Point2& measured, const gtsam::Cal3_S2* K, const gtsam::noiseModel::Base* model);
};


#include <gtsam_unstable/slam/Mechanization_bRn2.h>
class Mechanization_bRn2 {
  Mechanization_bRn2();
  Mechanization_bRn2(gtsam::Rot3& initial_bRn, gtsam::Vector initial_x_g,
      gtsam::Vector initial_x_a);
  gtsam::Vector b_g(double g_e);
  gtsam::Rot3 bRn();
  gtsam::Vector x_g();
  gtsam::Vector x_a();
  static gtsam::Mechanization_bRn2 initialize(gtsam::Matrix U, gtsam::Matrix F, double g_e);
  gtsam::Mechanization_bRn2 correct(gtsam::Vector dx) const;
  gtsam::Mechanization_bRn2 integrate(gtsam::Vector u, double dt) const;
  //void print(string s) const;
};

#include <gtsam_unstable/slam/AHRS.h>
class AHRS {
  AHRS(gtsam::Matrix U, gtsam::Matrix F, double g_e);
  pair<gtsam::Mechanization_bRn2, gtsam::GaussianDensity*> initialize(double g_e);
  pair<gtsam::Mechanization_bRn2, gtsam::GaussianDensity*> integrate(const gtsam::Mechanization_bRn2& mech, gtsam::GaussianDensity* state, gtsam::Vector u, double dt);
  pair<gtsam::Mechanization_bRn2, gtsam::GaussianDensity*> aid(const gtsam::Mechanization_bRn2& mech, gtsam::GaussianDensity* state, gtsam::Vector f, bool Farrel);
  pair<gtsam::Mechanization_bRn2, gtsam::GaussianDensity*> aidGeneral(const gtsam::Mechanization_bRn2& mech, gtsam::GaussianDensity* state, gtsam::Vector f, gtsam::Vector f_expected, const gtsam::Rot3& increment);
  //void print(string s) const;
};

#include <gtsam_unstable/slam/PartialPriorFactor.h>
template <T = {gtsam::Pose2, gtsam::Pose3}>
virtual class PartialPriorFactor : gtsam::NoiseModelFactor {
  PartialPriorFactor(gtsam::Key key, size_t idx, double prior,
                     const gtsam::noiseModel::Base* model);
  PartialPriorFactor(gtsam::Key key, const std::vector<size_t>& indices,
                     const gtsam::Vector& prior,
                     const gtsam::noiseModel::Base* model);

  // enabling serialization functionality
  void serialize() const;

  const gtsam::Vector& prior();
};

// Tectonic SAM Factors

#include <gtsam_unstable/slam/TSAMFactors.h>
//typedef gtsam::NoiseModelFactorN<gtsam::Pose2, gtsam::Point2> NLPosePose;
virtual class DeltaFactor : gtsam::NoiseModelFactor {
  DeltaFactor(gtsam::Key i, gtsam::Key j, const gtsam::Point2& measured,
      const gtsam::noiseModel::Base* noiseModel);
  //void print(string s) const;
};

//typedef gtsam::NoiseModelFactorN<gtsam::Pose2, gtsam::Pose2, gtsam::Pose2,
//    gtsam::Point2> NLPosePosePosePoint;
virtual class DeltaFactorBase : gtsam::NoiseModelFactor {
  DeltaFactorBase(gtsam::Key b1, gtsam::Key i, gtsam::Key b2, gtsam::Key j,
      const gtsam::Point2& measured, const gtsam::noiseModel::Base* noiseModel);
  //void print(string s) const;
};

//typedef gtsam::NoiseModelFactorN<gtsam::Pose2, gtsam::Pose2, gtsam::Pose2,
//    gtsam::Pose2> NLPosePosePosePose;
virtual class OdometryFactorBase : gtsam::NoiseModelFactor {
  OdometryFactorBase(gtsam::Key b1, gtsam::Key i, gtsam::Key b2, gtsam::Key j,
      const gtsam::Pose2& measured, const gtsam::noiseModel::Base* noiseModel);
  //void print(string s) const;
};

#include <gtsam/geometry/Cal3DS2.h>
#include <gtsam_unstable/slam/ProjectionFactorPPP.h>
#include <gtsam/geometry/Cal3Fisheye.h>
template<POSE, LANDMARK, CALIBRATION>
virtual class ProjectionFactorPPP : gtsam::NoiseModelFactor {
  ProjectionFactorPPP(const gtsam::Point2& measured, const gtsam::noiseModel::Base* noiseModel,
    gtsam::Key poseKey, gtsam::Key transformKey, gtsam::Key pointKey, const CALIBRATION* k);

  ProjectionFactorPPP(const gtsam::Point2& measured, const gtsam::noiseModel::Base* noiseModel,
      gtsam::Key poseKey, gtsam::Key transformKey, gtsam::Key pointKey, const CALIBRATION* k, bool throwCheirality, bool verboseCheirality);

  gtsam::Point2 measured() const;
  CALIBRATION* calibration() const;
  bool verboseCheirality() const;
  bool throwCheirality() const;

  // enabling serialization functionality
  void serialize() const;
};
typedef gtsam::ProjectionFactorPPP<gtsam::Pose3, gtsam::Point3, gtsam::Cal3_S2> ProjectionFactorPPPCal3_S2;
typedef gtsam::ProjectionFactorPPP<gtsam::Pose3, gtsam::Point3, gtsam::Cal3DS2> ProjectionFactorPPPCal3DS2;
typedef gtsam::ProjectionFactorPPP<gtsam::Pose3, gtsam::Point3, gtsam::Cal3Fisheye> ProjectionFactorPPPCal3Fisheye;

#include <gtsam_unstable/slam/ProjectionFactorPPPC.h>
template<POSE, LANDMARK, CALIBRATION>
virtual class ProjectionFactorPPPC : gtsam::NoiseModelFactor {
  ProjectionFactorPPPC(const gtsam::Point2& measured, const gtsam::noiseModel::Base* noiseModel,
    gtsam::Key poseKey, gtsam::Key transformKey, gtsam::Key pointKey, gtsam::Key calibKey);

  ProjectionFactorPPPC(const gtsam::Point2& measured, const gtsam::noiseModel::Base* noiseModel,
      gtsam::Key poseKey, gtsam::Key transformKey, gtsam::Key pointKey, gtsam::Key calibKey, bool throwCheirality, bool verboseCheirality);

  gtsam::Point2 measured() const;
  bool verboseCheirality() const;
  bool throwCheirality() const;

  // enabling serialization functionality
  void serialize() const;
};
typedef gtsam::ProjectionFactorPPPC<gtsam::Pose3, gtsam::Point3, gtsam::Cal3_S2> ProjectionFactorPPPCCal3_S2;
typedef gtsam::ProjectionFactorPPPC<gtsam::Pose3, gtsam::Point3, gtsam::Cal3DS2> ProjectionFactorPPPCCal3DS2;
typedef gtsam::ProjectionFactorPPPC<gtsam::Pose3, gtsam::Point3, gtsam::Cal3Fisheye> ProjectionFactorPPPCCal3Fisheye;

#include <gtsam_unstable/slam/ProjectionFactorRollingShutter.h>
virtual class ProjectionFactorRollingShutter : gtsam::NoiseModelFactor {
  ProjectionFactorRollingShutter(const gtsam::Point2& measured, double alpha, const gtsam::noiseModel::Base* noiseModel,
      gtsam::Key poseKey_a, gtsam::Key poseKey_b, gtsam::Key pointKey, const gtsam::Cal3_S2* K);

  ProjectionFactorRollingShutter(const gtsam::Point2& measured, double alpha, const gtsam::noiseModel::Base* noiseModel,
    gtsam::Key poseKey_a, gtsam::Key poseKey_b, gtsam::Key pointKey, const gtsam::Cal3_S2* K, gtsam::Pose3& body_P_sensor);

  ProjectionFactorRollingShutter(const gtsam::Point2& measured, double alpha, const gtsam::noiseModel::Base* noiseModel,
        gtsam::Key poseKey_a, gtsam::Key poseKey_b, gtsam::Key pointKey, const gtsam::Cal3_S2* K, bool throwCheirality,
        bool verboseCheirality);

  ProjectionFactorRollingShutter(const gtsam::Point2& measured, double alpha, const gtsam::noiseModel::Base* noiseModel,
      gtsam::Key poseKey_a, gtsam::Key poseKey_b, gtsam::Key pointKey, const gtsam::Cal3_S2* K, bool throwCheirality,
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
