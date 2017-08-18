namespace gtsam {

#include <gtsam/inference/Key.h>
typedef size_t Key;

#include <gtsam/base/FastVector.h>
template<T> class FastVector {
  FastVector();
  FastVector(const This& f);
  void push_back(const T& e);
  //T& operator[](int);
  T at(int i);
  size_t size() const;
};

typedef gtsam::FastVector<gtsam::Key> KeyVector;

//*************************************************************************
// geometry
//*************************************************************************

#include <gtsam/geometry/Point2.h>
class Point2 {
  // Standard Constructors
  Point2();
  Point2(double x, double y);
  Point2(Vector v);
  //Point2(const gtsam::Point2& l);

  // Testable
  void print(string s) const;
  bool equals(const gtsam::Point2& pose, double tol) const;

  // Group
  static gtsam::Point2 identity();

  // Standard Interface
  double x() const;
  double y() const;
  Vector vector() const;
  double distance(const gtsam::Point2& p2) const;
  double norm() const;

  // enabling serialization functionality
  void serialize() const;
};

#include <gtsam/geometry/Point3.h>
class Point3 {
  // Standard Constructors
  Point3();
  Point3(double x, double y, double z);
  Point3(Vector v);
  //Point3(const gtsam::Point3& l);

  // Testable
  void print(string s) const;
  bool equals(const gtsam::Point3& p, double tol) const;

  // Group
  static gtsam::Point3 identity();

  // Standard Interface
  Vector vector() const;
  double x() const;
  double y() const;
  double z() const;

  // enabling serialization functionality
  void serialize() const;
};

#include <gtsam/geometry/Rot2.h>
class Rot2 {
  // Standard Constructors and Named Constructors
  Rot2();
  Rot2(double theta);
  //Rot2(const gtsam::Rot2& l);

  static gtsam::Rot2 fromAngle(double theta);
  static gtsam::Rot2 fromDegrees(double theta);
  static gtsam::Rot2 fromCosSin(double c, double s);

  // Testable
  void print(string s) const;
  bool equals(const gtsam::Rot2& rot, double tol) const;

  // Group
  static gtsam::Rot2 identity();
  gtsam::Rot2 inverse();
  gtsam::Rot2 compose(const gtsam::Rot2& p2) const;
  gtsam::Rot2 between(const gtsam::Rot2& p2) const;

  // Manifold
  gtsam::Rot2 retract(Vector v) const;
  Vector localCoordinates(const gtsam::Rot2& p) const;

  // Lie Group
  static gtsam::Rot2 Expmap(Vector v);
  static Vector Logmap(const gtsam::Rot2& p);

  // Group Action on Point2
  gtsam::Point2 rotate(const gtsam::Point2& point) const;
  gtsam::Point2 unrotate(const gtsam::Point2& point) const;

  // Standard Interface
  static gtsam::Rot2 relativeBearing(const gtsam::Point2& d); // Ignoring derivative
  static gtsam::Rot2 atan2(double y, double x);
  double theta() const;
  double degrees() const;
  double c() const;
  double s() const;
  Matrix matrix() const;

  // enabling serialization functionality
  void serialize() const;
};

#include <gtsam/geometry/Rot3.h>
class Rot3 {
  // Standard Constructors and Named Constructors
  Rot3();
  Rot3(Matrix R);
  //Rot3(const gtsam::Rot3& l);

  static gtsam::Rot3 Rx(double t);
  static gtsam::Rot3 Ry(double t);
  static gtsam::Rot3 Rz(double t);
  static gtsam::Rot3 RzRyRx(double x, double y, double z);
  static gtsam::Rot3 RzRyRx(Vector xyz);
  static gtsam::Rot3 Yaw(double t); // positive yaw is to right (as in aircraft heading)
  static gtsam::Rot3 Pitch(double t); // positive pitch is up (increasing aircraft altitude)
  static gtsam::Rot3 Roll(double t); // positive roll is to right (increasing yaw in aircraft)
  static gtsam::Rot3 Ypr(double y, double p, double r);
  static gtsam::Rot3 Quaternion(double w, double x, double y, double z);
  static gtsam::Rot3 Rodrigues(Vector v);

  // Testable
  void print(string s) const;
  bool equals(const gtsam::Rot3& rot, double tol) const;

  // Group
  static gtsam::Rot3 identity();
    gtsam::Rot3 inverse() const;
  gtsam::Rot3 compose(const gtsam::Rot3& p2) const;
  gtsam::Rot3 between(const gtsam::Rot3& p2) const;

  // Manifold
  //gtsam::Rot3 retractCayley(Vector v) const; // FIXME, does not exist in both Matrix and Quaternion options
  gtsam::Rot3 retract(Vector v) const;
  Vector localCoordinates(const gtsam::Rot3& p) const;

  // Group Action on Point3
  gtsam::Point3 rotate(const gtsam::Point3& p) const;
  gtsam::Point3 unrotate(const gtsam::Point3& p) const;

  // Standard Interface
  static gtsam::Rot3 Expmap(Vector v);
  static Vector Logmap(const gtsam::Rot3& p);
  Matrix matrix() const;
  Matrix transpose() const;
  gtsam::Point3 column(size_t index) const;
  Vector xyz() const;
  Vector ypr() const;
  Vector rpy() const;
  double roll() const;
  double pitch() const;
  double yaw() const;
//  Vector toQuaternion() const;  // FIXME: Can't cast to Vector properly
  Vector quaternion() const;

  // enabling serialization functionality
  void serialize() const;
};

#include <gtsam/geometry/Pose2.h>
class Pose2 {
  // Standard Constructor
  Pose2();
  //Pose2(const gtsam::Pose2& pose);
  Pose2(double x, double y, double theta);
  Pose2(double theta, const gtsam::Point2& t);
  Pose2(const gtsam::Rot2& r, const gtsam::Point2& t);
  Pose2(Vector v);

  // Testable
  void print(string s) const;
  bool equals(const gtsam::Pose2& pose, double tol) const;

  // Group
  static gtsam::Pose2 identity();
  gtsam::Pose2 inverse() const;
  gtsam::Pose2 compose(const gtsam::Pose2& p2) const;
  gtsam::Pose2 between(const gtsam::Pose2& p2) const;

  // Manifold
  gtsam::Pose2 retract(Vector v) const;
  Vector localCoordinates(const gtsam::Pose2& p) const;

  // Lie Group
  static gtsam::Pose2 Expmap(Vector v);
  static Vector Logmap(const gtsam::Pose2& p);
  Matrix AdjointMap() const;
  Vector Adjoint(const Vector& xi) const;
  static Matrix wedge(double vx, double vy, double w);

  // Group Actions on Point2
  gtsam::Point2 transform_from(const gtsam::Point2& p) const;
  gtsam::Point2 transform_to(const gtsam::Point2& p) const;

  // Standard Interface
  double x() const;
  double y() const;
  double theta() const;
  gtsam::Rot2 bearing(const gtsam::Point2& point) const;
  double range(const gtsam::Point2& point) const;
  gtsam::Point2 translation() const;
  gtsam::Rot2 rotation() const;
  Matrix matrix() const;

  // enabling serialization functionality
  void serialize() const;
};


#include <gtsam/geometry/Pose3.h>
class Pose3 {
  // Standard Constructors
  Pose3();
  //Pose3(const gtsam::Pose3& pose);
  Pose3(const gtsam::Rot3& r, const gtsam::Point3& t);
  Pose3(const gtsam::Pose2& pose2); // FIXME: shadows Pose3(Pose3 pose)
  Pose3(Matrix t);

  // Testable
  void print(string s) const;
  bool equals(const gtsam::Pose3& pose, double tol) const;

  // Group
  static gtsam::Pose3 identity();
  gtsam::Pose3 inverse() const;
  gtsam::Pose3 compose(const gtsam::Pose3& p2) const;
  gtsam::Pose3 between(const gtsam::Pose3& p2) const;

  // Manifold
  gtsam::Pose3 retract(Vector v) const;
  Vector localCoordinates(const gtsam::Pose3& T2) const;

  // Lie Group
  static gtsam::Pose3 Expmap(Vector v);
  static Vector Logmap(const gtsam::Pose3& p);
  Matrix AdjointMap() const;
  Vector Adjoint(Vector xi) const;
  static Matrix wedge(double wx, double wy, double wz, double vx, double vy, double vz);

  // Group Action on Point3
  gtsam::Point3 transform_from(const gtsam::Point3& p) const;
  gtsam::Point3 transform_to(const gtsam::Point3& p) const;

  // Standard Interface
  gtsam::Rot3 rotation() const;
  gtsam::Point3 translation() const;
  double x() const;
  double y() const;
  double z() const;
  Matrix matrix() const;
  gtsam::Pose3 transform_to(const gtsam::Pose3& pose) const; // FIXME: shadows other transform_to()
  double range(const gtsam::Point3& point);
  double range(const gtsam::Pose3& pose);

  // enabling serialization functionality
  void serialize() const;
};

//*************************************************************************
// noise
//*************************************************************************

namespace noiseModel {
#include <gtsam/linear/NoiseModel.h>
virtual class Base {
};

virtual class Gaussian : gtsam::noiseModel::Base {
  static gtsam::noiseModel::Gaussian* SqrtInformation(Matrix R);
  static gtsam::noiseModel::Gaussian* Covariance(Matrix R);
  Matrix R() const;
  bool equals(gtsam::noiseModel::Base& expected, double tol);
  void print(string s) const;

  // enabling serialization functionality
  void serializable() const;
};

virtual class Diagonal : gtsam::noiseModel::Gaussian {
  static gtsam::noiseModel::Diagonal* Sigmas(Vector sigmas);
  static gtsam::noiseModel::Diagonal* Variances(Vector variances);
  static gtsam::noiseModel::Diagonal* Precisions(Vector precisions);
  Matrix R() const;
  void print(string s) const;

  // enabling serialization functionality
  void serializable() const;
};

virtual class Constrained : gtsam::noiseModel::Diagonal {
    static gtsam::noiseModel::Constrained* MixedSigmas(const Vector& mu, const Vector& sigmas);
    static gtsam::noiseModel::Constrained* MixedSigmas(double m, const Vector& sigmas);
    static gtsam::noiseModel::Constrained* MixedVariances(const Vector& mu, const Vector& variances);
    static gtsam::noiseModel::Constrained* MixedVariances(const Vector& variances);
    static gtsam::noiseModel::Constrained* MixedPrecisions(const Vector& mu, const Vector& precisions);
    static gtsam::noiseModel::Constrained* MixedPrecisions(const Vector& precisions);

    static gtsam::noiseModel::Constrained* All(size_t dim);
    static gtsam::noiseModel::Constrained* All(size_t dim, double mu);

    gtsam::noiseModel::Constrained* unit() const;

    // enabling serialization functionality
    void serializable() const;
};

virtual class Isotropic : gtsam::noiseModel::Diagonal {
  static gtsam::noiseModel::Isotropic* Sigma(size_t dim, double sigma);
  static gtsam::noiseModel::Isotropic* Variance(size_t dim, double varianace);
  static gtsam::noiseModel::Isotropic* Precision(size_t dim, double precision);
  void print(string s) const;

  // enabling serialization functionality
  void serializable() const;
};

virtual class Unit : gtsam::noiseModel::Isotropic {
  static gtsam::noiseModel::Unit* Create(size_t dim);
  void print(string s) const;

  // enabling serialization functionality
  void serializable() const;
};

namespace mEstimator {
virtual class Base {
};

virtual class Null: gtsam::noiseModel::mEstimator::Base {
  Null();
  //Null(const gtsam::noiseModel::mEstimator::Null& other);
  void print(string s) const;
  static gtsam::noiseModel::mEstimator::Null* Create();

  // enabling serialization functionality
  void serializable() const;
};

virtual class Fair: gtsam::noiseModel::mEstimator::Base {
  Fair(double c);
  //Fair(const gtsam::noiseModel::mEstimator::Fair& other);
  void print(string s) const;
  static gtsam::noiseModel::mEstimator::Fair* Create(double c);

  // enabling serialization functionality
  void serializable() const;
};

virtual class Huber: gtsam::noiseModel::mEstimator::Base {
  Huber(double k);
  //Huber(const gtsam::noiseModel::mEstimator::Huber& other);

  void print(string s) const;
  static gtsam::noiseModel::mEstimator::Huber* Create(double k);

  // enabling serialization functionality
  void serializable() const;
};

virtual class Tukey: gtsam::noiseModel::mEstimator::Base {
  Tukey(double k);
  //Tukey(const gtsam::noiseModel::mEstimator::Tukey& other);

  void print(string s) const;
  static gtsam::noiseModel::mEstimator::Tukey* Create(double k);

  // enabling serialization functionality
  void serializable() const;
};

}///\namespace mEstimator

virtual class Robust : gtsam::noiseModel::Base {
  Robust(const gtsam::noiseModel::mEstimator::Base* robust, const gtsam::noiseModel::Base* noise);
  //Robust(const gtsam::noiseModel::Robust& other);

  static gtsam::noiseModel::Robust* Create(const gtsam::noiseModel::mEstimator::Base* robust, const gtsam::noiseModel::Base* noise);
  void print(string s) const;

  // enabling serialization functionality
  void serializable() const;
};

}///\namespace noiseModel

#include <gtsam/linear/Sampler.h>
class Sampler {
    //Constructors
  Sampler(gtsam::noiseModel::Diagonal* model, int seed);
  Sampler(Vector sigmas, int seed);
  Sampler(int seed);
  //Sampler(const gtsam::Sampler& other);


    //Standard Interface
  size_t dim() const;
  Vector sigmas() const;
  gtsam::noiseModel::Diagonal* model() const;
  Vector sample();
  Vector sampleNewModel(gtsam::noiseModel::Diagonal* model);
};

#include <gtsam/linear/VectorValues.h>
class VectorValues {
    //Constructors
  VectorValues();
  VectorValues(const gtsam::VectorValues& other);

  //Named Constructors
  static gtsam::VectorValues Zero(const gtsam::VectorValues& model);

  //Standard Interface
  size_t size() const;
  size_t dim(size_t j) const;
  bool exists(size_t j) const;
  void print(string s) const;
  bool equals(const gtsam::VectorValues& expected, double tol) const;
  void insert(size_t j, Vector value);
  Vector vector() const;
  Vector at(size_t j) const;
  void update(const gtsam::VectorValues& values);

  //Advanced Interface
  void setZero();

  gtsam::VectorValues add(const gtsam::VectorValues& c) const;
  void addInPlace(const gtsam::VectorValues& c);
  gtsam::VectorValues subtract(const gtsam::VectorValues& c) const;
  gtsam::VectorValues scale(double a) const;
  void scaleInPlace(double a);

  bool hasSameStructure(const gtsam::VectorValues& other)  const;
  double dot(const gtsam::VectorValues& V) const;
  double norm() const;
  double squaredNorm() const;

  // enabling serialization functionality
  void serialize() const;
};

#include <gtsam/linear/GaussianFactor.h>
virtual class GaussianFactor {
  gtsam::KeyVector keys() const;
  void print(string s) const;
  bool equals(const gtsam::GaussianFactor& lf, double tol) const;
  double error(const gtsam::VectorValues& c) const;
  gtsam::GaussianFactor* clone() const;
  gtsam::GaussianFactor* negate() const;
  Matrix augmentedInformation() const;
  Matrix information() const;
  Matrix augmentedJacobian() const;
  pair<Matrix, Vector> jacobian() const;
  size_t size() const;
  bool empty() const;
};

#include <gtsam/linear/JacobianFactor.h>
virtual class JacobianFactor : gtsam::GaussianFactor {
  //Constructors
  JacobianFactor();
  JacobianFactor(const gtsam::GaussianFactor& factor);
  JacobianFactor(Vector b_in);
  JacobianFactor(size_t i1, Matrix A1, Vector b,
      const gtsam::noiseModel::Diagonal* model);
  JacobianFactor(size_t i1, Matrix A1, size_t i2, Matrix A2, Vector b,
      const gtsam::noiseModel::Diagonal* model);
  JacobianFactor(size_t i1, Matrix A1, size_t i2, Matrix A2, size_t i3, Matrix A3,
      Vector b, const gtsam::noiseModel::Diagonal* model);
  //JacobianFactor(const gtsam::GaussianFactorGraph& graph);
  //JacobianFactor(const gtsam::JacobianFactor& other);

  //Testable
  void print(string s) const;
  void printKeys(string s) const;
  bool equals(const gtsam::GaussianFactor& lf, double tol) const;
  size_t size() const;
  Vector unweighted_error(const gtsam::VectorValues& c) const;
  Vector error_vector(const gtsam::VectorValues& c) const;
  double error(const gtsam::VectorValues& c) const;

  //Standard Interface
  Matrix getA() const;
  Vector getb() const;
  size_t rows() const;
  size_t cols() const;
  bool isConstrained() const;
  pair<Matrix, Vector> jacobianUnweighted() const;
  Matrix augmentedJacobianUnweighted() const;

  void transposeMultiplyAdd(double alpha, const Vector& e, gtsam::VectorValues& x) const;
  gtsam::JacobianFactor whiten() const;

  //pair<gtsam::GaussianConditional*, gtsam::JacobianFactor*> eliminate(const gtsam::Ordering& keys) const;

  void setModel(bool anyConstrained, const Vector& sigmas);

  gtsam::noiseModel::Diagonal* get_model() const;

  // enabling serialization functionality
  void serialize() const;
};

#include <gtsam/linear/HessianFactor.h>
virtual class HessianFactor : gtsam::GaussianFactor {
  //Constructors
  HessianFactor();
  HessianFactor(const gtsam::GaussianFactor& factor);
  HessianFactor(size_t j, Matrix G, Vector g, double f);
  HessianFactor(size_t j, Vector mu, Matrix Sigma);
  HessianFactor(size_t j1, size_t j2, Matrix G11, Matrix G12, Vector g1, Matrix G22,
      Vector g2, double f);
  HessianFactor(size_t j1, size_t j2, size_t j3, Matrix G11, Matrix G12, Matrix G13,
      Vector g1, Matrix G22, Matrix G23, Vector g2, Matrix G33, Vector g3,
      double f);
  //HessianFactor(const gtsam::GaussianFactorGraph& factors);
  //HessianFactor(const gtsam::HessianFactor& other);

  //Testable
  size_t size() const;
  void print(string s) const;
  void printKeys(string s) const;
  bool equals(const gtsam::GaussianFactor& lf, double tol) const;
  double error(const gtsam::VectorValues& c) const;

  //Standard Interface
  size_t rows() const;
  Matrix information() const;
  double constantTerm() const;
  Vector linearTerm() const;

  // enabling serialization functionality
  void serialize() const;
};

#include <gtsam/nonlinear/Values.h>
class Values {
  Values();
  //Values(const gtsam::Values& other);

  size_t size() const;
  bool empty() const;
  void clear();
  size_t dim() const;

  void print(string s) const;
  bool equals(const gtsam::Values& other, double tol) const;

  void insert(const gtsam::Values& values);
  void update(const gtsam::Values& values);
  void erase(size_t j);
  void swap(gtsam::Values& values);

  bool exists(size_t j) const;
  gtsam::KeyVector keys() const;

  gtsam::VectorValues zeroVectors() const;

  gtsam::Values retract(const gtsam::VectorValues& delta) const;
  gtsam::VectorValues localCoordinates(const gtsam::Values& cp) const;

  // enabling serialization functionality
  void serialize() const;

  // New in 4.0, we have to specialize every insert/update/at to generate wrappers
  // Instead of the old:
  // void insert(size_t j, const gtsam::Value& value);
  // void update(size_t j, const gtsam::Value& val);
  // gtsam::Value at(size_t j) const;

  // template <T = {gtsam::Point2, gtsam::Rot2, gtsam::Pose2, gtsam::Point3,
  //                gtsam::Rot3, gtsam::Pose3}>
  // void insert(size_t j, const T& t);

  // template <T = {gtsam::Point2, gtsam::Rot2, gtsam::Pose2, gtsam::Point3,
  //                gtsam::Rot3, gtsam::Pose3}>
  // void update(size_t j, const T& t);
  void insert(size_t j, const gtsam::Point2& t);
  void insert(size_t j, const gtsam::Point3& t);
  void insert(size_t j, const gtsam::Rot2& t);
  void insert(size_t j, const gtsam::Pose2& t);
  void insert(size_t j, const gtsam::Rot3& t);
  void insert(size_t j, const gtsam::Pose3& t);
  void insert(size_t j, Vector t);
  void insert(size_t j, Matrix t);

  void update(size_t j, const gtsam::Point2& t);
  void update(size_t j, const gtsam::Point3& t);
  void update(size_t j, const gtsam::Rot2& t);
  void update(size_t j, const gtsam::Pose2& t);
  void update(size_t j, const gtsam::Rot3& t);
  void update(size_t j, const gtsam::Pose3& t);
  void update(size_t j, Vector t);
  void update(size_t j, Matrix t);

  template <T = {gtsam::Point2, gtsam::Rot2, gtsam::Pose2, gtsam::Point3,
                 gtsam::Rot3, gtsam::Pose3, Vector, Matrix}>
  T at(size_t j);

  /// version for double
  void insertDouble(size_t j, double c);
  double atDouble(size_t j) const;
};

#include <gtsam/nonlinear/NonlinearFactor.h>
virtual class NonlinearFactor {
  // Factor base class
  size_t size() const;
  gtsam::KeyVector keys() const;
  void print(string s) const;
  void printKeys(string s) const;
  // NonlinearFactor
  bool equals(const gtsam::NonlinearFactor& other, double tol) const;

  double error(const gtsam::Values& c) const;
  size_t dim() const;
  bool active(const gtsam::Values& c) const;
  gtsam::GaussianFactor* linearize(const gtsam::Values& c) const;
  gtsam::NonlinearFactor* clone() const;
  // gtsam::NonlinearFactor* rekey(const gtsam::KeyVector& newKeys) const; //FIXME: Conversion from KeyVector to std::vector does not happen
};

#include <gtsam/nonlinear/NonlinearFactorGraph.h>
class NonlinearFactorGraph {
  NonlinearFactorGraph();
  //NonlinearFactorGraph(const gtsam::NonlinearFactorGraph& graph);

  // FactorGraph
  void print(string s) const;
  bool equals(const gtsam::NonlinearFactorGraph& fg, double tol) const;
  size_t size() const;
  bool empty() const;
  void remove(size_t i);
  size_t nrFactors() const;
  gtsam::NonlinearFactor* at(size_t idx) const;
  void push_back(const gtsam::NonlinearFactorGraph& factors);
  void push_back(gtsam::NonlinearFactor* factor);
  void add(gtsam::NonlinearFactor* factor);
  bool exists(size_t idx) const;
  // gtsam::KeySet keys() const;

  // NonlinearFactorGraph
  double error(const gtsam::Values& values) const;
  double probPrime(const gtsam::Values& values) const;
  //gtsam::Ordering orderingCOLAMD() const;
  // Ordering* orderingCOLAMDConstrained(const gtsam::Values& c, const std::map<gtsam::Key,int>& constraints) const;
  //gtsam::GaussianFactorGraph* linearize(const gtsam::Values& values) const;
  gtsam::NonlinearFactorGraph clone() const;

  // enabling serialization functionality
  void serialize() const;
};

#include <gtsam/nonlinear/NonlinearFactor.h>
virtual class NoiseModelFactor: gtsam::NonlinearFactor {
  void equals(const gtsam::NoiseModelFactor& other, double tol) const;
  gtsam::noiseModel::Base* get_noiseModel() const; // deprecated by below
  gtsam::noiseModel::Base* noiseModel() const;
  Vector unwhitenedError(const gtsam::Values& x) const;
  Vector whitenedError(const gtsam::Values& x) const;
};

#include <gtsam/slam/PriorFactor.h>
template<T = {Vector, gtsam::Point2, gtsam::Point3, gtsam::Rot2, gtsam::Rot3, gtsam::Pose2, gtsam::Pose3}>
virtual class PriorFactor : gtsam::NoiseModelFactor {
  PriorFactor(size_t key, const T& prior, const gtsam::noiseModel::Base* noiseModel);
  //PriorFactor(const This& other);
  T prior() const;

  // enabling serialization functionality
  void serialize() const;
};


#include <gtsam/slam/BetweenFactor.h>
template<T = {Vector, gtsam::Point2, gtsam::Point3, gtsam::Rot2, gtsam::Rot3, gtsam::Pose2, gtsam::Pose3}>
virtual class BetweenFactor : gtsam::NoiseModelFactor {
  BetweenFactor(size_t key1, size_t key2, const T& relativePose, const gtsam::noiseModel::Base* noiseModel);
  //BetweenFactor(const This& other);
  T measured() const;

  // enabling serialization functionality
  void serialize() const;
};

#include <gtsam/inference/Symbol.h>
size_t symbol(char chr, size_t index);
char symbolChr(size_t key);
size_t symbolIndex(size_t key);

#include <gtsam/inference/Key.h>
// Default keyformatter
void PrintKeyVector(const gtsam::KeyVector& keys);
void PrintKeyVector(const gtsam::KeyVector& keys, string s);

#include <gtsam/nonlinear/NonlinearOptimizer.h>
bool checkConvergence(double relativeErrorTreshold,
    double absoluteErrorTreshold, double errorThreshold,
    double currentError, double newError);

#include <gtsam/slam/dataset.h>
pair<gtsam::NonlinearFactorGraph*, gtsam::Values*> load2D(string filename,
    gtsam::noiseModel::Diagonal* model, int maxID, bool addNoise, bool smart);
pair<gtsam::NonlinearFactorGraph*, gtsam::Values*> load2D(string filename,
    gtsam::noiseModel::Diagonal* model, int maxID, bool addNoise);
pair<gtsam::NonlinearFactorGraph*, gtsam::Values*> load2D(string filename,
    gtsam::noiseModel::Diagonal* model, int maxID);
pair<gtsam::NonlinearFactorGraph*, gtsam::Values*> load2D(string filename,
    gtsam::noiseModel::Diagonal* model);
pair<gtsam::NonlinearFactorGraph*, gtsam::Values*> load2D(string filename);
pair<gtsam::NonlinearFactorGraph*, gtsam::Values*> load2D_robust(string filename,
    gtsam::noiseModel::Base* model);
void save2D(const gtsam::NonlinearFactorGraph& graph,
    const gtsam::Values& config, gtsam::noiseModel::Diagonal* model,
    string filename);

pair<gtsam::NonlinearFactorGraph*, gtsam::Values*> readG2o(string filename);
void writeG2o(const gtsam::NonlinearFactorGraph& graph,
    const gtsam::Values& estimate, string filename);

//*************************************************************************
// Utilities
//*************************************************************************

namespace utilities {

  #include <gtsam/nonlinear/utilities.h>
  // gtsam::KeyList createKeyList(Vector I);
  // gtsam::KeyList createKeyList(string s, Vector I);
  gtsam::KeyVector createKeyVector(Vector I);
  gtsam::KeyVector createKeyVector(string s, Vector I);
  // gtsam::KeySet createKeySet(Vector I);
  // gtsam::KeySet createKeySet(string s, Vector I);
  Matrix extractPoint2(const gtsam::Values& values);
  Matrix extractPoint3(const gtsam::Values& values);
  Matrix extractPose2(const gtsam::Values& values);
  gtsam::Values allPose3s(gtsam::Values& values);
  Matrix extractPose3(const gtsam::Values& values);
  void perturbPoint2(gtsam::Values& values, double sigma, int seed);
  void perturbPose2 (gtsam::Values& values, double sigmaT, double sigmaR, int seed);
  void perturbPoint3(gtsam::Values& values, double sigma, int seed);
  // void insertBackprojections(gtsam::Values& values, const gtsam::SimpleCamera& c, Vector J, Matrix Z, double depth);
  // void insertProjectionFactors(gtsam::NonlinearFactorGraph& graph, size_t i, Vector J, Matrix Z, const gtsam::noiseModel::Base* model, const gtsam::Cal3_S2* K);
  // void insertProjectionFactors(gtsam::NonlinearFactorGraph& graph, size_t i, Vector J, Matrix Z, const gtsam::noiseModel::Base* model, const gtsam::Cal3_S2* K, const gtsam::Pose3& body_P_sensor);
  Matrix reprojectionErrors(const gtsam::NonlinearFactorGraph& graph, const gtsam::Values& values);
  gtsam::Values localToWorld(const gtsam::Values& local, const gtsam::Pose2& base);
  gtsam::Values localToWorld(const gtsam::Values& local, const gtsam::Pose2& base, const gtsam::KeyVector& keys);

} //\namespace utilities

#include <gtsam/nonlinear/utilities.h>
class RedirectCout {
  RedirectCout();
  string str();
};

} //\namespace gtsam
