namespace gtsam {

#include <gtsam/base/FastVector.h>
template<T> class FastVector{};
typedef gtsam::FastVector<Point3> KeyVector;

#include <gtsam/base/FastList.h>
template<T> class FastList{};
typedef gtsam::FastList<size_t> KeyList;

#include <gtsam/base/FastSet.h>
template<T> class FastSet{};
typedef gtsam::FastSet<size_t> KeySet;

#include <gtsam/base/FastMap.h>
template<K,V> class FastMap{};

#include <gtsam/geometry/Point3.h>
class Point3 {
  // Standard Constructors
  Point3();
  Point3(double x, double y, double z);
  Point3(Vector v);

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

#include <gtsam/geometry/Rot3.h>
class Rot3 {
  // Standard Constructors and Named Constructors
  Rot3();
  Rot3(Matrix R);
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

namespace mEstimator {
virtual class Base {
};

virtual class Null: gtsam::noiseModel::mEstimator::Base {
  Null();
  void print(string s) const;
  static gtsam::noiseModel::mEstimator::Null* Create();

  // enabling serialization functionality
  void serializable() const;
};

virtual class Fair: gtsam::noiseModel::mEstimator::Base {
  Fair(double c);
  void print(string s) const;
  static gtsam::noiseModel::mEstimator::Fair* Create(double c);

  // enabling serialization functionality
  void serializable() const;
};

virtual class Huber: gtsam::noiseModel::mEstimator::Base {
  Huber(double k);
  void print(string s) const;
  static gtsam::noiseModel::mEstimator::Huber* Create(double k);

  // enabling serialization functionality
  void serializable() const;
};
} // namespace mEstimator

virtual class Robust : gtsam::noiseModel::Base {
  Robust(const gtsam::noiseModel::mEstimator::Base* robust, const gtsam::noiseModel::Base* noise);
  static gtsam::noiseModel::Robust* Create(const gtsam::noiseModel::mEstimator::Base* robust, const gtsam::noiseModel::Base* noise);
  void print(string s) const;

  // enabling serialization functionality
  void serializable() const;
};
} // namespace noiseModel

#include <gtsam/linear/GaussianFactor.h>
virtual class GaussianFactor {
  // gtsam::KeyVector keys() const;
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
  JacobianFactor(const gtsam::GaussianFactorGraph& graph);

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

  // pair<gtsam::GaussianConditional*, gtsam::JacobianFactor*> eliminate(const gtsam::Ordering& keys) const;

  void setModel(bool anyConstrained, const Vector& sigmas);

  gtsam::noiseModel::Diagonal* get_model() const;

  // enabling serialization functionality
  void serialize() const;
};

#include <gtsam/nonlinear/Values.h>
class Values {
  Values();
  Values(const gtsam::Values& other);

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
  // gtsam::KeyVector keys() const;

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

  template<T = {gtsam::Point3, gtsam::Rot3, Vector, Matrix}>
  void insert(size_t j, const T& t);

  template<T = {gtsam::Point3, gtsam::Rot3, Vector, Matrix}>
  void update(size_t j, const T& t);

  template<T = {gtsam::Point3, gtsam::Rot3, Vector, Matrix}>
  T at(size_t j);

  /// version for double
  void insertDouble(size_t j, double c);
  double atDouble(size_t j) const;
};

#include <gtsam/nonlinear/NonlinearFactor.h>
virtual class NonlinearFactor {
  // Factor base class
  size_t size() const;
  // gtsam::KeyVector keys() const;
  void print(string s) const;
  void printKeys(string s) const;
  // NonlinearFactor
  void equals(const gtsam::NonlinearFactor& other, double tol) const;
  double error(const gtsam::Values& c) const;
  size_t dim() const;
  bool active(const gtsam::Values& c) const;
  // gtsam::GaussianFactor* linearize(const gtsam::Values& c) const;
  gtsam::NonlinearFactor* clone() const;
  // gtsam::NonlinearFactor* rekey(const gtsam::KeyVector& newKeys) const; //FIXME: Conversion from KeyVector to std::vector does not happen
};


#include <gtsam/nonlinear/NonlinearFactorGraph.h>
class NonlinearFactorGraph {
  NonlinearFactorGraph();
  NonlinearFactorGraph(const gtsam::NonlinearFactorGraph& graph);

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
  // gtsam::Ordering orderingCOLAMD() const;
  // // Ordering* orderingCOLAMDConstrained(const gtsam::Values& c, const std::map<gtsam::Key,int>& constraints) const;
  // gtsam::GaussianFactorGraph* linearize(const gtsam::Values& values) const;
  gtsam::NonlinearFactorGraph clone() const;

  // enabling serialization functionality
  void serialize() const;
};


virtual class NoiseModelFactor: gtsam::NonlinearFactor {
  void equals(const gtsam::NoiseModelFactor& other, double tol) const;
  gtsam::noiseModel::Base* get_noiseModel() const; // deprecated by below
  gtsam::noiseModel::Base* noiseModel() const;
  Vector unwhitenedError(const gtsam::Values& x) const;
  Vector whitenedError(const gtsam::Values& x) const;
};

#include <gtsam/slam/BetweenFactor.h>
template<T = {gtsam::Point3, gtsam::Rot3}>
virtual class BetweenFactor : gtsam::NoiseModelFactor {
  BetweenFactor(size_t key1, size_t key2, const T& relativePose, const gtsam::noiseModel::Base* noiseModel);
  T measured() const;

  // enabling serialization functionality
  void serialize() const;
};

#include <gtsam/sam/BearingFactor.h>
template<POSE, POINT, BEARING>
virtual class BearingFactor : gtsam::NoiseModelFactor {
  BearingFactor(size_t key1, size_t key2, const BEARING& measured, const gtsam::noiseModel::Base* noiseModel);

  // enabling serialization functionality
  void serialize() const;
};

typedef gtsam::BearingFactor<gtsam::Pose2, gtsam::Point2, gtsam::Rot2> BearingFactor2D;


}