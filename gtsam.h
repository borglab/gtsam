/**

 * GTSAM Wrap Module Definition
 *
 * These are the current classes available through the matlab and python wrappers,
 * add more functions/classes as they are available.
 *
 * IMPORTANT: the python wrapper supports keyword arguments for functions/methods. Hence, the
 *            argument names matter. An implementation restriction is that in overloaded methods
 *            or functions, arguments of different types *have* to have different names.
 *
 * Requirements:
 *   Classes must start with an uppercase letter
 *      - Can wrap a typedef
 *   Only one Method/Constructor per line, though methods/constructors can extend across multiple lines
 *   Methods can return
 *     - Eigen types:       Matrix, Vector
 *     - C/C++ basic types: string, bool, size_t, int, double, char, unsigned char
 *     - void
 *     - Any class with which be copied with boost::make_shared()
 *     - boost::shared_ptr of any object type
 *   Constructors
 *     - Overloads are supported, but arguments of different types *have* to have different names
 *     - A class with no constructors can be returned from other functions but not allocated directly in MATLAB
 *   Methods
 *     - Constness has no effect
 *     - Specify by-value (not reference) return types, even if C++ method returns reference
 *     - Must start with a letter (upper or lowercase)
 *     - Overloads are supported
 *   Static methods
 *     - Must start with a letter (upper or lowercase) and use the "static" keyword
 *     - The first letter will be made uppercase in the generated MATLAB interface
 *     - Overloads are supported, but arguments of different types *have* to have different names
 *   Arguments to functions any of
 *      - Eigen types:       Matrix, Vector
 *      - Eigen types and classes as an optionally const reference
 *     - C/C++ basic types: string, bool, size_t, size_t, double, char, unsigned char
 *     - Any class with which be copied with boost::make_shared() (except Eigen)
 *     - boost::shared_ptr of any object type (except Eigen)
 *   Comments can use either C++ or C style, with multiple lines
 *   Namespace definitions
 *     - Names of namespaces must start with a lowercase letter
 *      - start a namespace with "namespace {"
 *      - end a namespace with exactly "}"
 *      - Namespaces can be nested
 *   Namespace usage
 *      - Namespaces can be specified for classes in arguments and return values
 *      - In each case, the namespace must be fully specified, e.g., "namespace1::namespace2::ClassName"
 *   Includes in C++ wrappers
 *     - All includes will be collected and added in a single file
 *     - All namespaces must have angle brackets: <path>
 *     - No default includes will be added
 *   Global/Namespace functions
 *     - Functions specified outside of a class are global
 *     - Can be overloaded with different arguments
 *     - Can have multiple functions of the same name in different namespaces
 *   Using classes defined in other modules
 *     - If you are using a class 'OtherClass' not wrapped in this definition file, add "class OtherClass;" to avoid a dependency error
 *   Virtual inheritance
 *     - Specify fully-qualified base classes, i.e. "virtual class Derived : ns::Base {" where "ns" is the namespace
 *     - Mark with 'virtual' keyword, e.g. "virtual class Base {", and also "virtual class Derived : ns::Base {"
 *     - Forward declarations must also be marked virtual, e.g. "virtual class ns::Base;" and
 *       also "virtual class ns::Derived;"
 *     - Pure virtual (abstract) classes should list no constructors in this interface file
 *     - Virtual classes must have a clone() function in C++ (though it does not have to be included
 *       in the MATLAB interface).  clone() will be called whenever an object copy is needed, instead
 *       of using the copy constructor (which is used for non-virtual objects).
 *     - Signature of clone function - will be called virtually, so must appear at least at the top of the inheritance tree
 *           virtual boost::shared_ptr<CLASS_NAME> clone() const;
 *   Class Templates
 *     - Basic templates are supported either with an explicit list of types to instantiate,
 *       e.g. template<T = {gtsam::Pose2, gtsam::Rot2, gtsam::Point3}> class Class1 { ... };
 *       or with typedefs, e.g.
 *       template<T, U> class Class2 { ... };
 *       typedef Class2<Type1, Type2> MyInstantiatedClass;
 *     - In the class definition, appearances of the template argument(s) will be replaced with their
 *       instantiated types, e.g. 'void setValue(const T& value);'.
 *     - To refer to the instantiation of the template class itself, use 'This', i.e. 'static This Create();'
 *     - To create new instantiations in other modules, you must copy-and-paste the whole class definition
 *       into the new module, but use only your new instantiation types.
 *     - When forward-declaring template instantiations, use the generated/typedefed name, e.g.
 *       class gtsam::Class1Pose2;
 *       class gtsam::MyInstantiatedClass;
 *   Boost.serialization within Matlab:
 *     - you need to mark classes as being serializable in the markup file (see this file for an example).
 *     - There are two options currently, depending on the class.  To "mark" a class as serializable,
 *       add a function with a particular signature so that wrap will catch it.
 *        - Add "void serialize()" to a class to create serialization functions for a class.
 *          Adding this flag subsumes the serializable() flag below. Requirements:
 *             - A default constructor must be publicly accessible
 *             - Must not be an abstract base class
 *             - The class must have an actual boost.serialization serialize() function.
 *        - Add "void serializable()" to a class if you only want the class to be serialized as a
 *          part of a container (such as noisemodel). This version does not require a publicly
 *          accessible default constructor.
 *   Forward declarations and class definitions for Cython:
 *     - Need to specify the base class (both this forward class and base class are declared in an external cython header)
 *       This is so Cython can generate proper inheritance.
 *       Example when wrapping a gtsam-based project:
 *          // forward declarations
 *          virtual class gtsam::NonlinearFactor
 *          virtual class gtsam::NoiseModelFactor : gtsam::NonlinearFactor
 *          // class definition
 *          #include <MyFactor.h>
 *          virtual class MyFactor : gtsam::NoiseModelFactor {...};
 *    - *DO NOT* re-define overriden function already declared in the external (forward-declared) base class
 *        - This will cause an ambiguity problem in Cython pxd header file
 */

/**
 * Status:
 *  - TODO: default values for arguments
 *    - WORKAROUND: make multiple versions of the same function for different configurations of default arguments
 *  - TODO: Handle gtsam::Rot3M conversions to quaternions
 *  - TODO: Parse return of const ref arguments
 *  - TODO: Parse std::string variants and convert directly to special string
 *  - TODO: Add enum support
 *  - TODO: Add generalized serialization support via boost.serialization with hooks to matlab save/load
 */

namespace gtsam {

// Actually a FastList<Key>
#include <gtsam/inference/Key.h>
class KeyList {
  KeyList();
  KeyList(const gtsam::KeyList& other);

  // Note: no print function

  // common STL methods
  size_t size() const;
  bool empty() const;
  void clear();

  // structure specific methods
  size_t front() const;
  size_t back() const;
  void push_back(size_t key);
  void push_front(size_t key);
  void pop_back();
  void pop_front();
  void sort();
  void remove(size_t key);

  void serialize() const;
};

// Actually a FastSet<Key>
class KeySet {
  KeySet();
  KeySet(const gtsam::KeySet& set);
  KeySet(const gtsam::KeyVector& vector);
  KeySet(const gtsam::KeyList& list);

  // Testable
  void print(string s) const;
  bool equals(const gtsam::KeySet& other) const;

  // common STL methods
  size_t size() const;
  bool empty() const;
  void clear();

  // structure specific methods
  void insert(size_t key);
  void merge(const gtsam::KeySet& other);
  bool erase(size_t key); // returns true if value was removed
  bool count(size_t key) const; // returns true if value exists

  void serialize() const;
};

// Actually a vector<Key>
class KeyVector {
  KeyVector();
  KeyVector(const gtsam::KeyVector& other);

  // Note: no print function

  // common STL methods
  size_t size() const;
  bool empty() const;
  void clear();

  // structure specific methods
  size_t at(size_t i) const;
  size_t front() const;
  size_t back() const;
  void push_back(size_t key) const;

  void serialize() const;
};

// Actually a FastMap<Key,int>
class KeyGroupMap {
  KeyGroupMap();

  // Note: no print function

  // common STL methods
  size_t size() const;
  bool empty() const;
  void clear();

  // structure specific methods
  size_t at(size_t key) const;
  int erase(size_t key);
  bool insert2(size_t key, int val);
};

// Actually a FastSet<FactorIndex>
class FactorIndexSet {
  FactorIndexSet();
  FactorIndexSet(const gtsam::FactorIndexSet& set);

  // common STL methods
  size_t size() const;
  bool empty() const;
  void clear();

  // structure specific methods
  void insert(size_t factorIndex);
  bool erase(size_t factorIndex); // returns true if value was removed
  bool count(size_t factorIndex) const; // returns true if value exists
};

// Actually a vector<FactorIndex>
class FactorIndices {
  FactorIndices();
  FactorIndices(const gtsam::FactorIndices& other);

  // common STL methods
  size_t size() const;
  bool empty() const;
  void clear();

  // structure specific methods
  size_t at(size_t i) const;
  size_t front() const;
  size_t back() const;
  void push_back(size_t factorIndex) const;
};

//*************************************************************************
// base
//*************************************************************************

/** gtsam namespace functions */

#include <gtsam/base/debug.h>
bool isDebugVersion();

#include <gtsam/base/DSFMap.h>
class IndexPair {
  IndexPair();
  IndexPair(size_t i, size_t j);
  size_t i() const;
  size_t j() const;
};

template<KEY = {gtsam::IndexPair}>
class DSFMap {
  DSFMap();
  KEY find(const KEY& key) const;
  void merge(const KEY& x, const KEY& y);
};

#include <gtsam/base/Matrix.h>
bool linear_independent(Matrix A, Matrix B, double tol);

#include <gtsam/base/Value.h>
virtual class Value {
  // No constructors because this is an abstract class

  // Testable
  void print(string s) const;

  // Manifold
  size_t dim() const;
};

#include <gtsam/base/GenericValue.h>
template<T = {Vector, Matrix, gtsam::Point2, gtsam::Point3, gtsam::Rot2, gtsam::Rot3, gtsam::Pose2, gtsam::Pose3, gtsam::StereoPoint2, gtsam::Cal3_S2, gtsam::Cal3DS2, gtsam::Cal3Bundler, gtsam::EssentialMatrix, gtsam::CalibratedCamera, gtsam::SimpleCamera, gtsam::imuBias::ConstantBias}>
virtual class GenericValue : gtsam::Value {
  void serializable() const;
};

#include <gtsam/base/deprecated/LieScalar.h>
class LieScalar {
  // Standard constructors
  LieScalar();
  LieScalar(double d);

  // Standard interface
  double value() const;

  // Testable
  void print(string s) const;
  bool equals(const gtsam::LieScalar& expected, double tol) const;

  // Group
  static gtsam::LieScalar identity();
  gtsam::LieScalar inverse() const;
  gtsam::LieScalar compose(const gtsam::LieScalar& p) const;
  gtsam::LieScalar between(const gtsam::LieScalar& l2) const;

  // Manifold
  size_t dim() const;
  gtsam::LieScalar retract(Vector v) const;
  Vector localCoordinates(const gtsam::LieScalar& t2) const;

  // Lie group
  static gtsam::LieScalar Expmap(Vector v);
  static Vector Logmap(const gtsam::LieScalar& p);
};

#include <gtsam/base/deprecated/LieVector.h>
class LieVector {
  // Standard constructors
  LieVector();
  LieVector(Vector v);

  // Standard interface
  Vector vector() const;

  // Testable
  void print(string s) const;
  bool equals(const gtsam::LieVector& expected, double tol) const;

  // Group
  static gtsam::LieVector identity();
  gtsam::LieVector inverse() const;
  gtsam::LieVector compose(const gtsam::LieVector& p) const;
  gtsam::LieVector between(const gtsam::LieVector& l2) const;

  // Manifold
  size_t dim() const;
  gtsam::LieVector retract(Vector v) const;
  Vector localCoordinates(const gtsam::LieVector& t2) const;

  // Lie group
  static gtsam::LieVector Expmap(Vector v);
  static Vector Logmap(const gtsam::LieVector& p);

  // enabling serialization functionality
  void serialize() const;
};

#include <gtsam/base/deprecated/LieMatrix.h>
class LieMatrix {
  // Standard constructors
  LieMatrix();
  LieMatrix(Matrix v);

  // Standard interface
  Matrix matrix() const;

  // Testable
  void print(string s) const;
  bool equals(const gtsam::LieMatrix& expected, double tol) const;

  // Group
  static gtsam::LieMatrix identity();
  gtsam::LieMatrix inverse() const;
  gtsam::LieMatrix compose(const gtsam::LieMatrix& p) const;
  gtsam::LieMatrix between(const gtsam::LieMatrix& l2) const;

  // Manifold
  size_t dim() const;
  gtsam::LieMatrix retract(Vector v) const;
  Vector localCoordinates(const gtsam::LieMatrix & t2) const;

  // Lie group
  static gtsam::LieMatrix Expmap(Vector v);
  static Vector Logmap(const gtsam::LieMatrix& p);

  // enabling serialization functionality
  void serialize() const;
};

//*************************************************************************
// geometry
//*************************************************************************

#include <gtsam/geometry/Point2.h>
class Point2 {
  // Standard Constructors
  Point2();
  Point2(double x, double y);
  Point2(Vector v);

  // Testable
  void print(string s) const;
  bool equals(const gtsam::Point2& point, double tol) const;

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

// std::vector<gtsam::Point2>
#include <gtsam/geometry/Point2.h>
class Point2Vector
{
  // Constructors
  Point2Vector();
  Point2Vector(const gtsam::Point2Vector& v);

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

#include <gtsam/geometry/StereoPoint2.h>
class StereoPoint2 {
  // Standard Constructors
  StereoPoint2();
  StereoPoint2(double uL, double uR, double v);

  // Testable
  void print(string s) const;
  bool equals(const gtsam::StereoPoint2& point, double tol) const;

  // Group
  static gtsam::StereoPoint2 identity();
  gtsam::StereoPoint2 inverse() const;
  gtsam::StereoPoint2 compose(const gtsam::StereoPoint2& p2) const;
  gtsam::StereoPoint2 between(const gtsam::StereoPoint2& p2) const;

  // Manifold
  gtsam::StereoPoint2 retract(Vector v) const;
  Vector localCoordinates(const gtsam::StereoPoint2& p) const;

  // Lie Group
  static gtsam::StereoPoint2 Expmap(Vector v);
  static Vector Logmap(const gtsam::StereoPoint2& p);

  // Standard Interface
  Vector vector() const;
  double uL() const;
  double uR() const;
  double v() const;

  // enabling serialization functionality
  void serialize() const;
};

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

#include <gtsam/geometry/Rot2.h>
class Rot2 {
  // Standard Constructors and Named Constructors
  Rot2();
  Rot2(double theta);
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

#include <gtsam/geometry/SO3.h>
class SO3 {
  // Standard Constructors
  SO3();
  SO3(Matrix R);
  static gtsam::SO3 FromMatrix(Matrix R);
  static gtsam::SO3 AxisAngle(const Vector axis, double theta);
  static gtsam::SO3 ClosestTo(const Matrix M);

  // Testable
  void print(string s) const;
  bool equals(const gtsam::SO3& other, double tol) const;

  // Group
  static gtsam::SO3 identity();
  gtsam::SO3 inverse() const;
  gtsam::SO3 between(const gtsam::SO3& R) const;
  gtsam::SO3 compose(const gtsam::SO3& R) const;

  // Manifold
  gtsam::SO3 retract(Vector v) const;
  Vector localCoordinates(const gtsam::SO3& R) const;
  static gtsam::SO3 Expmap(Vector v);

  // Other methods
  Vector vec() const;
  Matrix matrix() const;
};

#include <gtsam/geometry/SO4.h>
class SO4 {
  // Standard Constructors
  SO4();
  SO4(Matrix R);
  static gtsam::SO4 FromMatrix(Matrix R);

  // Testable
  void print(string s) const;
  bool equals(const gtsam::SO4& other, double tol) const;

  // Group
  static gtsam::SO4 identity();
  gtsam::SO4 inverse() const;
  gtsam::SO4 between(const gtsam::SO4& Q) const;
  gtsam::SO4 compose(const gtsam::SO4& Q) const;

  // Manifold
  gtsam::SO4 retract(Vector v) const;
  Vector localCoordinates(const gtsam::SO4& Q) const;
  static gtsam::SO4 Expmap(Vector v);

  // Other methods
  Vector vec() const;
  Matrix matrix() const;
};

#include <gtsam/geometry/SOn.h>
class SOn {
  // Standard Constructors
  SOn(size_t n);
  static gtsam::SOn FromMatrix(Matrix R);
  static gtsam::SOn Lift(size_t n, Matrix R);

  // Testable
  void print(string s) const;
  bool equals(const gtsam::SOn& other, double tol) const;

  // Group
  static gtsam::SOn identity();
  gtsam::SOn inverse() const;
  gtsam::SOn between(const gtsam::SOn& Q) const;
  gtsam::SOn compose(const gtsam::SOn& Q) const;

  // Manifold
  gtsam::SOn retract(Vector v) const;
  Vector localCoordinates(const gtsam::SOn& Q) const;
  static gtsam::SOn Expmap(Vector v);

  // Other methods
  Vector vec() const;
  Matrix matrix() const;
};

#include <gtsam/geometry/Rot3.h>
class Rot3 {
  // Standard Constructors and Named Constructors
  Rot3();
  Rot3(Matrix R);
  Rot3(const gtsam::Point3& col1, const gtsam::Point3& col2, const gtsam::Point3& col3);
  Rot3(double R11, double R12, double R13,
      double R21, double R22, double R23,
      double R31, double R32, double R33);

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
  static gtsam::Rot3 AxisAngle(const gtsam::Point3& axis, double angle);
  static gtsam::Rot3 Rodrigues(Vector v);
  static gtsam::Rot3 Rodrigues(double wx, double wy, double wz);
  static gtsam::Rot3 ClosestTo(const Matrix M);

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
  pair<gtsam::Unit3, double> axisAngle() const;
//  Vector toQuaternion() const;  // FIXME: Can't cast to Vector properly
  Vector quaternion() const;
  gtsam::Rot3 slerp(double t, const gtsam::Rot3& other) const;

  // enabling serialization functionality
  void serialize() const;
};

#include <gtsam/geometry/Pose2.h>
class Pose2 {
  // Standard Constructor
  Pose2();
  Pose2(const gtsam::Pose2& other);
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
  static Matrix ExpmapDerivative(Vector v);
  static Matrix LogmapDerivative(const gtsam::Pose2& v);
  Matrix AdjointMap() const;
  Vector Adjoint(Vector xi) const;
  static Matrix adjointMap_(Vector v);
  static Vector adjoint_(Vector xi, Vector y);
  static Vector adjointTranspose(Vector xi, Vector y);
  static Matrix wedge(double vx, double vy, double w);

  // Group Actions on Point2
  gtsam::Point2 transformFrom(const gtsam::Point2& p) const;
  gtsam::Point2 transformTo(const gtsam::Point2& p) const;

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
  Pose3(const gtsam::Pose3& other);
  Pose3(const gtsam::Rot3& r, const gtsam::Point3& t);
  Pose3(const gtsam::Pose2& pose2); // FIXME: shadows Pose3(Pose3 pose)
  Pose3(Matrix mat);

  // Testable
  void print(string s) const;
  bool equals(const gtsam::Pose3& pose, double tol) const;

  // Group
  static gtsam::Pose3 identity();
  gtsam::Pose3 inverse() const;
  gtsam::Pose3 compose(const gtsam::Pose3& pose) const;
  gtsam::Pose3 between(const gtsam::Pose3& pose) const;

  // Manifold
  gtsam::Pose3 retract(Vector v) const;
  Vector localCoordinates(const gtsam::Pose3& pose) const;

  // Lie Group
  static gtsam::Pose3 Expmap(Vector v);
  static Vector Logmap(const gtsam::Pose3& pose);
  Matrix AdjointMap() const;
  Vector Adjoint(Vector xi) const;
  static Matrix adjointMap_(Vector xi);
  static Vector adjoint_(Vector xi, Vector y);
  static Vector adjointTranspose(Vector xi, Vector y);
  static Matrix ExpmapDerivative(Vector xi);
  static Matrix LogmapDerivative(const gtsam::Pose3& xi);
  static Matrix wedge(double wx, double wy, double wz, double vx, double vy, double vz);

  // Group Action on Point3
  gtsam::Point3 transformFrom(const gtsam::Point3& point) const;
  gtsam::Point3 transformTo(const gtsam::Point3& point) const;

  // Standard Interface
  gtsam::Rot3 rotation() const;
  gtsam::Point3 translation() const;
  double x() const;
  double y() const;
  double z() const;
  Matrix matrix() const;
  gtsam::Pose3 transformPoseFrom(const gtsam::Pose3& pose) const;
  gtsam::Pose3 transformPoseTo(const gtsam::Pose3& pose) const;
  double range(const gtsam::Point3& point);
  double range(const gtsam::Pose3& pose);

  // enabling serialization functionality
  void serialize() const;
};

// std::vector<gtsam::Pose3>
#include <gtsam/geometry/Pose3.h>
class Pose3Vector
{
  Pose3Vector();
  size_t size() const;
  bool empty() const;
  gtsam::Pose3 at(size_t n) const;
  void push_back(const gtsam::Pose3& pose);
};

#include <gtsam/geometry/Unit3.h>
class Unit3 {
  // Standard Constructors
  Unit3();
  Unit3(const gtsam::Point3& pose);

  // Testable
  void print(string s) const;
  bool equals(const gtsam::Unit3& pose, double tol) const;

  // Other functionality
  Matrix basis() const;
  Matrix skew() const;
  gtsam::Point3 point3() const;

  // Manifold
  static size_t Dim();
  size_t dim() const;
  gtsam::Unit3 retract(Vector v) const;
  Vector localCoordinates(const gtsam::Unit3& s) const;
};

#include <gtsam/geometry/EssentialMatrix.h>
class EssentialMatrix {
  // Standard Constructors
  EssentialMatrix(const gtsam::Rot3& aRb, const gtsam::Unit3& aTb);

  // Testable
  void print(string s) const;
  bool equals(const gtsam::EssentialMatrix& pose, double tol) const;

  // Manifold
  static size_t Dim();
  size_t dim() const;
  gtsam::EssentialMatrix retract(Vector v) const;
  Vector localCoordinates(const gtsam::EssentialMatrix& s) const;

  // Other methods:
  gtsam::Rot3 rotation() const;
  gtsam::Unit3 direction() const;
  Matrix matrix() const;
  double error(Vector vA, Vector vB);
};

#include <gtsam/geometry/Cal3_S2.h>
class Cal3_S2 {
  // Standard Constructors
  Cal3_S2();
  Cal3_S2(double fx, double fy, double s, double u0, double v0);
  Cal3_S2(Vector v);
  Cal3_S2(double fov, int w, int h);

  // Testable
  void print(string s) const;
  bool equals(const gtsam::Cal3_S2& rhs, double tol) const;

  // Manifold
  static size_t Dim();
  size_t dim() const;
  gtsam::Cal3_S2 retract(Vector v) const;
  Vector localCoordinates(const gtsam::Cal3_S2& c) const;

  // Action on Point2
  gtsam::Point2 calibrate(const gtsam::Point2& p) const;
  gtsam::Point2 uncalibrate(const gtsam::Point2& p) const;

  // Standard Interface
  double fx() const;
  double fy() const;
  double skew() const;
  double px() const;
  double py() const;
  gtsam::Point2 principalPoint() const;
  Vector vector() const;
  Matrix K() const;
  Matrix matrix() const;
  Matrix matrix_inverse() const;

  // enabling serialization functionality
  void serialize() const;
};

#include <gtsam/geometry/Cal3DS2_Base.h>
virtual class Cal3DS2_Base {
  // Standard Constructors
  Cal3DS2_Base();

  // Testable
  void print(string s) const;

  // Standard Interface
  double fx() const;
  double fy() const;
  double skew() const;
  double px() const;
  double py() const;
  double k1() const;
  double k2() const;
  Matrix K() const;
  Vector k() const;
  Vector vector() const;

  // Action on Point2
  gtsam::Point2 uncalibrate(const gtsam::Point2& p) const;
  gtsam::Point2 calibrate(const gtsam::Point2& p, double tol) const;

  // enabling serialization functionality
  void serialize() const;
};

#include <gtsam/geometry/Cal3DS2.h>
virtual class Cal3DS2 : gtsam::Cal3DS2_Base {
  // Standard Constructors
  Cal3DS2();
  Cal3DS2(double fx, double fy, double s, double u0, double v0, double k1, double k2);
  Cal3DS2(double fx, double fy, double s, double u0, double v0, double k1, double k2, double p1, double p2);
  Cal3DS2(Vector v);

  // Testable
  bool equals(const gtsam::Cal3DS2& rhs, double tol) const;

  // Manifold
  size_t dim() const;
  static size_t Dim();
  gtsam::Cal3DS2 retract(Vector v) const;
  Vector localCoordinates(const gtsam::Cal3DS2& c) const;

  // enabling serialization functionality
  void serialize() const;
};

#include <gtsam/geometry/Cal3Unified.h>
virtual class Cal3Unified : gtsam::Cal3DS2_Base {
  // Standard Constructors
  Cal3Unified();
  Cal3Unified(double fx, double fy, double s, double u0, double v0, double k1, double k2);
  Cal3Unified(double fx, double fy, double s, double u0, double v0, double k1, double k2, double p1, double p2, double xi);
  Cal3Unified(Vector v);

  // Testable
  bool equals(const gtsam::Cal3Unified& rhs, double tol) const;

  // Standard Interface
  double xi() const;
  gtsam::Point2 spaceToNPlane(const gtsam::Point2& p) const;
  gtsam::Point2 nPlaneToSpace(const gtsam::Point2& p) const;

  // Manifold
  size_t dim() const;
  static size_t Dim();
  gtsam::Cal3Unified retract(Vector v) const;
  Vector localCoordinates(const gtsam::Cal3Unified& c) const;

  // enabling serialization functionality
  void serialize() const;
};

#include <gtsam/geometry/Cal3_S2Stereo.h>
class Cal3_S2Stereo {
  // Standard Constructors
  Cal3_S2Stereo();
  Cal3_S2Stereo(double fx, double fy, double s, double u0, double v0, double b);
  Cal3_S2Stereo(Vector v);

  // Testable
  void print(string s) const;
  bool equals(const gtsam::Cal3_S2Stereo& K, double tol) const;

  // Standard Interface
  double fx() const;
  double fy() const;
  double skew() const;
  double px() const;
  double py() const;
  gtsam::Point2 principalPoint() const;
  double baseline() const;
};

#include <gtsam/geometry/Cal3Bundler.h>
class Cal3Bundler {
  // Standard Constructors
  Cal3Bundler();
  Cal3Bundler(double fx, double k1, double k2, double u0, double v0);

  // Testable
  void print(string s) const;
  bool equals(const gtsam::Cal3Bundler& rhs, double tol) const;

  // Manifold
  static size_t Dim();
  size_t dim() const;
  gtsam::Cal3Bundler retract(Vector v) const;
  Vector localCoordinates(const gtsam::Cal3Bundler& c) const;

  // Action on Point2
  gtsam::Point2 calibrate(const gtsam::Point2& p, double tol) const;
  gtsam::Point2 uncalibrate(const gtsam::Point2& p) const;

  // Standard Interface
  double fx() const;
  double fy() const;
  double k1() const;
  double k2() const;
  double u0() const;
  double v0() const;
  Vector vector() const;
  Vector k() const;
  //Matrix K() const; //FIXME: Uppercase

  // enabling serialization functionality
  void serialize() const;
};

#include <gtsam/geometry/CalibratedCamera.h>
class CalibratedCamera {
  // Standard Constructors and Named Constructors
  CalibratedCamera();
  CalibratedCamera(const gtsam::Pose3& pose);
  CalibratedCamera(Vector v);
  static gtsam::CalibratedCamera Level(const gtsam::Pose2& pose2, double height);

  // Testable
  void print(string s) const;
  bool equals(const gtsam::CalibratedCamera& camera, double tol) const;

  // Manifold
  static size_t Dim();
  size_t dim() const;
  gtsam::CalibratedCamera retract(Vector d) const;
  Vector localCoordinates(const gtsam::CalibratedCamera& T2) const;

  // Action on Point3
  gtsam::Point2 project(const gtsam::Point3& point) const;
  static gtsam::Point2 Project(const gtsam::Point3& cameraPoint);

  // Standard Interface
  gtsam::Pose3 pose() const;
  double range(const gtsam::Point3& p) const; // TODO: Other overloaded range methods

  // enabling serialization functionality
  void serialize() const;
};

#include <gtsam/geometry/PinholeCamera.h>
template<CALIBRATION>
class PinholeCamera {
  // Standard Constructors and Named Constructors
  PinholeCamera();
  PinholeCamera(const gtsam::Pose3& pose);
  PinholeCamera(const gtsam::Pose3& pose, const CALIBRATION& K);
  static This Level(const CALIBRATION& K, const gtsam::Pose2& pose, double height);
  static This Level(const gtsam::Pose2& pose, double height);
  static This Lookat(const gtsam::Point3& eye, const gtsam::Point3& target,
      const gtsam::Point3& upVector, const CALIBRATION& K);

  // Testable
  void print(string s) const;
  bool equals(const This& camera, double tol) const;

  // Standard Interface
  gtsam::Pose3 pose() const;
  CALIBRATION calibration() const;

  // Manifold
  This retract(Vector d) const;
  Vector localCoordinates(const This& T2) const;
  size_t dim() const;
  static size_t Dim();

  // Transformations and measurement functions
  static gtsam::Point2 Project(const gtsam::Point3& cameraPoint);
  pair<gtsam::Point2,bool> projectSafe(const gtsam::Point3& pw) const;
  gtsam::Point2 project(const gtsam::Point3& point);
  gtsam::Point3 backproject(const gtsam::Point2& p, double depth) const;
  double range(const gtsam::Point3& point);
  double range(const gtsam::Pose3& pose);

  // enabling serialization functionality
  void serialize() const;
};

#include <gtsam/geometry/SimpleCamera.h>
virtual class SimpleCamera {
  // Standard Constructors and Named Constructors
  SimpleCamera();
  SimpleCamera(const gtsam::Pose3& pose);
  SimpleCamera(const gtsam::Pose3& pose, const gtsam::Cal3_S2& K);
  static gtsam::SimpleCamera Level(const gtsam::Cal3_S2& K, const gtsam::Pose2& pose, double height);
  static gtsam::SimpleCamera Level(const gtsam::Pose2& pose, double height);
  static gtsam::SimpleCamera Lookat(const gtsam::Point3& eye, const gtsam::Point3& target,
      const gtsam::Point3& upVector, const gtsam::Cal3_S2& K);
  static gtsam::SimpleCamera Lookat(const gtsam::Point3& eye, const gtsam::Point3& target,
      const gtsam::Point3& upVector);

  // Testable
  void print(string s) const;
  bool equals(const gtsam::SimpleCamera& camera, double tol) const;

  // Standard Interface
  gtsam::Pose3 pose() const;
  gtsam::Cal3_S2 calibration() const;

  // Manifold
  gtsam::SimpleCamera retract(Vector d) const;
  Vector localCoordinates(const gtsam::SimpleCamera& T2) const;
  size_t dim() const;
  static size_t Dim();

  // Transformations and measurement functions
  static gtsam::Point2 Project(const gtsam::Point3& cameraPoint);
  pair<gtsam::Point2,bool> projectSafe(const gtsam::Point3& pw) const;
  gtsam::Point2 project(const gtsam::Point3& point);
  gtsam::Point3 backproject(const gtsam::Point2& p, double depth) const;
  double range(const gtsam::Point3& point);
  double range(const gtsam::Pose3& pose);

  // enabling serialization functionality
  void serialize() const;

};

gtsam::SimpleCamera simpleCamera(const Matrix& P);

// Some typedefs for common camera types
// PinholeCameraCal3_S2 is the same as SimpleCamera above
typedef gtsam::PinholeCamera<gtsam::Cal3_S2> PinholeCameraCal3_S2;
// due to lack of jacobians of Cal3DS2_Base::calibrate, PinholeCamera does not apply to Cal3DS2/Unified
//typedef gtsam::PinholeCamera<gtsam::Cal3DS2> PinholeCameraCal3DS2;
//typedef gtsam::PinholeCamera<gtsam::Cal3Unified> PinholeCameraCal3Unified;
//typedef gtsam::PinholeCamera<gtsam::Cal3Bundler> PinholeCameraCal3Bundler;

#include <gtsam/geometry/StereoCamera.h>
class StereoCamera {
  // Standard Constructors and Named Constructors
  StereoCamera();
  StereoCamera(const gtsam::Pose3& pose, const gtsam::Cal3_S2Stereo* K);

  // Testable
  void print(string s) const;
  bool equals(const gtsam::StereoCamera& camera, double tol) const;

  // Standard Interface
  gtsam::Pose3 pose() const;
  double baseline() const;
  gtsam::Cal3_S2Stereo calibration() const;

  // Manifold
  gtsam::StereoCamera retract(Vector d) const;
  Vector localCoordinates(const gtsam::StereoCamera& T2) const;
  size_t dim() const;
  static size_t Dim();

  // Transformations and measurement functions
  gtsam::StereoPoint2 project(const gtsam::Point3& point);
  gtsam::Point3 backproject(const gtsam::StereoPoint2& p) const;

  // enabling serialization functionality
  void serialize() const;
};

#include <gtsam/geometry/triangulation.h>

// Templates appear not yet supported for free functions
gtsam::Point3 triangulatePoint3(const gtsam::Pose3Vector& poses,
    gtsam::Cal3_S2* sharedCal, const gtsam::Point2Vector& measurements,
    double rank_tol, bool optimize);
gtsam::Point3 triangulatePoint3(const gtsam::Pose3Vector& poses,
    gtsam::Cal3DS2* sharedCal, const gtsam::Point2Vector& measurements,
    double rank_tol, bool optimize);
gtsam::Point3 triangulatePoint3(const gtsam::Pose3Vector& poses,
    gtsam::Cal3Bundler* sharedCal, const gtsam::Point2Vector& measurements,
    double rank_tol, bool optimize);

//*************************************************************************
// Symbolic
//*************************************************************************

#include <gtsam/symbolic/SymbolicFactor.h>
virtual class SymbolicFactor {
  // Standard Constructors and Named Constructors
  SymbolicFactor(const gtsam::SymbolicFactor& f);
  SymbolicFactor();
  SymbolicFactor(size_t j);
  SymbolicFactor(size_t j1, size_t j2);
  SymbolicFactor(size_t j1, size_t j2, size_t j3);
  SymbolicFactor(size_t j1, size_t j2, size_t j3, size_t j4);
  SymbolicFactor(size_t j1, size_t j2, size_t j3, size_t j4, size_t j5);
  SymbolicFactor(size_t j1, size_t j2, size_t j3, size_t j4, size_t j5, size_t j6);
  static gtsam::SymbolicFactor FromKeys(const gtsam::KeyVector& js);

  // From Factor
  size_t size() const;
  void print(string s) const;
  bool equals(const gtsam::SymbolicFactor& other, double tol) const;
  gtsam::KeyVector keys();
};

#include <gtsam/symbolic/SymbolicFactorGraph.h>
virtual class SymbolicFactorGraph {
  SymbolicFactorGraph();
  SymbolicFactorGraph(const gtsam::SymbolicBayesNet& bayesNet);
  SymbolicFactorGraph(const gtsam::SymbolicBayesTree& bayesTree);

  // From FactorGraph
  void push_back(gtsam::SymbolicFactor* factor);
  void print(string s) const;
  bool equals(const gtsam::SymbolicFactorGraph& rhs, double tol) const;
  size_t size() const;
  bool exists(size_t idx) const;

  // Standard interface
  gtsam::KeySet keys() const;
  void push_back(const gtsam::SymbolicFactorGraph& graph);
  void push_back(const gtsam::SymbolicBayesNet& bayesNet);
  void push_back(const gtsam::SymbolicBayesTree& bayesTree);

  //Advanced Interface
  void push_factor(size_t key);
  void push_factor(size_t key1, size_t key2);
  void push_factor(size_t key1, size_t key2, size_t key3);
  void push_factor(size_t key1, size_t key2, size_t key3, size_t key4);

  gtsam::SymbolicBayesNet* eliminateSequential();
  gtsam::SymbolicBayesNet* eliminateSequential(const gtsam::Ordering& ordering);
  gtsam::SymbolicBayesTree* eliminateMultifrontal();
  gtsam::SymbolicBayesTree* eliminateMultifrontal(const gtsam::Ordering& ordering);
  pair<gtsam::SymbolicBayesNet*, gtsam::SymbolicFactorGraph*> eliminatePartialSequential(
      const gtsam::Ordering& ordering);
  pair<gtsam::SymbolicBayesNet*, gtsam::SymbolicFactorGraph*> eliminatePartialSequential(
      const gtsam::KeyVector& keys);
  pair<gtsam::SymbolicBayesTree*, gtsam::SymbolicFactorGraph*> eliminatePartialMultifrontal(
      const gtsam::Ordering& ordering);
  pair<gtsam::SymbolicBayesTree*, gtsam::SymbolicFactorGraph*> eliminatePartialMultifrontal(
      const gtsam::KeyVector& keys);
  gtsam::SymbolicBayesNet* marginalMultifrontalBayesNet(const gtsam::Ordering& ordering);
  gtsam::SymbolicBayesNet* marginalMultifrontalBayesNet(const gtsam::KeyVector& key_vector);
  gtsam::SymbolicBayesNet* marginalMultifrontalBayesNet(const gtsam::Ordering& ordering,
      const gtsam::Ordering& marginalizedVariableOrdering);
  gtsam::SymbolicBayesNet* marginalMultifrontalBayesNet(const gtsam::KeyVector& key_vector,
      const gtsam::Ordering& marginalizedVariableOrdering);
  gtsam::SymbolicFactorGraph* marginal(const gtsam::KeyVector& key_vector);
};

#include <gtsam/symbolic/SymbolicConditional.h>
virtual class SymbolicConditional : gtsam::SymbolicFactor {
  // Standard Constructors and Named Constructors
  SymbolicConditional();
  SymbolicConditional(const gtsam::SymbolicConditional& other);
  SymbolicConditional(size_t key);
  SymbolicConditional(size_t key, size_t parent);
  SymbolicConditional(size_t key, size_t parent1, size_t parent2);
  SymbolicConditional(size_t key, size_t parent1, size_t parent2, size_t parent3);
  static gtsam::SymbolicConditional FromKeys(const gtsam::KeyVector& keys, size_t nrFrontals);

  // Testable
  void print(string s) const;
  bool equals(const gtsam::SymbolicConditional& other, double tol) const;

  // Standard interface
  size_t nrFrontals() const;
  size_t nrParents() const;
};

#include <gtsam/symbolic/SymbolicBayesNet.h>
class SymbolicBayesNet {
  SymbolicBayesNet();
  SymbolicBayesNet(const gtsam::SymbolicBayesNet& other);
  // Testable
  void print(string s) const;
  bool equals(const gtsam::SymbolicBayesNet& other, double tol) const;

  // Standard interface
  size_t size() const;
  void saveGraph(string s) const;
  gtsam::SymbolicConditional* at(size_t idx) const;
  gtsam::SymbolicConditional* front() const;
  gtsam::SymbolicConditional* back() const;
  void push_back(gtsam::SymbolicConditional* conditional);
  void push_back(const gtsam::SymbolicBayesNet& bayesNet);
};

#include <gtsam/symbolic/SymbolicBayesTree.h>
class SymbolicBayesTree {

    //Constructors
    SymbolicBayesTree();
    SymbolicBayesTree(const gtsam::SymbolicBayesTree& other);

    // Testable
    void print(string s);
    bool equals(const gtsam::SymbolicBayesTree& other, double tol) const;

    //Standard Interface
    //size_t findParentClique(const gtsam::IndexVector& parents) const;
    size_t size();
    void saveGraph(string s) const;
    void clear();
    void deleteCachedShortcuts();
    size_t numCachedSeparatorMarginals() const;

  gtsam::SymbolicConditional* marginalFactor(size_t key) const;
  gtsam::SymbolicFactorGraph* joint(size_t key1, size_t key2) const;
  gtsam::SymbolicBayesNet* jointBayesNet(size_t key1, size_t key2) const;
};

// class SymbolicBayesTreeClique {
//   BayesTreeClique();
//   BayesTreeClique(CONDITIONAL* conditional);
// //  BayesTreeClique(const pair<typename ConditionalType::shared_ptr, typename ConditionalType::FactorType::shared_ptr>& result) : Base(result) {}
//
//   bool equals(const This& other, double tol) const;
//   void print(string s) const;
//   void printTree() const; // Default indent of ""
//   void printTree(string indent) const;
//   size_t numCachedSeparatorMarginals() const;
//
//   CONDITIONAL* conditional() const;
//   bool isRoot() const;
//   size_t treeSize() const;
// //  const std::list<derived_ptr>& children() const { return children_; }
// //  derived_ptr parent() const { return parent_.lock(); }
//
//   // FIXME: need wrapped versions graphs, BayesNet
// //  BayesNet<ConditionalType> shortcut(derived_ptr root, Eliminate function) const;
// //  FactorGraph<FactorType> marginal(derived_ptr root, Eliminate function) const;
// //  FactorGraph<FactorType> joint(derived_ptr C2, derived_ptr root, Eliminate function) const;
//
//   void deleteCachedShortcuts();
// };

#include <gtsam/inference/VariableIndex.h>
class VariableIndex {
  // Standard Constructors and Named Constructors
  VariableIndex();
  // TODO: Templetize constructor when wrap supports it
  //template<T = {gtsam::FactorGraph}>
  //VariableIndex(const T& factorGraph, size_t nVariables);
  //VariableIndex(const T& factorGraph);
  VariableIndex(const gtsam::SymbolicFactorGraph& sfg);
  VariableIndex(const gtsam::GaussianFactorGraph& gfg);
  VariableIndex(const gtsam::NonlinearFactorGraph& fg);
  VariableIndex(const gtsam::VariableIndex& other);

  // Testable
  bool equals(const gtsam::VariableIndex& other, double tol) const;
  void print(string s) const;

  // Standard interface
  size_t size() const;
  size_t nFactors() const;
  size_t nEntries() const;
};

//*************************************************************************
// linear
//*************************************************************************

namespace noiseModel {
#include <gtsam/linear/NoiseModel.h>
virtual class Base {
  void print(string s) const;
  // Methods below are available for all noise models. However, can't add them
  // because wrap (incorrectly) thinks robust classes derive from this Base as well.
  // bool isConstrained() const;
  // bool isUnit() const;
  // size_t dim() const;
  // Vector sigmas() const;
};

virtual class Gaussian : gtsam::noiseModel::Base {
  static gtsam::noiseModel::Gaussian* Information(Matrix R);
  static gtsam::noiseModel::Gaussian* SqrtInformation(Matrix R);
  static gtsam::noiseModel::Gaussian* Covariance(Matrix R);

  bool equals(gtsam::noiseModel::Base& expected, double tol);

  // access to noise model
  Matrix R() const;
  Matrix information() const;
  Matrix covariance() const;

  // Whitening operations
  Vector whiten(Vector v) const;
  Vector unwhiten(Vector v) const;
  Matrix Whiten(Matrix H) const;

  // enabling serialization functionality
  void serializable() const;
};

virtual class Diagonal : gtsam::noiseModel::Gaussian {
  static gtsam::noiseModel::Diagonal* Sigmas(Vector sigmas);
  static gtsam::noiseModel::Diagonal* Variances(Vector variances);
  static gtsam::noiseModel::Diagonal* Precisions(Vector precisions);
  Matrix R() const;

  // access to noise model
  Vector sigmas() const;
  Vector invsigmas() const;
  Vector precisions() const;

  // enabling serialization functionality
  void serializable() const;
};

virtual class Constrained : gtsam::noiseModel::Diagonal {
    static gtsam::noiseModel::Constrained* MixedSigmas(Vector mu, Vector sigmas);
    static gtsam::noiseModel::Constrained* MixedSigmas(double m, Vector sigmas);
    static gtsam::noiseModel::Constrained* MixedVariances(Vector mu, Vector variances);
    static gtsam::noiseModel::Constrained* MixedVariances(Vector variances);
    static gtsam::noiseModel::Constrained* MixedPrecisions(Vector mu, Vector precisions);
    static gtsam::noiseModel::Constrained* MixedPrecisions(Vector precisions);

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

  // access to noise model
  double sigma() const;

  // enabling serialization functionality
  void serializable() const;
};

virtual class Unit : gtsam::noiseModel::Isotropic {
  static gtsam::noiseModel::Unit* Create(size_t dim);

  // enabling serialization functionality
  void serializable() const;
};

namespace mEstimator {
virtual class Base {
  void print(string s) const;
};

virtual class Null: gtsam::noiseModel::mEstimator::Base {
  Null();
  static gtsam::noiseModel::mEstimator::Null* Create();

  // enabling serialization functionality
  void serializable() const;

  double weight(double error) const;
  double loss(double error) const;
};

virtual class Fair: gtsam::noiseModel::mEstimator::Base {
  Fair(double c);
  static gtsam::noiseModel::mEstimator::Fair* Create(double c);

  // enabling serialization functionality
  void serializable() const;

  double weight(double error) const;
  double loss(double error) const;
};

virtual class Huber: gtsam::noiseModel::mEstimator::Base {
  Huber(double k);
  static gtsam::noiseModel::mEstimator::Huber* Create(double k);

  // enabling serialization functionality
  void serializable() const;

  double weight(double error) const;
  double loss(double error) const;
};

virtual class Cauchy: gtsam::noiseModel::mEstimator::Base {
  Cauchy(double k);
  static gtsam::noiseModel::mEstimator::Cauchy* Create(double k);

  // enabling serialization functionality
  void serializable() const;

  double weight(double error) const;
  double loss(double error) const;
};

virtual class Tukey: gtsam::noiseModel::mEstimator::Base {
  Tukey(double k);
  static gtsam::noiseModel::mEstimator::Tukey* Create(double k);

  // enabling serialization functionality
  void serializable() const;

  double weight(double error) const;
  double loss(double error) const;
};

virtual class Welsch: gtsam::noiseModel::mEstimator::Base {
  Welsch(double k);
  static gtsam::noiseModel::mEstimator::Welsch* Create(double k);

  // enabling serialization functionality
  void serializable() const;

  double weight(double error) const;
  double loss(double error) const;
};

virtual class GemanMcClure: gtsam::noiseModel::mEstimator::Base {
  GemanMcClure(double c);
  static gtsam::noiseModel::mEstimator::GemanMcClure* Create(double c);

  // enabling serialization functionality
  void serializable() const;

  double weight(double error) const;
  double loss(double error) const;
};

virtual class DCS: gtsam::noiseModel::mEstimator::Base {
  DCS(double c);
  static gtsam::noiseModel::mEstimator::DCS* Create(double c);

  // enabling serialization functionality
  void serializable() const;

  double weight(double error) const;
  double loss(double error) const;
};

virtual class L2WithDeadZone: gtsam::noiseModel::mEstimator::Base {
  L2WithDeadZone(double k);
  static gtsam::noiseModel::mEstimator::L2WithDeadZone* Create(double k);

  // enabling serialization functionality
  void serializable() const;

  double weight(double error) const;
  double loss(double error) const;
};

}///\namespace mEstimator

virtual class Robust : gtsam::noiseModel::Base {
  Robust(const gtsam::noiseModel::mEstimator::Base* robust, const gtsam::noiseModel::Base* noise);
  static gtsam::noiseModel::Robust* Create(const gtsam::noiseModel::mEstimator::Base* robust, const gtsam::noiseModel::Base* noise);

  // enabling serialization functionality
  void serializable() const;
};

}///\namespace noiseModel

#include <gtsam/linear/Sampler.h>
class Sampler {
  // Constructors
  Sampler(gtsam::noiseModel::Diagonal* model, int seed);
  Sampler(Vector sigmas, int seed);

  // Standard Interface
  size_t dim() const;
  Vector sigmas() const;
  gtsam::noiseModel::Diagonal* model() const;
  Vector sample();
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

  void transposeMultiplyAdd(double alpha, Vector e, gtsam::VectorValues& x) const;
  gtsam::JacobianFactor whiten() const;

  pair<gtsam::GaussianConditional*, gtsam::JacobianFactor*> eliminate(const gtsam::Ordering& keys) const;

  void setModel(bool anyConstrained, Vector sigmas);

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
  HessianFactor(const gtsam::GaussianFactorGraph& factors);

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

#include <gtsam/linear/GaussianFactorGraph.h>
class GaussianFactorGraph {
  GaussianFactorGraph();
  GaussianFactorGraph(const gtsam::GaussianBayesNet& bayesNet);
  GaussianFactorGraph(const gtsam::GaussianBayesTree& bayesTree);

  // From FactorGraph
  void print(string s) const;
  bool equals(const gtsam::GaussianFactorGraph& lfgraph, double tol) const;
  size_t size() const;
  gtsam::GaussianFactor* at(size_t idx) const;
  gtsam::KeySet keys() const;
  gtsam::KeyVector keyVector() const;
  bool exists(size_t idx) const;

  // Building the graph
  void push_back(const gtsam::GaussianFactor* factor);
  void push_back(const gtsam::GaussianConditional* conditional);
  void push_back(const gtsam::GaussianFactorGraph& graph);
  void push_back(const gtsam::GaussianBayesNet& bayesNet);
  void push_back(const gtsam::GaussianBayesTree& bayesTree);
  void add(const gtsam::GaussianFactor& factor);
  void add(Vector b);
  void add(size_t key1, Matrix A1, Vector b, const gtsam::noiseModel::Diagonal* model);
  void add(size_t key1, Matrix A1, size_t key2, Matrix A2, Vector b,
      const gtsam::noiseModel::Diagonal* model);
  void add(size_t key1, Matrix A1, size_t key2, Matrix A2, size_t key3, Matrix A3,
      Vector b, const gtsam::noiseModel::Diagonal* model);

  // error and probability
  double error(const gtsam::VectorValues& c) const;
  double probPrime(const gtsam::VectorValues& c) const;

  gtsam::GaussianFactorGraph clone() const;
  gtsam::GaussianFactorGraph negate() const;

  // Optimizing and linear algebra
  gtsam::VectorValues optimize() const;
  gtsam::VectorValues optimize(const gtsam::Ordering& ordering) const;
  gtsam::VectorValues optimizeGradientSearch() const;
  gtsam::VectorValues gradient(const gtsam::VectorValues& x0) const;
  gtsam::VectorValues gradientAtZero() const;

  // Elimination and marginals
  gtsam::GaussianBayesNet* eliminateSequential();
  gtsam::GaussianBayesNet* eliminateSequential(const gtsam::Ordering& ordering);
  gtsam::GaussianBayesTree* eliminateMultifrontal();
  gtsam::GaussianBayesTree* eliminateMultifrontal(const gtsam::Ordering& ordering);
  pair<gtsam::GaussianBayesNet*, gtsam::GaussianFactorGraph*> eliminatePartialSequential(
    const gtsam::Ordering& ordering);
  pair<gtsam::GaussianBayesNet*, gtsam::GaussianFactorGraph*> eliminatePartialSequential(
    const gtsam::KeyVector& keys);
  pair<gtsam::GaussianBayesTree*, gtsam::GaussianFactorGraph*> eliminatePartialMultifrontal(
    const gtsam::Ordering& ordering);
  pair<gtsam::GaussianBayesTree*, gtsam::GaussianFactorGraph*> eliminatePartialMultifrontal(
    const gtsam::KeyVector& keys);
  gtsam::GaussianBayesNet* marginalMultifrontalBayesNet(const gtsam::Ordering& ordering);
  gtsam::GaussianBayesNet* marginalMultifrontalBayesNet(const gtsam::KeyVector& key_vector);
  gtsam::GaussianBayesNet* marginalMultifrontalBayesNet(const gtsam::Ordering& ordering,
    const gtsam::Ordering& marginalizedVariableOrdering);
  gtsam::GaussianBayesNet* marginalMultifrontalBayesNet(const gtsam::KeyVector& key_vector,
    const gtsam::Ordering& marginalizedVariableOrdering);
  gtsam::GaussianFactorGraph* marginal(const gtsam::KeyVector& key_vector);

  // Conversion to matrices
  Matrix sparseJacobian_() const;
  Matrix augmentedJacobian() const;
  Matrix augmentedJacobian(const gtsam::Ordering& ordering) const;
  pair<Matrix,Vector> jacobian() const;
  pair<Matrix,Vector> jacobian(const gtsam::Ordering& ordering) const;
  Matrix augmentedHessian() const;
  Matrix augmentedHessian(const gtsam::Ordering& ordering) const;
  pair<Matrix,Vector> hessian() const;
  pair<Matrix,Vector> hessian(const gtsam::Ordering& ordering) const;

  // enabling serialization functionality
  void serialize() const;
};

#include <gtsam/linear/GaussianConditional.h>
virtual class GaussianConditional : gtsam::GaussianFactor {
  //Constructors
  GaussianConditional(size_t key, Vector d, Matrix R, const gtsam::noiseModel::Diagonal* sigmas);
  GaussianConditional(size_t key, Vector d, Matrix R, size_t name1, Matrix S,
      const gtsam::noiseModel::Diagonal* sigmas);
  GaussianConditional(size_t key, Vector d, Matrix R, size_t name1, Matrix S,
      size_t name2, Matrix T, const gtsam::noiseModel::Diagonal* sigmas);

  //Constructors with no noise model
  GaussianConditional(size_t key, Vector d, Matrix R);
    GaussianConditional(size_t key, Vector d, Matrix R, size_t name1, Matrix S);
    GaussianConditional(size_t key, Vector d, Matrix R, size_t name1, Matrix S,
        size_t name2, Matrix T);

  //Standard Interface
  void print(string s) const;
  bool equals(const gtsam::GaussianConditional &cg, double tol) const;

  //Advanced Interface
  gtsam::VectorValues solve(const gtsam::VectorValues& parents) const;
  gtsam::VectorValues solveOtherRHS(const gtsam::VectorValues& parents, const gtsam::VectorValues& rhs) const;
  void solveTransposeInPlace(gtsam::VectorValues& gy) const;
  void scaleFrontalsBySigma(gtsam::VectorValues& gy) const;
  Matrix R() const;
  Matrix S() const;
  Vector d() const;

  // enabling serialization functionality
  void serialize() const;
};

#include <gtsam/linear/GaussianDensity.h>
virtual class GaussianDensity : gtsam::GaussianConditional {
    //Constructors
  GaussianDensity(size_t key, Vector d, Matrix R, const gtsam::noiseModel::Diagonal* sigmas);

  //Standard Interface
  void print(string s) const;
  bool equals(const gtsam::GaussianDensity &cg, double tol) const;
  Vector mean() const;
  Matrix covariance() const;
};

#include <gtsam/linear/GaussianBayesNet.h>
virtual class GaussianBayesNet {
    //Constructors
  GaussianBayesNet();
  GaussianBayesNet(const gtsam::GaussianConditional* conditional);

  // Testable
  void print(string s) const;
  bool equals(const gtsam::GaussianBayesNet& other, double tol) const;
  size_t size() const;

  // FactorGraph derived interface
  // size_t size() const;
  gtsam::GaussianConditional* at(size_t idx) const;
  gtsam::KeySet keys() const;
  bool exists(size_t idx) const;

  gtsam::GaussianConditional* front() const;
  gtsam::GaussianConditional* back() const;
  void push_back(gtsam::GaussianConditional* conditional);
  void push_back(const gtsam::GaussianBayesNet& bayesNet);

  gtsam::VectorValues optimize() const;
  gtsam::VectorValues optimize(gtsam::VectorValues& solutionForMissing) const;
  gtsam::VectorValues optimizeGradientSearch() const;
  gtsam::VectorValues gradient(const gtsam::VectorValues& x0) const;
  gtsam::VectorValues gradientAtZero() const;
  double error(const gtsam::VectorValues& x) const;
  double determinant() const;
  double logDeterminant() const;
  gtsam::VectorValues backSubstitute(const gtsam::VectorValues& gx) const;
  gtsam::VectorValues backSubstituteTranspose(const gtsam::VectorValues& gx) const;
};

#include <gtsam/linear/GaussianBayesTree.h>
virtual class GaussianBayesTree {
  // Standard Constructors and Named Constructors
  GaussianBayesTree();
  GaussianBayesTree(const gtsam::GaussianBayesTree& other);
  bool equals(const gtsam::GaussianBayesTree& other, double tol) const;
  void print(string s);
  size_t size() const;
  bool empty() const;
  size_t numCachedSeparatorMarginals() const;
  void saveGraph(string s) const;

  gtsam::VectorValues optimize() const;
  gtsam::VectorValues optimizeGradientSearch() const;
  gtsam::VectorValues gradient(const gtsam::VectorValues& x0) const;
  gtsam::VectorValues gradientAtZero() const;
  double error(const gtsam::VectorValues& x) const;
  double determinant() const;
  double logDeterminant() const;
  Matrix marginalCovariance(size_t key) const;
  gtsam::GaussianConditional* marginalFactor(size_t key) const;
  gtsam::GaussianFactorGraph* joint(size_t key1, size_t key2) const;
  gtsam::GaussianBayesNet* jointBayesNet(size_t key1, size_t key2) const;
};

#include <gtsam/linear/Errors.h>
class Errors {
    //Constructors
    Errors();
    Errors(const gtsam::VectorValues& V);

    //Testable
    void print(string s);
    bool equals(const gtsam::Errors& expected, double tol) const;
};

#include <gtsam/linear/GaussianISAM.h>
class GaussianISAM {
  //Constructor
  GaussianISAM();

  //Standard Interface
  void update(const gtsam::GaussianFactorGraph& newFactors);
  void saveGraph(string s) const;
  void clear();
};

#include <gtsam/linear/IterativeSolver.h>
virtual class IterativeOptimizationParameters {
  string getVerbosity() const;
  void setVerbosity(string s) ;
  void print() const;
};

//virtual class IterativeSolver {
//  IterativeSolver();
//  gtsam::VectorValues optimize ();
//};

#include <gtsam/linear/ConjugateGradientSolver.h>
virtual class ConjugateGradientParameters : gtsam::IterativeOptimizationParameters {
  ConjugateGradientParameters();
  int getMinIterations() const ;
  int getMaxIterations() const ;
  int getReset() const;
  double getEpsilon_rel() const;
  double getEpsilon_abs() const;

  void setMinIterations(int value);
  void setMaxIterations(int value);
  void setReset(int value);
  void setEpsilon_rel(double value);
  void setEpsilon_abs(double value);
  void print() const;
};

#include <gtsam/linear/Preconditioner.h>
virtual class PreconditionerParameters {
  PreconditionerParameters();
};

virtual class DummyPreconditionerParameters : gtsam::PreconditionerParameters {
  DummyPreconditionerParameters();
};

#include <gtsam/linear/PCGSolver.h>
virtual class PCGSolverParameters : gtsam::ConjugateGradientParameters {
  PCGSolverParameters();
  void print(string s);
  void setPreconditionerParams(gtsam::PreconditionerParameters* preconditioner);
};

#include <gtsam/linear/SubgraphSolver.h>
virtual class SubgraphSolverParameters : gtsam::ConjugateGradientParameters {
  SubgraphSolverParameters();
  void print() const;
};

virtual class SubgraphSolver  {
  SubgraphSolver(const gtsam::GaussianFactorGraph &A, const gtsam::SubgraphSolverParameters &parameters, const gtsam::Ordering& ordering);
  SubgraphSolver(const gtsam::GaussianFactorGraph &Ab1, const gtsam::GaussianFactorGraph* Ab2, const gtsam::SubgraphSolverParameters &parameters, const gtsam::Ordering& ordering);
  gtsam::VectorValues optimize() const;
};

#include <gtsam/linear/KalmanFilter.h>
class KalmanFilter {
  KalmanFilter(size_t n);
  // gtsam::GaussianDensity* init(Vector x0, const gtsam::SharedDiagonal& P0);
  gtsam::GaussianDensity* init(Vector x0, Matrix P0);
  void print(string s) const;
  static size_t step(gtsam::GaussianDensity* p);
  gtsam::GaussianDensity* predict(gtsam::GaussianDensity* p, Matrix F,
      Matrix B, Vector u, const gtsam::noiseModel::Diagonal* modelQ);
  gtsam::GaussianDensity* predictQ(gtsam::GaussianDensity* p, Matrix F,
      Matrix B, Vector u, Matrix Q);
  gtsam::GaussianDensity* predict2(gtsam::GaussianDensity* p, Matrix A0,
      Matrix A1, Vector b, const gtsam::noiseModel::Diagonal* model);
  gtsam::GaussianDensity* update(gtsam::GaussianDensity* p, Matrix H,
      Vector z, const gtsam::noiseModel::Diagonal* model);
  gtsam::GaussianDensity* updateQ(gtsam::GaussianDensity* p, Matrix H,
      Vector z, Matrix Q);
};

//*************************************************************************
// nonlinear
//*************************************************************************

#include <gtsam/inference/Symbol.h>
size_t symbol(char chr, size_t index);
char symbolChr(size_t key);
size_t symbolIndex(size_t key);

namespace symbol_shorthand {
  size_t A(size_t j);
  size_t B(size_t j);
  size_t C(size_t j);
  size_t D(size_t j);
  size_t E(size_t j);
  size_t F(size_t j);
  size_t G(size_t j);
  size_t H(size_t j);
  size_t I(size_t j);
  size_t J(size_t j);
  size_t K(size_t j);
  size_t L(size_t j);
  size_t M(size_t j);
  size_t N(size_t j);
  size_t O(size_t j);
  size_t P(size_t j);
  size_t Q(size_t j);
  size_t R(size_t j);
  size_t S(size_t j);
  size_t T(size_t j);
  size_t U(size_t j);
  size_t V(size_t j);
  size_t W(size_t j);
  size_t X(size_t j);
  size_t Y(size_t j);
  size_t Z(size_t j);
}///\namespace symbol

// Default keyformatter
void PrintKeyList  (const gtsam::KeyList& keys);
void PrintKeyList  (const gtsam::KeyList& keys, string s);
void PrintKeyVector(const gtsam::KeyVector& keys);
void PrintKeyVector(const gtsam::KeyVector& keys, string s);
void PrintKeySet   (const gtsam::KeySet& keys);
void PrintKeySet   (const gtsam::KeySet& keys, string s);

#include <gtsam/inference/LabeledSymbol.h>
class LabeledSymbol {
  LabeledSymbol(size_t full_key);
  LabeledSymbol(const gtsam::LabeledSymbol& key);
  LabeledSymbol(unsigned char valType, unsigned char label, size_t j);

  size_t key() const;
  unsigned char label() const;
  unsigned char chr() const;
  size_t index() const;

  gtsam::LabeledSymbol upper() const;
  gtsam::LabeledSymbol lower() const;
  gtsam::LabeledSymbol newChr(unsigned char c) const;
  gtsam::LabeledSymbol newLabel(unsigned char label) const;

  void print(string s) const;
};

size_t mrsymbol(unsigned char c, unsigned char label, size_t j);
unsigned char mrsymbolChr(size_t key);
unsigned char mrsymbolLabel(size_t key);
size_t mrsymbolIndex(size_t key);

#include <gtsam/inference/Ordering.h>
class Ordering {
  // Standard Constructors and Named Constructors
  Ordering();
  Ordering(const gtsam::Ordering& other);

  // Testable
  void print(string s) const;
  bool equals(const gtsam::Ordering& ord, double tol) const;

  // Standard interface
  size_t size() const;
  size_t at(size_t key) const;
  void push_back(size_t key);

  // enabling serialization functionality
  void serialize() const;
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
  void replace(size_t i, gtsam::NonlinearFactor* factors);
  void resize(size_t size);
  size_t nrFactors() const;
  gtsam::NonlinearFactor* at(size_t idx) const;
  void push_back(const gtsam::NonlinearFactorGraph& factors);
  void push_back(gtsam::NonlinearFactor* factor);
  void add(gtsam::NonlinearFactor* factor);
  bool exists(size_t idx) const;
  gtsam::KeySet keys() const;
  gtsam::KeyVector keyVector() const;

  template<T = {Vector, gtsam::Point2, gtsam::StereoPoint2, gtsam::Point3, gtsam::Rot2, gtsam::SO3, gtsam::SO4, gtsam::Rot3, gtsam::Pose2, gtsam::Pose3, gtsam::Cal3_S2,gtsam::CalibratedCamera, gtsam::SimpleCamera, gtsam::PinholeCameraCal3_S2, gtsam::imuBias::ConstantBias}>
  void addPrior(size_t key, const T& prior, const gtsam::noiseModel::Base* noiseModel);

  // NonlinearFactorGraph
  void printErrors(const gtsam::Values& values) const;
  double error(const gtsam::Values& values) const;
  double probPrime(const gtsam::Values& values) const;
  gtsam::Ordering orderingCOLAMD() const;
  // Ordering* orderingCOLAMDConstrained(const gtsam::Values& c, const std::map<gtsam::Key,int>& constraints) const;
  gtsam::GaussianFactorGraph* linearize(const gtsam::Values& values) const;
  gtsam::NonlinearFactorGraph clone() const;

  // enabling serialization functionality
  void serialize() const;
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

#include <gtsam/nonlinear/NonlinearFactor.h>
virtual class NoiseModelFactor: gtsam::NonlinearFactor {
  bool equals(const gtsam::NoiseModelFactor& other, double tol) const;
  gtsam::noiseModel::Base* noiseModel() const;
  Vector unwhitenedError(const gtsam::Values& x) const;
  Vector whitenedError(const gtsam::Values& x) const;
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

  void insert(size_t j, const gtsam::Point2& point2);
  void insert(size_t j, const gtsam::Point3& point3);
  void insert(size_t j, const gtsam::Rot2& rot2);
  void insert(size_t j, const gtsam::Pose2& pose2);
  void insert(size_t j, const gtsam::SO3& R);
  void insert(size_t j, const gtsam::SO4& Q);
  void insert(size_t j, const gtsam::SOn& P);
  void insert(size_t j, const gtsam::Rot3& rot3);
  void insert(size_t j, const gtsam::Pose3& pose3);
  void insert(size_t j, const gtsam::Cal3_S2& cal3_s2);
  void insert(size_t j, const gtsam::Cal3DS2& cal3ds2);
  void insert(size_t j, const gtsam::Cal3Bundler& cal3bundler);
  void insert(size_t j, const gtsam::EssentialMatrix& essential_matrix);
  void insert(size_t j, const gtsam::PinholeCameraCal3_S2& simple_camera);
  void insert(size_t j, const gtsam::imuBias::ConstantBias& constant_bias);
  void insert(size_t j, const gtsam::NavState& nav_state);
  void insert(size_t j, Vector vector);
  void insert(size_t j, Matrix matrix);

  void update(size_t j, const gtsam::Point2& point2);
  void update(size_t j, const gtsam::Point3& point3);
  void update(size_t j, const gtsam::Rot2& rot2);
  void update(size_t j, const gtsam::Pose2& pose2);
  void update(size_t j, const gtsam::SO3& R);
  void update(size_t j, const gtsam::SO4& Q);
  void update(size_t j, const gtsam::SOn& P);
  void update(size_t j, const gtsam::Rot3& rot3);
  void update(size_t j, const gtsam::Pose3& pose3);
  void update(size_t j, const gtsam::Cal3_S2& cal3_s2);
  void update(size_t j, const gtsam::Cal3DS2& cal3ds2);
  void update(size_t j, const gtsam::Cal3Bundler& cal3bundler);
  void update(size_t j, const gtsam::EssentialMatrix& essential_matrix);
  void update(size_t j, const gtsam::imuBias::ConstantBias& constant_bias);
  void update(size_t j, const gtsam::NavState& nav_state);
  void update(size_t j, Vector vector);
  void update(size_t j, Matrix matrix);

  template<T = {gtsam::Point2, gtsam::Point3, gtsam::Rot2, gtsam::Pose2, gtsam::SO3, gtsam::SO4, gtsam::SOn, gtsam::Rot3, gtsam::Pose3, gtsam::Cal3_S2, gtsam::Cal3DS2, gtsam::Cal3Bundler, gtsam::EssentialMatrix, gtsam::imuBias::ConstantBias, gtsam::NavState, Vector, Matrix}>
  T at(size_t j);

  /// version for double
  void insertDouble(size_t j, double c);
  double atDouble(size_t j) const;
};

#include <gtsam/nonlinear/Marginals.h>
class Marginals {
  Marginals(const gtsam::NonlinearFactorGraph& graph,
      const gtsam::Values& solution);
  Marginals(const gtsam::GaussianFactorGraph& gfgraph,
      const gtsam::Values& solution);
  Marginals(const gtsam::GaussianFactorGraph& gfgraph,
      const gtsam::VectorValues& solutionvec);

  void print(string s) const;
  Matrix marginalCovariance(size_t variable) const;
  Matrix marginalInformation(size_t variable) const;
  gtsam::JointMarginal jointMarginalCovariance(const gtsam::KeyVector& variables) const;
  gtsam::JointMarginal jointMarginalInformation(const gtsam::KeyVector& variables) const;
};

class JointMarginal {
  Matrix at(size_t iVariable, size_t jVariable) const;
  Matrix fullMatrix() const;
  void print(string s) const;
  void print() const;
};

#include <gtsam/nonlinear/LinearContainerFactor.h>
virtual class LinearContainerFactor : gtsam::NonlinearFactor {

  LinearContainerFactor(gtsam::GaussianFactor* factor, const gtsam::Values& linearizationPoint);
  LinearContainerFactor(gtsam::GaussianFactor* factor);

  gtsam::GaussianFactor* factor() const;
//  const boost::optional<Values>& linearizationPoint() const;

  bool isJacobian() const;
  gtsam::JacobianFactor* toJacobian() const;
  gtsam::HessianFactor* toHessian() const;

  static gtsam::NonlinearFactorGraph ConvertLinearGraph(const gtsam::GaussianFactorGraph& linear_graph,
      const gtsam::Values& linearizationPoint);

  static gtsam::NonlinearFactorGraph ConvertLinearGraph(const gtsam::GaussianFactorGraph& linear_graph);

  // enabling serialization functionality
  void serializable() const;
}; // \class LinearContainerFactor

// Summarization functionality
//#include <gtsam/nonlinear/summarization.h>
//
//// Uses partial QR approach by default
//gtsam::GaussianFactorGraph summarize(
//    const gtsam::NonlinearFactorGraph& graph, const gtsam::Values& values,
//    const gtsam::KeySet& saved_keys);
//
//gtsam::NonlinearFactorGraph summarizeAsNonlinearContainer(
//    const gtsam::NonlinearFactorGraph& graph, const gtsam::Values& values,
//    const gtsam::KeySet& saved_keys);

//*************************************************************************
// Nonlinear optimizers
//*************************************************************************
#include <gtsam/nonlinear/NonlinearOptimizerParams.h>
virtual class NonlinearOptimizerParams {
  NonlinearOptimizerParams();
  void print(string s) const;

  int getMaxIterations() const;
  double getRelativeErrorTol() const;
  double getAbsoluteErrorTol() const;
  double getErrorTol() const;
  string getVerbosity() const;

  void setMaxIterations(int value);
  void setRelativeErrorTol(double value);
  void setAbsoluteErrorTol(double value);
  void setErrorTol(double value);
  void setVerbosity(string s);

  string getLinearSolverType() const;
  void setLinearSolverType(string solver);

  void setIterativeParams(gtsam::IterativeOptimizationParameters* params);
  void setOrdering(const gtsam::Ordering& ordering);
  string getOrderingType() const;
  void setOrderingType(string ordering);

  bool isMultifrontal() const;
  bool isSequential() const;
  bool isCholmod() const;
  bool isIterative() const;
};

bool checkConvergence(double relativeErrorTreshold,
                      double absoluteErrorTreshold, double errorThreshold,
                      double currentError, double newError);
bool checkConvergence(const gtsam::NonlinearOptimizerParams& params,
                      double currentError, double newError);

#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
virtual class GaussNewtonParams : gtsam::NonlinearOptimizerParams {
  GaussNewtonParams();
};

#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
virtual class LevenbergMarquardtParams : gtsam::NonlinearOptimizerParams {
  LevenbergMarquardtParams();

  bool getDiagonalDamping() const;
  double getlambdaFactor() const;
  double getlambdaInitial() const;
  double getlambdaLowerBound() const;
  double getlambdaUpperBound() const;
  bool getUseFixedLambdaFactor();
  string getLogFile() const;
  string getVerbosityLM() const;

  void setDiagonalDamping(bool flag);
  void setlambdaFactor(double value);
  void setlambdaInitial(double value);
  void setlambdaLowerBound(double value);
  void setlambdaUpperBound(double value);
  void setUseFixedLambdaFactor(bool flag);
  void setLogFile(string s);
  void setVerbosityLM(string s);

  static gtsam::LevenbergMarquardtParams LegacyDefaults();
  static gtsam::LevenbergMarquardtParams CeresDefaults();

  static gtsam::LevenbergMarquardtParams EnsureHasOrdering(
      gtsam::LevenbergMarquardtParams params,
      const gtsam::NonlinearFactorGraph& graph);
  static gtsam::LevenbergMarquardtParams ReplaceOrdering(
      gtsam::LevenbergMarquardtParams params, const gtsam::Ordering& ordering);
};

#include <gtsam/nonlinear/DoglegOptimizer.h>
virtual class DoglegParams : gtsam::NonlinearOptimizerParams {
  DoglegParams();

  double getDeltaInitial() const;
  string getVerbosityDL() const;

  void setDeltaInitial(double deltaInitial) const;
  void setVerbosityDL(string verbosityDL) const;
};

#include <gtsam/nonlinear/NonlinearOptimizer.h>
virtual class NonlinearOptimizer {
  gtsam::Values optimize();
  gtsam::Values optimizeSafely();
  double error() const;
  int iterations() const;
  gtsam::Values values() const;
  gtsam::GaussianFactorGraph* iterate() const;
};

#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
virtual class GaussNewtonOptimizer : gtsam::NonlinearOptimizer {
  GaussNewtonOptimizer(const gtsam::NonlinearFactorGraph& graph, const gtsam::Values& initialValues);
  GaussNewtonOptimizer(const gtsam::NonlinearFactorGraph& graph, const gtsam::Values& initialValues, const gtsam::GaussNewtonParams& params);
};

#include <gtsam/nonlinear/DoglegOptimizer.h>
virtual class DoglegOptimizer : gtsam::NonlinearOptimizer {
  DoglegOptimizer(const gtsam::NonlinearFactorGraph& graph, const gtsam::Values& initialValues);
  DoglegOptimizer(const gtsam::NonlinearFactorGraph& graph, const gtsam::Values& initialValues, const gtsam::DoglegParams& params);
  double getDelta() const;
};

#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
virtual class LevenbergMarquardtOptimizer : gtsam::NonlinearOptimizer {
  LevenbergMarquardtOptimizer(const gtsam::NonlinearFactorGraph& graph, const gtsam::Values& initialValues);
  LevenbergMarquardtOptimizer(const gtsam::NonlinearFactorGraph& graph, const gtsam::Values& initialValues, const gtsam::LevenbergMarquardtParams& params);
  double lambda() const;
  void print(string str) const;
};

#include <gtsam/nonlinear/ISAM2.h>
class ISAM2GaussNewtonParams {
  ISAM2GaussNewtonParams();

  void print(string str) const;

  /** Getters and Setters for all properties */
  double getWildfireThreshold() const;
  void setWildfireThreshold(double wildfireThreshold);
};

class ISAM2DoglegParams {
  ISAM2DoglegParams();

  void print(string str) const;

  /** Getters and Setters for all properties */
  double getWildfireThreshold() const;
  void setWildfireThreshold(double wildfireThreshold);
  double getInitialDelta() const;
  void setInitialDelta(double initialDelta);
  string getAdaptationMode() const;
  void setAdaptationMode(string adaptationMode);
  bool isVerbose() const;
  void setVerbose(bool verbose);
};

class ISAM2ThresholdMapValue {
  ISAM2ThresholdMapValue(char c, Vector thresholds);
  ISAM2ThresholdMapValue(const gtsam::ISAM2ThresholdMapValue& other);
};

class ISAM2ThresholdMap {
  ISAM2ThresholdMap();
  ISAM2ThresholdMap(const gtsam::ISAM2ThresholdMap& other);

  // Note: no print function

  // common STL methods
  size_t size() const;
  bool empty() const;
  void clear();

  // structure specific methods
  void insert(const gtsam::ISAM2ThresholdMapValue& value) const;
};

class ISAM2Params {
  ISAM2Params();

  void print(string str) const;

  /** Getters and Setters for all properties */
  void setOptimizationParams(const gtsam::ISAM2GaussNewtonParams& gauss_newton__params);
  void setOptimizationParams(const gtsam::ISAM2DoglegParams& dogleg_params);
  void setRelinearizeThreshold(double threshold);
  void setRelinearizeThreshold(const gtsam::ISAM2ThresholdMap& threshold_map);
  int getRelinearizeSkip() const;
  void setRelinearizeSkip(int relinearizeSkip);
  bool isEnableRelinearization() const;
  void setEnableRelinearization(bool enableRelinearization);
  bool isEvaluateNonlinearError() const;
  void setEvaluateNonlinearError(bool evaluateNonlinearError);
  string getFactorization() const;
  void setFactorization(string factorization);
  bool isCacheLinearizedFactors() const;
  void setCacheLinearizedFactors(bool cacheLinearizedFactors);
  bool isEnableDetailedResults() const;
  void setEnableDetailedResults(bool enableDetailedResults);
  bool isEnablePartialRelinearizationCheck() const;
  void setEnablePartialRelinearizationCheck(bool enablePartialRelinearizationCheck);
};

class ISAM2Clique {

    //Constructors
    ISAM2Clique();

    //Standard Interface
    Vector gradientContribution() const;
    void print(string s);
};

class ISAM2Result {
  ISAM2Result();

  void print(string str) const;

  /** Getters and Setters for all properties */
  size_t getVariablesRelinearized() const;
  size_t getVariablesReeliminated() const;
  size_t getCliques() const;
};

class ISAM2 {
  ISAM2();
  ISAM2(const gtsam::ISAM2Params& params);
  ISAM2(const gtsam::ISAM2& other);

  bool equals(const gtsam::ISAM2& other, double tol) const;
  void print(string s) const;
  void printStats() const;
  void saveGraph(string s) const;

  gtsam::ISAM2Result update();
  gtsam::ISAM2Result update(const gtsam::NonlinearFactorGraph& newFactors, const gtsam::Values& newTheta);
  gtsam::ISAM2Result update(const gtsam::NonlinearFactorGraph& newFactors, const gtsam::Values& newTheta, const gtsam::FactorIndices& removeFactorIndices);
  gtsam::ISAM2Result update(const gtsam::NonlinearFactorGraph& newFactors, const gtsam::Values& newTheta, const gtsam::FactorIndices& removeFactorIndices, const gtsam::KeyGroupMap& constrainedKeys);
  // TODO: wrap the full version of update
 //void update(const gtsam::NonlinearFactorGraph& newFactors, const gtsam::Values& newTheta, const gtsam::KeyVector& removeFactorIndices, FastMap<Key,int>& constrainedKeys);
  //void update(const gtsam::NonlinearFactorGraph& newFactors, const gtsam::Values& newTheta, const gtsam::KeyVector& removeFactorIndices, FastMap<Key,int>& constrainedKeys, bool force_relinearize);

  gtsam::Values getLinearizationPoint() const;
  gtsam::Values calculateEstimate() const;
  template <VALUE = {gtsam::Point2, gtsam::Rot2, gtsam::Pose2, gtsam::Point3,
                     gtsam::Rot3, gtsam::Pose3, gtsam::Cal3_S2, gtsam::Cal3DS2,
                     gtsam::Cal3Bundler, gtsam::EssentialMatrix,
                     gtsam::SimpleCamera, gtsam::PinholeCameraCal3_S2, Vector, Matrix}>
  VALUE calculateEstimate(size_t key) const;
  gtsam::Values calculateBestEstimate() const;
  Matrix marginalCovariance(size_t key) const;
  gtsam::VectorValues getDelta() const;
  gtsam::NonlinearFactorGraph getFactorsUnsafe() const;
  gtsam::VariableIndex getVariableIndex() const;
  gtsam::ISAM2Params params() const;
};

#include <gtsam/nonlinear/NonlinearISAM.h>
class NonlinearISAM {
  NonlinearISAM();
  NonlinearISAM(int reorderInterval);
  void print(string s) const;
  void printStats() const;
  void saveGraph(string s) const;
  gtsam::Values estimate() const;
  Matrix marginalCovariance(size_t key) const;
  int reorderInterval() const;
  int reorderCounter() const;
  void update(const gtsam::NonlinearFactorGraph& newFactors, const gtsam::Values& initialValues);
  void reorder_relinearize();

  // These might be expensive as instead of a reference the wrapper will make a copy
  gtsam::GaussianISAM bayesTree() const;
  gtsam::Values getLinearizationPoint() const;
  gtsam::NonlinearFactorGraph getFactorsUnsafe() const;
};

//*************************************************************************
// Nonlinear factor types
//*************************************************************************
#include <gtsam/geometry/SimpleCamera.h>
#include <gtsam/geometry/CalibratedCamera.h>
#include <gtsam/geometry/StereoPoint2.h>

#include <gtsam/nonlinear/PriorFactor.h>
template<T = {Vector, gtsam::Point2, gtsam::StereoPoint2, gtsam::Point3, gtsam::Rot2, gtsam::SO3, gtsam::SO4, gtsam::SOn, gtsam::Rot3, gtsam::Pose2, gtsam::Pose3, gtsam::Cal3_S2,gtsam::CalibratedCamera, gtsam::SimpleCamera, gtsam::PinholeCameraCal3_S2, gtsam::imuBias::ConstantBias}>
virtual class PriorFactor : gtsam::NoiseModelFactor {
  PriorFactor(size_t key, const T& prior, const gtsam::noiseModel::Base* noiseModel);
  T prior() const;

  // enabling serialization functionality
  void serialize() const;
};


#include <gtsam/slam/BetweenFactor.h>
template<T = {Vector, gtsam::Point2, gtsam::Point3, gtsam::Rot2, gtsam::SO3, gtsam::SO4, gtsam::Rot3, gtsam::Pose2, gtsam::Pose3, gtsam::imuBias::ConstantBias}>
virtual class BetweenFactor : gtsam::NoiseModelFactor {
  BetweenFactor(size_t key1, size_t key2, const T& relativePose, const gtsam::noiseModel::Base* noiseModel);
  T measured() const;

  // enabling serialization functionality
  void serialize() const;
};



#include <gtsam/nonlinear/NonlinearEquality.h>
template<T = {gtsam::Point2, gtsam::StereoPoint2, gtsam::Point3, gtsam::Rot2, gtsam::SO3, gtsam::SO4, gtsam::SOn, gtsam::Rot3, gtsam::Pose2, gtsam::Pose3, gtsam::Cal3_S2, gtsam::CalibratedCamera, gtsam::SimpleCamera, gtsam::PinholeCameraCal3_S2, gtsam::imuBias::ConstantBias}>
virtual class NonlinearEquality : gtsam::NoiseModelFactor {
  // Constructor - forces exact evaluation
  NonlinearEquality(size_t j, const T& feasible);
  // Constructor - allows inexact evaluation
  NonlinearEquality(size_t j, const T& feasible, double error_gain);

  // enabling serialization functionality
  void serialize() const;
};


#include <gtsam/sam/RangeFactor.h>
template<POSE, POINT>
virtual class RangeFactor : gtsam::NoiseModelFactor {
  RangeFactor(size_t key1, size_t key2, double measured, const gtsam::noiseModel::Base* noiseModel);

  // enabling serialization functionality
  void serialize() const;
};

typedef gtsam::RangeFactor<gtsam::Pose2, gtsam::Point2> RangeFactor2D;
typedef gtsam::RangeFactor<gtsam::Pose3, gtsam::Point3> RangeFactor3D;
typedef gtsam::RangeFactor<gtsam::Pose2, gtsam::Pose2> RangeFactorPose2;
typedef gtsam::RangeFactor<gtsam::Pose3, gtsam::Pose3> RangeFactorPose3;
typedef gtsam::RangeFactor<gtsam::CalibratedCamera, gtsam::Point3> RangeFactorCalibratedCameraPoint;
typedef gtsam::RangeFactor<gtsam::PinholeCameraCal3_S2, gtsam::Point3> RangeFactorSimpleCameraPoint;
typedef gtsam::RangeFactor<gtsam::CalibratedCamera, gtsam::CalibratedCamera> RangeFactorCalibratedCamera;
typedef gtsam::RangeFactor<gtsam::PinholeCameraCal3_S2, gtsam::PinholeCameraCal3_S2> RangeFactorSimpleCamera;


#include <gtsam/sam/RangeFactor.h>
template<POSE, POINT>
virtual class RangeFactorWithTransform : gtsam::NoiseModelFactor {
  RangeFactorWithTransform(size_t key1, size_t key2, double measured, const gtsam::noiseModel::Base* noiseModel, const POSE& body_T_sensor);

  // enabling serialization functionality
  void serialize() const;
};

typedef gtsam::RangeFactorWithTransform<gtsam::Pose2, gtsam::Point2> RangeFactorWithTransform2D;
typedef gtsam::RangeFactorWithTransform<gtsam::Pose3, gtsam::Point3> RangeFactorWithTransform3D;
typedef gtsam::RangeFactorWithTransform<gtsam::Pose2, gtsam::Pose2> RangeFactorWithTransformPose2;
typedef gtsam::RangeFactorWithTransform<gtsam::Pose3, gtsam::Pose3> RangeFactorWithTransformPose3;

#include <gtsam/sam/BearingFactor.h>
template<POSE, POINT, BEARING>
virtual class BearingFactor : gtsam::NoiseModelFactor {
  BearingFactor(size_t key1, size_t key2, const BEARING& measured, const gtsam::noiseModel::Base* noiseModel);

  // enabling serialization functionality
  void serialize() const;
};

typedef gtsam::BearingFactor<gtsam::Pose2, gtsam::Point2, gtsam::Rot2> BearingFactor2D;
typedef gtsam::BearingFactor<gtsam::Pose2, gtsam::Pose2, gtsam::Rot2> BearingFactorPose2;

#include <gtsam/geometry/BearingRange.h>
template <POSE, POINT, BEARING, RANGE>
class BearingRange {
  BearingRange(const BEARING& b, const RANGE& r);
  BEARING bearing() const;
  RANGE range() const;
  // TODO(frank): can't class instance itself?
  // static gtsam::BearingRange Measure(const POSE& pose, const POINT& point);
  static BEARING MeasureBearing(const POSE& pose, const POINT& point);
  static RANGE MeasureRange(const POSE& pose, const POINT& point);
  void print(string s) const;
};

typedef gtsam::BearingRange<gtsam::Pose2, gtsam::Point2, gtsam::Rot2, double> BearingRange2D;

#include <gtsam/sam/BearingRangeFactor.h>
template<POSE, POINT, BEARING, RANGE>
virtual class BearingRangeFactor : gtsam::NoiseModelFactor {
  BearingRangeFactor(size_t poseKey, size_t pointKey,
      const BEARING& measuredBearing, const RANGE& measuredRange,
      const gtsam::noiseModel::Base* noiseModel);

  // enabling serialization functionality
  void serialize() const;
};

typedef gtsam::BearingRangeFactor<gtsam::Pose2, gtsam::Point2, gtsam::Rot2, double> BearingRangeFactor2D;
typedef gtsam::BearingRangeFactor<gtsam::Pose2, gtsam::Pose2, gtsam::Rot2, double> BearingRangeFactorPose2;


#include <gtsam/slam/ProjectionFactor.h>
template<POSE, LANDMARK, CALIBRATION>
virtual class GenericProjectionFactor : gtsam::NoiseModelFactor {
  GenericProjectionFactor(const gtsam::Point2& measured, const gtsam::noiseModel::Base* noiseModel,
    size_t poseKey, size_t pointKey, const CALIBRATION* k);
  GenericProjectionFactor(const gtsam::Point2& measured, const gtsam::noiseModel::Base* noiseModel,
    size_t poseKey, size_t pointKey, const CALIBRATION* k, const POSE& body_P_sensor);

  GenericProjectionFactor(const gtsam::Point2& measured, const gtsam::noiseModel::Base* noiseModel,
      size_t poseKey, size_t pointKey, const CALIBRATION* k, bool throwCheirality, bool verboseCheirality);
    GenericProjectionFactor(const gtsam::Point2& measured, const gtsam::noiseModel::Base* noiseModel,
      size_t poseKey, size_t pointKey, const CALIBRATION* k, bool throwCheirality, bool verboseCheirality,
      const POSE& body_P_sensor);

  gtsam::Point2 measured() const;
  CALIBRATION* calibration() const;
  bool verboseCheirality() const;
  bool throwCheirality() const;

  // enabling serialization functionality
  void serialize() const;
};
typedef gtsam::GenericProjectionFactor<gtsam::Pose3, gtsam::Point3, gtsam::Cal3_S2> GenericProjectionFactorCal3_S2;
typedef gtsam::GenericProjectionFactor<gtsam::Pose3, gtsam::Point3, gtsam::Cal3DS2> GenericProjectionFactorCal3DS2;


#include <gtsam/slam/GeneralSFMFactor.h>
template<CAMERA, LANDMARK>
virtual class GeneralSFMFactor : gtsam::NoiseModelFactor {
  GeneralSFMFactor(const gtsam::Point2& measured, const gtsam::noiseModel::Base* model, size_t cameraKey, size_t landmarkKey);
  gtsam::Point2 measured() const;
};
typedef gtsam::GeneralSFMFactor<gtsam::PinholeCameraCal3_S2, gtsam::Point3> GeneralSFMFactorCal3_S2;
// due to lack of jacobians of Cal3DS2_Base::calibrate, GeneralSFMFactor does not apply to Cal3DS2
//typedef gtsam::GeneralSFMFactor<gtsam::PinholeCameraCal3DS2, gtsam::Point3> GeneralSFMFactorCal3DS2;

template<CALIBRATION = {gtsam::Cal3_S2}>
virtual class GeneralSFMFactor2 : gtsam::NoiseModelFactor {
  GeneralSFMFactor2(const gtsam::Point2& measured, const gtsam::noiseModel::Base* model, size_t poseKey, size_t landmarkKey, size_t calibKey);
  gtsam::Point2 measured() const;

  // enabling serialization functionality
  void serialize() const;
};

#include <gtsam/slam/SmartProjectionFactor.h>
class SmartProjectionParams {
  SmartProjectionParams();
  // TODO(frank): make these work:
  //  void setLinearizationMode(LinearizationMode linMode);
  //  void setDegeneracyMode(DegeneracyMode degMode);
  void setRankTolerance(double rankTol);
  void setEnableEPI(bool enableEPI);
  void setLandmarkDistanceThreshold(bool landmarkDistanceThreshold);
  void setDynamicOutlierRejectionThreshold(bool dynOutRejectionThreshold);
};

#include <gtsam/slam/SmartProjectionPoseFactor.h>
template<CALIBRATION>
virtual class SmartProjectionPoseFactor: gtsam::NonlinearFactor {

  SmartProjectionPoseFactor(const gtsam::noiseModel::Base* noise,
      const CALIBRATION* K);
  SmartProjectionPoseFactor(const gtsam::noiseModel::Base* noise,
      const CALIBRATION* K,
      const gtsam::Pose3& body_P_sensor);
  SmartProjectionPoseFactor(const gtsam::noiseModel::Base* noise,
      const CALIBRATION* K,
      const gtsam::SmartProjectionParams& params);
  SmartProjectionPoseFactor(const gtsam::noiseModel::Base* noise,
      const CALIBRATION* K,
      const gtsam::Pose3& body_P_sensor,
      const gtsam::SmartProjectionParams& params);

  void add(const gtsam::Point2& measured_i, size_t poseKey_i);

  // enabling serialization functionality
  //void serialize() const;
};

typedef gtsam::SmartProjectionPoseFactor<gtsam::Cal3_S2> SmartProjectionPose3Factor;


#include <gtsam/slam/StereoFactor.h>
template<POSE, LANDMARK>
virtual class GenericStereoFactor : gtsam::NoiseModelFactor {
  GenericStereoFactor(const gtsam::StereoPoint2& measured, const gtsam::noiseModel::Base* noiseModel,
    size_t poseKey, size_t landmarkKey, const gtsam::Cal3_S2Stereo* K);
  gtsam::StereoPoint2 measured() const;
  gtsam::Cal3_S2Stereo* calibration() const;

  // enabling serialization functionality
  void serialize() const;
};
typedef gtsam::GenericStereoFactor<gtsam::Pose3, gtsam::Point3> GenericStereoFactor3D;

#include <gtsam/slam/PoseTranslationPrior.h>
template<POSE>
virtual class PoseTranslationPrior : gtsam::NoiseModelFactor {
  PoseTranslationPrior(size_t key, const POSE& pose_z, const gtsam::noiseModel::Base* noiseModel);
};

typedef gtsam::PoseTranslationPrior<gtsam::Pose2> PoseTranslationPrior2D;
typedef gtsam::PoseTranslationPrior<gtsam::Pose3> PoseTranslationPrior3D;

#include <gtsam/slam/PoseRotationPrior.h>
template<POSE>
virtual class PoseRotationPrior : gtsam::NoiseModelFactor {
  PoseRotationPrior(size_t key, const POSE& pose_z, const gtsam::noiseModel::Base* noiseModel);
};

typedef gtsam::PoseRotationPrior<gtsam::Pose2> PoseRotationPrior2D;
typedef gtsam::PoseRotationPrior<gtsam::Pose3> PoseRotationPrior3D;

#include <gtsam/slam/EssentialMatrixFactor.h>
virtual class EssentialMatrixFactor : gtsam::NoiseModelFactor {
  EssentialMatrixFactor(size_t key, const gtsam::Point2& pA, const gtsam::Point2& pB,
      const gtsam::noiseModel::Base* noiseModel);
};

#include <gtsam/slam/dataset.h>
class SfmTrack {
  size_t number_measurements() const;
  pair<size_t, gtsam::Point2> measurement(size_t idx) const;
  pair<size_t, size_t> siftIndex(size_t idx) const;
};

class SfmData {
  size_t number_cameras() const;
  size_t number_tracks() const;
  //TODO(Varun) Need to fix issue #237 first before this can work
  // gtsam::PinholeCamera<gtsam::Cal3Bundler> camera(size_t idx) const;
  gtsam::SfmTrack track(size_t idx) const;
};

string findExampleDataFile(string name);
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

// std::vector<gtsam::BetweenFactor<Pose3>::shared_ptr>
class BetweenFactorPose3s
{
  BetweenFactorPose3s();
  size_t size() const;
  gtsam::BetweenFactorPose3* at(size_t i) const;
  void push_back(const gtsam::BetweenFactorPose3* factor);
};

#include <gtsam/slam/InitializePose3.h>
class InitializePose3 {
  static gtsam::Values computeOrientationsChordal(
      const gtsam::NonlinearFactorGraph& pose3Graph);
  static gtsam::Values computeOrientationsGradient(
      const gtsam::NonlinearFactorGraph& pose3Graph,
      const gtsam::Values& givenGuess, size_t maxIter, const bool setRefFrame);
  static gtsam::Values computeOrientationsGradient(
      const gtsam::NonlinearFactorGraph& pose3Graph,
      const gtsam::Values& givenGuess);
  static gtsam::NonlinearFactorGraph buildPose3graph(
      const gtsam::NonlinearFactorGraph& graph);
  static gtsam::Values initializeOrientations(
      const gtsam::NonlinearFactorGraph& graph);
  static gtsam::Values initialize(const gtsam::NonlinearFactorGraph& graph,
                                  const gtsam::Values& givenGuess,
                                  bool useGradient);
  static gtsam::Values initialize(const gtsam::NonlinearFactorGraph& graph);
};

gtsam::BetweenFactorPose3s parse3DFactors(string filename);
pair<gtsam::NonlinearFactorGraph*, gtsam::Values*> load3D(string filename);

pair<gtsam::NonlinearFactorGraph*, gtsam::Values*> readG2o(string filename);
pair<gtsam::NonlinearFactorGraph*, gtsam::Values*> readG2o(string filename, bool is3D);
void writeG2o(const gtsam::NonlinearFactorGraph& graph,
    const gtsam::Values& estimate, string filename);

#include <gtsam/slam/KarcherMeanFactor-inl.h>
template<T = {gtsam::Point2, gtsam::Rot2, gtsam::Pose2, gtsam::Point3, gtsam::SO3, gtsam::SO4, gtsam::Rot3, gtsam::Pose3}>
virtual class KarcherMeanFactor : gtsam::NonlinearFactor {
  KarcherMeanFactor(const gtsam::KeyVector& keys);
};

#include <gtsam/slam/FrobeniusFactor.h>
gtsam::noiseModel::Isotropic* ConvertPose3NoiseModel(
    gtsam::noiseModel::Base* model, size_t d);

template<T = {gtsam::SO3, gtsam::SO4}>
virtual class FrobeniusFactor : gtsam::NoiseModelFactor {
  FrobeniusFactor(size_t key1, size_t key2);
  FrobeniusFactor(size_t key1, size_t key2, gtsam::noiseModel::Base* model);

  Vector evaluateError(const T& R1, const T& R2);
};

template<T = {gtsam::SO3, gtsam::SO4}>
virtual class FrobeniusBetweenFactor : gtsam::NoiseModelFactor {
  FrobeniusBetweenFactor(size_t key1, size_t key2, const T& R12);
  FrobeniusBetweenFactor(size_t key1, size_t key2, const T& R12, gtsam::noiseModel::Base* model);

  Vector evaluateError(const T& R1, const T& R2);
};

virtual class FrobeniusWormholeFactor : gtsam::NoiseModelFactor {
  FrobeniusWormholeFactor(size_t key1, size_t key2, const gtsam::Rot3& R12,
                          size_t p);
  FrobeniusWormholeFactor(size_t key1, size_t key2, const gtsam::Rot3& R12,
                          size_t p, gtsam::noiseModel::Base* model);
  Vector evaluateError(const gtsam::SOn& Q1, const gtsam::SOn& Q2);
};

//*************************************************************************
// Navigation
//*************************************************************************
namespace imuBias {
#include <gtsam/navigation/ImuBias.h>

class ConstantBias {
  // Constructors
  ConstantBias();
  ConstantBias(Vector biasAcc, Vector biasGyro);

  // Testable
  void print(string s) const;
  bool equals(const gtsam::imuBias::ConstantBias& expected, double tol) const;

  // Group
  static gtsam::imuBias::ConstantBias identity();
  gtsam::imuBias::ConstantBias inverse() const;
  gtsam::imuBias::ConstantBias compose(const gtsam::imuBias::ConstantBias& b) const;
  gtsam::imuBias::ConstantBias between(const gtsam::imuBias::ConstantBias& b) const;

  // Manifold
  gtsam::imuBias::ConstantBias retract(Vector v) const;
  Vector localCoordinates(const gtsam::imuBias::ConstantBias& b) const;

  // Lie Group
  static gtsam::imuBias::ConstantBias Expmap(Vector v);
  static Vector Logmap(const gtsam::imuBias::ConstantBias& b);

  // Standard Interface
  Vector vector() const;
  Vector accelerometer() const;
  Vector gyroscope() const;
  Vector correctAccelerometer(Vector measurement) const;
  Vector correctGyroscope(Vector measurement) const;
};

}///\namespace imuBias

#include <gtsam/navigation/NavState.h>
class NavState {
  // Constructors
  NavState();
  NavState(const gtsam::Rot3& R, const gtsam::Point3& t, Vector v);
  NavState(const gtsam::Pose3& pose, Vector v);

  // Testable
  void print(string s) const;
  bool equals(const gtsam::NavState& expected, double tol) const;

  // Access
  gtsam::Rot3 attitude() const;
  gtsam::Point3 position() const;
  Vector velocity() const;
  gtsam::Pose3 pose() const;
};

#include <gtsam/navigation/PreintegratedRotation.h>
virtual class PreintegratedRotationParams {
  PreintegratedRotationParams();

  // Testable
  void print(string s) const;
  bool equals(const gtsam::PreintegratedRotationParams& expected, double tol);

  void setGyroscopeCovariance(Matrix cov);
  void setOmegaCoriolis(Vector omega);
  void setBodyPSensor(const gtsam::Pose3& pose);

  Matrix getGyroscopeCovariance() const;

  // TODO(frank): allow optional
  //  boost::optional<Vector> getOmegaCoriolis() const;
  //  boost::optional<Pose3>   getBodyPSensor()   const;
};

#include <gtsam/navigation/PreintegrationParams.h>
virtual class PreintegrationParams : gtsam::PreintegratedRotationParams {
  PreintegrationParams(Vector n_gravity);

  static gtsam::PreintegrationParams* MakeSharedD(double g);
  static gtsam::PreintegrationParams* MakeSharedU(double g);
  static gtsam::PreintegrationParams* MakeSharedD();  // default g = 9.81
  static gtsam::PreintegrationParams* MakeSharedU();  // default g = 9.81

  // Testable
  void print(string s) const;
  bool equals(const gtsam::PreintegrationParams& expected, double tol);

  void setAccelerometerCovariance(Matrix cov);
  void setIntegrationCovariance(Matrix cov);
  void setUse2ndOrderCoriolis(bool flag);

  Matrix getAccelerometerCovariance() const;
  Matrix getIntegrationCovariance()   const;
  bool   getUse2ndOrderCoriolis()     const;
};

#include <gtsam/navigation/ImuFactor.h>
class PreintegratedImuMeasurements {
  // Constructors
  PreintegratedImuMeasurements(const gtsam::PreintegrationParams* params);
  PreintegratedImuMeasurements(const gtsam::PreintegrationParams* params,
      const gtsam::imuBias::ConstantBias& bias);

  // Testable
  void print(string s) const;
  bool equals(const gtsam::PreintegratedImuMeasurements& expected, double tol);

  // Standard Interface
  void integrateMeasurement(Vector measuredAcc, Vector measuredOmega,
      double deltaT);
  void resetIntegration();
  void resetIntegrationAndSetBias(const gtsam::imuBias::ConstantBias& biasHat);

  Matrix preintMeasCov() const;
  double deltaTij() const;
  gtsam::Rot3 deltaRij() const;
  Vector deltaPij() const;
  Vector deltaVij() const;
  gtsam::imuBias::ConstantBias biasHat() const;
  Vector biasHatVector() const;
  gtsam::NavState predict(const gtsam::NavState& state_i,
      const gtsam::imuBias::ConstantBias& bias) const;
};

virtual class ImuFactor: gtsam::NonlinearFactor {
  ImuFactor(size_t pose_i, size_t vel_i, size_t pose_j, size_t vel_j,
      size_t bias,
      const gtsam::PreintegratedImuMeasurements& preintegratedMeasurements);

  // Standard Interface
  gtsam::PreintegratedImuMeasurements preintegratedMeasurements() const;
  Vector evaluateError(const gtsam::Pose3& pose_i, Vector vel_i,
      const gtsam::Pose3& pose_j, Vector vel_j,
      const gtsam::imuBias::ConstantBias& bias);
};

#include <gtsam/navigation/CombinedImuFactor.h>
virtual class PreintegrationCombinedParams : gtsam::PreintegrationParams {
  PreintegrationCombinedParams(Vector n_gravity);

  static gtsam::PreintegrationCombinedParams* MakeSharedD(double g);
  static gtsam::PreintegrationCombinedParams* MakeSharedU(double g);
  static gtsam::PreintegrationCombinedParams* MakeSharedD();  // default g = 9.81
  static gtsam::PreintegrationCombinedParams* MakeSharedU();  // default g = 9.81

  // Testable
  void print(string s) const;
  bool equals(const gtsam::PreintegrationCombinedParams& expected, double tol);

  void setBiasAccCovariance(Matrix cov);
  void setBiasOmegaCovariance(Matrix cov);
  void setBiasAccOmegaInt(Matrix cov);
  
  Matrix getBiasAccCovariance() const ;
  Matrix getBiasOmegaCovariance() const ;
  Matrix getBiasAccOmegaInt() const;
 
};

class PreintegratedCombinedMeasurements {
// Constructors
  PreintegratedCombinedMeasurements(const gtsam::PreintegrationCombinedParams* params);
  PreintegratedCombinedMeasurements(const gtsam::PreintegrationCombinedParams* params,
				    const gtsam::imuBias::ConstantBias& bias);
  // Testable
  void print(string s) const;
  bool equals(const gtsam::PreintegratedCombinedMeasurements& expected,
      double tol);

  // Standard Interface
  void integrateMeasurement(Vector measuredAcc, Vector measuredOmega,
      double deltaT);
  void resetIntegration();
  void resetIntegrationAndSetBias(const gtsam::imuBias::ConstantBias& biasHat);

  Matrix preintMeasCov() const;
  double deltaTij() const;
  gtsam::Rot3 deltaRij() const;
  Vector deltaPij() const;
  Vector deltaVij() const;
  gtsam::imuBias::ConstantBias biasHat() const;
  Vector biasHatVector() const;
  gtsam::NavState predict(const gtsam::NavState& state_i,
      const gtsam::imuBias::ConstantBias& bias) const;
};

virtual class CombinedImuFactor: gtsam::NonlinearFactor {
  CombinedImuFactor(size_t pose_i, size_t vel_i, size_t pose_j, size_t vel_j,
      size_t bias_i, size_t bias_j,
      const gtsam::PreintegratedCombinedMeasurements& CombinedPreintegratedMeasurements);

  // Standard Interface
  gtsam::PreintegratedCombinedMeasurements preintegratedMeasurements() const;
  Vector evaluateError(const gtsam::Pose3& pose_i, Vector vel_i,
      const gtsam::Pose3& pose_j, Vector vel_j,
      const gtsam::imuBias::ConstantBias& bias_i,
      const gtsam::imuBias::ConstantBias& bias_j);
};

#include <gtsam/navigation/AHRSFactor.h>
class PreintegratedAhrsMeasurements {
  // Standard Constructor
  PreintegratedAhrsMeasurements(Vector bias, Matrix measuredOmegaCovariance);
  PreintegratedAhrsMeasurements(const gtsam::PreintegratedAhrsMeasurements& rhs);

  // Testable
  void print(string s) const;
  bool equals(const gtsam::PreintegratedAhrsMeasurements& expected, double tol);

  // get Data
  gtsam::Rot3 deltaRij() const;
  double deltaTij() const;
  Vector biasHat() const;

  // Standard Interface
  void integrateMeasurement(Vector measuredOmega, double deltaT);
  void resetIntegration() ;
};

virtual class AHRSFactor : gtsam::NonlinearFactor {
  AHRSFactor(size_t rot_i, size_t rot_j,size_t bias,
      const gtsam::PreintegratedAhrsMeasurements& preintegratedMeasurements, Vector omegaCoriolis);
  AHRSFactor(size_t rot_i, size_t rot_j, size_t bias,
      const gtsam::PreintegratedAhrsMeasurements& preintegratedMeasurements, Vector omegaCoriolis,
      const gtsam::Pose3& body_P_sensor);

  // Standard Interface
  gtsam::PreintegratedAhrsMeasurements preintegratedMeasurements() const;
  Vector evaluateError(const gtsam::Rot3& rot_i, const gtsam::Rot3& rot_j,
      Vector bias) const;
  gtsam::Rot3 predict(const gtsam::Rot3& rot_i, Vector bias,
      const gtsam::PreintegratedAhrsMeasurements& preintegratedMeasurements,
      Vector omegaCoriolis) const;
};

#include <gtsam/navigation/AttitudeFactor.h>
//virtual class AttitudeFactor : gtsam::NonlinearFactor {
//  AttitudeFactor(const Unit3& nZ, const Unit3& bRef);
//  AttitudeFactor();
//};
virtual class Rot3AttitudeFactor : gtsam::NonlinearFactor{
  Rot3AttitudeFactor(size_t key, const gtsam::Unit3& nZ, const gtsam::noiseModel::Diagonal* model,
      const gtsam::Unit3& bRef);
  Rot3AttitudeFactor(size_t key, const gtsam::Unit3& nZ, const gtsam::noiseModel::Diagonal* model);
  Rot3AttitudeFactor();
  void print(string s) const;
  bool equals(const gtsam::NonlinearFactor& expected, double tol) const;
  gtsam::Unit3 nZ() const;
  gtsam::Unit3 bRef() const;
};

virtual class Pose3AttitudeFactor : gtsam::NonlinearFactor {
  Pose3AttitudeFactor(size_t key, const gtsam::Unit3& nZ,
                      const gtsam::noiseModel::Diagonal* model,
                      const gtsam::Unit3& bRef);
  Pose3AttitudeFactor(size_t key, const gtsam::Unit3& nZ,
                      const gtsam::noiseModel::Diagonal* model);
  Pose3AttitudeFactor();
  void print(string s) const;
  bool equals(const gtsam::NonlinearFactor& expected, double tol) const;
  gtsam::Unit3 nZ() const;
  gtsam::Unit3 bRef() const;
};

#include <gtsam/navigation/GPSFactor.h>
virtual class GPSFactor : gtsam::NonlinearFactor{
  GPSFactor(size_t key, const gtsam::Point3& gpsIn,
            const gtsam::noiseModel::Base* model);

  // Testable
  void print(string s) const;
  bool equals(const gtsam::GPSFactor& expected, double tol);

  // Standard Interface
  gtsam::Point3 measurementIn() const;
};

virtual class GPSFactor2 : gtsam::NonlinearFactor {
  GPSFactor2(size_t key, const gtsam::Point3& gpsIn,
            const gtsam::noiseModel::Base* model);

  // Testable
  void print(string s) const;
  bool equals(const gtsam::GPSFactor2& expected, double tol);

  // Standard Interface
  gtsam::Point3 measurementIn() const;
};

#include <gtsam/navigation/Scenario.h>
virtual class Scenario {
  gtsam::Pose3 pose(double t) const;
  Vector omega_b(double t) const;
  Vector velocity_n(double t) const;
  Vector acceleration_n(double t) const;
  gtsam::Rot3 rotation(double t) const;
  gtsam::NavState navState(double t) const;
  Vector velocity_b(double t) const;
  Vector acceleration_b(double t) const;
};

virtual class ConstantTwistScenario : gtsam::Scenario {
  ConstantTwistScenario(Vector w, Vector v);
  ConstantTwistScenario(Vector w, Vector v,
                        const gtsam::Pose3& nTb0);
};

virtual class AcceleratingScenario : gtsam::Scenario {
  AcceleratingScenario(const gtsam::Rot3& nRb, const gtsam::Point3& p0,
                       Vector v0, Vector a_n,
                       Vector omega_b);
};

#include <gtsam/navigation/ScenarioRunner.h>
class ScenarioRunner {
  ScenarioRunner(const gtsam::Scenario& scenario,
                 const gtsam::PreintegrationParams* p,
                 double imuSampleTime,
                 const gtsam::imuBias::ConstantBias& bias);
  Vector gravity_n() const;
  Vector actualAngularVelocity(double t) const;
  Vector actualSpecificForce(double t) const;
  Vector measuredAngularVelocity(double t) const;
  Vector measuredSpecificForce(double t) const;
  double imuSampleTime() const;
  gtsam::PreintegratedImuMeasurements integrate(
      double T, const gtsam::imuBias::ConstantBias& estimatedBias,
      bool corrupted) const;
  gtsam::NavState predict(
      const gtsam::PreintegratedImuMeasurements& pim,
      const gtsam::imuBias::ConstantBias& estimatedBias) const;
  Matrix estimateCovariance(
      double T, size_t N,
      const gtsam::imuBias::ConstantBias& estimatedBias) const;
  Matrix estimateNoiseCovariance(size_t N) const;
};

//*************************************************************************
// Utilities
//*************************************************************************

namespace utilities {

  #include <gtsam/nonlinear/utilities.h>
  gtsam::KeyList createKeyList(Vector I);
  gtsam::KeyList createKeyList(string s, Vector I);
  gtsam::KeyVector createKeyVector(Vector I);
  gtsam::KeyVector createKeyVector(string s, Vector I);
  gtsam::KeySet createKeySet(Vector I);
  gtsam::KeySet createKeySet(string s, Vector I);
  Matrix extractPoint2(const gtsam::Values& values);
  Matrix extractPoint3(const gtsam::Values& values);
  Matrix extractPose2(const gtsam::Values& values);
  gtsam::Values allPose3s(gtsam::Values& values);
  Matrix extractPose3(const gtsam::Values& values);
  void perturbPoint2(gtsam::Values& values, double sigma, int seed);
  void perturbPose2 (gtsam::Values& values, double sigmaT, double sigmaR, int seed);
  void perturbPoint3(gtsam::Values& values, double sigma, int seed);
  void insertBackprojections(gtsam::Values& values, const gtsam::PinholeCameraCal3_S2& c, Vector J, Matrix Z, double depth);
  void insertProjectionFactors(gtsam::NonlinearFactorGraph& graph, size_t i, Vector J, Matrix Z, const gtsam::noiseModel::Base* model, const gtsam::Cal3_S2* K);
  void insertProjectionFactors(gtsam::NonlinearFactorGraph& graph, size_t i, Vector J, Matrix Z, const gtsam::noiseModel::Base* model, const gtsam::Cal3_S2* K, const gtsam::Pose3& body_P_sensor);
  Matrix reprojectionErrors(const gtsam::NonlinearFactorGraph& graph, const gtsam::Values& values);
  gtsam::Values localToWorld(const gtsam::Values& local, const gtsam::Pose2& base);
  gtsam::Values localToWorld(const gtsam::Values& local, const gtsam::Pose2& base, const gtsam::KeyVector& keys);

} //\namespace utilities

#include <gtsam/nonlinear/utilities.h>
class RedirectCout {
  RedirectCout();
  string str();
};

}
