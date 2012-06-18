/**
 * GTSAM Wrap Module Definition
 *
 * These are the current classes available through the matlab toolbox interface,
 * add more functions/classes as they are available.
 *
 * Requirements:
 *   Classes must start with an uppercase letter
 *   Only one Method/Constructor per line
 *   Methods can return
 *     - Eigen types:       Matrix, Vector
 *     - C/C++ basic types: string, bool, size_t, size_t, double, char, unsigned char
 *     - void
 *     - Any class with which be copied with boost::make_shared()
 *     - boost::shared_ptr of any object type
 *   Limitations on methods
 *     - Parsing does not support overloading
 *     - There can only be one method (static or otherwise) with a given name
 *   Arguments to functions any of
 *   	 - Eigen types:       Matrix, Vector
 *   	 - Eigen types and classes as an optionally const reference
 *     - C/C++ basic types: string, bool, size_t, size_t, double, char, unsigned char
 *     - Any class with which be copied with boost::make_shared() (except Eigen)
 *     - boost::shared_ptr of any object type (except Eigen)
 *   Comments can use either C++ or C style, with multiple lines
 *   Namespace definitions
 *     - Names of namespaces must start with a lowercase letter
 *   	 - start a namespace with "namespace {"
 *   	 - end a namespace with exactly "}///\namespace [namespace_name]", optionally adding the name of the namespace
 *   	 - This ending is not C++ standard, and must contain "}///\namespace" to parse
 *   	 - Namespaces can be nested
 *   Namespace usage
 *   	 - Namespaces can be specified for classes in arguments and return values
 *   	 - In each case, the namespace must be fully specified, e.g., "namespace1::namespace2::ClassName"
 *   Using namespace
 *   	 - To use a namespace (e.g., generate a "using namespace x" line in cpp files), add "using namespace x;"
 *   	 - This declaration applies to all classes *after* the declaration, regardless of brackets
 *   Methods must start with a lowercase letter
 *   Static methods must start with a letter (upper or lowercase) and use the "static" keyword
 *   Includes in C++ wrappers
 *   	 - By default, the include will be <[classname].h>
 *   	 - All namespaces must have angle brackets: <path>
 *   	 - To override, add a full include statement just before the class statement
 *   	 - An override include can be added for a namespace by placing it just before the namespace statement
 *   	 - Both classes and namespace accept exactly one namespace
 *   Overriding type dependency checks
 *     - If you are using a class 'OtherClass' not wrapped in this definition file, add "class OtherClass;" to avoid a dependency error
 *     - Limitation: this only works if the class does not need a namespace specification
 */

/**
 * Status:
 *  - TODO: global functions
 *  - TODO: default values for arguments
 *  - TODO: overloaded functions
 *  - TODO: signatures for constructors can be ambiguous if two types have the same first letter
 *  - TODO: Handle gtsam::Rot3M conversions to quaternions
 */

namespace gtsam {

//*************************************************************************
// base
//*************************************************************************

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
};

//*************************************************************************
// geometry
//*************************************************************************

class Point2 {
  // Standard Constructors
	Point2();
	Point2(double x, double y);
	Point2(Vector v);

  // Testable
  void print(string s) const;
  bool equals(const gtsam::Point2& pose, double tol) const;

  // Group
  static gtsam::Point2 identity();
  gtsam::Point2 inverse() const;
  gtsam::Point2 compose(const gtsam::Point2& p2) const;
  gtsam::Point2 between(const gtsam::Point2& p2) const;

  // Manifold
  static size_t Dim();
  size_t dim() const;
  gtsam::Point2 retract(Vector v) const;
  Vector localCoordinates(const gtsam::Point2& p) const;

  // Lie Group
  static gtsam::Point2 Expmap(Vector v);
	static Vector Logmap(const gtsam::Point2& p);

  // Standard Interface
  double x() const;
	double y() const;
  Vector vector() const;
};

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
  static size_t Dim();
  size_t dim() const;
  gtsam::StereoPoint2 retract(Vector v) const;
  Vector localCoordinates(const gtsam::StereoPoint2& p) const;

  // Lie Group
  static gtsam::StereoPoint2 Expmap(Vector v);
  static Vector Logmap(const gtsam::StereoPoint2& p);

  // Standard Interface
  Vector vector() const;
};

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
  gtsam::Point3 inverse() const;
  gtsam::Point3 compose(const gtsam::Point3& p2) const;
  gtsam::Point3 between(const gtsam::Point3& p2) const;

  // Manifold
  static size_t Dim();
  size_t dim() const;
  gtsam::Point3 retract(Vector v) const;
  Vector localCoordinates(const gtsam::Point3& p) const;

  // Lie Group
  static gtsam::Point3 Expmap(Vector v);
	static Vector Logmap(const gtsam::Point3& p);

  // Standard Interface
	Vector vector() const;
	double x() const;
	double y() const;
	double z() const;
};

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
  static size_t Dim();
  size_t dim() const;
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
};

class Rot3 {
  // Standard Constructors and Named Constructors
	Rot3();
	Rot3(Matrix R);
	static gtsam::Rot3 Rx(double t);
	static gtsam::Rot3 Ry(double t);
	static gtsam::Rot3 Rz(double t);
//  static gtsam::Rot3 RzRyRx(double x, double y, double z); // FIXME: overloaded functions don't work yet
	static gtsam::Rot3 RzRyRx(Vector xyz);
	static gtsam::Rot3 yaw(double t); // positive yaw is to right (as in aircraft heading)
	static gtsam::Rot3 pitch(double t); // positive pitch is up (increasing aircraft altitude)
	static gtsam::Rot3 roll(double t); // positive roll is to right (increasing yaw in aircraft)
	static gtsam::Rot3 ypr(double y, double p, double r);
	static gtsam::Rot3 quaternion(double w, double x, double y, double z);
	static gtsam::Rot3 rodriguez(Vector v);

  // Testable
	void print(string s) const;
	bool equals(const gtsam::Rot3& rot, double tol) const;

  // Group
	static gtsam::Rot3 identity();
  gtsam::Rot3 inverse() const;
	gtsam::Rot3 compose(const gtsam::Rot3& p2) const;
	gtsam::Rot3 between(const gtsam::Rot3& p2) const;

  // Manifold
  static size_t Dim();
  size_t dim() const;
  // gtsam::Rot3 retractCayley(Vector v) const; // FIXME, does not exist in both Matrix and Quaternion options
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
};

class Pose2 {
  // Standard Constructor
	Pose2();
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
  static size_t Dim();
  size_t dim() const;
  gtsam::Pose2 retract(Vector v) const;
  Vector localCoordinates(const gtsam::Pose2& p) const;

  // Lie Group
	static gtsam::Pose2 Expmap(Vector v);
	static Vector Logmap(const gtsam::Pose2& p);
  Matrix adjointMap() const;
  Vector adjoint(const Vector& xi) const;
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
};

class Pose3 {
	// Standard Constructors
	Pose3();
	Pose3(const gtsam::Pose3& pose);
	Pose3(const gtsam::Rot3& r, const gtsam::Point3& t);
	// Pose3(const gtsam::Pose2& pose2); // FIXME: shadows Pose3(Pose3 pose)
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
	static size_t Dim();
	size_t dim() const;
	gtsam::Pose3 retract(Vector v) const;
	gtsam::Pose3 retractFirstOrder(Vector v) const;
	Vector localCoordinates(const gtsam::Pose3& T2) const;

	// Lie Group
	static gtsam::Pose3 Expmap(Vector v);
	static Vector Logmap(const gtsam::Pose3& p);
	Matrix adjointMap() const;
	Vector adjoint(const Vector& xi) const;
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
	// gtsam::Pose3 transform_to(const gtsam::Pose3& pose) const; // FIXME: shadows other transform_to()
	double range(const gtsam::Point3& point);
	// double range(const gtsam::Pose3& pose); // FIXME: shadows other range
};

class Cal3_S2 {
  // Standard Constructors
  Cal3_S2();
  Cal3_S2(double fx, double fy, double s, double u0, double v0);

  // Testable
  void print(string s) const;
  bool equals(const gtsam::Cal3_S2& pose, double tol) const;

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
  Matrix matrix() const;
  Matrix matrix_inverse() const;
};

class Cal3_S2Stereo {
  // Standard Constructors
  Cal3_S2Stereo();
  Cal3_S2Stereo(double fx, double fy, double s, double u0, double v0, double b);

  // Testable
  void print(string s) const;
  bool equals(const gtsam::Cal3_S2Stereo& pose, double tol) const;

  // Standard Interface
  double fx() const;
  double fy() const;
  double skew() const;
  double px() const;
  double py() const;
  gtsam::Point2 principalPoint() const;
  double baseline() const;
};

class CalibratedCamera {
  // Standard Constructors and Named Constructors
	CalibratedCamera();
	CalibratedCamera(const gtsam::Pose3& pose);
	CalibratedCamera(const Vector& v);
  gtsam::CalibratedCamera level(const gtsam::Pose2& pose2, double height);

  // Testable
	void print(string s) const;
	bool equals(const gtsam::CalibratedCamera& camera, double tol) const;

  // Manifold
  static size_t Dim();
  size_t dim() const;
  gtsam::CalibratedCamera retract(const Vector& d) const;
  Vector localCoordinates(const gtsam::CalibratedCamera& T2) const;

  // Group
  gtsam::CalibratedCamera compose(const gtsam::CalibratedCamera& c) const;
  gtsam::CalibratedCamera inverse() const;

  // Action on Point3
  gtsam::Point2 project(const gtsam::Point3& point) const;
  static gtsam::Point2 project_to_camera(const gtsam::Point3& cameraPoint);

  // Standard Interface
	gtsam::Pose3 pose() const;
  double range(const gtsam::Point3& p) const; // TODO: Other overloaded range methods
};


class SimpleCamera {
  // Standard Constructors and Named Constructors
	SimpleCamera();
  SimpleCamera(const gtsam::Pose3& pose);
  SimpleCamera(const gtsam::Pose3& pose, const gtsam::Cal3_S2& K);
  static gtsam::SimpleCamera level(const gtsam::Cal3_S2& K,
      const gtsam::Pose2& pose, double height);
  static gtsam::SimpleCamera level(const gtsam::Pose2& pose, double height);
  static gtsam::SimpleCamera lookat(const gtsam::Point3& eye,
      const gtsam::Point3& target, const gtsam::Point3& upVector,
      const gtsam::Cal3_S2& K);

  // Testable
	void print(string s) const;
	bool equals(const gtsam::SimpleCamera& camera, double tol) const;

  // Standard Interface
  gtsam::Pose3 pose() const;
  gtsam::Cal3_S2 calibration();

	// Manifold
  gtsam::SimpleCamera retract(const Vector& d) const;
  Vector localCoordinates(const gtsam::SimpleCamera& T2) const;
  size_t dim() const;
  static size_t Dim();

  // Transformations and measurement functions
  static gtsam::Point2 project_to_camera(const gtsam::Point3& cameraPoint);
  pair<gtsam::Point2,bool> projectSafe(const gtsam::Point3& pw) const;
	gtsam::Point2 project(const gtsam::Point3& point);
	gtsam::Point3 backproject(const gtsam::Point2& p, double depth) const;
	gtsam::Point3 backproject_from_camera(const gtsam::Point2& p, double depth) const;
  double range(const gtsam::Point3& point);
  // double range(const gtsam::Pose3& point); // FIXME, overload
};

//*************************************************************************
// inference
//*************************************************************************


//*************************************************************************
// linear
//*************************************************************************

class SharedGaussian {
	SharedGaussian(Matrix covariance);
	void print(string s) const;
};

class SharedDiagonal {
	SharedDiagonal(Vector sigmas);
	void print(string s) const;
	Vector sample() const;
};

class SharedNoiseModel {
	static gtsam::SharedNoiseModel Sigmas(Vector sigmas);
	static gtsam::SharedNoiseModel Sigma(size_t dim, double sigma);
	static gtsam::SharedNoiseModel Precisions(Vector precisions);
	static gtsam::SharedNoiseModel Precision(size_t dim, double precision);
	static gtsam::SharedNoiseModel Unit(size_t dim);
	static gtsam::SharedNoiseModel SqrtInformation(Matrix R);
	static gtsam::SharedNoiseModel Covariance(Matrix covariance);
	void print(string s) const;
};

class VectorValues {
	VectorValues();
	VectorValues(size_t nVars, size_t varDim);
	void print(string s) const;
	bool equals(const gtsam::VectorValues& expected, double tol) const;
	size_t size() const;
	void insert(size_t j, Vector value);
};

class GaussianConditional {
	GaussianConditional(size_t key, Vector d, Matrix R, Vector sigmas);
	GaussianConditional(size_t key, Vector d, Matrix R, size_t name1, Matrix S,
			Vector sigmas);
	GaussianConditional(size_t key, Vector d, Matrix R, size_t name1, Matrix S,
			size_t name2, Matrix T, Vector sigmas);
	void print(string s) const;
	bool equals(const gtsam::GaussianConditional &cg, double tol) const;
};

class GaussianDensity {
	GaussianDensity(size_t key, Vector d, Matrix R, Vector sigmas);
	void print(string s) const;
	Vector mean() const;
	Matrix information() const;
	Matrix covariance() const;
};

class GaussianBayesNet {
	GaussianBayesNet();
	void print(string s) const;
	bool equals(const gtsam::GaussianBayesNet& cbn, double tol) const;
	void push_back(gtsam::GaussianConditional* conditional);
	void push_front(gtsam::GaussianConditional* conditional);
};

class GaussianFactor {
	void print(string s) const;
	bool equals(const gtsam::GaussianFactor& lf, double tol) const;
	double error(const gtsam::VectorValues& c) const;
};

class JacobianFactor {
	JacobianFactor();
	JacobianFactor(Vector b_in);
	JacobianFactor(size_t i1, Matrix A1, Vector b,
			const gtsam::SharedDiagonal& model);
	JacobianFactor(size_t i1, Matrix A1, size_t i2, Matrix A2, Vector b,
			const gtsam::SharedDiagonal& model);
	JacobianFactor(size_t i1, Matrix A1, size_t i2, Matrix A2, size_t i3, Matrix A3,
			Vector b, const gtsam::SharedDiagonal& model);
	void print(string s) const;
	bool equals(const gtsam::GaussianFactor& lf, double tol) const;
	bool empty() const;
	Vector getb() const;
	double error(const gtsam::VectorValues& c) const;
	gtsam::GaussianConditional* eliminateFirst();
};

class HessianFactor {
	HessianFactor(const gtsam::HessianFactor& gf);
	HessianFactor();
	HessianFactor(size_t j, Matrix G, Vector g, double f);
	HessianFactor(size_t j, Vector mu, Matrix Sigma);
	HessianFactor(size_t j1, size_t j2, Matrix G11, Matrix G12, Vector g1, Matrix G22,
			Vector g2, double f);
	HessianFactor(size_t j1, size_t j2, size_t j3, Matrix G11, Matrix G12, Matrix G13,
			Vector g1, Matrix G22, Matrix G23, Vector g2, Matrix G33, Vector g3,
			double f);
	HessianFactor(const gtsam::GaussianConditional& cg);
	HessianFactor(const gtsam::GaussianFactor& factor);
	void print(string s) const;
	bool equals(const gtsam::GaussianFactor& lf, double tol) const;
	double error(const gtsam::VectorValues& c) const;
};

class GaussianFactorGraph {
	GaussianFactorGraph();
	GaussianFactorGraph(const gtsam::GaussianBayesNet& CBN);

	// From FactorGraph
	void push_back(gtsam::GaussianFactor* factor);
	void print(string s) const;
	bool equals(const gtsam::GaussianFactorGraph& lfgraph, double tol) const;
	size_t size() const;

	// Building the graph
	void add(gtsam::JacobianFactor* factor);
	// all these won't work as MATLAB can't handle overloading
//	void add(Vector b);
//	void add(size_t key1, Matrix A1, Vector b, const gtsam::SharedDiagonal& model);
//	void add(size_t key1, Matrix A1, size_t key2, Matrix A2, Vector b,
//			const gtsam::SharedDiagonal& model);
//	void add(size_t key1, Matrix A1, size_t key2, Matrix A2, size_t key3, Matrix A3,
//			Vector b, const gtsam::SharedDiagonal& model);
//	void add(gtsam::HessianFactor* factor);

	// error and probability
	double error(const gtsam::VectorValues& c) const;
	double probPrime(const gtsam::VectorValues& c) const;

	// combining
	static gtsam::GaussianFactorGraph combine2(
			const gtsam::GaussianFactorGraph& lfg1,
			const gtsam::GaussianFactorGraph& lfg2);
	void combine(const gtsam::GaussianFactorGraph& lfg);

	// Conversion to matrices
	Matrix sparseJacobian_() const;
	Matrix denseJacobian() const;
	Matrix denseHessian() const;
};

class GaussianISAM {
  GaussianISAM();
  void saveGraph(string s) const;
  gtsam::GaussianFactor* marginalFactor(size_t j) const;
  gtsam::GaussianBayesNet* marginalBayesNet(size_t key) const;
  Matrix marginalCovariance(size_t key) const;
  gtsam::GaussianBayesNet* jointBayesNet(size_t key1, size_t key2) const;
};

class GaussianSequentialSolver {
	GaussianSequentialSolver(const gtsam::GaussianFactorGraph& graph,
			bool useQR);
	gtsam::GaussianBayesNet* eliminate() const;
	gtsam::VectorValues* optimize() const;
	gtsam::GaussianFactor* marginalFactor(size_t j) const;
	Matrix marginalCovariance(size_t j) const;
};

class KalmanFilter {
	KalmanFilter(size_t n);
	// gtsam::GaussianDensity* init(Vector x0, const gtsam::SharedDiagonal& P0);
	gtsam::GaussianDensity* init(Vector x0, Matrix P0);
	void print(string s) const;
	static size_t step(gtsam::GaussianDensity* p);
	gtsam::GaussianDensity* predict(gtsam::GaussianDensity* p, Matrix F,
			Matrix B, Vector u, const gtsam::SharedDiagonal& modelQ);
	gtsam::GaussianDensity* predictQ(gtsam::GaussianDensity* p, Matrix F,
			Matrix B, Vector u, Matrix Q);
	gtsam::GaussianDensity* predict2(gtsam::GaussianDensity* p, Matrix A0,
			Matrix A1, Vector b, const gtsam::SharedDiagonal& model);
	gtsam::GaussianDensity* update(gtsam::GaussianDensity* p, Matrix H,
			Vector z, const gtsam::SharedDiagonal& model);
	gtsam::GaussianDensity* updateQ(gtsam::GaussianDensity* p, Matrix H,
			Vector z, Matrix Q);
};

//*************************************************************************
// nonlinear
//*************************************************************************

class Symbol {
	Symbol(char c, size_t j);
	Symbol(size_t k);
	void print(string s) const;
	size_t key() const;
	size_t index() const;
	char chr() const;
};

class Ordering {
	Ordering();
	void print(string s) const;
	bool equals(const gtsam::Ordering& ord, double tol) const;
	void push_back(size_t key);
};

class NonlinearFactorGraph {
	NonlinearFactorGraph();
  void print(string s) const;
};

class NonlinearFactor {
//	NonlinearFactor(); // FIXME: don't use this - creates an abstract class
	void print(string s) const;
	void equals(const gtsam::NonlinearFactor& other, double tol) const;
	gtsam::KeyVector keys() const;
	size_t size() const;
	size_t dim() const;
};

class Values {
	Values();
	size_t size() const;
	void print(string s) const;
	bool exists(size_t j) const;
};

// Actually a FastList<Key>
#include <gtsam/nonlinear/Values.h>
class KeyList {
	KeyList();
	KeyList(const gtsam::KeyList& other);

	// Note: no print function
	size_t size() const;
};

// Actually a KeyVector<Key>
#include <gtsam/nonlinear/Values.h>
class KeyVector {
	KeyVector();
	KeyVector(const gtsam::KeyVector& other);

	// Note: no print function
	size_t size() const;
	size_t at(size_t i) const;
};

class Marginals {
	Marginals(const gtsam::NonlinearFactorGraph& graph,
			const gtsam::Values& solution);
	void print(string s) const;
	Matrix marginalCovariance(size_t variable) const;
	Matrix marginalInformation(size_t variable) const;
};

}///\namespace gtsam

//*************************************************************************
// Pose2SLAM
//*************************************************************************

#include <gtsam/slam/pose2SLAM.h>
namespace pose2SLAM {

class Values {
	Values();
	size_t size() const;
	void print(string s) const;
	static pose2SLAM::Values Circle(size_t n, double R);
	void insertPose(size_t key, const gtsam::Pose2& pose);
	void updatePose(size_t key, const gtsam::Pose2& pose);
	gtsam::Pose2 pose(size_t i);
  Vector xs() const;
  Vector ys() const;
  Vector thetas() const;
};

class Graph {
	Graph();

	// FactorGraph
	void print(string s) const;
	bool equals(const pose2SLAM::Graph& fg, double tol) const;
	size_t size() const;
	bool empty() const;
	void remove(size_t i);
	size_t nrFactors() const;
	gtsam::NonlinearFactor* at(size_t i) const;

	// NonlinearFactorGraph
	double error(const pose2SLAM::Values& values) const;
	double probPrime(const pose2SLAM::Values& values) const;
	gtsam::Ordering* orderingCOLAMD(const pose2SLAM::Values& values) const;
	gtsam::GaussianFactorGraph* linearize(const pose2SLAM::Values& values,
			const gtsam::Ordering& ordering) const;

	// pose2SLAM-specific
	void addPrior(size_t key, const gtsam::Pose2& pose, const gtsam::SharedNoiseModel& noiseModel);
	void addPoseConstraint(size_t key, const gtsam::Pose2& pose);
	void addOdometry(size_t key1, size_t key2, const gtsam::Pose2& odometry, const gtsam::SharedNoiseModel& noiseModel);
	void addConstraint(size_t key1, size_t key2, const gtsam::Pose2& odometry, const gtsam::SharedNoiseModel& noiseModel);
	pose2SLAM::Values optimize(const pose2SLAM::Values& initialEstimate) const;
	pose2SLAM::Values optimizeSPCG(const pose2SLAM::Values& initialEstimate) const;
	gtsam::Marginals marginals(const pose2SLAM::Values& solution) const;
};

}///\namespace pose2SLAM

//*************************************************************************
// Pose3SLAM
//*************************************************************************

#include <gtsam/slam/pose3SLAM.h>
namespace pose3SLAM {

class Values {
	Values();
	size_t size() const;
	void print(string s) const;
	static pose3SLAM::Values Circle(size_t n, double R);
	void insertPose(size_t key, const gtsam::Pose3& pose);
	void updatePose(size_t key, const gtsam::Pose3& pose);
	gtsam::Pose3 pose(size_t i);
  Vector xs() const;
  Vector ys() const;
  Vector zs() const;
};

class Graph {
	Graph();

	// FactorGraph
	void print(string s) const;
	bool equals(const pose3SLAM::Graph& fg, double tol) const;
	size_t size() const;
	bool empty() const;
	void remove(size_t i);
	size_t nrFactors() const;
	gtsam::NonlinearFactor* at(size_t i) const;

	// NonlinearFactorGraph
	double error(const pose3SLAM::Values& values) const;
	double probPrime(const pose3SLAM::Values& values) const;
	gtsam::Ordering* orderingCOLAMD(const pose3SLAM::Values& values) const;
	gtsam::GaussianFactorGraph* linearize(const pose3SLAM::Values& values,
			const gtsam::Ordering& ordering) const;

	// pose3SLAM-specific
	void addPrior(size_t key, const gtsam::Pose3& p, const gtsam::SharedNoiseModel& model);
	void addConstraint(size_t key1, size_t key2, const gtsam::Pose3& z, const gtsam::SharedNoiseModel& model);
	void addHardConstraint(size_t i, const gtsam::Pose3& p);
	pose3SLAM::Values optimize(const pose3SLAM::Values& initialEstimate) const;
	gtsam::Marginals marginals(const pose3SLAM::Values& solution) const;
};

}///\namespace pose3SLAM

//*************************************************************************
// planarSLAM
//*************************************************************************

#include <gtsam/slam/planarSLAM.h>
namespace planarSLAM {

class Values {
	Values();
	size_t size() const;
	void print(string s) const;
	void insertPose(size_t key, const gtsam::Pose2& pose);
	void insertPoint(size_t key, const gtsam::Point2& point);
	void updatePose(size_t key, const gtsam::Pose2& pose);
	void updatePoint(size_t key, const gtsam::Point2& point);
	gtsam::Pose2 pose(size_t key) const;
	gtsam::Point2 point(size_t key) const;
};

class Graph {
	Graph();

	// FactorGraph
	void print(string s) const;
	bool equals(const planarSLAM::Graph& fg, double tol) const;
	size_t size() const;
	bool empty() const;
	void remove(size_t i);
	size_t nrFactors() const;
	gtsam::NonlinearFactor* at(size_t i) const;

	// NonlinearFactorGraph
	double error(const planarSLAM::Values& values) const;
	double probPrime(const planarSLAM::Values& values) const;
	gtsam::Ordering* orderingCOLAMD(const planarSLAM::Values& values) const;
	gtsam::GaussianFactorGraph* linearize(const planarSLAM::Values& values,
			const gtsam::Ordering& ordering) const;

	// planarSLAM-specific
	void addPrior(size_t key, const gtsam::Pose2& pose, const gtsam::SharedNoiseModel& noiseModel);
	void addPoseConstraint(size_t key, const gtsam::Pose2& pose);
	void addOdometry(size_t key1, size_t key2, const gtsam::Pose2& odometry, const gtsam::SharedNoiseModel& noiseModel);
	void addBearing(size_t poseKey, size_t pointKey, const gtsam::Rot2& bearing, const gtsam::SharedNoiseModel& noiseModel);
	void addRange(size_t poseKey, size_t pointKey, double range, const gtsam::SharedNoiseModel& noiseModel);
	void addBearingRange(size_t poseKey, size_t pointKey, const gtsam::Rot2& bearing,double range, const gtsam::SharedNoiseModel& noiseModel);
	planarSLAM::Values optimize(const planarSLAM::Values& initialEstimate);
	gtsam::Marginals marginals(const planarSLAM::Values& solution) const;
};

class Odometry {
	Odometry(size_t key1, size_t key2, const gtsam::Pose2& measured,
			const gtsam::SharedNoiseModel& model);
	void print(string s) const;
	gtsam::GaussianFactor* linearize(const planarSLAM::Values& center,
			const gtsam::Ordering& ordering) const;
};

}///\namespace planarSLAM

//*************************************************************************
// VisualSLAM
//*************************************************************************

#include <gtsam/slam/visualSLAM.h>
namespace visualSLAM {

class Values {
  Values();
  void insertPose(size_t key, const gtsam::Pose3& pose);
  void insertPoint(size_t key, const gtsam::Point3& pose);
  void updatePose(size_t key, const gtsam::Pose3& pose);
  void updatePoint(size_t key, const gtsam::Point3& pose);
  size_t size() const;
  size_t nrPoses() const;
  size_t nrPoints() const;
  void print(string s) const;
  gtsam::Pose3 pose(size_t i);
  gtsam::Point3 point(size_t j);
  visualSLAM::Values allPoses() const;
  visualSLAM::Values allPoints() const;
  gtsam::KeyVector keys() const; // Note the switch to KeyVector, rather than KeyList
  bool exists(size_t key);
  Vector xs() const;
  Vector ys() const;
  Vector zs() const;
  Matrix points() const;
};

class Graph {
  Graph();

	// FactorGraph
	void print(string s) const;
	bool equals(const visualSLAM::Graph& fg, double tol) const;
	size_t size() const;
	bool empty() const;
	void remove(size_t i);
	size_t nrFactors() const;
	gtsam::NonlinearFactor* at(size_t i) const;

  double error(const visualSLAM::Values& values) const;
  gtsam::Ordering* orderingCOLAMD(const visualSLAM::Values& values) const;
  gtsam::GaussianFactorGraph* linearize(const visualSLAM::Values& values,
      const gtsam::Ordering& ordering) const;

  // Measurements
  void addMeasurement(const gtsam::Point2& measured, const gtsam::SharedNoiseModel& model,
      size_t poseKey, size_t pointKey, const gtsam::Cal3_S2* K);
  void addStereoMeasurement(const gtsam::StereoPoint2& measured, const gtsam::SharedNoiseModel& model,
      size_t poseKey, size_t pointKey, const gtsam::Cal3_S2Stereo* K);

  // Constraints
  void addPoseConstraint(size_t poseKey, const gtsam::Pose3& p);
  void addPointConstraint(size_t pointKey, const gtsam::Point3& p);

  // Priors
  void addPosePrior(size_t poseKey, const gtsam::Pose3& p, const gtsam::SharedNoiseModel& model);
  void addPointPrior(size_t pointKey, const gtsam::Point3& p, const gtsam::SharedNoiseModel& model);
  void addRangeFactor(size_t poseKey, size_t pointKey, double range, const gtsam::SharedNoiseModel& model);
  void addOdometry(size_t poseKey1, size_t poseKey2, const gtsam::Pose3& odometry, const gtsam::SharedNoiseModel& model);

  visualSLAM::Values optimize(const visualSLAM::Values& initialEstimate) const;
  gtsam::Marginals marginals(const visualSLAM::Values& solution) const;
};

class ISAM {
	ISAM();
	ISAM(int reorderInterval);
  void print(string s) const;
  void printStats() const;
  void saveGraph(string s) const;
	visualSLAM::Values estimate() const;
  Matrix marginalCovariance(size_t key) const;
  int reorderInterval() const;
  int reorderCounter() const;
  void update(const visualSLAM::Graph& newFactors, const visualSLAM::Values& initialValues);
  void reorder_relinearize();
  void addKey(size_t key);
  void setOrdering(const gtsam::Ordering& new_ordering);

  // These might be expensive as instead of a reference the wrapper will make a copy
  gtsam::GaussianISAM bayesTree() const;
  visualSLAM::Values getLinearizationPoint() const;
  gtsam::Ordering getOrdering() const;
  gtsam::NonlinearFactorGraph getFactorsUnsafe() const;
};

}///\namespace visualSLAM
