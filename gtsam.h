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
 *     - C/C++ basic types: string, bool, size_t, int, double
 *     - void
 *     - Any class with which be copied with boost::make_shared()
 *     - boost::shared_ptr of any object type
 *   Limitations on methods
 *     - Parsing does not support overloading
 *     - There can only be one method with a given name
 *   Arguments to functions any of
 *   	 - Eigen types:       Matrix, Vector
 *   	 - Eigen types and classes as an optionally const reference
 *     - C/C++ basic types: string, bool, size_t, int, double
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
 *  - TODO: Handle Rot3M conversions to quaternions
 */

// Everything is in the gtsam namespace, so we avoid copying everything in
using namespace gtsam;

//*************************************************************************
// base
//*************************************************************************

//*************************************************************************
// geometry
//*************************************************************************

class Point2 {
	Point2();
	Point2(double x, double y);
	static Point2 Expmap(Vector v);
	static Vector Logmap(const Point2& p);
	void print(string s) const;
	double x();
	double y();
	Vector localCoordinates(const Point2& p);
	Point2 compose(const Point2& p2);
	Point2 between(const Point2& p2);
	Point2 retract(Vector v);
};

class Point3 {
	Point3();
	Point3(double x, double y, double z);
	Point3(Vector v);
	static Point3 Expmap(Vector v);
	static Vector Logmap(const Point3& p);
	void print(string s) const;
	bool equals(const Point3& p, double tol);
	Vector vector() const;
	double x();
	double y();
	double z();
	Vector localCoordinates(const Point3& p);
	Point3 retract(Vector v);
	Point3 compose(const Point3& p2);
	Point3 between(const Point3& p2);
};

class Rot2 {
	Rot2();
	Rot2(double theta);
	static Rot2 Expmap(Vector v);
	static Vector Logmap(const Rot2& p);
	static Rot2 fromAngle(double theta);
	static Rot2 fromDegrees(double theta);
	static Rot2 fromCosSin(double c, double s);
	static Rot2 relativeBearing(const Point2& d); // Ignoring derivative
	static Rot2 atan2(double y, double x);
	void print(string s) const;
	bool equals(const Rot2& rot, double tol) const;
	double theta() const;
	double degrees() const;
	double c() const;
	double s() const;
	Vector localCoordinates(const Rot2& p);
	Rot2 retract(Vector v);
	Rot2 compose(const Rot2& p2);
	Rot2 between(const Rot2& p2);
};

class Rot3 {
	Rot3();
	Rot3(Matrix R);
  static Rot3 Rx(double t);
  static Rot3 Ry(double t);
  static Rot3 Rz(double t);
//  static Rot3 RzRyRx(double x, double y, double z); // FIXME: overloaded functions don't work yet
  static Rot3 RzRyRx(Vector xyz);
  static Rot3 yaw  (double t); // positive yaw is to right (as in aircraft heading)
  static Rot3 pitch(double t); // positive pitch is up (increasing aircraft altitude)
  static Rot3 roll (double t); // positive roll is to right (increasing yaw in aircraft)
  static Rot3 ypr(double y, double p, double r);
  static Rot3 quaternion(double w, double x, double y, double z);
  static Rot3 rodriguez(Vector v);
	void print(string s) const;
	bool equals(const Rot3& rot, double tol) const;
  static Rot3 identity();
	Rot3 compose(const Rot3& p2) const;
	Rot3 inverse() const;
	Rot3 between(const Rot3& p2) const;
	Point3 rotate(const Point3& p) const;
	Point3 unrotate(const Point3& p) const;
	Rot3 retractCayley(Vector v) const;
	Rot3 retract(Vector v) const;
	Vector localCoordinates(const Rot3& p) const;
	static Rot3 Expmap(Vector v);
	static Vector Logmap(const Rot3& p);
	Matrix matrix() const;
	Matrix transpose() const;
	Point3 column(int index) const;
	Vector xyz() const;
	Vector ypr() const;
	Vector rpy() const;
  double roll() const;
  double pitch() const;
  double yaw() const;
//  Vector toQuaternion() const;  // FIXME: Can't cast to Vector properly
};

class Pose2 {
	Pose2();
	Pose2(double x, double y, double theta);
	Pose2(double theta, const Point2& t);
	Pose2(const Rot2& r, const Point2& t);
	Pose2(Vector v);
	static Pose2 Expmap(Vector v);
	static Vector Logmap(const Pose2& p);
	void print(string s) const;
	bool equals(const Pose2& pose, double tol) const;
	double x() const;
	double y() const;
	double theta() const;
	int dim() const;
	Vector localCoordinates(const Pose2& p);
	Pose2 retract(Vector v);
	Pose2 compose(const Pose2& p2);
	Pose2 between(const Pose2& p2);
	Rot2 bearing(const Point2& point);
	double range(const Point2& point);
	Point2 translation() const;
	Rot2 rotation() const;
};

class Pose3 {
	Pose3();
	Pose3(const Rot3& r, const Point3& t);
	Pose3(Vector v);
	Pose3(Matrix t);
	Pose3(const Pose2& pose2);
	static Pose3 Expmap(Vector v);
	static Vector Logmap(const Pose3& p);
	void print(string s) const;
	bool equals(const Pose3& pose, double tol) const;
	double x() const;
	double y() const;
	double z() const;
	Matrix matrix() const;
	Matrix adjointMap() const;
	Pose3 compose(const Pose3& p2);
	Pose3 between(const Pose3& p2);
	Pose3 retract(Vector v);
	Pose3 retractFirstOrder(Vector v);
	Point3 translation() const;
	Rot3 rotation() const;
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
	static SharedNoiseModel sharedSigmas(Vector sigmas);
	static SharedNoiseModel sharedSigma(size_t dim, double sigma);
	static SharedNoiseModel sharedPrecisions(Vector precisions);
	static SharedNoiseModel sharedPrecision(size_t dim, double precision);
	static SharedNoiseModel sharedUnit(size_t dim);
	static SharedNoiseModel sharedSqrtInformation(Matrix R);
	static SharedNoiseModel sharedCovariance(Matrix covariance);
	void print(string s) const;
};

class VectorValues {
	VectorValues();
	VectorValues(int nVars, int varDim);
	void print(string s) const;
	bool equals(const VectorValues& expected, double tol) const;
	int size() const;
	void insert(int j, Vector value);
};

class GaussianConditional {
	GaussianConditional(int key, Vector d, Matrix R, Vector sigmas);
	GaussianConditional(int key, Vector d, Matrix R, int name1, Matrix S,
			Vector sigmas);
	GaussianConditional(int key, Vector d, Matrix R, int name1, Matrix S,
			int name2, Matrix T, Vector sigmas);
	void print(string s) const;
	bool equals(const GaussianConditional &cg, double tol) const;
};

class GaussianBayesNet {
	GaussianBayesNet();
	void print(string s) const;
	bool equals(const GaussianBayesNet& cbn, double tol) const;
	void push_back(GaussianConditional* conditional);
	void push_front(GaussianConditional* conditional);
};

class GaussianFactor {
	void print(string s) const;
	bool equals(const GaussianFactor& lf, double tol) const;
	double error(const VectorValues& c) const;
};

class JacobianFactor {
	JacobianFactor();
	JacobianFactor(Vector b_in);
	JacobianFactor(int i1, Matrix A1, Vector b, const SharedDiagonal& model);
	JacobianFactor(int i1, Matrix A1, int i2, Matrix A2, Vector b,
			const SharedDiagonal& model);
	JacobianFactor(int i1, Matrix A1, int i2, Matrix A2, int i3, Matrix A3,
			Vector b, const SharedDiagonal& model);
	void print(string s) const;
	bool equals(const GaussianFactor& lf, double tol) const;
	bool empty() const;
	Vector getb() const;
	double error(const VectorValues& c) const;
	GaussianConditional* eliminateFirst();
};

class HessianFactor {
	HessianFactor(const HessianFactor& gf);
	HessianFactor();
	HessianFactor(int j, Matrix G, Vector g, double f);
	HessianFactor(int j, Vector mu, Matrix Sigma);
	HessianFactor(int j1, int j2, Matrix G11, Matrix G12, Vector g1, Matrix G22,
			Vector g2, double f);
	HessianFactor(int j1, int j2, int j3, Matrix G11, Matrix G12, Matrix G13,
			Vector g1, Matrix G22, Matrix G23, Vector g2, Matrix G33, Vector g3,
			double f);
	HessianFactor(const GaussianConditional& cg);
	HessianFactor(const GaussianFactor& factor);
  void print(string s) const;
  bool equals(const GaussianFactor& lf, double tol) const;
  double error(const VectorValues& c) const;
};

class GaussianFactorGraph {
	GaussianFactorGraph();
  GaussianFactorGraph(const GaussianBayesNet& CBN);

  // From FactorGraph
	void push_back(GaussianFactor* factor);
	void print(string s) const;
	bool equals(const GaussianFactorGraph& lfgraph, double tol) const;
	int size() const;

	// Building the graph
	void add(JacobianFactor* factor);
	void add(Vector b);
	void add(int key1, Matrix A1, Vector b, const SharedDiagonal& model);
	void add(int key1, Matrix A1, int key2, Matrix A2, Vector b,
			const SharedDiagonal& model);
	void add(int key1, Matrix A1, int key2, Matrix A2, int key3, Matrix A3,
			Vector b, const SharedDiagonal& model);
	void add(HessianFactor* factor);

	// error and probability
	double error(const VectorValues& c) const;
	double probPrime(const VectorValues& c) const;

	// combining
  static GaussianFactorGraph combine2(const GaussianFactorGraph& lfg1,
      const GaussianFactorGraph& lfg2);
	void combine(const GaussianFactorGraph& lfg);

	// Conversion to matrices
	Matrix sparseJacobian_() const;
	Matrix denseJacobian() const;
	Matrix denseHessian() const;
};

class GaussianSequentialSolver {
  GaussianSequentialSolver(const GaussianFactorGraph& graph, bool useQR);
  GaussianBayesNet* eliminate() const;
  VectorValues* optimize() const;
  GaussianFactor* marginalFactor(int j) const;
  Matrix marginalCovariance(int j) const;
};

class KalmanFilter {
	KalmanFilter(Vector x0, const SharedDiagonal& P0);
	KalmanFilter(Vector x0, Matrix P0);
	void print(string s) const;
	Vector mean() const;
	Matrix information() const;
	Matrix covariance() const;
	KalmanFilter predict(Matrix F, Matrix B, Vector u, const SharedDiagonal& model);
	KalmanFilter predictQ(Matrix F, Matrix B, Vector u, Matrix Q);
	KalmanFilter predict2(Matrix A0, Matrix A1, Vector b, const SharedDiagonal& model);
	KalmanFilter update(Matrix H, Vector z, const SharedDiagonal& model);
};

//*************************************************************************
// nonlinear
//*************************************************************************

class Ordering {
	Ordering();
	void print(string s) const;
	bool equals(const Ordering& ord, double tol) const;
	void push_back(string key);
};

//*************************************************************************
// slam
//*************************************************************************

// Planar SLAM example domain
#include <gtsam/slam/planarSLAM.h>
namespace planarSLAM {

class Values {
	Values();
	void print(string s) const;
	Pose2 pose(int key) const;
	Point2 point(int key) const;
	void insertPose(int key, const Pose2& pose);
	void insertPoint(int key, const Point2& point);
};

class Graph {
	Graph();

	void print(string s) const;

	double error(const planarSLAM::Values& values) const;
	Ordering* orderingCOLAMD(const planarSLAM::Values& values) const;
	GaussianFactorGraph* linearize(const planarSLAM::Values& values,
			const Ordering& ordering) const;

	void addPrior(int key, const Pose2& pose, const SharedNoiseModel& noiseModel);
	void addPoseConstraint(int key, const Pose2& pose);
	void addOdometry(int key1, int key2, const Pose2& odometry, const SharedNoiseModel& noiseModel);
	void addBearing(int poseKey, int pointKey, const Rot2& bearing, const SharedNoiseModel& noiseModel);
	void addRange(int poseKey, int pointKey, double range, const SharedNoiseModel& noiseModel);
	void addBearingRange(int poseKey, int pointKey, const Rot2& bearing, double range,
			const SharedNoiseModel& noiseModel);
	planarSLAM::Values optimize(const planarSLAM::Values& initialEstimate);
};

class Odometry {
	Odometry(int key1, int key2, const Pose2& measured,
			const SharedNoiseModel& model);
	void print(string s) const;
	GaussianFactor* linearize(const planarSLAM::Values& center, const Ordering& ordering) const;
};

}///\namespace planarSLAM

// Simulated2D Example Domain
#include <gtsam/slam/simulated2D.h>
namespace simulated2D {

class Values {
	Values();
	void insertPose(int i, const Point2& p);
	void insertPoint(int j, const Point2& p);
	int nrPoses() const;
	int nrPoints() const;
	Point2 pose(int i);
	Point2 point(int j);
};

class Graph {
	Graph();
};

// TODO: add factors, etc.

}///\namespace simulated2D

// Simulated2DOriented Example Domain
#include <gtsam/slam/simulated2DOriented.h>
namespace simulated2DOriented {

class Values {
	Values();
	void insertPose(int i, const Pose2& p);
	void insertPoint(int j, const Point2& p);
	int nrPoses() const;
	int nrPoints() const;
	Pose2 pose(int i);
	Point2 point(int j);
};

class Graph {
	Graph();
};

// TODO: add factors, etc.

}///\namespace simulated2DOriented

//// These are considered to be broken and will be added back as they start working
//// It's assumed that there have been interface changes that might break this
//
//class Ordering{
//	Ordering(string key);
//  void print(string s) const;
//  bool equals(const Ordering& ord, double tol) const;
//	Ordering subtract(const Ordering& keys) const;
//	void unique ();
//	void reverse ();
//  void push_back(string s);
//};
//
//class GaussianFactorSet {
//  GaussianFactorSet();
//  void push_back(GaussianFactor* factor);
//};
//
//class Simulated2DPosePrior {
//	Simulated2DPosePrior(Point2& mu, const SharedDiagonal& model, int i);
//  void print(string s) const;
//  double error(const Simulated2DValues& c) const;
//};
//
//class Simulated2DOrientedPosePrior {
//	Simulated2DOrientedPosePrior(Pose2& mu, const SharedDiagonal& model, int i);
//  void print(string s) const;
//  double error(const Simulated2DOrientedValues& c) const;
//};
//
//class Simulated2DPointPrior {
//	Simulated2DPointPrior(Point2& mu, const SharedDiagonal& model, int i);
//  void print(string s) const;
//  double error(const Simulated2DValues& c) const;
//};
//
//class Simulated2DOdometry {
//  Simulated2DOdometry(Point2& mu, const SharedDiagonal& model, int i1, int i2);
//  void print(string s) const;
//  double error(const Simulated2DValues& c) const;
//};
//
//class Simulated2DOrientedOdometry {
//	Simulated2DOrientedOdometry(Pose2& mu, const SharedDiagonal& model, int i1, int i2);
//  void print(string s) const;
//  double error(const Simulated2DOrientedValues& c) const;
//};
//
//class Simulated2DMeasurement {
//  Simulated2DMeasurement(Point2& mu, const SharedDiagonal& model, int i, int j);
//  void print(string s) const;
//  double error(const Simulated2DValues& c) const;
//};
//
//class GaussianFactor {
//	GaussianFactor(string key1,
//			Matrix A1,
//			Vector b_in,
//			const SharedDiagonal& model);
//	GaussianFactor(string key1,
//			Matrix A1,
//			string key2,
//			Matrix A2,
//			Vector b_in,
//			const SharedDiagonal& model);
//	GaussianFactor(string key1,
//			Matrix A1,
//			string key2,
//			Matrix A2,
//			string key3,
//			Matrix A3,
//			Vector b_in,
//			const SharedDiagonal& model);
//	bool involves(string key) const;
//	Matrix getA(string key) const;
//	pair<Matrix,Vector> matrix(const Ordering& ordering) const;
//	pair<GaussianConditional*,GaussianFactor*> eliminate(string key) const;
//};
//
//class GaussianFactorGraph {
//	GaussianConditional* eliminateOne(string key);
//	GaussianBayesNet* eliminate_(const Ordering& ordering);
//	VectorValues* optimize_(const Ordering& ordering);
//	pair<Matrix,Vector> matrix(const Ordering& ordering) const;
//	VectorValues* steepestDescent_(const VectorValues& x0) const;
//	VectorValues* conjugateGradientDescent_(const VectorValues& x0) const;
//};
//
//class Pose2Values{
//	Pose2Values();
//	Pose2 get(string key) const;
//	void insert(string name, const Pose2& val);
//	void print(string s) const;
//	void clear();
//	int size();
//};
//
//class Pose2Factor {
//	Pose2Factor(string key1, string key2,
//			const Pose2& measured, Matrix measurement_covariance);
//	void print(string name) const;
//	double error(const Pose2Values& c) const;
//	size_t size() const;
//	GaussianFactor* linearize(const Pose2Values& config) const;
//};
//
//class Pose2Graph{
//	Pose2Graph();
//	void print(string s) const;
//	GaussianFactorGraph* linearize_(const Pose2Values& config) const;
//	void push_back(Pose2Factor* factor);
//};
//
//class SymbolicFactor{
//	SymbolicFactor(const Ordering& keys);
//	void print(string s) const;
//};
//
//class Simulated2DPosePrior {
//	GaussianFactor* linearize(const Simulated2DValues& config) const;
//};
//
//class Simulated2DOrientedPosePrior {
//	GaussianFactor* linearize(const Simulated2DOrientedValues& config) const;
//};
//
//class Simulated2DPointPrior {
//	GaussianFactor* linearize(const Simulated2DValues& config) const;
//};
//
//class Simulated2DOdometry {
//	GaussianFactor* linearize(const Simulated2DValues& config) const;
//};
//
//class Simulated2DOrientedOdometry {
//	GaussianFactor* linearize(const Simulated2DOrientedValues& config) const;
//};
//
//class Simulated2DMeasurement {
//	GaussianFactor* linearize(const Simulated2DValues& config) const;
//};
//
//class Pose2SLAMOptimizer {
//	Pose2SLAMOptimizer(string dataset_name);
//	void print(string s) const;
//	void update(Vector x) const;
//	Vector optimize() const;
//	double error() const;
//	Matrix a1() const;
//	Matrix a2() const;
//	Vector b1() const;
//	Vector b2() const;
//};
