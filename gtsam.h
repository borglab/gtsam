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
 *     - C/C++ basic types: string, bool, size_t, int, double, char
 *     - void
 *     - Any class with which be copied with boost::make_shared()
 *     - boost::shared_ptr of any object type
 *   Limitations on methods
 *     - Parsing does not support overloading
 *     - There can only be one method with a given name
 *   Arguments to functions any of
 *   	 - Eigen types:       Matrix, Vector
 *   	 - Eigen types and classes as an optionally const reference
 *     - C/C++ basic types: string, bool, size_t, int, double, char
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

//*************************************************************************
// geometry
//*************************************************************************

class Point2 {
  Point2();
  Point2(double x, double y);
  static gtsam::Point2 Expmap(Vector v);
	static Vector Logmap(const gtsam::Point2& p);
	void print(string s) const;
	double x();
	double y();
  Vector localCoordinates(const gtsam::Point2& p);
	gtsam::Point2 compose(const gtsam::Point2& p2);
	gtsam::Point2 between(const gtsam::Point2& p2);
  gtsam::Point2 retract(Vector v);
};

class Point3 {
	Point3();
	Point3(double x, double y, double z);
	Point3(Vector v);
	static gtsam::Point3 Expmap(Vector v);
	static Vector Logmap(const gtsam::Point3& p);
	void print(string s) const;
	bool equals(const gtsam::Point3& p, double tol);
	Vector vector() const;
	double x();
	double y();
	double z();
	Vector localCoordinates(const gtsam::Point3& p);
	gtsam::Point3 retract(Vector v);
	gtsam::Point3 compose(const gtsam::Point3& p2);
	gtsam::Point3 between(const gtsam::Point3& p2);
};

class Rot2 {
	Rot2();
	Rot2(double theta);
	static gtsam::Rot2 Expmap(Vector v);
	static Vector Logmap(const gtsam::Rot2& p);
	static gtsam::Rot2 fromAngle(double theta);
	static gtsam::Rot2 fromDegrees(double theta);
	static gtsam::Rot2 fromCosSin(double c, double s);
	static gtsam::Rot2 relativeBearing(const gtsam::Point2& d); // Ignoring derivative
	static gtsam::Rot2 atan2(double y, double x);
	void print(string s) const;
	bool equals(const gtsam::Rot2& rot, double tol) const;
	double theta() const;
	double degrees() const;
	double c() const;
	double s() const;
	Vector localCoordinates(const gtsam::Rot2& p);
	gtsam::Rot2 retract(Vector v);
	gtsam::Rot2 compose(const gtsam::Rot2& p2);
	gtsam::Rot2 between(const gtsam::Rot2& p2);
};

class Rot3 {
	Rot3();
	Rot3(Matrix R);
  static gtsam::Rot3 Rx(double t);
  static gtsam::Rot3 Ry(double t);
  static gtsam::Rot3 Rz(double t);
//  static gtsam::Rot3 RzRyRx(double x, double y, double z); // FIXME: overloaded functions don't work yet
  static gtsam::Rot3 RzRyRx(Vector xyz);
  static gtsam::Rot3 yaw  (double t); // positive yaw is to right (as in aircraft heading)
  static gtsam::Rot3 pitch(double t); // positive pitch is up (increasing aircraft altitude)
  static gtsam::Rot3 roll (double t); // positive roll is to right (increasing yaw in aircraft)
  static gtsam::Rot3 ypr(double y, double p, double r);
  static gtsam::Rot3 quaternion(double w, double x, double y, double z);
  static gtsam::Rot3 rodriguez(Vector v);
	void print(string s) const;
	bool equals(const gtsam::Rot3& rot, double tol) const;
  static gtsam::Rot3 identity();
  gtsam::Rot3 compose(const gtsam::Rot3& p2) const;
  gtsam::Rot3 inverse() const;
	gtsam::Rot3 between(const gtsam::Rot3& p2) const;
	gtsam::Point3 rotate(const gtsam::Point3& p) const;
	gtsam::Point3 unrotate(const gtsam::Point3& p) const;
	gtsam::Rot3 retractCayley(Vector v) const;
	gtsam::Rot3 retract(Vector v) const;
	Vector localCoordinates(const gtsam::Rot3& p) const;
	static gtsam::Rot3 Expmap(Vector v);
	static Vector Logmap(const gtsam::Rot3& p);
	Matrix matrix() const;
	Matrix transpose() const;
	gtsam::Point3 column(int index) const;
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
	Pose2(double theta, const gtsam::Point2& t);
	Pose2(const gtsam::Rot2& r, const gtsam::Point2& t);
	Pose2(Vector v);
	static gtsam::Pose2 Expmap(Vector v);
	static Vector Logmap(const gtsam::Pose2& p);
	void print(string s) const;
	bool equals(const gtsam::Pose2& pose, double tol) const;
	double x() const;
	double y() const;
	double theta() const;
	int dim() const;
	Vector localCoordinates(const gtsam::Pose2& p);
	gtsam::Pose2 retract(Vector v);
	gtsam::Pose2 compose(const gtsam::Pose2& p2);
	gtsam::Pose2 between(const gtsam::Pose2& p2);
	gtsam::Rot2 bearing(const gtsam::Point2& point);
	double range(const gtsam::Point2& point);
	gtsam::Point2 translation() const;
	gtsam::Rot2 rotation() const;
};

class Pose3 {
	Pose3();
	Pose3(const gtsam::Rot3& r, const gtsam::Point3& t);
	Pose3(Matrix t);
	Pose3(const gtsam::Pose2& pose2);
	static gtsam::Pose3 Expmap(Vector v);
	static Vector Logmap(const gtsam::Pose3& p);
	void print(string s) const;
	bool equals(const gtsam::Pose3& pose, double tol) const;
	double x() const;
	double y() const;
	double z() const;
	Matrix matrix() const;
	Matrix adjointMap() const;
	gtsam::Pose3 compose(const gtsam::Pose3& p2);
	gtsam::Pose3 between(const gtsam::Pose3& p2);
	gtsam::Pose3 retract(Vector v);
	gtsam::Pose3 retractFirstOrder(Vector v);
  Vector localCoordinates(const gtsam::Pose3& T2) const;
	gtsam::Point3 translation() const;
	gtsam::Rot3 rotation() const;
};

class CalibratedCamera {
    
    CalibratedCamera();
    CalibratedCamera(const gtsam::Pose3& pose);
    CalibratedCamera(const Vector& v);

    void print(string s) const;
    bool equals(const gtsam::CalibratedCamera& camera, double tol) const;

    gtsam::Pose3 pose() const;

    gtsam::CalibratedCamera compose(const gtsam::CalibratedCamera& c) const;
    gtsam::CalibratedCamera inverse() const;
    gtsam::CalibratedCamera level(const gtsam::Pose2& pose2, double height);
    gtsam::CalibratedCamera retract(const Vector& d) const;
    Vector localCoordinates(const gtsam::CalibratedCamera& T2) const;

    gtsam::Point2 project(const gtsam::Point3& point) const;
    static gtsam::Point2 project_to_camera(const gtsam::Point3& cameraPoint);
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
	VectorValues(int nVars, int varDim);
	void print(string s) const;
	bool equals(const gtsam::VectorValues& expected, double tol) const;
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
	bool equals(const gtsam::GaussianConditional &cg, double tol) const;
};

class GaussianDensity {
	GaussianDensity(int key, Vector d, Matrix R, Vector sigmas);
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
	JacobianFactor(int i1, Matrix A1, Vector b, const gtsam::SharedDiagonal& model);
	JacobianFactor(int i1, Matrix A1, int i2, Matrix A2, Vector b,
			const gtsam::SharedDiagonal& model);
	JacobianFactor(int i1, Matrix A1, int i2, Matrix A2, int i3, Matrix A3,
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
	HessianFactor(int j, Matrix G, Vector g, double f);
	HessianFactor(int j, Vector mu, Matrix Sigma);
	HessianFactor(int j1, int j2, Matrix G11, Matrix G12, Vector g1, Matrix G22,
			Vector g2, double f);
	HessianFactor(int j1, int j2, int j3, Matrix G11, Matrix G12, Matrix G13,
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
	int size() const;

	// Building the graph
	void add(gtsam::JacobianFactor* factor);
	// all these won't work as MATLAB can't handle overloading
//	void add(Vector b);
//	void add(int key1, Matrix A1, Vector b, const gtsam::SharedDiagonal& model);
//	void add(int key1, Matrix A1, int key2, Matrix A2, Vector b,
//			const gtsam::SharedDiagonal& model);
//	void add(int key1, Matrix A1, int key2, Matrix A2, int key3, Matrix A3,
//			Vector b, const gtsam::SharedDiagonal& model);
//	void add(gtsam::HessianFactor* factor);

	// error and probability
	double error(const gtsam::VectorValues& c) const;
	double probPrime(const gtsam::VectorValues& c) const;

	// combining
  static gtsam::GaussianFactorGraph combine2(const gtsam::GaussianFactorGraph& lfg1,
      const gtsam::GaussianFactorGraph& lfg2);
	void combine(const gtsam::GaussianFactorGraph& lfg);

	// Conversion to matrices
	Matrix sparseJacobian_() const;
	Matrix denseJacobian() const;
	Matrix denseHessian() const;
};

class GaussianSequentialSolver {
  GaussianSequentialSolver(const gtsam::GaussianFactorGraph& graph, bool useQR);
  gtsam::GaussianBayesNet* eliminate() const;
  gtsam::VectorValues* optimize() const;
  gtsam::GaussianFactor* marginalFactor(int j) const;
  Matrix marginalCovariance(int j) const;
};

class KalmanFilter {
	KalmanFilter(size_t n);
	// gtsam::GaussianDensity* init(Vector x0, const gtsam::SharedDiagonal& P0);
	gtsam::GaussianDensity* init(Vector x0, Matrix P0);
	void print(string s) const;
	static int step(gtsam::GaussianDensity* p);
	gtsam::GaussianDensity* predict(gtsam::GaussianDensity* p, Matrix F, Matrix B, Vector u,
			const gtsam::SharedDiagonal& modelQ);
	gtsam::GaussianDensity* predictQ(gtsam::GaussianDensity* p, Matrix F, Matrix B, Vector u,
			Matrix Q);
	gtsam::GaussianDensity* predict2(gtsam::GaussianDensity* p, Matrix A0, Matrix A1, Vector b,
			const gtsam::SharedDiagonal& model);
	gtsam::GaussianDensity* update(gtsam::GaussianDensity* p, Matrix H, Vector z,
			const gtsam::SharedDiagonal& model);
	gtsam::GaussianDensity* updateQ(gtsam::GaussianDensity* p, Matrix H, Vector z,
	    Matrix Q);
};

//*************************************************************************
// nonlinear
//*************************************************************************

class Symbol {
  Symbol(char c, size_t j);
	void print(string s) const;
  size_t key() const;
};

class Ordering {
	Ordering();
	void print(string s) const;
	bool equals(const gtsam::Ordering& ord, double tol) const;
	void push_back(size_t key);
};

class NonlinearFactorGraph {
    NonlinearFactorGraph();
};

class Values {
    Values();
  	void print(string s) const;
    bool exists(size_t j) const;
};

class Marginals {
	Marginals(const gtsam::NonlinearFactorGraph& graph, const gtsam::Values& solution);
  void print(string s) const;
  Matrix marginalCovariance(size_t variable) const;
  Matrix marginalInformation(size_t variable) const;
};

}///\namespace gtsam

//*************************************************************************
// planarSLAM
//*************************************************************************

#include <gtsam/slam/planarSLAM.h>
namespace planarSLAM {

class Values {
	Values();
	void print(string s) const;
	gtsam::Pose2 pose(int key) const;
	gtsam::Point2 point(int key) const;
	void insertPose(int key, const gtsam::Pose2& pose);
	void insertPoint(int key, const gtsam::Point2& point);
};

class Graph {
	Graph();

	void print(string s) const;

	double error(const planarSLAM::Values& values) const;
	gtsam::Ordering* orderingCOLAMD(const planarSLAM::Values& values) const;
	gtsam::GaussianFactorGraph* linearize(const planarSLAM::Values& values,
			const gtsam::Ordering& ordering) const;

	void addPrior(int key, const gtsam::Pose2& pose, const gtsam::SharedNoiseModel& noiseModel);
	void addPoseConstraint(int key, const gtsam::Pose2& pose);
	void addOdometry(int key1, int key2, const gtsam::Pose2& odometry, const gtsam::SharedNoiseModel& noiseModel);
	void addBearing(int poseKey, int pointKey, const gtsam::Rot2& bearing, const gtsam::SharedNoiseModel& noiseModel);
	void addRange(int poseKey, int pointKey, double range, const gtsam::SharedNoiseModel& noiseModel);
	void addBearingRange(int poseKey, int pointKey, const gtsam::Rot2& bearing, double range,
			const gtsam::SharedNoiseModel& noiseModel);
	planarSLAM::Values optimize(const planarSLAM::Values& initialEstimate);
};

class Odometry {
	Odometry(int key1, int key2, const gtsam::Pose2& measured,
			const gtsam::SharedNoiseModel& model);
	void print(string s) const;
	gtsam::GaussianFactor* linearize(const planarSLAM::Values& center, const gtsam::Ordering& ordering) const;
};

}///\namespace planarSLAM

//*************************************************************************
// Pose2SLAM
//*************************************************************************

#include <gtsam/slam/pose2SLAM.h>
namespace pose2SLAM {

class Values {
  Values();
  void print(string s) const;
  void insertPose(int key, const gtsam::Pose2& pose);
  gtsam::Pose2 pose(int i);
};

class Graph {
  Graph();

  void print(string s) const;

  double error(const pose2SLAM::Values& values) const;
  gtsam::Ordering* orderingCOLAMD(const pose2SLAM::Values& values) const;
  gtsam::GaussianFactorGraph* linearize(const pose2SLAM::Values& values,
      const gtsam::Ordering& ordering) const;

  void addPrior(int key, const gtsam::Pose2& pose, const gtsam::SharedNoiseModel& noiseModel);
  void addPoseConstraint(int key, const gtsam::Pose2& pose);
  void addOdometry(int key1, int key2, const gtsam::Pose2& odometry, const gtsam::SharedNoiseModel& noiseModel);
  pose2SLAM::Values optimize(const pose2SLAM::Values& initialEstimate) const;
  gtsam::Marginals marginals(const pose2SLAM::Values& solution) const;
};

}///\namespace pose2SLAM

//*************************************************************************
// Simulated2D
//*************************************************************************

#include <gtsam/slam/simulated2D.h>
namespace simulated2D {

class Values {
	Values();
	void insertPose(int i, const gtsam::Point2& p);
	void insertPoint(int j, const gtsam::Point2& p);
	int nrPoses() const;
	int nrPoints() const;
	gtsam::Point2 pose(int i);
	gtsam::Point2 point(int j);
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
	void insertPose(int i, const gtsam::Pose2& p);
	void insertPoint(int j, const gtsam::Point2& p);
	int nrPoses() const;
	int nrPoints() const;
	gtsam::Pose2 pose(int i);
	gtsam::Point2 point(int j);
};

class Graph {
	Graph();
};

// TODO: add factors, etc.

}///\namespace simulated2DOriented
