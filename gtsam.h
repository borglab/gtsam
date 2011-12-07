/**
 * GTSAM Wrap Module definition
 *
 * These are the current classes available through the matlab toolbox interface,
 * add more functions/classes as they are available.
 *
 * Requirements:
 *   Constructors must appear in a class before any methods
 *   Methods can only return Matrix, Vector, double, int, void, and a shared_ptr to any other object
 *   Comments can use either C++ or C style
 *   Methods must start with a lowercase letter
 *   Static methods must start with a letter (upper or lowercase) and use the "static" keyword
 *   Classes must start with an uppercase letter
 */

class Point2 {
	Point2();
	Point2(double x, double y);
	static Point2* Expmap_(Vector v);
	static Vector Logmap(const Point2& p);
	void print(string s) const;
	double x();
	double y();
	Point2* compose_(const Point2& p2);
	Point2* between_(const Point2& p2);
	Vector localCoordinates(const Point2& p);
	Point2* retract_(Vector v);
};

class Point3 {
	Point3();
	Point3(double x, double y, double z);
	Point3(Vector v);
	static Point3* Expmap_(Vector v);
	static Vector Logmap(const Point3& p);
	void print(string s) const;
	bool equals(const Point3& p, double tol);
	Vector vector() const;
	double x();
	double y();
	double z();
	Point3* compose_(const Point3& p2);
	Point3* between_(const Point3& p2);
	Vector localCoordinates(const Point3& p);
	Point3* retract_(Vector v);
};

class Rot2 {
	Rot2();
	Rot2(double theta);
	static Rot2* Expmap_(Vector v);
	static Vector Logmap(const Rot2& p);
	void print(string s) const;
	bool equals(const Rot2& rot, double tol) const;
	double c() const;
	double s() const;
	Rot2* compose_(const Rot2& p2);
	Rot2* between_(const Rot2& p2);
	Vector localCoordinates(const Rot2& p);
	Rot2* retract_(Vector v);
};

class Rot3 {
	Rot3();
	Rot3(Matrix R);
	static Rot3* Expmap_(Vector v);
	static Vector Logmap(const Rot3& p);
  static Rot3 ypr(double y, double p, double r);
	Matrix matrix() const;
	Matrix transpose() const;
	Vector xyz() const;
	Vector ypr() const;
	void print(string s) const;
	bool equals(const Rot3& rot, double tol) const;
	Rot3* compose_(const Rot3& p2);
	Rot3* between_(const Rot3& p2);
	Vector localCoordinates(const Rot3& p);
	Rot3* retract_(Vector v);
};

class Pose2 {
	Pose2();
	Pose2(double x, double y, double theta);
	Pose2(double theta, const Point2& t);
	Pose2(const Rot2& r, const Point2& t);
	Pose2(Vector v);
	static Pose2* Expmap_(Vector v);
	static Vector Logmap(const Pose2& p);
	void print(string s) const;
	bool equals(const Pose2& pose, double tol) const;
	double x() const;
	double y() const;
	double theta() const;
	int dim() const;
	Pose2* compose_(const Pose2& p2);
	Pose2* between_(const Pose2& p2);
	Vector localCoordinates(const Pose2& p);
	Pose2* retract_(Vector v);
};

class Pose3 {
	Pose3();
	Pose3(const Rot3& r, const Point3& t);
	Pose3(Vector v);
	Pose3(Matrix t);
	static Pose3* Expmap_(Vector v);
	static Vector Logmap(const Pose3& p);
	void print(string s) const;
	bool equals(const Pose3& pose, double tol) const;
	double x() const;
	double y() const;
	double z() const;
	Matrix matrix() const;
	Matrix adjointMap() const;
	Pose3* compose_(const Pose3& p2);
	Pose3* between_(const Pose3& p2);
	Vector localCoordinates(const Pose3& p);
	Pose3* retract_(Vector v);
	Point3* translation_() const;
	Rot3* rotation_() const;
};

class SharedGaussian {
	SharedGaussian(Matrix covariance);
	void print(string s) const;
};

class SharedDiagonal {
	SharedDiagonal(Vector sigmas);
	void print(string s) const;
	Vector sample() const;
};

class VectorValues {
	VectorValues();
	VectorValues(int nVars, int varDim);
	void print(string s) const;
	bool equals(const VectorValues& expected, double tol) const;
	int size() const;
	void insert(int j, const Vector& value);
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

class GaussianFactorGraph {
	GaussianFactorGraph();
	void print(string s) const;
	bool equals(const GaussianFactorGraph& lfgraph, double tol) const;

	int size() const;
	void push_back(GaussianFactor* ptr_f);
	double error(const VectorValues& c) const;
	double probPrime(const VectorValues& c) const;
	void combine(const GaussianFactorGraph& lfg);
	Matrix denseJacobian() const;
	Matrix denseHessian() const;
	Matrix sparseJacobian_() const;
};

class KalmanFilter {
	KalmanFilter(Vector x, const SharedDiagonal& model);
	void print(string s) const;
	Vector mean() const;
	Matrix information() const;
	Matrix covariance() const;
	void predict(Matrix F, Matrix B, Vector u, const SharedDiagonal& model);
	void predict2(Matrix A0, Matrix A1, Vector b, const SharedDiagonal& model);
	void update(Matrix H, Vector z, const SharedDiagonal& model);
};

class Landmark2 {
	Landmark2();
	Landmark2(double x, double y);
	void print(string s) const;
	double x();
	double y();
};

class Ordering {
	Ordering();
	void print(string s) const;
	bool equals(const Ordering& ord, double tol) const;
	void push_back(string key);
};

class PlanarSLAMValues {
	PlanarSLAMValues();
	void print(string s) const;
	Pose2* pose(int key);
	void insertPose(int key, const Pose2& pose);
	void insertPoint(int key, const Point2& point);
};

class PlanarSLAMGraph {
	PlanarSLAMGraph();

	void print(string s) const;

	double error(const PlanarSLAMValues& values) const;
	Ordering* orderingCOLAMD(const PlanarSLAMValues& values) const;
	GaussianFactorGraph* linearize(const PlanarSLAMValues& values,
			const Ordering& ordering) const;

	void addPrior(int key, const Pose2& pose, const SharedNoiseModel& noiseModel);
	void addPoseConstraint(int key, const Pose2& pose);
	void addOdometry(int key1, int key2, const Pose2& odometry, const SharedNoiseModel& noiseModel);
	void addBearing(int poseKey, int pointKey, const Rot2& bearing, const SharedNoiseModel& noiseModel);
	void addRange(int poseKey, int pointKey, double range, const SharedNoiseModel& noiseModel);
	void addBearingRange(int poseKey, int pointKey, const Rot2& bearing, double range,
			const SharedNoiseModel& noiseModel);
	PlanarSLAMValues* optimize_(const PlanarSLAMValues& initialEstimate);
};

class PlanarSLAMOdometry {
	PlanarSLAMOdometry(int key1, int key2, const Pose2& measured,
			const SharedNoiseModel& model);
	void print(string s) const;
	GaussianFactor* linearize(const PlanarSLAMValues& center, const Ordering& ordering) const;
};

class GaussianSequentialSolver {
  GaussianSequentialSolver(const GaussianFactorGraph& graph, bool useQR);
  GaussianBayesNet* eliminate() const;
//  VectorValues* optimize() const; // FAIL: parse error here
  GaussianFactor* marginalFactor(int j) const;
  Matrix marginalCovariance(int j) const;
};







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
//class Simulated2DValues {
//	Simulated2DValues();
//  void print(string s) const;
//	void insertPose(int i, const Point2& p);
//	void insertPoint(int j, const Point2& p);
//	int nrPoses() const;
//	int nrPoints() const;
//	Point2* pose(int i);
//	Point2* point(int j);
//};
//
//class Simulated2DOrientedValues {
//	Simulated2DOrientedValues();
//  void print(string s) const;
//	void insertPose(int i, const Pose2& p);
//	void insertPoint(int j, const Point2& p);
//	int nrPoses() const;
//	int nrPoints() const;
//	Pose2* pose(int i);
//	Point2* point(int j);
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
//// These are currently broken
//// Solve by parsing a namespace pose2SLAM::Values and making a Pose2SLAMValues class
//// We also have to solve the shared pointer mess to avoid duplicate methods
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
