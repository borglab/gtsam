class Point2 {
	Point2();
	Point2(double x, double y);
	void print(string s) const;
	double x();
	double y();
};

class Point3 {
	Point3();
	Point3(double x, double y, double z);
	Point3(Vector v);
	void print(string s) const;
	Vector vector() const;
	double x();
	double y();
	double z();
};

class Rot2 {
	Rot2();
	Rot2(double theta);
	void print(string s) const;
	bool equals(const Rot2& pose, double tol) const;
	double c() const;
	double s() const;
};

class Pose2 {
	Pose2();
	Pose2(double x, double y, double theta);
	Pose2(double theta, const Point2& t);
	Pose2(const Rot2& r, const Point2& t);
	Pose2(Vector v);
	void print(string s) const;
	bool equals(const Pose2& pose, double tol) const;
	double x() const;
	double y() const;
	double theta() const;
	int dim() const;
	Pose2* compose_(const Pose2& p2);
	Pose2* between_(const Pose2& p2);
	Vector logmap(const Pose2& p);
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
