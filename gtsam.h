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
	Pose2(const Pose2& pose);
	Pose2(double x, double y, double theta);
	Pose2(double theta, const Point2& t);
	Pose2(const Rot2& r, const Point2& t);
	void print(string s) const;
	bool equals(const Pose2& pose, double tol) const;
	double x() const;
	double y() const;
	double theta() const;
	size_t dim() const;
};

class SharedGaussian {
	SharedGaussian(Matrix covariance);
	SharedGaussian(Vector sigmas);
};

class SharedDiagonal {
	SharedDiagonal(Vector sigmas);
};

class VectorValues {
	VectorValues();
	VectorValues(size_t nVars, size_t varDim);
	void print(string s) const;
	bool equals(const VectorValues& expected, double tol) const;
	size_t size() const;
  void insert(size_t j, const Vector& value);
};

class GaussianConditional {
	GaussianConditional(size_t key, Vector d, Matrix R, Vector sigmas);
	GaussianConditional(size_t key, Vector d, Matrix R, size_t name1, Matrix S,
			Vector sigmas);
	GaussianConditional(size_t key, Vector d, Matrix R, size_t name1, Matrix S,
			size_t name2, Matrix T, Vector sigmas);
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
	JacobianFactor(size_t i1, Matrix A1, Vector b, const SharedDiagonal& model);
	JacobianFactor(size_t i1, Matrix A1, size_t i2, Matrix A2, Vector b,
			const SharedDiagonal& model);
	JacobianFactor(size_t i1, Matrix A1, size_t i2, Matrix A2, size_t i3,
			Matrix A3, Vector b, const SharedDiagonal& model);
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

	size_t size() const;
	void push_back(GaussianFactor* ptr_f);
	double error(const VectorValues& c) const;
	double probPrime(const VectorValues& c) const;
	void combine(const GaussianFactorGraph& lfg);
  Matrix denseJacobian() const;
  Matrix denseHessian() const;
	Matrix sparse(Vector columnIndices) const;
};

class Landmark2 {
	Landmark2();
	Landmark2(double x, double y);
	void print(string s) const;
	double x();
	double y();
};

class Ordering{
  void print(string s) const;
  bool equals(const Ordering& ord, double tol) const;
};

class PlanarSLAMValues {
	PlanarSLAMValues();
	void print(string s) const;
	void insertPose(int key, const Pose2& pose);
	void insertPoint(int key, const Point2& point);
};

class PlanarSLAMGraph {
	PlanarSLAMGraph();

	void print(string s) const;

	double error(const PlanarSLAMValues& c) const;
	Ordering* orderingCOLAMD(const PlanarSLAMValues& config) const;
	GaussianFactorGraph* linearize(const PlanarSLAMValues& config, const Ordering& ordering) const;

	void addPrior(size_t i, const Pose2& p, const SharedNoiseModel& model);
	void addPoseConstraint(size_t i, const Pose2& p);
	void addOdometry(size_t i, size_t j, const Pose2& z,
			const SharedNoiseModel& model);
	void addBearing(size_t i, size_t j, const Rot2& z,
			const SharedNoiseModel& model);
	void addRange(size_t i, size_t j, double z, const SharedNoiseModel& model);
	void addBearingRange(size_t i, size_t j, const Rot2& z1, double z2,
			const SharedNoiseModel& model);
	PlanarSLAMValues* optimize_(const PlanarSLAMValues& initialEstimate);
};

