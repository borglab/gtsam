// These are considered to be broken and will be added back as they start working
// It's assumed that there have been interface changes that might break this

class Ordering{
	Ordering(string key);
  void print(string s) const;
  bool equals(const Ordering& ord, double tol) const;
	Ordering subtract(const Ordering& keys) const;
	void unique ();
	void reverse ();
  void push_back(string s);
};

class GaussianFactorSet {
  GaussianFactorSet();
  void push_back(GaussianFactor* factor);
};

class Simulated2DValues {
	Simulated2DValues();
  void print(string s) const;
	void insertPose(int i, const Point2& p);
	void insertPoint(int j, const Point2& p);
	int nrPoses() const;
	int nrPoints() const;
	Point2* pose(int i);
	Point2* point(int j);
};

class Simulated2DOrientedValues {
	Simulated2DOrientedValues();
  void print(string s) const;
	void insertPose(int i, const Pose2& p);
	void insertPoint(int j, const Point2& p);
	int nrPoses() const;
	int nrPoints() const;
	Pose2* pose(int i);
	Point2* point(int j);
};

class Simulated2DPosePrior {
	Simulated2DPosePrior(Point2& mu, const SharedDiagonal& model, int i);
  void print(string s) const;
  double error(const Simulated2DValues& c) const;
};

class Simulated2DOrientedPosePrior {
	Simulated2DOrientedPosePrior(Pose2& mu, const SharedDiagonal& model, int i);
  void print(string s) const;
  double error(const Simulated2DOrientedValues& c) const;
};

class Simulated2DPointPrior {
	Simulated2DPointPrior(Point2& mu, const SharedDiagonal& model, int i);
  void print(string s) const;
  double error(const Simulated2DValues& c) const;
};

class Simulated2DOdometry {
  Simulated2DOdometry(Point2& mu, const SharedDiagonal& model, int i1, int i2);
  void print(string s) const;
  double error(const Simulated2DValues& c) const;
};

class Simulated2DOrientedOdometry {
	Simulated2DOrientedOdometry(Pose2& mu, const SharedDiagonal& model, int i1, int i2);
  void print(string s) const;
  double error(const Simulated2DOrientedValues& c) const;
};

class Simulated2DMeasurement {
  Simulated2DMeasurement(Point2& mu, const SharedDiagonal& model, int i, int j);
  void print(string s) const;
  double error(const Simulated2DValues& c) const;
};

// These are currently broken
// Solve by parsing a namespace pose2SLAM::Values and making a Pose2SLAMValues class
// We also have to solve the shared pointer mess to avoid duplicate methods

class GaussianFactor {
	GaussianFactor(string key1,
			Matrix A1,
			Vector b_in,
			const SharedDiagonal& model);
	GaussianFactor(string key1,
			Matrix A1,
			string key2,
			Matrix A2,
			Vector b_in,
			const SharedDiagonal& model);
	GaussianFactor(string key1,
			Matrix A1,
			string key2,
			Matrix A2,
			string key3,
			Matrix A3,
			Vector b_in,
			const SharedDiagonal& model);
	bool involves(string key) const;
	Matrix getA(string key) const;
	pair<Matrix,Vector> matrix(const Ordering& ordering) const;
	pair<GaussianConditional*,GaussianFactor*> eliminate(string key) const;
};

class GaussianFactorGraph {
	GaussianConditional* eliminateOne(string key);
	GaussianBayesNet* eliminate_(const Ordering& ordering);
	VectorValues* optimize_(const Ordering& ordering);
	pair<Matrix,Vector> matrix(const Ordering& ordering) const;
	VectorValues* steepestDescent_(const VectorValues& x0) const;
	VectorValues* conjugateGradientDescent_(const VectorValues& x0) const;
};


class Pose2Values{
	Pose2Values();
	Pose2 get(string key) const;
	void insert(string name, const Pose2& val);
	void print(string s) const;
	void clear();
	int size();
};

class Pose2Factor {
	Pose2Factor(string key1, string key2,
			const Pose2& measured, Matrix measurement_covariance);
	void print(string name) const;
	double error(const Pose2Values& c) const;
	size_t size() const;
	GaussianFactor* linearize(const Pose2Values& config) const;
};

class Pose2Graph{
	Pose2Graph();
	void print(string s) const;
	GaussianFactorGraph* linearize_(const Pose2Values& config) const;
	void push_back(Pose2Factor* factor);
};

class SymbolicFactor{
	SymbolicFactor(const Ordering& keys);
	void print(string s) const;
};

class Simulated2DPosePrior {
	GaussianFactor* linearize(const Simulated2DValues& config) const;
};

class Simulated2DOrientedPosePrior {
	GaussianFactor* linearize(const Simulated2DOrientedValues& config) const;
};

class Simulated2DPointPrior {
	GaussianFactor* linearize(const Simulated2DValues& config) const;
};

class Simulated2DOdometry {
	GaussianFactor* linearize(const Simulated2DValues& config) const;
};

class Simulated2DOrientedOdometry {
	GaussianFactor* linearize(const Simulated2DOrientedValues& config) const;
};

class Simulated2DMeasurement {
	GaussianFactor* linearize(const Simulated2DValues& config) const;
};

class Pose2SLAMOptimizer {
	Pose2SLAMOptimizer(string dataset_name);
	void print(string s) const;
	void update(Vector x) const;
	Vector optimize() const;
	double error() const;
	Matrix a1() const;
	Matrix a2() const;
	Vector b1() const;
	Vector b2() const;
};


