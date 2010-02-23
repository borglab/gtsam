class SharedGaussian {
	SharedGaussian(Matrix covariance);
	SharedGaussian(Vector sigmas);
};

class SharedDiagonal {
	SharedDiagonal(Vector sigmas);
};

class Ordering {
  Ordering();
  Ordering(string key);
  Ordering subtract(const Ordering& keys) const;
  void print(string s) const;
  bool equals(const Ordering& ord, double tol) const;
  void push_back(string s);
  void unique ();
  void reverse ();
};

class SymbolicFactor{
	SymbolicFactor(const Ordering& keys);
	void print(string s) const;
};

class VectorConfig {
  VectorConfig();
  void print(string s) const;
  bool equals(const VectorConfig& expected, double tol) const;
  void insert(string name, Vector val);
  Vector get(string name) const;
  bool contains(string name) const;
  size_t size() const;
};

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
  void print(string s) const;
  bool equals(const GaussianFactor& lf, double tol) const;
  bool empty() const;
  Vector get_b() const;
  Matrix get_A(string key) const;
  double error(const VectorConfig& c) const;
  bool involves(string key) const;
  pair<Matrix,Vector> matrix(const Ordering& ordering) const;
  pair<GaussianConditional*,GaussianFactor*> eliminate(string key) const;
};

class GaussianFactorSet {
  GaussianFactorSet();
  void push_back(GaussianFactor* factor);
};

class GaussianConditional {
  GaussianConditional();
  GaussianConditional(string key,
  		    Vector d,
		      Matrix R,
		      Vector sigmas);
  GaussianConditional(string key,
  		    Vector d,
		      Matrix R,
		      string name1,
		      Matrix S,
		      Vector sigmas);
  GaussianConditional(string key,
  		    Vector d,
		      Matrix R,
		      string name1,
		      Matrix S,
		      string name2,
		      Matrix T,
		      Vector sigmas);
  void print(string s) const;
  bool equals(const GaussianConditional &cg, double tol) const;
  void add(string key, Matrix S);
  Vector solve(const VectorConfig& x);
};

class GaussianBayesNet {
  GaussianBayesNet();
  void print(string s) const;
  bool equals(const GaussianBayesNet& cbn, double tol) const;
  void push_back(GaussianConditional* conditional);
  void push_front(GaussianConditional* conditional);
};

class GaussianFactorGraph {
  GaussianFactorGraph();
  void print(string s) const;
  bool equals(const GaussianFactorGraph& lfgraph, double tol) const;

  size_t size() const;
  void push_back(GaussianFactor* ptr_f);
  double error(const VectorConfig& c) const;
  double probPrime(const VectorConfig& c) const;
  void combine(const GaussianFactorGraph& lfg);

  GaussianConditional* eliminateOne(string key);
  GaussianBayesNet* eliminate_(const Ordering& ordering);
  VectorConfig* optimize_(const Ordering& ordering);
  pair<Matrix,Vector> matrix(const Ordering& ordering) const;
  Matrix sparse(const Ordering& ordering) const;
  VectorConfig* steepestDescent_(const VectorConfig& x0) const;
  VectorConfig* conjugateGradientDescent_(const VectorConfig& x0) const;
};

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
  Pose2 expmap(const Vector& v) const;
  Vector logmap(const Pose2& pose) const;
  Point2 t() const;
  Rot2 r() const;
};

class Simulated2DConfig {
	Simulated2DConfig();
  void print(string s) const;
	void insertPose(int i, const Point2& p);
	void insertPoint(int j, const Point2& p);
	int nrPoses() const;
	int nrPoints() const;
	Point2* pose(int i);
	Point2* point(int j);
};

class Simulated2DPosePrior {
	Simulated2DPosePrior(Point2& mu, const SharedDiagonal& model, int i);
  void print(string s) const;
	GaussianFactor* linearize(const Simulated2DConfig& config) const;
  double error(const Simulated2DConfig& c) const;
};

class Simulated2DPointPrior {
	Simulated2DPointPrior(Point2& mu, const SharedDiagonal& model, int i);
  void print(string s) const;
	GaussianFactor* linearize(const Simulated2DConfig& config) const;
  double error(const Simulated2DConfig& c) const;
};

class Simulated2DOdometry {
  Simulated2DOdometry(Point2& mu, const SharedDiagonal& model, int i1, int i2);
  void print(string s) const;
	GaussianFactor* linearize(const Simulated2DConfig& config) const;
  double error(const Simulated2DConfig& c) const;
};

class Simulated2DMeasurement {
  Simulated2DMeasurement(Point2& mu, const SharedDiagonal& model, int i, int j);
  void print(string s) const;
	GaussianFactor* linearize(const Simulated2DConfig& config) const;
  double error(const Simulated2DConfig& c) const;
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

