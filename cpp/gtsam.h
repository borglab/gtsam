class Ordering {
  Ordering();
  void push_back(string s);
  void print(string s) const;
};

class SymbolicFactor{
	SymbolicFactor(const Ordering& keys);
	void print(string s) const;
};

class VectorConfig {
  VectorConfig();
  Vector get(string name) const;
  bool contains(string name) const;
  size_t size() const;
  void insert(string name, Vector val);
  void print(string s) const;
  bool equals(const VectorConfig& expected, double tol) const;
  void clear();
};

class GaussianFactorSet {
  GaussianFactorSet();
  void push_back(GaussianFactor* factor);
};

class GaussianFactor {
  GaussianFactor(string key1,
	       Matrix A1,
	       Vector b_in,
	       double sigma);
  GaussianFactor(string key1,
	       Matrix A1,
	       string key2,
	       Matrix A2,
	       Vector b_in,
	       double sigma);
  GaussianFactor(string key1,
	       Matrix A1,
	       string key2,
	       Matrix A2,
	       string key3,
	       Matrix A3,
	       Vector b_in,
	       double sigma);
  bool empty() const;
  Vector get_b() const;
  Matrix get_A(string key) const;
  double error(const VectorConfig& c) const;
  bool involves(string key) const;
  void print(string s) const;
  bool equals(const GaussianFactor& lf, double tol) const;
  pair<Matrix,Vector> matrix(const Ordering& ordering) const;
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
  Vector solve(const VectorConfig& x);
  void add(string key, Matrix S);
  bool equals(const GaussianConditional &cg, double tol) const;
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

  size_t size() const;
  void push_back(GaussianFactor* ptr_f);
  double error(const VectorConfig& c) const;
  double probPrime(const VectorConfig& c) const;
  void print(string s) const;
  bool equals(const GaussianFactorGraph& lfgraph, double tol) const;
  void combine(const GaussianFactorGraph& lfg);

  GaussianConditional* eliminateOne(string key);
  GaussianBayesNet* eliminate_(const Ordering& ordering);
  VectorConfig* optimize_(const Ordering& ordering);
  pair<Matrix,Vector> matrix(const Ordering& ordering) const;
  Matrix sparse(const Ordering& ordering) const;
};

class Point2 {
  Point2();
  Point2(double x, double y);
  double x();
  double y();
  size_t dim() const;
  void print(string s) const;
};

class Point3 {
  Point3();
  Point3(double x, double y, double z);
  Point3(Vector v);
  size_t dim() const;
  Point3 exmap(Vector d) const;
  Vector vector() const;
  double x();
  double y();
  double z();
  void print(string s) const;
}; 

class Point2Prior {
  Point2Prior(Vector mu, double sigma, string key);
  Vector error_vector(const VectorConfig& c) const;
  GaussianFactor* linearize(const VectorConfig& c) const;
  double sigma();
  Vector measurement();
  double error(const VectorConfig& c) const;
  void print(string s) const;
};

class Simulated2DOdometry {
  Simulated2DOdometry(Vector odo, double sigma, string key, string key2);
  Vector error_vector(const VectorConfig& c) const;
  GaussianFactor* linearize(const VectorConfig& c) const;
  double sigma();
  Vector measurement();
  double error(const VectorConfig& c) const;
  void print(string s) const;
};

class Simulated2DMeasurement {
  Simulated2DMeasurement(Vector odo, double sigma, string key, string key2);
  Vector error_vector(const VectorConfig& c) const;
  GaussianFactor* linearize(const VectorConfig& c) const;
  double sigma();
  Vector measurement();
  double error(const VectorConfig& c) const;
  void print(string s) const;
};

class Pose2Config{
	Pose2 get(string key) const;
};

class Pose2Constraint {
	Pose2Constraint(string key1, string key2,
			const Pose2& measured, const Matrix& measurement_covariance);
	void print(string name) const;
	bool equals(const Pose2Constraint& expected, double tol) const;
	double error(const Pose2Config& c) const;
	size_t size() const;
	GaussianFactor* linearize(const Pose2Config& config) const;
};
