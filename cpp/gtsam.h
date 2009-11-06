class VectorConfig {
  VectorConfig();
  Vector get(string name) const;
  bool contains(string name) const;
  size_t size() const;
  void insert(string name, Vector val);
  void print() const;
  bool equals(const VectorConfig& expected, double tol) const;
  void clear();
};

class LinearFactorSet {
  LinearFactorSet();
  void push_back(LinearFactor* factor);
};

class LinearFactor {
  LinearFactor(string key1,
	       Matrix A1,
	       Vector b_in,
	       double sigma);
  LinearFactor(string key1,
	       Matrix A1,
	       string key2,
	       Matrix A2,
	       Vector b_in,
	       double sigma);
  LinearFactor(string key1,
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
  void print() const;
  bool equals(const LinearFactor& lf, double tol) const;
  pair<Matrix,Vector> matrix(const Ordering& ordering) const;
};

class ConditionalGaussian {
  ConditionalGaussian();
  ConditionalGaussian(string key,
  		    Vector d,
		      Matrix R,
		      Vector precisions);
  ConditionalGaussian(string key,
  		    Vector d,
		      Matrix R,
		      string name1,
		      Matrix S,
		      Vector precisions);
  ConditionalGaussian(string key,
  		    Vector d,
		      Matrix R,
		      string name1,
		      Matrix S,
		      string name2,
		      Matrix T,
		      Vector precisions);
  void print() const;
  Vector solve(const VectorConfig& x);
  void add(string key, Matrix S);
  bool equals(const ConditionalGaussian &cg) const;
};

class Ordering {
  Ordering();
  void push_back(string s);
  void print() const;
};

class GaussianBayesNet {
  GaussianBayesNet();
  VectorConfig* optimize();
  void print() const;
  bool equals(const GaussianBayesNet& cbn) const;
  pair<Matrix,Vector> matrix() const;
};

class LinearFactorGraph {
  LinearFactorGraph();

  size_t size() const;
  void push_back(LinearFactor* ptr_f);
  double error(const VectorConfig& c) const;
  double probPrime(const VectorConfig& c) const;
  void print() const;
  bool equals(const LinearFactorGraph& lfgraph) const;

  VectorConfig optimize(const Ordering& ordering);
  GaussianBayesNet* eliminate(const Ordering& ordering);
  pair<Matrix,Vector> matrix(const Ordering& ordering) const;
  Matrix sparse(const Ordering& ordering) const;
};

class Point2 {
  Point2();
  Point2(double x, double y);
  double x();
  double y();
  size_t dim() const;
  void print() const;
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
  void print() const;
}; 

class Point2Prior {
  Point2Prior(Vector mu, double sigma, string key);
  Vector error_vector(const VectorConfig& c) const;
  LinearFactor* linearize(const VectorConfig& c) const;
  double sigma();
  Vector measurement();
  double error(const VectorConfig& c) const;
  void print() const;
};

class Simulated2DOdometry {
  Simulated2DOdometry(Vector odo, double sigma, string key, string key2);
  Vector error_vector(const VectorConfig& c) const;
  LinearFactor* linearize(const VectorConfig& c) const;
  double sigma();
  Vector measurement();
  double error(const VectorConfig& c) const;
  void print() const;
};

class Simulated2DMeasurement {
  Simulated2DMeasurement(Vector odo, double sigma, string key, string key2);
  Vector error_vector(const VectorConfig& c) const;
  LinearFactor* linearize(const VectorConfig& c) const;
  double sigma();
  Vector measurement();
  double error(const VectorConfig& c) const;
  void print() const;
};

