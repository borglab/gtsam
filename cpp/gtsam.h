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
  void push_back(string s);
  void print(string s) const;
  bool equals(const Ordering& ord, double tol) const;
  void unique ();
  void reverse ();
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
  bool empty() const;
  Vector get_b() const;
  Matrix get_A(string key) const;
  double error(const VectorConfig& c) const;
  bool involves(string key) const;
  void print(string s) const;
  bool equals(const GaussianFactor& lf, double tol) const;
  pair<Matrix,Vector> matrix(const Ordering& ordering) const;
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
  VectorConfig* steepestDescent_(const VectorConfig& x0) const;
  VectorConfig* conjugateGradientDescent_(const VectorConfig& x0) const;
};

class Point2 {
  Point2();
  Point2(double x, double y);
  double x();
  double y();
  void print(string s) const;
};

class Point3 {
  Point3();
  Point3(double x, double y, double z);
  Point3(Vector v);
  Vector vector() const;
  double x();
  double y();
  double z();
  void print(string s) const;
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
