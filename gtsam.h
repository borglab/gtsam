class SharedGaussian {
	SharedGaussian(Matrix covariance);
	SharedGaussian(Vector sigmas);
};

class SharedDiagonal {
	SharedDiagonal(Vector sigmas);
};

class Ordering {
  Ordering();
  void print(string s) const;
  bool equals(const Ordering& ord, double tol) const;
  void push_back(string s);
};


class VectorValues {
  VectorValues();
  void print(string s) const;
  bool equals(const VectorValues& expected, double tol) const;
  size_t size() const;
};

class GaussianFactorSet {
  GaussianFactorSet();
  void push_back(GaussianFactor* factor);
};

class GaussianConditional {
  GaussianConditional();
  void print(string s) const;
  bool equals(const GaussianConditional &cg, double tol) const;
  Vector solve(const VectorValues& x);
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
  double error(const VectorValues& c) const;
  double probPrime(const VectorValues& c) const;
  void combine(const GaussianFactorGraph& lfg);
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

