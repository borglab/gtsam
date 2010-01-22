// These are currently broken
// Solve by parsing a namespace pose2SLAM::Config and making a Pose2SLAMConfig class
// We also have to solve the shared pointer mess to avoid duplicate methods

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
	Pose2Config();
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
	double error(const Pose2Config& c) const;
	size_t size() const;
	GaussianFactor* linearize(const Pose2Config& config) const;
};

class Pose2Graph{
	Pose2Graph();
	void print(string s) const;
	GaussianFactorGraph* linearize_(const Pose2Config& config) const;
	void push_back(Pose2Factor* factor);
};


