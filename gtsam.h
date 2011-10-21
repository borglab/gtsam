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
	void reserve(size_t nVars, size_t totalDims);
	size_t push_back_preallocated(Vector vector);
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
	double error(const PlanarSLAMValues& c) const;
	Ordering* orderingCOLAMD(const PlanarSLAMValues& config) const;
	void print(string s) const;
	void addPrior(size_t i, const Pose2& p, const SharedNoiseModel& model);
	void addPoseConstraint(size_t i, const Pose2& p);
	void addOdometry(size_t i, size_t j, const Pose2& z,
			const SharedNoiseModel& model);
	void addBearing(size_t i, size_t j, const Rot2& z,
			const SharedNoiseModel& model);
	void addRange(size_t i, size_t j, double z, const SharedNoiseModel& model);
	void addBearingRange(size_t i, size_t j, const Rot2& z1, double z2,
			const SharedNoiseModel& model);
	PlanarSLAMValues* optimize(const PlanarSLAMValues& initialEstimate);
};

