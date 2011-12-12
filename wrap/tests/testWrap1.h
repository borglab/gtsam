//Header file to test dependency checking
//
class Pose3 {
	Pose3();
	Pose3(const Rot3& r, const Point3& t);//What is Rot3? Throw here
	Pose3(Vector v);
	Pose3(Matrix t);
	static Pose3 Expmap(Vector v);
	static Vector Logmap(const Pose3& p);
	static Rot3 testStaticDep(Rot3& r);//What is Rot3? Throw here
	void print(string s) const;
	bool equals(const Pose3& pose, double tol) const;
	double x() const;
	double y() const;
	double z() const;
	Matrix matrix() const;
	Matrix adjointMap() const;
	Pose3 compose(const Pose3& p2);
	Pose3 between(const Pose3& p2);
	Pose3 retract(Vector v);
	Point3 translation() const;
	Rot3 rotation() const; //What is Rot3? Throw here
};
