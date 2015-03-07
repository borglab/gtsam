//Header file to test dependency checking
//
class Pose3 {
	Pose3(const Rot3& r, const Point3& t); //What is Rot3? Throw here
	static Rot3 testStaticDep(Rot3& r);   //What is Rot3? Throw here
	Rot3 testReturnType() const;          // Throw here
	void testMethodArg(const Rot3& r) const;
};
