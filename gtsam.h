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
  Pose2 expmap(Vector v) const;
  Vector logmap(const Pose2& pose) const;
  Point2 t() const;
  Rot2 r() const;
};

