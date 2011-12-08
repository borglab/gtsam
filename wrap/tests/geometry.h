	// comments!

class Point2 {
 Point2();
 Point2(double x, double y);
 double x() const;
 double y() const;
 int dim() const;
 VectorNotEigen vectorConfusion();
};

namespace ns_inner {

class Point3 {
  Point3(double x, double y, double z);
  double norm() const;

  // static functions - use static keyword and uppercase
  static double staticFunction();
  static Point3 StaticFunctionRet(double z);
};

// another comment

// NOTE: you *must* end namespaces as follows:
}//\namespace

// another comment

/**
 * A multi-line comment!
 */

class Test {
	/* a comment! */
	// another comment
  Test();

  pair<Vector,Matrix> return_pair (Vector v, Matrix A) const; // intentionally the first method

  bool   return_bool   (bool   value) const; // comment after a line!
  size_t return_size_t (size_t value) const;
  int    return_int    (int    value) const;
  double return_double (double value) const;

  Test(double a, Matrix b); // a constructor in the middle of a class

  // comments in the middle!

  // (more) comments in the middle!

  string return_string (string value) const;
  Vector return_vector1(Vector value) const;
  Matrix return_matrix1(Matrix value) const;
  Vector return_vector2(Vector value) const;
  Matrix return_matrix2(Matrix value) const;
  void arg_EigenConstRef(const Matrix& value) const;

  bool return_field(const Test& t) const;

  Test* return_TestPtr(Test* value) const;
  Test  return_Test(Test* value) const;

  Point2* return_Point2Ptr(bool value) const;

  pair<Test*,Test*> create_ptrs () const;
  pair<Test ,Test*> create_MixedPtrs () const;
  pair<Test*,Test*> return_ptrs (Test* p1, Test* p2) const;

  void print() const;

  // comments at the end!

  // even more comments at the end!
};

// comments at the end!

// even more comments at the end!
