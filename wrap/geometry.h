/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */


class Point2 {
 Point2();
 Point2(double x, double y);
 double x();
 double y();
 int dim() const;
};

class Point3 {
  Point3(double x, double y, double z);
  double norm() const;
};

class Test {
  Test();

  bool   return_bool   (bool   value);
  size_t return_size_t (size_t value);
  int    return_int    (int    value);
  double return_double (double value);

  string return_string (string value);
  Vector return_vector1(Vector value);
  Matrix return_matrix1(Matrix value);
  Vector return_vector2(Vector value);
  Matrix return_matrix2(Matrix value);

  pair<Vector,Matrix> return_pair (Vector v, Matrix A);

  bool return_field(const Test& t) const;

  Test* return_TestPtr(Test* value);

  pair<Test*,Test*> create_ptrs ();
  pair<Test*,Test*> return_ptrs (Test* p1, Test* p2);

  void print();
};
