// comments!

class VectorNotEigen;

namespace gtsam {

#include <gtsam/geometry/Point2.h>
class Point2 {
 Point2();
 Point2(double x, double y);
 double x() const;
 double y() const;
 int dim() const;
 char returnChar() const;
 void argChar(char a) const;
 void argChar(char* a) const;
 void argChar(char& a) const;
 void argChar(char@ a) const;
 void argChar(const char* a) const;
 void argChar(const char& a) const;
 void argChar(const char@ a) const;
 void argUChar(unsigned char a) const;
 void eigenArguments(Vector v, Matrix m) const;
 VectorNotEigen vectorConfusion();

 void serializable() const; // Sets flag and creates export, but does not make serialization functions
};

#include <gtsam/geometry/Point3.h>
class Point3 {
  Point3(double x, double y, double z);
  double norm() const;

  // static functions - use static keyword and uppercase
  static double staticFunction();
  static gtsam::Point3 StaticFunctionRet(double z);

  // enabling serialization functionality
  void serialize() const; // Just triggers a flag internally and removes actual function
};

}
// another comment
