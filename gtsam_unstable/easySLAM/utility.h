/***********************************************************************
Written by Alireza Fathi,
Last Modified: Oct 14, 2008.
These are utility functions that I wrote for myself, though since they are
written fantastic! somebody else can use them too!
***********************************************************************/
#pragma once

/*STL/C++*/
#include <gtsam/Matrix.h>
#include <gtsam/Rot3.h>

#include <iostream>
#include <map>
#include <string>
#include <utility>
#include <vector>

// Boost
namespace ublas = boost::numeric::ublas;
// typedef double DATA_TYPE;
// #define Matrix ublas::matrix<DATA_TYPE, ublas::row_major,
// ublas::unbounded_array<DATA_TYPE> > #define Vector ublas::vector<DATA_TYPE,
// ublas::unbounded_array<DATA_TYPE> >

/***********************************************************************
                                Functions
***********************************************************************/

double det3(Matrix M);    // compute the determinant of a 3by3 matrix
double l2norm(Vector V);  // compute the l2norm of a vector

Matrix transformationVectorToMatrix(
    Vector wTrvector /*6*/); /*return 4 by 4 matrix*/
/*
  receives a 6 dimensional vector, which its first 3 dimensions are translation
  and its last 3 dimensions are rotations. This creates a 4by4 transformation
  matrix out of that and returns it.
*/

Vector transformationMatrixToVector(
    Matrix wTr /*4 by 4*/); /*return a 6 dimensional vector*/
/*
  gets a transformation matrix, extracts the rotation and translation parameters
  and returns them in a 6 dimensional vector.
*/

Matrix getRotationFromGivens(double thetax, double thetay, double thetaz);
// input angles are in radian, returns the rotation matrix

Matrix Drot3(Matrix R /*3by3*/, Matrix x /*3*/); /*returns a 3 by 3 matrix*/
/*
  returns the derivative of rotation
*/
inline Matrix rotationTranslation2Transformation(Matrix R /*3by3*/,
                                                 Vector t /*3*/); /*4by4 <--*/

/***********************classes**********************************/
class CantOpenFile : public std::exception {
 private:
  std::string filename_;

 public:
  CantOpenFile(const std::string& filename) : filename_(filename) {}
  ~CantOpenFile() throw() {}
  virtual const char* what() const throw() {
    return ("Can't open file " + filename_).c_str();
  }
};
