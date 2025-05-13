#include "utility.h"

using namespace std;
using namespace gtsam;

/***********************************************************************
                               DEFINITIONS
***********************************************************************/

double det3(Matrix M) {
  double result = 0;
  result += M(0, 0) * M(1, 1) * M(2, 2);
  result += M(1, 0) * M(2, 1) * M(0, 2);
  result += M(2, 0) * M(0, 1) * M(1, 2);
  result -= M(2, 0) * M(1, 1) * M(0, 2);
  result -= M(0, 0) * M(2, 1) * M(1, 2);
  result -= M(1, 0) * M(0, 1) * M(2, 2);
  return result;
}

double l2norm(Vector V) {
  double result = 0;
  for (int i = 0; i < V.size(); i++)
    result += (double)(pow((double)(V(i)), 2.0));
  return (double)(sqrt((double)(result)));
}

Matrix getRotationFromGivens(double thetax, double thetay, double thetaz) {
  // input angles are in radian, returns the rotation matrix
  Matrix myQxT = zeros(3, 3);
  Matrix myQyT = zeros(3, 3);
  Matrix myQzT = zeros(3, 3);
  myQxT(0, 0) = 1.0;
  myQxT(1, 1) = cos(thetax);
  myQxT(1, 2) = sin(thetax);
  myQxT(2, 1) = -sin(thetax);
  myQxT(2, 2) = cos(thetax);
  myQyT(1, 1) = 1.0;
  myQyT(0, 0) = cos(thetay);
  myQyT(0, 2) = -sin(thetay);
  myQyT(2, 0) = sin(thetay);
  myQyT(2, 2) = cos(thetay);
  myQzT(2, 2) = 1.0;
  myQzT(0, 0) = cos(thetaz);
  myQzT(0, 1) = sin(thetaz);
  myQzT(1, 0) = -sin(thetaz);
  myQzT(1, 1) = cos(thetaz);
  return myQzT * myQyT * myQxT;
}

Matrix transformationVectorToMatrix(Vector wTrvector /*6*/) {
  Matrix wTr(4, 4);
  for (int i = 0; i < 4; i++)
    for (int j = 0; j < 4; j++) wTr(i, j) = 0;
  wTr(3, 3) = 1;
  Matrix wRr(3, 3);
  Vector wTrRvector(3);
  for (int i = 0; i < 3; i++) wTrRvector(i) = wTrvector(i + 3);
  wRr = getRotationFromGivens(wTrRvector(0), wTrRvector(1), wTrRvector(2));
  for (int i = 0; i < 3; i++)
    for (int j = 0; j < 3; j++) wTr(i, j) = wRr(i, j);
  for (int i = 0; i < 3; i++) wTr(i, 3) = wTrvector(i);
  return wTr;
}

Vector transformationMatrixToVector(Matrix wTr /*4 by 4*/) {
  Vector wTrvector(6);
  Matrix wRr(3, 3);
  for (int i = 0; i < 3; i++)
    for (int j = 0; j < 3; j++) wRr(i, j) = wTr(i, j);
  Vector wTrRvector(3);
  wTrRvector = RQ(wRr);
  for (int i = 0; i < 3; i++) wTrvector(i + 3) = wTrRvector(i);
  for (int i = 0; i < 3; i++) wTrvector(i) = wTr(i, 3);
  return wTrvector;
}

Matrix Drot3(Matrix R /*3by3*/, Vector x /*3*/) {
  Vector Rx = R * x;
  Vector minusRx = -Rx;
  return skewSymmetric(minusRx);
}

inline Matrix rotationTranslation2Transformation(Matrix R, Vector t) {
  Matrix result(4, 4);
  for (int i = 0; i < 4; i++)
    for (int j = 0; j < 4; j++) result(i, j) = 0;
  result(3, 3) = 1;
  for (int i = 0; i < 3; i++)
    for (int j = 0; j < 3; j++) result(i, j) = R(i, j);
  for (int i = 0; i < 3; i++) result(i, 3) = t(i);
  return result;
}
