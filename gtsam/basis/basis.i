//*************************************************************************
// basis
//*************************************************************************

#include <gtsam/basis/Fourier.h>

class FourierBasis {
  static Vector CalculateWeights(size_t N, double x);
  static Matrix WeightMatrix(size_t N, Vector x);

  static Matrix DifferentiationMatrix(size_t N);
  static Vector DerivativeWeights(size_t N, double x);
};

#include <gtsam/basis/Chebyshev.h>

class Chebyshev1Basis {
  static Matrix CalculateWeights(size_t N, double x);
  static Matrix WeightMatrix(size_t N, Vector X);

  // TODO need support for nested classes
  // class Derivative {
  //   Derivative(size_t N, double x);
  // };
};

class Chebyshev2Basis {
  static Matrix CalculateWeights(size_t N, double x);
  static Matrix WeightMatrix(size_t N, Vector x);
};

#include <gtsam/basis/Chebyshev2.h>
class Chebyshev2 {
  static double Point(size_t N, int j);
  static double Point(size_t N, int j, double a, double b);

  static Vector Points(size_t N);
  static Vector Points(size_t N, double a, double b);

  static Matrix WeightMatrix(size_t N, Vector X);
  static Matrix WeightMatrix(size_t N, Vector X, double a, double b);

  static Matrix CalculateWeights(size_t N, double x, double a, double b);
  static Matrix DerivativeWeights(size_t N, double x, double a, double b);
  static Matrix IntegrationWeights(size_t N, double a, double b);
  static Matrix DifferentiationMatrix(size_t N, double a, double b);

  // TODO Needs OptionalJacobian
  // static double Derivative(double x, Vector f);
};

#include <gtsam/basis/ParameterMatrix.h>

template <M = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15}>
class ParameterMatrix {
  ParameterMatrix(const size_t N);
  ParameterMatrix(const Matrix& matrix);

  Matrix matrix() const;

  void print(const string& s="") const;
};
