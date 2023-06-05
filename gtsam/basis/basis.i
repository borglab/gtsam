//*************************************************************************
// basis
//*************************************************************************

namespace gtsam {

// TODO(gerry): add all the Functors to the Basis interfaces, e.g.
// `EvaluationFunctor`

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
};

#include <gtsam/basis/ParameterMatrix.h>

template <M = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15}>
class ParameterMatrix {
  ParameterMatrix(const size_t N);
  ParameterMatrix(const Matrix& matrix);

  Matrix matrix() const;

  void print(const string& s = "") const;
};

#include <gtsam/basis/BasisFactors.h>

template <BASIS = {gtsam::Chebyshev2, gtsam::Chebyshev1Basis,
                   gtsam::Chebyshev2Basis, gtsam::FourierBasis}>
virtual class EvaluationFactor : gtsam::NoiseModelFactor {
  EvaluationFactor();
  EvaluationFactor(gtsam::Key key, const double z,
                   const gtsam::noiseModel::Base* model, const size_t N,
                   double x);
  EvaluationFactor(gtsam::Key key, const double z,
                   const gtsam::noiseModel::Base* model, const size_t N,
                   double x, double a, double b);
};

template <BASIS, M>
virtual class VectorEvaluationFactor : gtsam::NoiseModelFactor {
  VectorEvaluationFactor();
  VectorEvaluationFactor(gtsam::Key key, const Vector& z,
                         const gtsam::noiseModel::Base* model, const size_t N,
                         double x);
  VectorEvaluationFactor(gtsam::Key key, const Vector& z,
                         const gtsam::noiseModel::Base* model, const size_t N,
                         double x, double a, double b);
};

// TODO(Varun) Better way to support arbitrary dimensions?
// Especially if users mainly do `pip install gtsam` for the Python wrapper.
typedef gtsam::VectorEvaluationFactor<gtsam::Chebyshev2, 3>
    VectorEvaluationFactorChebyshev2D3;
typedef gtsam::VectorEvaluationFactor<gtsam::Chebyshev2, 4>
    VectorEvaluationFactorChebyshev2D4;
typedef gtsam::VectorEvaluationFactor<gtsam::Chebyshev2, 12>
    VectorEvaluationFactorChebyshev2D12;

template <BASIS, P>
virtual class VectorComponentFactor : gtsam::NoiseModelFactor {
  VectorComponentFactor();
  VectorComponentFactor(gtsam::Key key, const double z,
                        const gtsam::noiseModel::Base* model, const size_t N,
                        size_t i, double x);
  VectorComponentFactor(gtsam::Key key, const double z,
                        const gtsam::noiseModel::Base* model, const size_t N,
                        size_t i, double x, double a, double b);
};

typedef gtsam::VectorComponentFactor<gtsam::Chebyshev2, 3>
    VectorComponentFactorChebyshev2D3;
typedef gtsam::VectorComponentFactor<gtsam::Chebyshev2, 4>
    VectorComponentFactorChebyshev2D4;
typedef gtsam::VectorComponentFactor<gtsam::Chebyshev2, 12>
    VectorComponentFactorChebyshev2D12;

template <BASIS, T>
virtual class ManifoldEvaluationFactor : gtsam::NoiseModelFactor {
  ManifoldEvaluationFactor();
  ManifoldEvaluationFactor(gtsam::Key key, const T& z,
                           const gtsam::noiseModel::Base* model, const size_t N,
                           double x);
  ManifoldEvaluationFactor(gtsam::Key key, const T& z,
                           const gtsam::noiseModel::Base* model, const size_t N,
                           double x, double a, double b);
};

#include <gtsam/geometry/Pose3.h>

typedef gtsam::ManifoldEvaluationFactor<gtsam::Chebyshev2, gtsam::Rot3>
    ManifoldEvaluationFactorChebyshev2Rot3;
typedef gtsam::ManifoldEvaluationFactor<gtsam::Chebyshev2, gtsam::Pose3>
    ManifoldEvaluationFactorChebyshev2Pose3;

// TODO(gerry): Add `DerivativeFactor`, `VectorDerivativeFactor`, and
// `ComponentDerivativeFactor`

#include <gtsam/basis/FitBasis.h>
template <BASIS = {gtsam::FourierBasis, gtsam::Chebyshev1Basis,
                   gtsam::Chebyshev2Basis, gtsam::Chebyshev2}>
class FitBasis {
  FitBasis(const std::map<double, double>& sequence,
           const gtsam::noiseModel::Base* model, size_t N);

  static gtsam::NonlinearFactorGraph NonlinearGraph(
      const std::map<double, double>& sequence,
      const gtsam::noiseModel::Base* model, size_t N);
  static gtsam::GaussianFactorGraph::shared_ptr LinearGraph(
      const std::map<double, double>& sequence,
      const gtsam::noiseModel::Base* model, size_t N);
  gtsam::This::Parameters parameters() const;
};

}  // namespace gtsam
