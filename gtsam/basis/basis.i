//*************************************************************************
// basis
//*************************************************************************

namespace gtsam {

// TODO(gerry): add all the Functors to the Basis interfaces, e.g.
// `EvaluationFunctor`

#include <gtsam/basis/Fourier.h>

class FourierBasis {
  static gtsam::Vector CalculateWeights(size_t N, double x);
  static gtsam::Matrix WeightMatrix(size_t N, gtsam::Vector x);

  static gtsam::Matrix DifferentiationMatrix(size_t N);
  static gtsam::Vector DerivativeWeights(size_t N, double x);
};

#include <gtsam/basis/Chebyshev.h>

class Chebyshev1Basis {
  static gtsam::Matrix CalculateWeights(size_t N, double x);
  static gtsam::Matrix WeightMatrix(size_t N, gtsam::Vector X);
};

class Chebyshev2Basis {
  static gtsam::Matrix CalculateWeights(size_t N, double x);
  static gtsam::Matrix WeightMatrix(size_t N, gtsam::Vector x);
};

#include <gtsam/basis/Chebyshev2.h>
class Chebyshev2 {
  static double Point(size_t N, int j);
  static double Point(size_t N, int j, double a, double b);

  static gtsam::Vector Points(size_t N);
  static gtsam::Vector Points(size_t N, double a, double b);

  static gtsam::Matrix WeightMatrix(size_t N, gtsam::Vector X);
  static gtsam::Matrix WeightMatrix(size_t N, gtsam::Vector X, double a, double b);

  static gtsam::Matrix CalculateWeights(size_t N, double x, double a, double b);
  static gtsam::Matrix DerivativeWeights(size_t N, double x, double a, double b);
  static gtsam::Matrix IntegrationWeights(size_t N, double a, double b);
  static gtsam::Matrix DifferentiationMatrix(size_t N, double a, double b);
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

template <BASIS = {gtsam::FourierBasis, gtsam::Chebyshev1Basis,
                   gtsam::Chebyshev2Basis, gtsam::Chebyshev2}>
virtual class VectorEvaluationFactor : gtsam::NoiseModelFactor {
  VectorEvaluationFactor();
  VectorEvaluationFactor(gtsam::Key key, const gtsam::Vector& z,
                         const gtsam::noiseModel::Base* model, const size_t M,
                         const size_t N, double x);
  VectorEvaluationFactor(gtsam::Key key, const gtsam::Vector& z,
                         const gtsam::noiseModel::Base* model, const size_t M,
                         const size_t N, double x, double a, double b);
};

template <BASIS = {gtsam::FourierBasis, gtsam::Chebyshev1Basis,
                   gtsam::Chebyshev2Basis, gtsam::Chebyshev2}>
virtual class VectorComponentFactor : gtsam::NoiseModelFactor {
  VectorComponentFactor();
  VectorComponentFactor(gtsam::Key key, const double z,
                        const gtsam::noiseModel::Base* model, const size_t M,
                        const size_t N, size_t i, double x);
  VectorComponentFactor(gtsam::Key key, const double z,
                        const gtsam::noiseModel::Base* model, const size_t M,
                        const size_t N, size_t i, double x, double a, double b);
};

#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Pose3.h>

template <BASIS = {gtsam::FourierBasis, gtsam::Chebyshev1Basis,
                   gtsam::Chebyshev2Basis, gtsam::Chebyshev2},
          T = {gtsam::Rot2, gtsam::Rot3, gtsam::Pose2, gtsam::Pose3}>
virtual class ManifoldEvaluationFactor : gtsam::NoiseModelFactor {
  ManifoldEvaluationFactor();
  ManifoldEvaluationFactor(gtsam::Key key, const T& z,
                           const gtsam::noiseModel::Base* model, const size_t N,
                           double x);
  ManifoldEvaluationFactor(gtsam::Key key, const T& z,
                           const gtsam::noiseModel::Base* model, const size_t N,
                           double x, double a, double b);
};

template <BASIS = {gtsam::FourierBasis, gtsam::Chebyshev1Basis,
                   gtsam::Chebyshev2Basis, gtsam::Chebyshev2}>
virtual class DerivativeFactor : gtsam::NoiseModelFactor {
  DerivativeFactor();
  DerivativeFactor(gtsam::Key key, const double z,
                   const gtsam::noiseModel::Base* model, const size_t N,
                   double x);
  DerivativeFactor(gtsam::Key key, const double z,
                   const gtsam::noiseModel::Base* model, const size_t N,
                   double x, double a, double b);
};

template <BASIS = {gtsam::FourierBasis, gtsam::Chebyshev1Basis,
                   gtsam::Chebyshev2Basis, gtsam::Chebyshev2}>
virtual class VectorDerivativeFactor : gtsam::NoiseModelFactor {
  VectorDerivativeFactor();
  VectorDerivativeFactor(gtsam::Key key, const gtsam::Vector& z,
                         const gtsam::noiseModel::Base* model, const size_t M,
                         const size_t N, double x);
  VectorDerivativeFactor(gtsam::Key key, const gtsam::Vector& z,
                         const gtsam::noiseModel::Base* model, const size_t M,
                         const size_t N, double x, double a, double b);
};

template <BASIS = {gtsam::FourierBasis, gtsam::Chebyshev1Basis,
                   gtsam::Chebyshev2Basis, gtsam::Chebyshev2}>
virtual class ComponentDerivativeFactor : gtsam::NoiseModelFactor {
  ComponentDerivativeFactor();
  ComponentDerivativeFactor(gtsam::Key key, const double z,
                            const gtsam::noiseModel::Base* model,
                            const size_t P, const size_t N, size_t i, double x);
  ComponentDerivativeFactor(gtsam::Key key, const double z,
                            const gtsam::noiseModel::Base* model,
                            const size_t P, const size_t N, size_t i, double x,
                            double a, double b);
};

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
