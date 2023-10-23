/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file FitBasis.h
 * @date July 4, 2020
 * @author Varun Agrawal, Frank Dellaert
 * @brief Fit a Basis using least-squares
 */

/*
 *  Concept needed for LS. Parameters = Coefficients | Values
 *    - Parameters, Jacobian
 *    - PredictFactor(double x)(Parameters p, OptionalJacobian<1,N> H)
 */

#pragma once

#include <gtsam/basis/Basis.h>
#include <gtsam/basis/BasisFactors.h>
#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/linear/VectorValues.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

namespace gtsam {

/// Our sequence representation is a map of {x: y} values where y = f(x)
using Sequence = std::map<double, double>;
/// A sample is a key-value pair from a sequence.
using Sample = std::pair<double, double>;

/**
 * Class that does regression via least squares
 * Example usage:
 *  size_t N = 3;
 *  auto fit = FitBasis<Chebyshev2>(data_points, noise_model, N);
 *  Vector coefficients = fit.parameters();
 *
 * where `data_points` are a map from `x` to `y` values indicating a function
 * mapping at specific points, `noise_model` is the gaussian noise model, and
 * `N` is the degree of the polynomial basis used to fit the function.
 */
template <class Basis>
class FitBasis {
 public:
  using Parameters = typename Basis::Parameters;

 private:
  Parameters parameters_;

 public:
  /// Create nonlinear FG from Sequence
  static NonlinearFactorGraph NonlinearGraph(const Sequence& sequence,
                                             const SharedNoiseModel& model,
                                             size_t N) {
    NonlinearFactorGraph graph;
    for (const Sample sample : sequence) {
      graph.emplace_shared<EvaluationFactor<Basis>>(0, sample.second, model, N,
                                                    sample.first);
    }
    return graph;
  }

  /// Create linear FG from Sequence
  static GaussianFactorGraph::shared_ptr LinearGraph(
      const Sequence& sequence, const SharedNoiseModel& model, size_t N) {
    NonlinearFactorGraph graph = NonlinearGraph(sequence, model, N);
    Values values;
    values.insert<Parameters>(0, Basis::ParameterMatrix(N));
    GaussianFactorGraph::shared_ptr gfg = graph.linearize(values);
    return gfg;
  }

  /**
   * @brief Construct a new FitBasis object.
   *
   * @param sequence map of x->y values for a function, a.k.a. y = f(x).
   * @param model The noise model to use.
   * @param N The degree of the polynomial to fit.
   */
  FitBasis(const Sequence& sequence, const SharedNoiseModel& model, size_t N) {
    GaussianFactorGraph::shared_ptr gfg = LinearGraph(sequence, model, N);
    VectorValues solution = gfg->optimize();
    parameters_ = solution.at(0);
  }

  /// Return Fourier coefficients
  Parameters parameters() const { return parameters_; }
};

}  // namespace gtsam
