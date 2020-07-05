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
 * @author Varun Agrawal
 * @brief Fit a Basis using least-squares
 */

/*
 *  Concept needed for LS. Parameters = Coefficients | Values
 *    - Parameters, Jacobian
 *    - PredictFactor(double x)(Parameters p, OptionalJacobian<1,N> H)
 */

#pragma once

#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/linear/VectorValues.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

#include "Basis.h"
#include "BasisFactors.h"

namespace gtsam {

/// For now, this is our sequence representation
typedef std::map<double, double> Sequence;

/**
 * Class that does Fourier Decomposition via least squares
 */
template <class Basis>
class FitBasis {
 public:
  typedef std::pair<double, double> Sample;
  typedef typename Basis::Parameters Parameters;

 private:
  Parameters parameters_;

 public:
  /// Create nonlinear FG from Sequence
  static NonlinearFactorGraph NonlinearGraph(const Sequence& sequence,
                                             const SharedNoiseModel& model,
                                             size_t N) {
    NonlinearFactorGraph graph;
    for (const Sample sample : sequence) {
      graph.emplace_shared<PredictFactor<Basis>>(0, sample.second, model, N,
                                                 sample.first);
    }
    return std::move(graph);
  }

  /// Create linear FG from Sequence
  static GaussianFactorGraph::shared_ptr LinearGraph(
      const Sequence& sequence, const SharedNoiseModel& model, size_t N) {
    NonlinearFactorGraph graph = NonlinearGraph(sequence, model, N);
    Values values;
    values.insert<Parameters>(0, Parameters::Zero(N));
    GaussianFactorGraph::shared_ptr gfg = graph.linearize(values);
    return gfg;
  }

  /// Constructor
  FitBasis(size_t N, const Sequence& sequence, const SharedNoiseModel& model) {
    GaussianFactorGraph::shared_ptr gfg = LinearGraph(sequence, model, N);
    VectorValues solution = gfg->optimize();
    parameters_ = solution.at(0);
  }

  /// Return Fourier coefficients
  Parameters parameters() { return parameters_; }
};

}  // namespace gtsam
