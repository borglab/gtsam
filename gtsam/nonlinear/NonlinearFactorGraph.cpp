/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    NonlinearFactorGraph.cpp
 * @brief   Factor Graph Consisting of non-linear factors
 * @author  Frank Dellaert
 * @author  Carlos Nieto
 * @author  Christian Potthast
 */

#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/symbolic/SymbolicFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/linear/linearExceptions.h>
#include <gtsam/linear/VectorValues.h>
#include <gtsam/inference/Ordering.h>
#include <gtsam/inference/FactorGraph-inst.h>
#include <gtsam/config.h> // for GTSAM_USE_TBB

#ifdef GTSAM_USE_TBB
#  include <tbb/parallel_for.h>
#endif

#include <algorithm>
#include <cmath>
#include <fstream>
#include <set>

using namespace std;

namespace gtsam {

// Instantiate base classes
template class FactorGraph<NonlinearFactor>;

/* ************************************************************************* */
double NonlinearFactorGraph::probPrime(const Values& values) const {
  // NOTE the 0.5 constant is handled by the factor error.
  return exp(-error(values));
}

/* ************************************************************************* */
void NonlinearFactorGraph::print(const std::string& str, const KeyFormatter& keyFormatter) const {
  cout << str << "size: " << size() << endl << endl;
  for (size_t i = 0; i < factors_.size(); i++) {
    stringstream ss;
    ss << "Factor " << i << ": ";
    if (factors_[i] != nullptr) {
      factors_[i]->print(ss.str(), keyFormatter);
      cout << "\n";
    } else {
      cout << ss.str() << "nullptr\n";
    }
  }
  std::cout.flush();
}

/* ************************************************************************* */
void NonlinearFactorGraph::printErrors(const Values& values, const std::string& str,
    const KeyFormatter& keyFormatter,
    const std::function<bool(const Factor* /*factor*/, double /*whitenedError*/, size_t /*index*/)>& printCondition) const
{
  cout << str << "size: " << size() << endl
       << endl;
  for (size_t i = 0; i < factors_.size(); i++) {
    const sharedFactor& factor = factors_[i];
    const double errorValue = (factor != nullptr ? factors_[i]->error(values) : .0);
    if (!printCondition(factor.get(),errorValue,i))
      continue; // User-provided filter did not pass

    stringstream ss;
    ss << "Factor " << i << ": ";
    if (factor == nullptr) {
      cout << "nullptr" << "\n";
    } else {
      factor->print(ss.str(), keyFormatter);
      cout << "error = " << errorValue << "\n";
    }
    cout << "\n";
  }
  std::cout.flush();
}

/* ************************************************************************* */
bool NonlinearFactorGraph::equals(const NonlinearFactorGraph& other, double tol) const {
  return Base::equals(other, tol);
}

/* ************************************************************************* */
void NonlinearFactorGraph::dot(std::ostream& os, const Values& values,
                               const KeyFormatter& keyFormatter,
                               const GraphvizFormatting& writer) const {
  writer.graphPreamble(&os);

  // Find bounds (imperative)
  KeySet keys = this->keys();
  Vector2 min = writer.findBounds(values, keys);

  // Create nodes for each variable in the graph
  for (Key key : keys) {
    auto position = writer.variablePos(values, min, key);
    writer.drawVariable(key, keyFormatter, position, &os);
  }
  os << "\n";

  if (writer.mergeSimilarFactors) {
    // Remove duplicate factors
    std::set<KeyVector> structure;
    for (const sharedFactor& factor : factors_) {
      if (factor) {
        KeyVector factorKeys = factor->keys();
        std::sort(factorKeys.begin(), factorKeys.end());
        structure.insert(factorKeys);
      }
    }

    // Create factors and variable connections
    size_t i = 0;
    for (const KeyVector& factorKeys : structure) {
      writer.processFactor(i++, factorKeys, keyFormatter, {}, &os);
    }
  } else {
    // Create factors and variable connections
    for (size_t i = 0; i < size(); ++i) {
      const NonlinearFactor::shared_ptr& factor = at(i);
      if (factor) {
        const KeyVector& factorKeys = factor->keys();
        writer.processFactor(i, factorKeys, keyFormatter,
                             writer.factorPos(min, i), &os);
      }
    }
  }

  os << "}\n";
  std::flush(os);
}

/* ************************************************************************* */
std::string NonlinearFactorGraph::dot(const Values& values,
                                      const KeyFormatter& keyFormatter,
                                      const GraphvizFormatting& writer) const {
  std::stringstream ss;
  dot(ss, values, keyFormatter, writer);
  return ss.str();
}

/* ************************************************************************* */
void NonlinearFactorGraph::saveGraph(const std::string& filename,
                                     const Values& values,
                                     const KeyFormatter& keyFormatter,
                                     const GraphvizFormatting& writer) const {
  std::ofstream of(filename);
  dot(of, values, keyFormatter, writer);
  of.close();
}

/* ************************************************************************* */
double NonlinearFactorGraph::error(const Values& values) const {
  gttic(NonlinearFactorGraph_error);
  double total_error = 0.;
  // iterate over all the factors_ to accumulate the log probabilities
  for(const sharedFactor& factor: factors_) {
    if(factor)
      total_error += factor->error(values);
  }
  return total_error;
}

/* ************************************************************************* */
Ordering NonlinearFactorGraph::orderingCOLAMD() const
{
  return Ordering::Colamd(*this);
}

/* ************************************************************************* */
Ordering NonlinearFactorGraph::orderingCOLAMDConstrained(const FastMap<Key, int>& constraints) const
{
  return Ordering::ColamdConstrained(*this, constraints);
}

/* ************************************************************************* */
SymbolicFactorGraph::shared_ptr NonlinearFactorGraph::symbolic() const
{
  // Generate the symbolic factor graph
  SymbolicFactorGraph::shared_ptr symbolic = boost::make_shared<SymbolicFactorGraph>();
  symbolic->reserve(size());

  for (const sharedFactor& factor: factors_) {
    if(factor)
      *symbolic += SymbolicFactor(*factor);
    else
      *symbolic += SymbolicFactorGraph::sharedFactor();
  }

  return symbolic;
}

/* ************************************************************************* */
namespace {

#ifdef GTSAM_USE_TBB
class _LinearizeOneFactor {
  const NonlinearFactorGraph& nonlinearGraph_;
  const Values& linearizationPoint_;
  GaussianFactorGraph& result_;
public:
  // Create functor with constant parameters
  _LinearizeOneFactor(const NonlinearFactorGraph& graph,
      const Values& linearizationPoint, GaussianFactorGraph& result) :
      nonlinearGraph_(graph), linearizationPoint_(linearizationPoint), result_(result) {
  }
  // Operator that linearizes a given range of the factors
  void operator()(const tbb::blocked_range<size_t>& blocked_range) const {
    for (size_t i = blocked_range.begin(); i != blocked_range.end(); ++i) {
      if (nonlinearGraph_[i] && nonlinearGraph_[i]->sendable())
        result_[i] = nonlinearGraph_[i]->linearize(linearizationPoint_);
      else
        result_[i] = GaussianFactor::shared_ptr();
    }
  }
};
#endif

}

/* ************************************************************************* */
GaussianFactorGraph::shared_ptr NonlinearFactorGraph::linearize(const Values& linearizationPoint) const
{
  gttic(NonlinearFactorGraph_linearize);

  // create an empty linear FG
  GaussianFactorGraph::shared_ptr linearFG = boost::make_shared<GaussianFactorGraph>();

#ifdef GTSAM_USE_TBB

  linearFG->resize(size());
  TbbOpenMPMixedScope threadLimiter; // Limits OpenMP threads since we're mixing TBB and OpenMP

  // First linearize all sendable factors
  tbb::parallel_for(tbb::blocked_range<size_t>(0, size()),
    _LinearizeOneFactor(*this, linearizationPoint, *linearFG));

  // Linearize all non-sendable factors
  for(size_t i = 0; i < size(); i++) {
    auto& factor = (*this)[i];
    if(factor && !(factor->sendable())) {
      (*linearFG)[i] = factor->linearize(linearizationPoint);
    }
  }

#else

  linearFG->reserve(size());

  // linearize all factors
  for(const sharedFactor& factor: factors_) {
    if(factor) {
      (*linearFG) += factor->linearize(linearizationPoint);
    } else
    (*linearFG) += GaussianFactor::shared_ptr();
  }

#endif

  return linearFG;
}

/* ************************************************************************* */
static Scatter scatterFromValues(const Values& values) {
  gttic(scatterFromValues);

  Scatter scatter;
  scatter.reserve(values.size());

  // use "natural" ordering with keys taken from the initial values
  for (const auto& key_dim : values.dims()) {
    scatter.add(key_dim.first, key_dim.second);
  }

  return scatter;
}

/* ************************************************************************* */
static Scatter scatterFromValues(const Values& values, const Ordering& ordering) {
  gttic(scatterFromValues);

  Scatter scatter;
  scatter.reserve(values.size());

  // copy ordering into keys and lookup dimension in values, is O(n*log n)
  for (Key key : ordering) {
    const Value& value = values.at(key);
    scatter.add(key, value.dim());
  }

  return scatter;
}

/* ************************************************************************* */
HessianFactor::shared_ptr NonlinearFactorGraph::linearizeToHessianFactor(
    const Values& values, const Scatter& scatter, const Dampen& dampen) const {
  // NOTE(frank): we are heavily leaning on friendship below
  HessianFactor::shared_ptr hessianFactor(new HessianFactor(scatter));

  // Initialize so we can rank-update below
  hessianFactor->info_.setZero();

  // linearize all factors straight into the Hessian
  // TODO(frank): this saves on creating the graph, but still mallocs a gaussianFactor!
  for (const sharedFactor& nonlinearFactor : factors_) {
    if (nonlinearFactor) {
      const auto& gaussianFactor = nonlinearFactor->linearize(values);
      gaussianFactor->updateHessian(hessianFactor->keys_, &hessianFactor->info_);
    }
  }

  if (dampen) dampen(hessianFactor);

  return hessianFactor;
}

/* ************************************************************************* */
HessianFactor::shared_ptr NonlinearFactorGraph::linearizeToHessianFactor(
    const Values& values, const Ordering& order, const Dampen& dampen) const {
  gttic(NonlinearFactorGraph_linearizeToHessianFactor);

  Scatter scatter = scatterFromValues(values, order);
  return linearizeToHessianFactor(values, scatter, dampen);
}

/* ************************************************************************* */
HessianFactor::shared_ptr NonlinearFactorGraph::linearizeToHessianFactor(
    const Values& values, const Dampen& dampen) const {
  gttic(NonlinearFactorGraph_linearizeToHessianFactor);

  Scatter scatter = scatterFromValues(values);
  return linearizeToHessianFactor(values, scatter, dampen);
}

/* ************************************************************************* */
Values NonlinearFactorGraph::updateCholesky(const Values& values,
                                            const Dampen& dampen) const {
  gttic(NonlinearFactorGraph_updateCholesky);
  auto hessianFactor = linearizeToHessianFactor(values, dampen);
  VectorValues delta = hessianFactor->solve();
  return values.retract(delta);
}

/* ************************************************************************* */
Values NonlinearFactorGraph::updateCholesky(const Values& values,
                                            const Ordering& ordering,
                                            const Dampen& dampen) const {
  gttic(NonlinearFactorGraph_updateCholesky);
  auto hessianFactor = linearizeToHessianFactor(values, ordering, dampen);
  VectorValues delta = hessianFactor->solve();
  return values.retract(delta);
}

/* ************************************************************************* */
NonlinearFactorGraph NonlinearFactorGraph::clone() const {
  NonlinearFactorGraph result;
  for (const sharedFactor& f : factors_) {
    if (f)
      result.push_back(f->clone());
    else
      result.push_back(sharedFactor()); // Passes on null factors so indices remain valid
  }
  return result;
}

/* ************************************************************************* */
NonlinearFactorGraph NonlinearFactorGraph::rekey(const std::map<Key,Key>& rekey_mapping) const {
  NonlinearFactorGraph result;
  for (const sharedFactor& f : factors_) {
    if (f)
      result.push_back(f->rekey(rekey_mapping));
    else
      result.push_back(sharedFactor());
  }
  return result;
}

/* ************************************************************************* */

} // namespace gtsam
