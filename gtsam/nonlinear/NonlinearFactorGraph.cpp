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

#include <cmath>
#include <fstream>
#include <limits>

using namespace std;

namespace gtsam {

// Instantiate base classes
template class FactorGraph<NonlinearFactor>;

/* ************************************************************************* */
double NonlinearFactorGraph::probPrime(const Values& values) const {
  return exp(-0.5 * error(values));
}

/* ************************************************************************* */
void NonlinearFactorGraph::print(const std::string& str, const KeyFormatter& keyFormatter) const {
  cout << str << "size: " << size() << endl << endl;
  for (size_t i = 0; i < factors_.size(); i++) {
    stringstream ss;
    ss << "Factor " << i << ": ";
    if (factors_[i] != nullptr) factors_[i]->print(ss.str(), keyFormatter);
    cout << endl;
  }
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
    cout << endl; // only one "endl" at end might be faster, \n for each factor
  }
}

/* ************************************************************************* */
bool NonlinearFactorGraph::equals(const NonlinearFactorGraph& other, double tol) const {
  return Base::equals(other, tol);
}

/* ************************************************************************* */
void NonlinearFactorGraph::saveGraph(std::ostream &stm, const Values& values,
    const GraphvizFormatting& formatting,
    const KeyFormatter& keyFormatter) const
{
  stm << "graph {\n";
  stm << "  size=\"" << formatting.figureWidthInches << "," <<
    formatting.figureHeightInches << "\";\n\n";

  KeySet keys = this->keys();

  // Local utility function to extract x and y coordinates
  struct { boost::optional<Point2> operator()(
    const Value& value, const GraphvizFormatting& graphvizFormatting)
  {
    Vector3 t;
    if (const GenericValue<Pose2>* p = dynamic_cast<const GenericValue<Pose2>*>(&value)) {
      t << p->value().x(), p->value().y(), 0;
    } else if (const GenericValue<Point2>* p = dynamic_cast<const GenericValue<Point2>*>(&value)) {
      t << p->value().x(), p->value().y(), 0;
    } else if (const GenericValue<Pose3>* p = dynamic_cast<const GenericValue<Pose3>*>(&value)) {
      t = p->value().translation();
    } else if (const GenericValue<Point3>* p = dynamic_cast<const GenericValue<Point3>*>(&value)) {
      t = p->value();
    } else {
      return boost::none;
    }
    double x, y;
    switch (graphvizFormatting.paperHorizontalAxis) {
      case GraphvizFormatting::X: x = t.x(); break;
      case GraphvizFormatting::Y: x = t.y(); break;
      case GraphvizFormatting::Z: x = t.z(); break;
      case GraphvizFormatting::NEGX: x = -t.x(); break;
      case GraphvizFormatting::NEGY: x = -t.y(); break;
      case GraphvizFormatting::NEGZ: x = -t.z(); break;
      default: throw std::runtime_error("Invalid enum value");
    }
    switch (graphvizFormatting.paperVerticalAxis) {
      case GraphvizFormatting::X: y = t.x(); break;
      case GraphvizFormatting::Y: y = t.y(); break;
      case GraphvizFormatting::Z: y = t.z(); break;
      case GraphvizFormatting::NEGX: y = -t.x(); break;
      case GraphvizFormatting::NEGY: y = -t.y(); break;
      case GraphvizFormatting::NEGZ: y = -t.z(); break;
      default: throw std::runtime_error("Invalid enum value");
    }
    return Point2(x,y);
  }} getXY;

  // Find bounds
  double minX = numeric_limits<double>::infinity(), maxX = -numeric_limits<double>::infinity();
  double minY = numeric_limits<double>::infinity(), maxY = -numeric_limits<double>::infinity();
  for (const Key& key : keys) {
    if (values.exists(key)) {
      boost::optional<Point2> xy = getXY(values.at(key), formatting);
      if(xy) {
        if(xy->x() < minX)
          minX = xy->x();
        if(xy->x() > maxX)
          maxX = xy->x();
        if(xy->y() < minY)
          minY = xy->y();
        if(xy->y() > maxY)
          maxY = xy->y();
      }
    }
  }

  // Create nodes for each variable in the graph
  for(Key key: keys){
    // Label the node with the label from the KeyFormatter
    stm << "  var" << key << "[label=\"" << keyFormatter(key) << "\"";
    if(values.exists(key)) {
      boost::optional<Point2> xy = getXY(values.at(key), formatting);
      if(xy)
      stm << ", pos=\"" << formatting.scale*(xy->x() - minX) << "," << formatting.scale*(xy->y() - minY) << "!\"";
    }
    stm << "];\n";
  }
  stm << "\n";

  if (formatting.mergeSimilarFactors) {
    // Remove duplicate factors
    std::set<KeyVector > structure;
    for (const sharedFactor& factor : factors_) {
      if (factor) {
        KeyVector factorKeys = factor->keys();
        std::sort(factorKeys.begin(), factorKeys.end());
        structure.insert(factorKeys);
      }
    }

    // Create factors and variable connections
    size_t i = 0;
    for(const KeyVector& factorKeys: structure){
      // Make each factor a dot
      stm << "  factor" << i << "[label=\"\", shape=point";
      {
        map<size_t, Point2>::const_iterator pos = formatting.factorPositions.find(i);
        if(pos != formatting.factorPositions.end())
        stm << ", pos=\"" << formatting.scale*(pos->second.x() - minX) << ","
                          << formatting.scale*(pos->second.y() - minY) << "!\"";
      }
      stm << "];\n";

      // Make factor-variable connections
      for(Key key: factorKeys) {
        stm << "  var" << key << "--" << "factor" << i << ";\n";
      }

      ++ i;
    }
  } else {
    // Create factors and variable connections
    for(size_t i = 0; i < size(); ++i) {
      const NonlinearFactor::shared_ptr& factor = at(i);
      // If null pointer, move on to the next
      if (!factor) {
        continue;
      }

      if (formatting.plotFactorPoints) {
        const KeyVector& keys = factor->keys();
        if (formatting.binaryEdges && keys.size() == 2) {
          stm << "  var" << keys[0] << "--"
              << "var" << keys[1] << ";\n";
        } else {
          // Make each factor a dot
          stm << "  factor" << i << "[label=\"\", shape=point";
          {
            map<size_t, Point2>::const_iterator pos =
                formatting.factorPositions.find(i);
            if (pos != formatting.factorPositions.end())
              stm << ", pos=\"" << formatting.scale * (pos->second.x() - minX)
                  << "," << formatting.scale * (pos->second.y() - minY)
                  << "!\"";
          }
          stm << "];\n";

          // Make factor-variable connections
          if (formatting.connectKeysToFactor && factor) {
            for (Key key : *factor) {
              stm << "  var" << key << "--"
                  << "factor" << i << ";\n";
            }
          }
        }
      } else {
        Key k;
        bool firstTime = true;
        for (Key key : *this->at(i)) {
          if (firstTime) {
            k = key;
            firstTime = false;
            continue;
          }
          stm << "  var" << key << "--"
              << "var" << k << ";\n";
          k = key;
        }
      }
    }
  }

  stm << "}\n";
}

/* ************************************************************************* */
void NonlinearFactorGraph::saveGraph(
    const std::string& file, const Values& values,
    const GraphvizFormatting& graphvizFormatting,
    const KeyFormatter& keyFormatter) const {
  std::ofstream of(file);
  saveGraph(of, values, graphvizFormatting, keyFormatter);
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
