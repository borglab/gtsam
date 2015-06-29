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
#include <gtsam/inference/Ordering.h>
#include <gtsam/inference/FactorGraph-inst.h>
#include <gtsam/config.h> // for GTSAM_USE_TBB

#ifdef GTSAM_USE_TBB
#  include <tbb/parallel_for.h>
#endif

#include <boost/foreach.hpp>
#include <cmath>
#include <limits>

using namespace std;

namespace gtsam {

// Instantiate base classes
template class FactorGraph<NonlinearFactor>;

/* ************************************************************************* */
double NonlinearFactorGraph::probPrime(const Values& c) const {
  return exp(-0.5 * error(c));
}

/* ************************************************************************* */
void NonlinearFactorGraph::print(const std::string& str, const KeyFormatter& keyFormatter) const {
  cout << str << "size: " << size() << endl << endl;
  for (size_t i = 0; i < factors_.size(); i++) {
    stringstream ss;
    ss << "Factor " << i << ": ";
    if (factors_[i] != NULL) factors_[i]->print(ss.str(), keyFormatter);
    cout << endl;
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
      t = p->value().translation().vector();
    } else if (const GenericValue<Point3>* p = dynamic_cast<const GenericValue<Point3>*>(&value)) {
      t = p->value().vector();
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
  BOOST_FOREACH(Key key, keys) {
    if(values.exists(key)) {
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
  BOOST_FOREACH(Key key, keys){
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
    std::set<vector<Key> > structure;
    BOOST_FOREACH(const sharedFactor& factor, *this){
      if(factor) {
        vector<Key> factorKeys = factor->keys();
        std::sort(factorKeys.begin(), factorKeys.end());
        structure.insert(factorKeys);
      }
    }

    // Create factors and variable connections
    size_t i = 0;
    BOOST_FOREACH(const vector<Key>& factorKeys, structure){
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
      BOOST_FOREACH(Key key, factorKeys) {
        stm << "  var" << key << "--" << "factor" << i << ";\n";
      }

      ++ i;
    }
  } else {
    // Create factors and variable connections
    for(size_t i = 0; i < this->size(); ++i) {
      const NonlinearFactor::shared_ptr& factor = this->at(i);
      if(formatting.plotFactorPoints) {
        const FastVector<Key>& keys = factor->keys();
        if (formatting.binaryEdges && keys.size()==2) {
          stm << "  var" << keys[0] << "--" << "var" << keys[1] << ";\n";
        } else {
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
          if(formatting.connectKeysToFactor && factor) {
            BOOST_FOREACH(Key key, *factor) {
              stm << "  var" << key << "--" << "factor" << i << ";\n";
            }
          }
        }
      }
      else {
        if(factor) {
          Key k;
          bool firstTime = true;
          BOOST_FOREACH(Key key, *this->at(i)) {
            if(firstTime) {
              k = key;
              firstTime = false;
              continue;
            }
            stm << "  var" << key << "--" << "var" << k << ";\n";
            k = key;
          }
        }
      }
    }
  }

  stm << "}\n";
}

/* ************************************************************************* */
double NonlinearFactorGraph::error(const Values& c) const {
  gttic(NonlinearFactorGraph_error);
  double total_error = 0.;
  // iterate over all the factors_ to accumulate the log probabilities
  BOOST_FOREACH(const sharedFactor& factor, this->factors_) {
    if(factor)
      total_error += factor->error(c);
  }
  return total_error;
}

/* ************************************************************************* */
KeySet NonlinearFactorGraph::keys() const {
  KeySet keys;
  BOOST_FOREACH(const sharedFactor& factor, this->factors_) {
    if(factor)
      keys.insert(factor->begin(), factor->end());
  }
  return keys;
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
  symbolic->reserve(this->size());

  BOOST_FOREACH(const sharedFactor& factor, *this) {
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
      if (nonlinearGraph_[i])
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

  linearFG->resize(this->size());
  TbbOpenMPMixedScope threadLimiter; // Limits OpenMP threads since we're mixing TBB and OpenMP
  tbb::parallel_for(tbb::blocked_range<size_t>(0, this->size()),
    _LinearizeOneFactor(*this, linearizationPoint, *linearFG));

#else

  linearFG->reserve(this->size());

  // linearize all factors
  BOOST_FOREACH(const sharedFactor& factor, this->factors_) {
    if(factor) {
      (*linearFG) += factor->linearize(linearizationPoint);
    } else
    (*linearFG) += GaussianFactor::shared_ptr();
  }

#endif

  return linearFG;
}

/* ************************************************************************* */
NonlinearFactorGraph NonlinearFactorGraph::clone() const {
  NonlinearFactorGraph result;
  BOOST_FOREACH(const sharedFactor& f, *this) {
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
  BOOST_FOREACH(const sharedFactor& f, *this) {
    if (f)
      result.push_back(f->rekey(rekey_mapping));
    else
      result.push_back(sharedFactor());
  }
  return result;
}

/* ************************************************************************* */

} // namespace gtsam
