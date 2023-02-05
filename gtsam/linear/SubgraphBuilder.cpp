/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010-2019, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file SubgraphBuilder.cpp
 * @date Dec 31, 2009
 * @author Frank Dellaert, Yong-Dian Jian
 */

#include <gtsam/base/DSFVector.h>
#include <gtsam/base/FastMap.h>
#include <gtsam/base/WeightedSampler.h>
#include <gtsam/inference/Ordering.h>
#include <gtsam/inference/VariableIndex.h>
#include <gtsam/linear/Errors.h>
#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/linear/SubgraphBuilder.h>
#include <gtsam/base/kruskal.h>

#include <boost/algorithm/string.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>
#ifdef GTSAM_ENABLE_BOOST_SERIALIZATION
#include <boost/serialization/vector.hpp>
#endif

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <numeric>  // accumulate
#include <queue>
#include <random>
#include <set>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

using std::cout;
using std::endl;
using std::ostream;
using std::vector;

namespace gtsam {

/****************************************************************************/
Subgraph::Subgraph(const vector<size_t> &indices) {
  edges_.reserve(indices.size());
  for (const size_t &index : indices) {
    const Edge e{index, 1.0};
    edges_.push_back(e);
  }
}

/****************************************************************************/
vector<size_t> Subgraph::edgeIndices() const {
  vector<size_t> eid;
  eid.reserve(size());
  for (const Edge &edge : edges_) {
    eid.push_back(edge.index);
  }
  return eid;
}

#ifdef GTSAM_ENABLE_BOOST_SERIALIZATION
/****************************************************************************/
void Subgraph::save(const std::string &fn) const {
  std::ofstream os(fn.c_str());
  boost::archive::text_oarchive oa(os);
  oa << *this;
  os.close();
}

/****************************************************************************/
Subgraph Subgraph::load(const std::string &fn) {
  std::ifstream is(fn.c_str());
  boost::archive::text_iarchive ia(is);
  Subgraph subgraph;
  ia >> subgraph;
  is.close();
  return subgraph;
}
#endif

/****************************************************************************/
ostream &operator<<(ostream &os, const Subgraph::Edge &edge) {
  if (edge.weight != 1.0)
    os << edge.index << "(" << std::setprecision(2) << edge.weight << ")";
  else
    os << edge.index;
  return os;
}

/****************************************************************************/
ostream &operator<<(ostream &os, const Subgraph &subgraph) {
  os << "Subgraph" << endl;
  for (const auto &e : subgraph.edges()) {
    os << e << ", ";
  }
  return os;
}

/*****************************************************************************/
void SubgraphBuilderParameters::print() const { print(cout); }

/***************************************************************************************/
void SubgraphBuilderParameters::print(ostream &os) const {
  os << "SubgraphBuilderParameters" << endl
     << "skeleton:            " << skeletonTranslator(skeletonType) << endl
     << "skeleton weight:     " << skeletonWeightTranslator(skeletonWeight)
     << endl
     << "augmentation weight: "
     << augmentationWeightTranslator(augmentationWeight) << endl;
}

/*****************************************************************************/
ostream &operator<<(ostream &os, const SubgraphBuilderParameters &p) {
  p.print(os);
  return os;
}

/*****************************************************************************/
SubgraphBuilderParameters::Skeleton
SubgraphBuilderParameters::skeletonTranslator(const std::string &src) {
  std::string s = src;
  boost::algorithm::to_upper(s);
  if (s == "NATURALCHAIN")
    return NATURALCHAIN;
  else if (s == "BFS")
    return BFS;
  else if (s == "KRUSKAL")
    return KRUSKAL;
  throw std::invalid_argument(
      "SubgraphBuilderParameters::skeletonTranslator undefined string " + s);
  return KRUSKAL;
}

/****************************************************************/
std::string SubgraphBuilderParameters::skeletonTranslator(Skeleton s) {
  if (s == NATURALCHAIN)
    return "NATURALCHAIN";
  else if (s == BFS)
    return "BFS";
  else if (s == KRUSKAL)
    return "KRUSKAL";
  else
    return "UNKNOWN";
}

/****************************************************************/
SubgraphBuilderParameters::SkeletonWeight
SubgraphBuilderParameters::skeletonWeightTranslator(const std::string &src) {
  std::string s = src;
  boost::algorithm::to_upper(s);
  if (s == "EQUAL")
    return EQUAL;
  else if (s == "RHS")
    return RHS_2NORM;
  else if (s == "LHS")
    return LHS_FNORM;
  else if (s == "RANDOM")
    return RANDOM;
  throw std::invalid_argument(
      "SubgraphBuilderParameters::skeletonWeightTranslator undefined string " +
      s);
  return EQUAL;
}

/****************************************************************/
std::string SubgraphBuilderParameters::skeletonWeightTranslator(
    SkeletonWeight w) {
  if (w == EQUAL)
    return "EQUAL";
  else if (w == RHS_2NORM)
    return "RHS";
  else if (w == LHS_FNORM)
    return "LHS";
  else if (w == RANDOM)
    return "RANDOM";
  else
    return "UNKNOWN";
}

/****************************************************************/
SubgraphBuilderParameters::AugmentationWeight
SubgraphBuilderParameters::augmentationWeightTranslator(
    const std::string &src) {
  std::string s = src;
  boost::algorithm::to_upper(s);
  if (s == "SKELETON") return SKELETON;
  //  else if (s == "STRETCH")  return STRETCH;
  //  else if (s == "GENERALIZED_STRETCH")  return GENERALIZED_STRETCH;
  throw std::invalid_argument(
      "SubgraphBuilder::Parameters::augmentationWeightTranslator undefined "
      "string " +
      s);
  return SKELETON;
}

/****************************************************************/
std::string SubgraphBuilderParameters::augmentationWeightTranslator(
    AugmentationWeight w) {
  if (w == SKELETON) return "SKELETON";
  //  else if ( w == STRETCH )              return "STRETCH";
  //  else if ( w == GENERALIZED_STRETCH )  return "GENERALIZED_STRETCH";
  else
    return "UNKNOWN";
}

/****************************************************************/
vector<size_t> SubgraphBuilder::buildTree(const GaussianFactorGraph &gfg,
                                          const vector<double> &weights) const {
  const SubgraphBuilderParameters &p = parameters_;
  switch (p.skeletonType) {
    case SubgraphBuilderParameters::NATURALCHAIN:
      return natural_chain(gfg);
      break;
    case SubgraphBuilderParameters::BFS:
      return bfs(gfg);
      break;
    case SubgraphBuilderParameters::KRUSKAL:
      return kruskal(gfg, weights);
      break;
    default:
      std::cerr << "SubgraphBuilder::buildTree undefined skeleton type" << endl;
      break;
  }
  return vector<size_t>();
}

/****************************************************************/
vector<size_t> SubgraphBuilder::unary(const GaussianFactorGraph &gfg) const {
  vector<size_t> unaryFactorIndices;
  size_t index = 0;
  for (const auto &factor : gfg) {
    if (factor->size() == 1) {
      unaryFactorIndices.push_back(index);
    }
    index++;
  }
  return unaryFactorIndices;
}

/****************************************************************/
vector<size_t> SubgraphBuilder::natural_chain(
    const GaussianFactorGraph &gfg) const {
  vector<size_t> chainFactorIndices;
  size_t index = 0;
  for (const GaussianFactor::shared_ptr &gf : gfg) {
    if (gf->size() == 2) {
      const Key k0 = gf->keys()[0], k1 = gf->keys()[1];
      if ((k1 - k0) == 1 || (k0 - k1) == 1) chainFactorIndices.push_back(index);
    }
    index++;
  }
  return chainFactorIndices;
}

/****************************************************************/
vector<size_t> SubgraphBuilder::bfs(const GaussianFactorGraph &gfg) const {
  const VariableIndex variableIndex(gfg);
  /* start from the first key of the first factor */
  Key seed = gfg[0]->keys()[0];

  const size_t n = variableIndex.size();

  /* each vertex has self as the predecessor */
  vector<size_t> bfsFactorIndices;
  bfsFactorIndices.reserve(n - 1);

  /* Initialize */
  std::queue<size_t> q;
  q.push(seed);

  std::set<size_t> flags;
  flags.insert(seed);

  /* traversal */
  while (!q.empty()) {
    const size_t head = q.front();
    q.pop();
    for (const size_t index : variableIndex[head]) {
      const GaussianFactor &gf = *gfg[index];
      for (const size_t key : gf.keys()) {
        if (flags.count(key) == 0) {
          q.push(key);
          flags.insert(key);
          bfsFactorIndices.push_back(index);
        }
      }
    }
  }
  return bfsFactorIndices;
}

/****************************************************************/
vector<size_t> SubgraphBuilder::kruskal(const GaussianFactorGraph &gfg,
                                        const vector<double> &weights) const {
  return utils::kruskal(gfg, weights);
}

/****************************************************************/
vector<size_t> SubgraphBuilder::sample(const vector<double> &weights,
                                       const size_t t) const {
  std::mt19937 rng(42);  // TODO(frank): allow us to use a different seed
  WeightedSampler<std::mt19937> sampler(&rng);
  return sampler.sampleWithoutReplacement(t, weights);
}

/****************************************************************/
Subgraph SubgraphBuilder::operator()(const GaussianFactorGraph &gfg) const {
  const auto &p = parameters_;
  const auto inverse_ordering = Ordering::Natural(gfg);
  const FastMap<Key, size_t> forward_ordering = inverse_ordering.invert();
  const size_t n = inverse_ordering.size(), m = gfg.size();

  // Make sure the subgraph preconditioner does not include more than half of
  // the edges beyond the spanning tree, or we might as well solve the whole
  // thing.
  size_t numExtraEdges = n * p.augmentationFactor;
  const size_t numRemaining = m - (n - 1);
  numExtraEdges = std::min(numExtraEdges, numRemaining / 2);

  // Calculate weights
  vector<double> weights = this->weights(gfg);

  // Build spanning tree.
  const vector<size_t> tree = buildTree(gfg, weights);
  if (tree.size() != n - 1) {
    throw std::runtime_error(
        "SubgraphBuilder::operator() failure: tree.size() != n-1, might be caused by disconnected graph");
  }

  // Downweight the tree edges to zero.
  for (const size_t index : tree) {
    weights[index] = 0.0;
  }

  /* decide how many edges to augment */
  vector<size_t> offTree = sample(weights, numExtraEdges);

  vector<size_t> subgraph = unary(gfg);
  subgraph.insert(subgraph.end(), tree.begin(), tree.end());
  subgraph.insert(subgraph.end(), offTree.begin(), offTree.end());

  return Subgraph(subgraph);
}

/****************************************************************/
SubgraphBuilder::Weights SubgraphBuilder::weights(
    const GaussianFactorGraph &gfg) const {

  const size_t m = gfg.size();
  Weights weight;
  weight.reserve(m);

  for (const GaussianFactor::shared_ptr &gf : gfg) {
    switch (parameters_.skeletonWeight) {
      case SubgraphBuilderParameters::EQUAL:
        weight.push_back(1.0);
        break;
      case SubgraphBuilderParameters::RHS_2NORM: {
        if (JacobianFactor::shared_ptr jf =
                std::dynamic_pointer_cast<JacobianFactor>(gf)) {
          weight.push_back(jf->getb().norm());
        } else if (HessianFactor::shared_ptr hf =
                       std::dynamic_pointer_cast<HessianFactor>(gf)) {
          weight.push_back(hf->linearTerm().norm());
        }
      } break;
      case SubgraphBuilderParameters::LHS_FNORM: {
        if (JacobianFactor::shared_ptr jf =
                std::dynamic_pointer_cast<JacobianFactor>(gf)) {
          weight.push_back(std::sqrt(jf->getA().squaredNorm()));
        } else if (HessianFactor::shared_ptr hf =
                       std::dynamic_pointer_cast<HessianFactor>(gf)) {
          weight.push_back(std::sqrt(hf->information().squaredNorm()));
        }
      } break;

      case SubgraphBuilderParameters::RANDOM:
        weight.push_back(std::rand() % 100 + 1.0);
        break;

      default:
        throw std::invalid_argument(
            "SubgraphBuilder::weights: undefined weight scheme ");
        break;
    }
  }
  return weight;
}

/*****************************************************************************/
GaussianFactorGraph buildFactorSubgraph(const GaussianFactorGraph &gfg,
                                        const Subgraph &subgraph,
                                        const bool clone) {
  GaussianFactorGraph subgraphFactors;
  subgraphFactors.reserve(subgraph.size());
  for (const auto &e : subgraph) {
    const auto factor = gfg[e.index];
    subgraphFactors.push_back(clone ? factor->clone() : factor);
  }
  return subgraphFactors;
}

/**************************************************************************************************/
std::pair<GaussianFactorGraph, GaussianFactorGraph> splitFactorGraph(
    const GaussianFactorGraph &factorGraph, const Subgraph &subgraph) {
  // Get the subgraph by calling cheaper method
  auto subgraphFactors = buildFactorSubgraph(factorGraph, subgraph, false);

  // Now, copy all factors then set subGraph factors to zero
  GaussianFactorGraph remaining = factorGraph;

  for (const auto &e : subgraph) {
    remaining.remove(e.index);
  }

  return std::make_pair(subgraphFactors, remaining);
}

/*****************************************************************************/

}  // namespace gtsam
