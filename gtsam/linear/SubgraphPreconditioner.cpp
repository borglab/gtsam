/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file SubgraphPreconditioner.cpp
 * @date Dec 31, 2009
 * @author: Frank Dellaert
 */

#include <gtsam/linear/SubgraphPreconditioner.h>
#include <gtsam/linear/GaussianConditional.h>
#include <gtsam/linear/HessianFactor.h>
#include <gtsam/linear/JacobianFactor.h>
#include <gtsam/linear/GaussianEliminationTree.h>
#include <gtsam/inference/Ordering.h>
#include <gtsam/inference/VariableIndex.h>
#include <gtsam/base/DSFVector.h>
#include <gtsam/base/FastMap.h>
#include <gtsam/base/FastVector.h>
#include <gtsam/base/types.h>
#include <gtsam/base/Vector.h>

#include <boost/algorithm/string.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/range/adaptor/reversed.hpp>
#include <boost/shared_ptr.hpp>

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <iterator>
#include <list>
#include <map>
#include <numeric> // accumulate
#include <queue>
#include <set>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

using std::cout;
using std::endl;
using std::vector;
using std::ostream;

namespace gtsam {

/* ************************************************************************* */
// Convert any non-Jacobian factors to Jacobians (e.g. Hessian -> Jacobian with Cholesky)
static GaussianFactorGraph::shared_ptr convertToJacobianFactors(const GaussianFactorGraph &gfg) {
  auto result = boost::make_shared<GaussianFactorGraph>();
  for (const auto &factor : gfg) {
    auto jf = boost::dynamic_pointer_cast<JacobianFactor>(factor);
    if( !jf ) {
      jf = boost::make_shared<JacobianFactor>(*factor); 
    }
    result->push_back(jf);
  }
  return result;
}

/*****************************************************************************/
static vector<size_t> iidSampler(const vector<double> &weight, const size_t n) {

  /* compute the sum of the weights */
  const double sum = std::accumulate(weight.begin(), weight.end(), 0.0);
  if (sum==0.0) {
    throw std::runtime_error("Weight vector has no non-zero weights");
  }

  /* make a normalized and accumulated version of the weight vector */
  const size_t m = weight.size();
  vector<double> cdf; cdf.reserve(m);
  for ( size_t i = 0 ; i < m ; ++i ) {
    cdf.push_back(weight[i]/sum);
  }

  vector<double> acc(m);
  std::partial_sum(cdf.begin(),cdf.end(),acc.begin());

  /* iid sample n times */
  vector<size_t> result; result.reserve(n);
  const double denominator = (double)RAND_MAX;
  for ( size_t i = 0 ; i < n ; ++i ) {
    const double value = rand() / denominator;
    /* binary search the interval containing "value" */
    const auto it = std::lower_bound(acc.begin(), acc.end(), value);
    const size_t idx = it - acc.begin();
    result.push_back(idx);
  }
  return result;
}

/*****************************************************************************/
static vector<size_t> UniqueSampler(const vector<double> &weight, const size_t n) {

  const size_t m = weight.size();
  if ( n > m ) throw std::invalid_argument("UniqueSampler: invalid input size");

  vector<size_t> result;

  size_t count = 0;
  vector<bool> touched(m, false);
  while ( count < n ) {
    vector<size_t> localIndices; localIndices.reserve(n-count);
    vector<double> localWeights; localWeights.reserve(n-count);

    /* collect data */
    for ( size_t i = 0 ; i < m ; ++i ) {
      if ( !touched[i] ) {
        localIndices.push_back(i);
        localWeights.push_back(weight[i]);
      }
    }

    /* sampling and cache results */
    vector<size_t> samples = iidSampler(localWeights, n-count);
    for ( const size_t &id: samples ) {
      if ( touched[id] == false ) {
        touched[id] = true ;
        result.push_back(id);
        if ( ++count >= n ) break;
      }
    }
  }
  return result;
}

/****************************************************************************/
Subgraph::Subgraph(const vector<size_t> &indices) {
  edges_.reserve(indices.size());
  for ( const size_t &idx: indices ) {
    edges_.emplace_back(idx, 1.0);
  }
}

/****************************************************************************/
vector<size_t> Subgraph::edgeIndices() const {
  vector<size_t> eid; eid.reserve(size());
  for ( const SubgraphEdge &edge: edges_ ) {
    eid.push_back(edge.index_);
  }
  return eid;
}

/****************************************************************************/
void Subgraph::save(const std::string &fn) const {
  std::ofstream os(fn.c_str());
  boost::archive::text_oarchive oa(os);
  oa << *this;
  os.close();
}

/****************************************************************************/
Subgraph::shared_ptr Subgraph::load(const std::string &fn) {
  std::ifstream is(fn.c_str());
  boost::archive::text_iarchive ia(is);
  Subgraph::shared_ptr subgraph(new Subgraph());
  ia >> *subgraph;
  is.close();
  return subgraph;
}

/****************************************************************************/
ostream &operator<<(ostream &os, const SubgraphEdge &edge) {
  if ( edge.weight() != 1.0 )
    os << edge.index() << "(" << std::setprecision(2) << edge.weight() << ")";
  else
    os << edge.index() ;
  return os;
}

/****************************************************************************/
ostream &operator<<(ostream &os, const Subgraph &subgraph) {
  os << "Subgraph" << endl;
  for ( const SubgraphEdge &e: subgraph.edges() ) {
    os << e << ", " ;
  }
  return os;
}

/*****************************************************************************/
void SubgraphBuilderParameters::print() const {
  print(cout);
}

/***************************************************************************************/
void SubgraphBuilderParameters::print(ostream &os) const {
  os << "SubgraphBuilderParameters" << endl
      << "skeleton:            " << skeletonTranslator(skeleton_) << endl
      << "skeleton weight:     " << skeletonWeightTranslator(skeletonWeight_) << endl
      << "augmentation weight: " << augmentationWeightTranslator(augmentationWeight_) << endl
      ;
}

/*****************************************************************************/
ostream& operator<<(ostream &os, const SubgraphBuilderParameters &p) {
  p.print(os);
  return os;
}

/*****************************************************************************/
SubgraphBuilderParameters::Skeleton SubgraphBuilderParameters::skeletonTranslator(const std::string &src){
  std::string s = src;  boost::algorithm::to_upper(s);
  if (s == "NATURALCHAIN")    return NATURALCHAIN;
  else if (s == "BFS")        return BFS;
  else if (s == "KRUSKAL")    return KRUSKAL;
  throw std::invalid_argument("SubgraphBuilderParameters::skeletonTranslator undefined string " + s);
  return KRUSKAL;
}

/****************************************************************/
std::string SubgraphBuilderParameters::skeletonTranslator(Skeleton s) {
  if ( s == NATURALCHAIN ) return "NATURALCHAIN";
  else if ( s == BFS )     return "BFS";
  else if ( s == KRUSKAL ) return "KRUSKAL";
  else                     return "UNKNOWN";
}

/****************************************************************/
SubgraphBuilderParameters::SkeletonWeight SubgraphBuilderParameters::skeletonWeightTranslator(const std::string &src) {
  std::string s = src;  boost::algorithm::to_upper(s);
  if (s == "EQUAL")       return EQUAL;
  else if (s == "RHS")    return RHS_2NORM;
  else if (s == "LHS")    return LHS_FNORM;
  else if (s == "RANDOM") return RANDOM;
  throw std::invalid_argument("SubgraphBuilderParameters::skeletonWeightTranslator undefined string " + s);
  return EQUAL;
}

/****************************************************************/
std::string SubgraphBuilderParameters::skeletonWeightTranslator(SkeletonWeight w) {
  if ( w == EQUAL )           return "EQUAL";
  else if ( w == RHS_2NORM )  return "RHS";
  else if ( w == LHS_FNORM )  return "LHS";
  else if ( w == RANDOM )     return "RANDOM";
  else                        return "UNKNOWN";
}

/****************************************************************/
SubgraphBuilderParameters::AugmentationWeight
SubgraphBuilderParameters::augmentationWeightTranslator(
    const std::string &src) {
  std::string s = src;  boost::algorithm::to_upper(s);
  if (s == "SKELETON")      return SKELETON;
//  else if (s == "STRETCH")  return STRETCH;
//  else if (s == "GENERALIZED_STRETCH")  return GENERALIZED_STRETCH;
  throw std::invalid_argument("SubgraphBuilder::Parameters::augmentationWeightTranslator undefined string " + s);
  return SKELETON;
}

/****************************************************************/
std::string SubgraphBuilderParameters::augmentationWeightTranslator(AugmentationWeight w) {
  if ( w == SKELETON )                  return "SKELETON";
//  else if ( w == STRETCH )              return "STRETCH";
//  else if ( w == GENERALIZED_STRETCH )  return "GENERALIZED_STRETCH";
  else                                  return "UNKNOWN";
}

/****************************************************************/
vector<size_t> SubgraphBuilder::buildTree(const GaussianFactorGraph &gfg,
                                          const FastMap<Key, size_t> &ordering,
                                          const vector<double> &weights) const {
  const SubgraphBuilderParameters &p = parameters_;
  switch (p.skeleton_) {
  case SubgraphBuilderParameters::NATURALCHAIN:
    return natural_chain(gfg);
    break;
  case SubgraphBuilderParameters::BFS:
    return bfs(gfg);
    break;
  case SubgraphBuilderParameters::KRUSKAL:
    return kruskal(gfg, ordering, weights);
    break;
  default:
    std::cerr << "SubgraphBuilder::buildTree undefined skeleton type" << endl;
    break;
  }
  return vector<size_t>();
}

/****************************************************************/
vector<size_t> SubgraphBuilder::unary(const GaussianFactorGraph &gfg) const {
  vector<size_t> result ;
  size_t idx = 0;
  for (const auto &factor : gfg) {
    if ( factor->size() == 1 ) {
      result.push_back(idx);
    }
    idx++;
  }
  return result;
}

/****************************************************************/
vector<size_t> SubgraphBuilder::natural_chain(const GaussianFactorGraph &gfg) const {
  vector<size_t> result ;
  size_t idx = 0;
  for ( const GaussianFactor::shared_ptr &gf: gfg ) {
    if ( gf->size() == 2 ) {
      const Key k0 = gf->keys()[0], k1 = gf->keys()[1];
      if ( (k1-k0) == 1 || (k0-k1) == 1 )
        result.push_back(idx);
    }
    idx++;
  }
  return result;
}

/****************************************************************/
vector<size_t> SubgraphBuilder::bfs(const GaussianFactorGraph &gfg) const {
  const VariableIndex variableIndex(gfg);
  /* start from the first key of the first factor */
  Key seed = gfg[0]->keys()[0];

  const size_t n = variableIndex.size();

  /* each vertex has self as the predecessor */
  vector<size_t> result;
  result.reserve(n-1);

  /* Initialize */
  std::queue<size_t> q;
  q.push(seed);

  std::set<size_t> flags;
  flags.insert(seed);

  /* traversal */
  while ( !q.empty() ) {
    const size_t head = q.front(); q.pop();
    for ( const size_t id: variableIndex[head] ) {
      const GaussianFactor &gf = *gfg[id];
      for ( const size_t key: gf.keys() ) {
        if ( flags.count(key) == 0 ) {
          q.push(key);
          flags.insert(key);
          result.push_back(id);
        }
      }
    }
  }
  return result;
}

/****************************************************************/
vector<size_t> SubgraphBuilder::kruskal(const GaussianFactorGraph &gfg,
                                        const FastMap<Key, size_t> &ordering,
                                        const vector<double> &weights) const {
  const VariableIndex variableIndex(gfg);
  const size_t n = variableIndex.size();
  const vector<size_t> idx = sort_idx(weights) ;

  /* initialize buffer */
  vector<size_t> result;
  result.reserve(n-1);

  // container for acsendingly sorted edges
  DSFVector dsf(n);

  size_t count = 0 ; double sum = 0.0 ;
  for (const size_t id: idx) {
    const GaussianFactor &gf = *gfg[id];
    const auto keys = gf.keys();
    if ( keys.size() != 2 ) continue;
    const size_t u = ordering.find(keys[0])->second,
                 v = ordering.find(keys[1])->second;
    if ( dsf.find(u) != dsf.find(v) ) {
      dsf.merge(u, v) ;
      result.push_back(id) ;
      sum += weights[id] ;
      if ( ++count == n-1 ) break ;
    }
  }
  return result;
}

/****************************************************************/
vector<size_t> SubgraphBuilder::sample(const vector<double> &weights, const size_t t) const {
  return UniqueSampler(weights, t);
}

/****************************************************************/
Subgraph::shared_ptr SubgraphBuilder::operator() (const GaussianFactorGraph &gfg) const {

  const auto &p = parameters_;
  const auto inverse_ordering = Ordering::Natural(gfg);
  const FastMap<Key, size_t> forward_ordering = inverse_ordering.invert();
  const size_t n = inverse_ordering.size(), m = gfg.size();

  // Make sure the subgraph preconditioner does not include more than half of
  // the edges beyond the spanning tree, or we might as well solve the whole thing.
  size_t numExtraEdges = n * p.complexity_;
  const size_t numRemaining = m - (n - 1);
  numExtraEdges = std::min(numExtraEdges, numRemaining/2);

  // Calculate weights 
  vector<double> weights = this->weights(gfg);

  // Build spanning tree.
  const vector<size_t> tree = buildTree(gfg, forward_ordering, weights);
  if ( tree.size() != n-1 ) {
    throw std::runtime_error("SubgraphBuilder::operator() failure: tree.size() != n-1");
  }

  // Downweight the tree edges to zero.
  for ( const size_t id: tree ) {
    weights[id] = 0.0;
  }

  /* decide how many edges to augment */
  vector<size_t> offTree = sample(weights, numExtraEdges);

  vector<size_t> subgraph = unary(gfg);
  subgraph.insert(subgraph.end(), tree.begin(), tree.end());
  subgraph.insert(subgraph.end(), offTree.begin(), offTree.end());

  return boost::make_shared<Subgraph>(subgraph);
}

/****************************************************************/
SubgraphBuilder::Weights SubgraphBuilder::weights(const GaussianFactorGraph &gfg) const
{
  const size_t m = gfg.size() ;
  Weights weight; weight.reserve(m);

  for(const GaussianFactor::shared_ptr &gf: gfg ) {
    switch ( parameters_.skeletonWeight_ ) {
    case SubgraphBuilderParameters::EQUAL:
      weight.push_back(1.0);
      break;
    case SubgraphBuilderParameters::RHS_2NORM:
    {
      if ( JacobianFactor::shared_ptr jf = boost::dynamic_pointer_cast<JacobianFactor>(gf) ) {
        weight.push_back(jf->getb().norm());
      }
      else if ( HessianFactor::shared_ptr hf = boost::dynamic_pointer_cast<HessianFactor>(gf) ) {
        weight.push_back(hf->linearTerm().norm());
      }
    }
      break;
    case SubgraphBuilderParameters::LHS_FNORM:
    {
      if ( JacobianFactor::shared_ptr jf = boost::dynamic_pointer_cast<JacobianFactor>(gf) ) {
        weight.push_back(std::sqrt(jf->getA().squaredNorm()));
      }
      else if ( HessianFactor::shared_ptr hf = boost::dynamic_pointer_cast<HessianFactor>(gf) ) {
        weight.push_back(std::sqrt(hf->information().squaredNorm()));
      }
    }
      break;

    case SubgraphBuilderParameters::RANDOM:
      weight.push_back(std::rand()%100 + 1.0);
      break;

    default:
      throw std::invalid_argument("SubgraphBuilder::weights: undefined weight scheme ");
      break;
    }
  }
  return weight;
}

/* ************************************************************************* */
SubgraphPreconditioner::SubgraphPreconditioner(const SubgraphPreconditionerParameters &p) :
         parameters_(p) {}

/* ************************************************************************* */
SubgraphPreconditioner::SubgraphPreconditioner(const sharedFG& Ab2,
    const sharedBayesNet& Rc1, const sharedValues& xbar, const SubgraphPreconditionerParameters &p) :
        Ab2_(convertToJacobianFactors(*Ab2)), Rc1_(Rc1), xbar_(xbar),
        b2bar_(new Errors(-Ab2_->gaussianErrors(*xbar))), parameters_(p) {
}

/* ************************************************************************* */
// x = xbar + inv(R1)*y
VectorValues SubgraphPreconditioner::x(const VectorValues& y) const {
  return *xbar_ + Rc1_->backSubstitute(y);
}

/* ************************************************************************* */
double SubgraphPreconditioner::error(const VectorValues& y) const {
  Errors e(y);
  VectorValues x = this->x(y);
  Errors e2 = Ab2()->gaussianErrors(x);
  return 0.5 * (dot(e, e) + dot(e2,e2));
}

/* ************************************************************************* */
// gradient is y + inv(R1')*A2'*(A2*inv(R1)*y-b2bar),
VectorValues SubgraphPreconditioner::gradient(const VectorValues &y) const {
  VectorValues x = Rc1()->backSubstitute(y); /* inv(R1)*y */
  Errors e = (*Ab2() * x - *b2bar());        /* (A2*inv(R1)*y-b2bar) */
  VectorValues v = VectorValues::Zero(x);
  Ab2()->transposeMultiplyAdd(1.0, e, v);    /* A2'*(A2*inv(R1)*y-b2bar) */
  return y + Rc1()->backSubstituteTranspose(v);
}

/* ************************************************************************* */
// Apply operator A, A*y = [I;A2*inv(R1)]*y = [y; A2*inv(R1)*y]
Errors SubgraphPreconditioner::operator*(const VectorValues& y) const {
  Errors e(y);
  VectorValues x = Rc1()->backSubstitute(y);   /* x=inv(R1)*y */
  Errors e2 = *Ab2() * x;                      /* A2*x */
  e.splice(e.end(), e2);
  return e;
}

/* ************************************************************************* */
// In-place version that overwrites e
void SubgraphPreconditioner::multiplyInPlace(const VectorValues& y, Errors& e) const {

  Errors::iterator ei = e.begin();
  for(const auto& key_value: y) {
    *ei = key_value.second;
    ++ei;
  }

  // Add A2 contribution
  VectorValues x = Rc1()->backSubstitute(y);      // x=inv(R1)*y
  Ab2()->multiplyInPlace(x, ei);                  // use iterator version
}

/* ************************************************************************* */
// Apply operator A', A'*e = [I inv(R1')*A2']*e = e1 + inv(R1')*A2'*e2
VectorValues SubgraphPreconditioner::operator^(const Errors& e) const {

  Errors::const_iterator it = e.begin();
  VectorValues y = zero();
  for(auto& key_value: y) {
    key_value.second = *it;
    ++it;
  }
  transposeMultiplyAdd2(1.0, it, e.end(), y);
  return y;
}

/* ************************************************************************* */
// y += alpha*A'*e
void SubgraphPreconditioner::transposeMultiplyAdd
(double alpha, const Errors& e, VectorValues& y) const {

  Errors::const_iterator it = e.begin();
  for(auto& key_value: y) {
    const Vector& ei = *it;
    axpy(alpha, ei, key_value.second);
    ++it;
  }
  transposeMultiplyAdd2(alpha, it, e.end(), y);
}

/* ************************************************************************* */
// y += alpha*inv(R1')*A2'*e2
void SubgraphPreconditioner::transposeMultiplyAdd2 (double alpha,
    Errors::const_iterator it, Errors::const_iterator end, VectorValues& y) const {

  // create e2 with what's left of e
  // TODO can we avoid creating e2 by passing iterator to transposeMultiplyAdd ?
  Errors e2;
  while (it != end) e2.push_back(*(it++));

  VectorValues x = VectorValues::Zero(y); // x = 0
  Ab2_->transposeMultiplyAdd(1.0,e2,x);   // x += A2'*e2
  axpy(alpha, Rc1_->backSubstituteTranspose(x), y); // y += alpha*inv(R1')*x
}

/* ************************************************************************* */
void SubgraphPreconditioner::print(const std::string& s) const {
  cout << s << endl;
  Ab2_->print();
}

/*****************************************************************************/
void SubgraphPreconditioner::solve(const Vector &y, Vector &x) const {
  assert(x.size() == y.size());

  /* back substitute */
  for (const auto &cg : boost::adaptors::reverse(*Rc1_)) {
    /* collect a subvector of x that consists of the parents of cg (S) */
    const KeyVector parentKeys(cg->beginParents(), cg->endParents());
    const KeyVector frontalKeys(cg->beginFrontals(), cg->endFrontals());
    const Vector xParent = getSubvector(x, keyInfo_, parentKeys);
    const Vector rhsFrontal = getSubvector(y, keyInfo_, frontalKeys);

    /* compute the solution for the current pivot */
    const Vector solFrontal = cg->get_R().triangularView<Eigen::Upper>().solve(
        rhsFrontal - cg->get_S() * xParent);

    /* assign subvector of sol to the frontal variables */
    setSubvector(solFrontal, keyInfo_, frontalKeys, x);
  }
}

/*****************************************************************************/
void SubgraphPreconditioner::transposeSolve(const Vector &y, Vector &x) const {
  /* copy first */
  assert(x.size() == y.size());
  std::copy(y.data(), y.data() + y.rows(), x.data());

  /* in place back substitute */
  for (const auto &cg : *Rc1_) {
    const Vector rhsFrontal = getSubvector(
        x, keyInfo_, KeyVector(cg->beginFrontals(), cg->endFrontals()));
    const Vector solFrontal =
        cg->get_R().transpose().triangularView<Eigen::Lower>().solve(
            rhsFrontal);

    // Check for indeterminant solution
    if (solFrontal.hasNaN())
      throw IndeterminantLinearSystemException(cg->keys().front());

    /* assign subvector of sol to the frontal variables */
    setSubvector(solFrontal, keyInfo_,
                 KeyVector(cg->beginFrontals(), cg->endFrontals()), x);

    /* substract from parent variables */
    for (auto it = cg->beginParents(); it != cg->endParents(); it++) {
      const KeyInfoEntry &info = keyInfo_.find(*it)->second;
      Eigen::Map<Vector> rhsParent(x.data() + info.colstart(), info.dim(), 1);
      rhsParent -= Matrix(cg->getA(it)).transpose() * solFrontal;
    }
  }
}

/*****************************************************************************/
void SubgraphPreconditioner::build(const GaussianFactorGraph &gfg, const KeyInfo &keyInfo, const std::map<Key,Vector> &lambda)
{
  /* identify the subgraph structure */
  const SubgraphBuilder builder(parameters_.builderParams_);
  Subgraph::shared_ptr subgraph = builder(gfg);

  keyInfo_ = keyInfo;

  /* build factor subgraph */
  GaussianFactorGraph::shared_ptr gfg_subgraph = buildFactorSubgraph(gfg, *subgraph, true);

  /* factorize and cache BayesNet */
  Rc1_ = gfg_subgraph->eliminateSequential();
}

/*****************************************************************************/
Vector getSubvector(const Vector &src, const KeyInfo &keyInfo,
                    const KeyVector &keys) {
  /* a cache of starting index and dim */
  vector<std::pair<size_t, size_t> > cache;
  cache.reserve(3);

  /* figure out dimension by traversing the keys */
  size_t dim = 0;
  for (const Key &key : keys) {
    const KeyInfoEntry &entry = keyInfo.find(key)->second;
    cache.emplace_back(entry.colstart(), entry.dim());
    dim += entry.dim();
  }

  /* use the cache to fill the result */
  Vector result = Vector::Zero(dim);
  size_t idx = 0;
  for (const auto &p : cache) {
    result.segment(idx, p.second) = src.segment(p.first, p.second);
    idx += p.second;
  }

  return result;
}

/*****************************************************************************/
void setSubvector(const Vector &src, const KeyInfo &keyInfo, const KeyVector &keys, Vector &dst) {
  size_t idx = 0;
  for ( const Key &key: keys ) {
    const KeyInfoEntry &entry = keyInfo.find(key)->second;
    dst.segment(entry.colstart(), entry.dim()) = src.segment(idx, entry.dim()) ;
    idx += entry.dim();
  }
}

/*****************************************************************************/
GaussianFactorGraph::shared_ptr buildFactorSubgraph(
    const GaussianFactorGraph &gfg, const Subgraph &subgraph,
    const bool clone) {
  auto result = boost::make_shared<GaussianFactorGraph>();
  result->reserve(subgraph.size());
  for ( const SubgraphEdge &e: subgraph ) {
    const size_t idx = e.index();
    if ( clone ) result->push_back(gfg[idx]->clone());
    else result->push_back(gfg[idx]);
  }
  return result;
}

} // nsamespace gtsam
