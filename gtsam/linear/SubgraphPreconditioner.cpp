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

using namespace std;

namespace gtsam {

/* ************************************************************************* */
static GaussianFactorGraph::shared_ptr convertToJacobianFactors(const GaussianFactorGraph &gfg) {
  GaussianFactorGraph::shared_ptr result(new GaussianFactorGraph());
  for(const GaussianFactor::shared_ptr &gf: gfg) {
    JacobianFactor::shared_ptr jf = boost::dynamic_pointer_cast<JacobianFactor>(gf);
    if( !jf ) {
      jf = boost::make_shared<JacobianFactor>(*gf); // Convert any non-Jacobian factors to Jacobians (e.g. Hessian -> Jacobian with Cholesky)
    }
    result->push_back(jf);
  }
  return result;
}

/*****************************************************************************/
static std::vector<size_t> iidSampler(const vector<double> &weight, const size_t n) {

  /* compute the sum of the weights */
  const double sum = std::accumulate(weight.begin(), weight.end(), 0.0);

  /* make a normalized and accumulated version of the weight vector */
  const size_t m = weight.size();
  vector<double> w; w.reserve(m);
  for ( size_t i = 0 ; i < m ; ++i ) {
    w.push_back(weight[i]/sum);
  }

  vector<double> acc(m);
  std::partial_sum(w.begin(),w.end(),acc.begin());

  /* iid sample n times */
  vector<size_t> result; result.reserve(n);
  const double denominator = (double)RAND_MAX;
  for ( size_t i = 0 ; i < n ; ++i ) {
    const double value = rand() / denominator;
    /* binary search the interval containing "value" */
    vector<double>::iterator it = std::lower_bound(acc.begin(), acc.end(), value);
    size_t idx = it - acc.begin();
    result.push_back(idx);
  }
  return result;
}

/*****************************************************************************/
vector<size_t> uniqueSampler(const vector<double> &weight, const size_t n) {

  const size_t m = weight.size();
  if ( n > m ) throw std::invalid_argument("uniqueSampler: invalid input size");

  vector<size_t> result;

  size_t count = 0;
  std::vector<bool> touched(m, false);
  while ( count < n ) {
    std::vector<size_t> localIndices; localIndices.reserve(n-count);
    std::vector<double> localWeights; localWeights.reserve(n-count);

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
Subgraph::Subgraph(const std::vector<size_t> &indices) {
  edges_.reserve(indices.size());
  for ( const size_t &idx: indices ) {
    edges_.push_back(SubgraphEdge(idx, 1.0));
  }
}

/****************************************************************************/
std::vector<size_t> Subgraph::edgeIndices() const {
  std::vector<size_t> eid; eid.reserve(size());
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
std::ostream &operator<<(std::ostream &os, const SubgraphEdge &edge) {
  if ( edge.weight() != 1.0 )
    os << edge.index() << "(" << std::setprecision(2) << edge.weight() << ")";
  else
    os << edge.index() ;
  return os;
}

/****************************************************************************/
std::ostream &operator<<(std::ostream &os, const Subgraph &subgraph) {
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
  throw invalid_argument("SubgraphBuilderParameters::skeletonTranslator undefined string " + s);
  return KRUSKAL;
}

/****************************************************************/
std::string SubgraphBuilderParameters::skeletonTranslator(Skeleton w) {
  if ( w == NATURALCHAIN )return "NATURALCHAIN";
  else if ( w == BFS )    return "BFS";
  else if ( w == KRUSKAL )return "KRUSKAL";
  else                    return "UNKNOWN";
}

/****************************************************************/
SubgraphBuilderParameters::SkeletonWeight SubgraphBuilderParameters::skeletonWeightTranslator(const std::string &src) {
  std::string s = src;  boost::algorithm::to_upper(s);
  if (s == "EQUAL")       return EQUAL;
  else if (s == "RHS")    return RHS_2NORM;
  else if (s == "LHS")    return LHS_FNORM;
  else if (s == "RANDOM") return RANDOM;
  throw invalid_argument("SubgraphBuilderParameters::skeletonWeightTranslator undefined string " + s);
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
SubgraphBuilderParameters::AugmentationWeight SubgraphBuilderParameters::augmentationWeightTranslator(const std::string &src) {
  std::string s = src;  boost::algorithm::to_upper(s);
  if (s == "SKELETON")      return SKELETON;
//  else if (s == "STRETCH")  return STRETCH;
//  else if (s == "GENERALIZED_STRETCH")  return GENERALIZED_STRETCH;
  throw invalid_argument("SubgraphBuilder::Parameters::augmentationWeightTranslator undefined string " + s);
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
std::vector<size_t> SubgraphBuilder::buildTree(const GaussianFactorGraph &gfg, const FastMap<Key, size_t> &ordering, const std::vector<double> &w) const {
  const SubgraphBuilderParameters &p = parameters_;
  switch (p.skeleton_) {
  case SubgraphBuilderParameters::NATURALCHAIN:
    return natural_chain(gfg);
    break;
  case SubgraphBuilderParameters::BFS:
    return bfs(gfg);
    break;
  case SubgraphBuilderParameters::KRUSKAL:
    return kruskal(gfg, ordering, w);
    break;
  default:
    cerr << "SubgraphBuilder::buildTree undefined skeleton type" << endl;
    break;
  }
  return vector<size_t>();
}

/****************************************************************/
std::vector<size_t> SubgraphBuilder::unary(const GaussianFactorGraph &gfg) const {
  std::vector<size_t> result ;
  size_t idx = 0;
  for ( const GaussianFactor::shared_ptr &gf: gfg ) {
    if ( gf->size() == 1 ) {
      result.push_back(idx);
    }
    idx++;
  }
  return result;
}

/****************************************************************/
std::vector<size_t> SubgraphBuilder::natural_chain(const GaussianFactorGraph &gfg) const {
  std::vector<size_t> result ;
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
std::vector<size_t> SubgraphBuilder::bfs(const GaussianFactorGraph &gfg) const {
  const VariableIndex variableIndex(gfg);
  /* start from the first key of the first factor */
  Key seed = gfg[0]->keys()[0];

  const size_t n = variableIndex.size();

  /* each vertex has self as the predecessor */
  std::vector<size_t> result;
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
std::vector<size_t> SubgraphBuilder::kruskal(const GaussianFactorGraph &gfg, const FastMap<Key, size_t> &ordering, const std::vector<double> &w) const {
  const VariableIndex variableIndex(gfg);
  const size_t n = variableIndex.size();
  const vector<size_t> idx = sort_idx(w) ;

  /* initialize buffer */
  vector<size_t> result;
  result.reserve(n-1);

  // container for acsendingly sorted edges
  DSFVector D(n) ;

  size_t count = 0 ; double sum = 0.0 ;
  for (const size_t id: idx) {
    const GaussianFactor &gf = *gfg[id];
    if ( gf.keys().size() != 2 ) continue;
    const size_t u = ordering.find(gf.keys()[0])->second,
                 u_root = D.find(u),
                 v = ordering.find(gf.keys()[1])->second,
                 v_root = D.find(v) ;
    if ( u_root != v_root ) {
      D.merge(u_root, v_root) ;
      result.push_back(id) ;
      sum += w[id] ;
      if ( ++count == n-1 ) break ;
    }
  }
  return result;
}

/****************************************************************/
std::vector<size_t> SubgraphBuilder::sample(const std::vector<double> &weights, const size_t t) const {
  return uniqueSampler(weights, t);
}

/****************************************************************/
Subgraph::shared_ptr SubgraphBuilder::operator() (const GaussianFactorGraph &gfg) const {

  const SubgraphBuilderParameters &p = parameters_;
  const Ordering inverse_ordering = Ordering::Natural(gfg);
  const FastMap<Key, size_t> forward_ordering = inverse_ordering.invert();
  const size_t n = inverse_ordering.size(), t = n * p.complexity_ ;

  vector<double> w = weights(gfg);
  const vector<size_t> tree = buildTree(gfg, forward_ordering, w);

  /* sanity check */
  if ( tree.size() != n-1 ) {
    throw runtime_error("SubgraphBuilder::operator() tree.size() != n-1 failed ");
  }

  /* down weight the tree edges to zero */
  for ( const size_t id: tree ) {
    w[id] = 0.0;
  }

  /* decide how many edges to augment */
  std::vector<size_t> offTree = sample(w, t);

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
      throw invalid_argument("SubgraphBuilder::weights: undefined weight scheme ");
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
VectorValues SubgraphPreconditioner::gradient(const VectorValues& y) const {
  VectorValues x = Rc1()->backSubstitute(y); /* inv(R1)*y */
  Errors e = (*Ab2()*x - *b2bar());               /* (A2*inv(R1)*y-b2bar) */
  VectorValues v = VectorValues::Zero(x);
  Ab2()->transposeMultiplyAdd(1.0, e, v);           /* A2'*(A2*inv(R1)*y-b2bar) */
  return y + Rc1()->backSubstituteTranspose(v);
}

/* ************************************************************************* */
// Apply operator A, A*y = [I;A2*inv(R1)]*y = [y; A2*inv(R1)*y]
Errors SubgraphPreconditioner::operator*(const VectorValues& y) const {

  Errors e(y);
  VectorValues x = Rc1()->backSubstitute(y);   /* x=inv(R1)*y */
  Errors e2 = *Ab2() * x;                              /* A2*x */
  e.splice(e.end(), e2);
  return e;
}

/* ************************************************************************* */
// In-place version that overwrites e
void SubgraphPreconditioner::multiplyInPlace(const VectorValues& y, Errors& e) const {

  Errors::iterator ei = e.begin();
  for (size_t i = 0; i < y.size(); ++i, ++ei)
    *ei = y[i];

  // Add A2 contribution
  VectorValues x = Rc1()->backSubstitute(y);      // x=inv(R1)*y
  Ab2()->multiplyInPlace(x, ei);                  // use iterator version
}

/* ************************************************************************* */
// Apply operator A', A'*e = [I inv(R1')*A2']*e = e1 + inv(R1')*A2'*e2
VectorValues SubgraphPreconditioner::operator^(const Errors& e) const {

  Errors::const_iterator it = e.begin();
  VectorValues y = zero();
  for (size_t i = 0; i < y.size(); ++i, ++it)
    y[i] = *it;
  transposeMultiplyAdd2(1.0, it, e.end(), y);
  return y;
}

/* ************************************************************************* */
// y += alpha*A'*e
void SubgraphPreconditioner::transposeMultiplyAdd
(double alpha, const Errors& e, VectorValues& y) const {

  Errors::const_iterator it = e.begin();
  for (size_t i = 0; i < y.size(); ++i, ++it) {
    const Vector& ei = *it;
    axpy(alpha, ei, y[i]);
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
void SubgraphPreconditioner::solve(const Vector& y, Vector &x) const
{
  /* copy first */
  std::copy(y.data(), y.data() + y.rows(), x.data());

  /* in place back substitute */
  for (auto cg: boost::adaptors::reverse(*Rc1_)) {
    /* collect a subvector of x that consists of the parents of cg (S) */
    const Vector xParent = getSubvector(x, keyInfo_, FastVector<Key>(cg->beginParents(), cg->endParents()));
    const Vector rhsFrontal = getSubvector(x, keyInfo_, FastVector<Key>(cg->beginFrontals(), cg->endFrontals()));

    /* compute the solution for the current pivot */
    const Vector solFrontal = cg->get_R().triangularView<Eigen::Upper>().solve(rhsFrontal - cg->get_S() * xParent);

    /* assign subvector of sol to the frontal variables */
    setSubvector(solFrontal, keyInfo_, FastVector<Key>(cg->beginFrontals(), cg->endFrontals()), x);
  }
}

/*****************************************************************************/
void SubgraphPreconditioner::transposeSolve(const Vector& y, Vector& x) const
{
  /* copy first */
  std::copy(y.data(), y.data() + y.rows(), x.data());

  /* in place back substitute */
  for(const boost::shared_ptr<GaussianConditional> & cg: *Rc1_) {
    const Vector rhsFrontal = getSubvector(x, keyInfo_, FastVector<Key>(cg->beginFrontals(), cg->endFrontals()));
//    const Vector solFrontal = cg->get_R().triangularView<Eigen::Upper>().transpose().solve(rhsFrontal);
    const Vector solFrontal = cg->get_R().transpose().triangularView<Eigen::Lower>().solve(rhsFrontal);

    // Check for indeterminant solution
    if ( solFrontal.hasNaN()) throw IndeterminantLinearSystemException(cg->keys().front());

    /* assign subvector of sol to the frontal variables */
    setSubvector(solFrontal, keyInfo_, FastVector<Key>(cg->beginFrontals(), cg->endFrontals()), x);

    /* substract from parent variables */
    for (GaussianConditional::const_iterator it = cg->beginParents(); it != cg->endParents(); it++) {
      KeyInfo::const_iterator it2 = keyInfo_.find(*it);
      Eigen::Map<Vector> rhsParent(x.data()+it2->second.colstart(), it2->second.dim(), 1);
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
Vector getSubvector(const Vector &src, const KeyInfo &keyInfo, const FastVector<Key> &keys) {

  /* a cache of starting index and dim */
  typedef vector<pair<size_t, size_t> > Cache;
  Cache cache;

  /* figure out dimension by traversing the keys */
  size_t d = 0;
  for ( const Key &key: keys ) {
    const KeyInfoEntry &entry = keyInfo.find(key)->second;
    cache.push_back(make_pair(entry.colstart(), entry.dim()));
    d += entry.dim();
  }

  /* use the cache to fill the result */
  Vector result = Vector::Zero(d, 1);
  size_t idx = 0;
  for ( const Cache::value_type &p: cache ) {
    result.segment(idx, p.second) = src.segment(p.first, p.second) ;
    idx += p.second;
  }

  return result;
}

/*****************************************************************************/
void setSubvector(const Vector &src, const KeyInfo &keyInfo, const FastVector<Key> &keys, Vector &dst) {
  /* use the cache  */
  size_t idx = 0;
  for ( const Key &key: keys ) {
    const KeyInfoEntry &entry = keyInfo.find(key)->second;
    dst.segment(entry.colstart(), entry.dim()) = src.segment(idx, entry.dim()) ;
    idx += entry.dim();
  }
}

/*****************************************************************************/
boost::shared_ptr<GaussianFactorGraph>
buildFactorSubgraph(const GaussianFactorGraph &gfg, const Subgraph &subgraph, const bool clone) {

  GaussianFactorGraph::shared_ptr result(new GaussianFactorGraph());
  result->reserve(subgraph.size());
  for ( const SubgraphEdge &e: subgraph ) {
    const size_t idx = e.index();
    if ( clone ) result->push_back(gfg[idx]->clone());
    else result->push_back(gfg[idx]);
  }
  return result;
}

} // nsamespace gtsam
