/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

#include <gtsam/linear/SimpleSPCGSolver.h>
#include <gtsam/linear/GaussianConditional.h>
#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/linear/GaussianBayesNet.h>
#include <gtsam/linear/JacobianFactor.h>
#include <gtsam/linear/VectorValues.h>
#include <gtsam/inference/EliminationTree.h>
#include <boost/foreach.hpp>
#include <boost/shared_ptr.hpp>

namespace gtsam {

/* utility function */
std::vector<size_t> extractRowSpec_(const FactorGraph<JacobianFactor>& jfg) {
  std::vector<size_t> spec; spec.reserve(jfg.size());
  BOOST_FOREACH ( const JacobianFactor::shared_ptr &jf, jfg ) {
    spec.push_back(jf->rows());
  }
  return spec;
}

std::vector<size_t> extractColSpec_(const FactorGraph<GaussianFactor>& gfg, const VariableIndex &index) {
  const size_t n = index.size();
  std::vector<size_t> spec(n, 0);
  for ( Index i = 0 ; i < n ; ++i ) {
    const GaussianFactor &gf = *(gfg[index[i].front()]);
    for ( GaussianFactor::const_iterator it = gf.begin() ; it != gf.end() ; ++it ) {
      spec[*it] = gf.getDim(it);
    }
  }
  return spec;
}

SimpleSPCGSolver::SimpleSPCGSolver(const GaussianFactorGraph &gfg, const Parameters::shared_ptr &parameters)
  : Base(parameters)
{
  std::vector<size_t> colSpec = extractColSpec_(gfg, VariableIndex(gfg));

  nVar_ = colSpec.size();

  /* Split the factor graph into At (tree) and Ac (constraints)
   * Note: This part has to be refined for your graph/application */
  GaussianFactorGraph::shared_ptr At;
  boost::tie(At, Ac_) = this->splitGraph(gfg);

  /* construct row vector spec of the new system */
  nAc_ = Ac_->size();
  std::vector<size_t> rowSpec = extractRowSpec_(*Ac_);
  for ( Index i = 0 ; i < nVar_ ; ++i ) {
    rowSpec.push_back(colSpec[i]);
  }

  /* solve the tree with direct solver. get preconditioner */
  Rt_ = EliminationTree<GaussianFactor>::Create(*At)->eliminate(EliminateQR);
  xt_ = boost::make_shared<VectorValues>(gtsam::optimize(*Rt_));

  /* initial value for the lspcg method */
  y0_ = boost::make_shared<VectorValues>(VectorValues::Zero(colSpec));

  /* the right hand side of the new system */
  by_ = boost::make_shared<VectorValues>(VectorValues::Zero(rowSpec));
  for ( Index i = 0 ; i < nAc_ ; ++i ) {
    JacobianFactor::shared_ptr jf = (*Ac_)[i];
    Vector xi = internal::extractVectorValuesSlices(*xt_, jf->begin(), jf->end());
    (*by_)[i] = jf->getb() - jf->getA()*xi;
  }

  /* allocate buffer for row and column vectors */
  tmpY_ = boost::make_shared<VectorValues>(VectorValues::Zero(colSpec));
  tmpB_ = boost::make_shared<VectorValues>(VectorValues::Zero(rowSpec));
}

/* implements the CGLS method in Section 7.4 of Bjork's book */
VectorValues::shared_ptr SimpleSPCGSolver::optimize (const VectorValues &initial) {

  VectorValues::shared_ptr x(new VectorValues(initial));
  VectorValues r = VectorValues::Zero(*by_),
               q = VectorValues::Zero(*by_),
               p = VectorValues::Zero(*y0_),
               s = VectorValues::Zero(*y0_);

  residual(*x, r);
  transposeMultiply(r, s) ;
  p.vector() = s.vector() ;

  double gamma = s.vector().squaredNorm(), new_gamma = 0.0, alpha = 0.0, beta = 0.0 ;

  const double threshold =
            ::max(parameters_->epsilon_abs(),
                  parameters_->epsilon() * parameters_->epsilon() * gamma);
  const size_t iMaxIterations = parameters_->maxIterations();

  if ( parameters_->verbosity() >= IterativeOptimizationParameters::ERROR )
    cout << "[SimpleSPCGSolver] epsilon = " << parameters_->epsilon()
         << ", max = " << parameters_->maxIterations()
         << ", ||r0|| = " << std::sqrt(gamma)
         << ", threshold = " << threshold << std::endl;

  size_t k ;
  for ( k = 1 ; k < iMaxIterations ; ++k ) {

    multiply(p, q);
    alpha = gamma / q.vector().squaredNorm() ;
    x->vector() += (alpha * p.vector());
    r.vector() -= (alpha * q.vector());
    transposeMultiply(r, s);
    new_gamma = s.vector().squaredNorm();
    beta = new_gamma / gamma ;
    p.vector() = s.vector() + beta * p.vector();
    gamma = new_gamma ;

    if ( parameters_->verbosity() >= IterativeOptimizationParameters::ERROR) {
      cout << "[SimpleSPCGSolver] iteration " << k << ": a = " << alpha << ": b = " << beta << ", ||r|| = " << std::sqrt(gamma) << endl;
    }

    if ( gamma < threshold ) break ;
  } // k

  if ( parameters_->verbosity() >= IterativeOptimizationParameters::ERROR )
    cout << "[SimpleSPCGSolver] iteration " << k << ": a = " << alpha << ": b = " << beta << ", ||r|| = " << std::sqrt(gamma) << endl;

  /* transform y back to x */
  return this->transform(*x);
}

void SimpleSPCGSolver::residual(const VectorValues &input, VectorValues &output) {
  output.vector() = by_->vector();
  this->multiply(input, *tmpB_);
  axpy(-1.0, *tmpB_, output);
}

void SimpleSPCGSolver::multiply(const VectorValues &input, VectorValues &output) {
  this->backSubstitute(input, *tmpY_);
  gtsam::multiply(*Ac_, *tmpY_, output);
  for ( Index i = 0 ; i < nVar_ ; ++i ) {
    output[i + nAc_] = input[i];
  }
}

void SimpleSPCGSolver::transposeMultiply(const VectorValues &input, VectorValues &output) {
  gtsam::transposeMultiply(*Ac_, input, *tmpY_);
  this->transposeBackSubstitute(*tmpY_, output);
  for ( Index i = 0 ; i < nVar_ ; ++i ) {
    output[i] += input[nAc_+i];
  }
}

void SimpleSPCGSolver::backSubstitute(const VectorValues &input, VectorValues &output) {
  for ( Index i = 0 ; i < input.size() ; ++i ) {
    output[i] = input[i] ;
  }
  BOOST_REVERSE_FOREACH(const boost::shared_ptr<const GaussianConditional> cg, *Rt_) {
    const Index key = *(cg->beginFrontals());
    Vector xS = internal::extractVectorValuesSlices(output, cg->beginParents(), cg->endParents());
    xS = input[key] - cg->get_S() * xS;
    output[key] = cg->get_R().triangularView<Eigen::Upper>().solve(xS);
  }
}

void SimpleSPCGSolver::transposeBackSubstitute(const VectorValues &input, VectorValues &output) {
  for ( Index i = 0 ; i < input.size() ; ++i ) {
    output[i] = input[i] ;
  }
  BOOST_FOREACH(const boost::shared_ptr<const GaussianConditional> cg, *Rt_) {
    const Index key = *(cg->beginFrontals());
    output[key] = gtsam::backSubstituteUpper(output[key], Matrix(cg->get_R()));
    const Vector d = internal::extractVectorValuesSlices(output, cg->beginParents(), cg->endParents())
                   - cg->get_S().transpose() * output[key];
    internal::writeVectorValuesSlices(d, output, cg->beginParents(), cg->endParents());
  }
}

VectorValues::shared_ptr SimpleSPCGSolver::transform(const VectorValues &y) {
  VectorValues::shared_ptr x = boost::make_shared<VectorValues>(VectorValues::Zero(y));
  this->backSubstitute(y, *x);
  axpy(1.0, *xt_, *x);
  return x;
}

boost::tuple<GaussianFactorGraph::shared_ptr, FactorGraph<JacobianFactor>::shared_ptr>
SimpleSPCGSolver::splitGraph(const GaussianFactorGraph &gfg) {

  VariableIndex index(gfg);
  size_t n = index.size();
  std::vector<bool> connected(n, false);

  GaussianFactorGraph::shared_ptr At(new GaussianFactorGraph());
  FactorGraph<JacobianFactor>::shared_ptr Ac( new FactorGraph<JacobianFactor>());

  BOOST_FOREACH ( const GaussianFactor::shared_ptr &gf, gfg ) {
    bool augment = false ;

    /* check whether this factor should be augmented to the "tree" graph */
    if ( gf->keys().size() == 1 ) augment = true;
    else {
      BOOST_FOREACH ( const Index key, *gf ) {
        if ( connected[key] == false ) {
          augment = true ;
        }
        connected[key] = true;
      }
    }

    if ( augment ) At->push_back(gf);
    else Ac->push_back(boost::dynamic_pointer_cast<JacobianFactor>(gf));
  }

//  gfg.print("gfg");
//  At->print("At");
//  Ac->print("Ac");

  return boost::tie(At, Ac);
}






}
