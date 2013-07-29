/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    GaussianFactorGraph.cpp
 * @brief   Linear Factor Graph where all factors are Gaussians
 * @author  Kai Ni
 * @author  Christian Potthast
 * @author  Richard Roberts
 * @author  Frank Dellaert
 */

#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/linear/VectorValues.h>
#include <gtsam/linear/GaussianBayesTree.h>
#include <gtsam/linear/GaussianEliminationTree.h>
#include <gtsam/linear/GaussianJunctionTree.h>
#include <gtsam/inference/FactorGraph-inst.h>
#include <gtsam/inference/EliminateableFactorGraph-inst.h>
#include <gtsam/base/debug.h>
#include <gtsam/base/timing.h>
#include <gtsam/base/cholesky.h>

using namespace std;
using namespace gtsam;

namespace gtsam {

  /* ************************************************************************* */
  bool GaussianFactorGraph::equals(const This& fg, double tol) const
  {
    return Base::equals(fg, tol);
  }

  /* ************************************************************************* */
  GaussianFactorGraph::Keys GaussianFactorGraph::keys() const {
    FastSet<Key> keys;
    BOOST_FOREACH(const sharedFactor& factor, *this)
    if (factor)
      keys.insert(factor->begin(), factor->end());
    return keys;
  }

  /* ************************************************************************* */
  std::vector<boost::tuple<size_t, size_t, double> > GaussianFactorGraph::sparseJacobian() const {
    // First find dimensions of each variable
    FastVector<size_t> dims;
    BOOST_FOREACH(const sharedFactor& factor, *this) {
      for(GaussianFactor::const_iterator pos = factor->begin(); pos != factor->end(); ++pos) {
        if(dims.size() <= *pos)
          dims.resize(*pos + 1, 0);
        dims[*pos] = factor->getDim(pos);
      }
    }

    // Compute first scalar column of each variable
    vector<size_t> columnIndices(dims.size()+1, 0);
    for(size_t j=1; j<dims.size()+1; ++j)
      columnIndices[j] = columnIndices[j-1] + dims[j-1];

    // Iterate over all factors, adding sparse scalar entries
    typedef boost::tuple<size_t, size_t, double> triplet;
    FastVector<triplet> entries;
    size_t row = 0;
    BOOST_FOREACH(const sharedFactor& factor, *this) {
      // Convert to JacobianFactor if necessary
      JacobianFactor::shared_ptr jacobianFactor(
          boost::dynamic_pointer_cast<JacobianFactor>(factor));
      if (!jacobianFactor) {
        //TODO : re-enable
        //HessianFactor::shared_ptr hessian(boost::dynamic_pointer_cast<HessianFactor>(factor));
        //if (hessian)
        //  jacobianFactor.reset(new JacobianFactor(*hessian));
        //else
          throw invalid_argument(
              "GaussianFactorGraph contains a factor that is neither a JacobianFactor nor a HessianFactor.");
      }

      // Whiten the factor and add entries for it
      // iterate over all variables in the factor
      const JacobianFactor whitened(jacobianFactor->whiten());
      for(JacobianFactor::const_iterator pos=whitened.begin(); pos<whitened.end(); ++pos) {
        JacobianFactor::constABlock whitenedA = whitened.getA(pos);
        // find first column index for this key
        size_t column_start = columnIndices[*pos];
        for (size_t i = 0; i < (size_t) whitenedA.rows(); i++)
          for (size_t j = 0; j < (size_t) whitenedA.cols(); j++) {
            double s = whitenedA(i,j);
            if (std::abs(s) > 1e-12) entries.push_back(
                boost::make_tuple(row+i, column_start+j, s));
          }
      }

      JacobianFactor::constBVector whitenedb(whitened.getb());
      size_t bcolumn = columnIndices.back();
      for (size_t i = 0; i < (size_t) whitenedb.size(); i++)
        entries.push_back(boost::make_tuple(row+i, bcolumn, whitenedb(i)));

      // Increment row index
      row += jacobianFactor->rows();
    }
    return vector<triplet>(entries.begin(), entries.end());
  }

  /* ************************************************************************* */
  Matrix GaussianFactorGraph::sparseJacobian_() const {

    // call sparseJacobian
    typedef boost::tuple<size_t, size_t, double> triplet;
    std::vector<triplet> result = sparseJacobian();

    // translate to base 1 matrix
    size_t nzmax = result.size();
    Matrix IJS(3,nzmax);
    for (size_t k = 0; k < result.size(); k++) {
      const triplet& entry = result[k];
      IJS(0,k) = double(entry.get<0>() + 1);
      IJS(1,k) = double(entry.get<1>() + 1);
      IJS(2,k) = entry.get<2>();
    }
    return IJS;
  }

  /* ************************************************************************* */
  Matrix GaussianFactorGraph::augmentedJacobian() const {
    // combine all factors
    JacobianFactor combined(*this);
    return combined.augmentedJacobian();
  }

  /* ************************************************************************* */
  std::pair<Matrix,Vector> GaussianFactorGraph::jacobian() const {
    Matrix augmented = augmentedJacobian();
    return make_pair(
      augmented.leftCols(augmented.cols()-1),
      augmented.col(augmented.cols()-1));
  }

  /* ************************************************************************* */
  //Matrix GaussianFactorGraph::augmentedHessian() const {
  //  // combine all factors and get upper-triangular part of Hessian
  //  HessianFactor combined(*this);
  //  Matrix result = combined.info();
  //  // Fill in lower-triangular part of Hessian
  //  result.triangularView<Eigen::StrictlyLower>() = result.transpose();
  //  return result;
  //}

  /* ************************************************************************* */
  //std::pair<Matrix,Vector> GaussianFactorGraph::hessian() const {
  //  Matrix augmented = augmentedHessian();
  //  return make_pair(
  //    augmented.topLeftCorner(augmented.rows()-1, augmented.rows()-1),
  //    augmented.col(augmented.rows()-1).head(augmented.rows()-1));
  //}

  /* ************************************************************************* */
  //GaussianFactorGraph::EliminationResult EliminateCholesky(const FactorGraph<
  //    GaussianFactor>& factors, size_t nrFrontals) {

  //  const bool debug = ISDEBUG("EliminateCholesky");

  //  // Form Ab' * Ab
  //  gttic(combine);
  //  HessianFactor::shared_ptr combinedFactor(new HessianFactor(factors));
  //  gttoc(combine);

  //  // Do Cholesky, note that after this, the lower triangle still contains
  //  // some untouched non-zeros that should be zero.  We zero them while
  //  // extracting submatrices next.
  //  gttic(partial_Cholesky);
  //  combinedFactor->partialCholesky(nrFrontals);

  //  gttoc(partial_Cholesky);

  //  // Extract conditional and fill in details of the remaining factor
  //  gttic(split);
  //  GaussianConditional::shared_ptr conditional =
  //      combinedFactor->splitEliminatedFactor(nrFrontals);
  //  if (debug) {
  //    conditional->print("Extracted conditional: ");
  //    combinedFactor->print("Eliminated factor (L piece): ");
  //  }
  //  gttoc(split);

  //  combinedFactor->assertInvariants();
  //  return make_pair(conditional, combinedFactor);
  //}
  
  /* ************************************************************************* */
  VectorValues GaussianFactorGraph::optimize(const Eliminate& function) const
  {
    gttic(GaussianFactorGraph_optimize);
    return BaseEliminateable::eliminateMultifrontal(boost::none, function)->optimize();
  }

  /* ************************************************************************* */
  VectorValues GaussianFactorGraph::gradient(const VectorValues& x0) const
  {
    VectorValues g = VectorValues::Zero(x0);
    Errors e = gaussianErrors(x0);
    transposeMultiplyAdd(1.0, e, g);
    return g;
  }

  /* ************************************************************************* */
  namespace {
    JacobianFactor::shared_ptr convertToJacobianFactorPtr(const GaussianFactor::shared_ptr &gf) {
      JacobianFactor::shared_ptr result = boost::dynamic_pointer_cast<JacobianFactor>(gf);
      if( !result ) {
        result = boost::make_shared<JacobianFactor>(*gf); // Convert any non-Jacobian factors to Jacobians (e.g. Hessian -> Jacobian with Cholesky)
      }
      return result;
    }
  }

  /* ************************************************************************* */
  VectorValues GaussianFactorGraph::gradientAtZero() const
  {
    // Zero-out the gradient
    VectorValues g;
    Errors e;
    BOOST_FOREACH(const sharedFactor& Ai_G, *this) {
      JacobianFactor::shared_ptr Ai = convertToJacobianFactorPtr(Ai_G);
      e.push_back(-Ai->getb());
    }
    transposeMultiplyAdd(1.0, e, g);
    return g;
  }

  /* ************************************************************************* */
  bool hasConstraints(const GaussianFactorGraph& factors) {
    typedef JacobianFactor J;
    BOOST_FOREACH(const GaussianFactor::shared_ptr& factor, factors) {
      J::shared_ptr jacobian(boost::dynamic_pointer_cast<J>(factor));
      if (jacobian && jacobian->get_model()->isConstrained()) {
        return true;
      }
    }
    return false;
  }

  /* ************************************************************************* */
  //GaussianFactorGraph::EliminationResult EliminatePreferCholesky(
  //    const FactorGraph<GaussianFactor>& factors, size_t nrFrontals) {

  //  // If any JacobianFactors have constrained noise models, we have to convert
  //  // all factors to JacobianFactors.  Otherwise, we can convert all factors
  //  // to HessianFactors.  This is because QR can handle constrained noise
  //  // models but Cholesky cannot.
  //  if (hasConstraints(factors))
  //    return EliminateQR(factors, nrFrontals);
  //  else {
  //    GaussianFactorGraph::EliminationResult ret;
  //    gttic(EliminateCholesky);
  //    ret = EliminateCholesky(factors, nrFrontals);
  //    gttoc(EliminateCholesky);
  //    return ret;
  //  }

  //} // \EliminatePreferCholesky



  ///* ************************************************************************* */
  //Errors operator*(const GaussianFactorGraph& fg, const VectorValues& x) {
  //  Errors e;
  //  BOOST_FOREACH(const GaussianFactor::shared_ptr& Ai_G, fg) {
  //    JacobianFactor::shared_ptr Ai = convertToJacobianFactorPtr(Ai_G);
  //    e.push_back((*Ai)*x);
  //  }
  //  return e;
  //}

  ///* ************************************************************************* */
  //void multiplyInPlace(const GaussianFactorGraph& fg, const VectorValues& x, Errors& e) {
  //  multiplyInPlace(fg,x,e.begin());
  //}

  ///* ************************************************************************* */
  //void multiplyInPlace(const GaussianFactorGraph& fg, const VectorValues& x, const Errors::iterator& e) {
  //  Errors::iterator ei = e;
  //  BOOST_FOREACH(const GaussianFactor::shared_ptr& Ai_G, fg) {
  //    JacobianFactor::shared_ptr Ai = convertToJacobianFactorPtr(Ai_G);
  //    *ei = (*Ai)*x;
  //    ei++;
  //  }
  //}

  /* ************************************************************************* */
  // x += alpha*A'*e
  void GaussianFactorGraph::transposeMultiplyAdd(double alpha, const Errors& e, VectorValues& x) const
  {
    // For each factor add the gradient contribution
    Errors::const_iterator ei = e.begin();
    BOOST_FOREACH(const sharedFactor& Ai_G, *this) {
      JacobianFactor::shared_ptr Ai = convertToJacobianFactorPtr(Ai_G);
      Ai->transposeMultiplyAdd(alpha, *(ei++), x);
    }
  }

  ///* ************************************************************************* */
  //void residual(const GaussianFactorGraph& fg, const VectorValues &x, VectorValues &r) {
  //  Key i = 0 ;
  //  BOOST_FOREACH(const GaussianFactor::shared_ptr& Ai_G, fg) {
  //    JacobianFactor::shared_ptr Ai = convertToJacobianFactorPtr(Ai_G);
  //    r[i] = Ai->getb();
  //    i++;
  //  }
  //  VectorValues Ax = VectorValues::SameStructure(r);
  //  multiply(fg,x,Ax);
  //  axpy(-1.0,Ax,r);
  //}

  ///* ************************************************************************* */
  //void multiply(const GaussianFactorGraph& fg, const VectorValues &x, VectorValues &r) {
  //  r.setZero();
  //  Key i = 0;
  //  BOOST_FOREACH(const GaussianFactor::shared_ptr& Ai_G, fg) {
  //    JacobianFactor::shared_ptr Ai = convertToJacobianFactorPtr(Ai_G);
  //    Vector &y = r[i];
  //    for(JacobianFactor::const_iterator j = Ai->begin(); j != Ai->end(); ++j) {
  //      y += Ai->getA(j) * x[*j];
  //    }
  //    ++i;
  //  }
  //}

  /* ************************************************************************* */
  VectorValues GaussianFactorGraph::transposeMultiply(const Errors& e) const
  {
    VectorValues x;
    Errors::const_iterator ei = e.begin();
    BOOST_FOREACH(const sharedFactor& Ai_G, *this) {
      JacobianFactor::shared_ptr Ai = convertToJacobianFactorPtr(Ai_G);
      for(JacobianFactor::const_iterator j = Ai->begin(); j != Ai->end(); ++j) {
        // Create the value as a zero vector if it does not exist.
        pair<VectorValues::iterator, bool> xi = x.tryInsert(*j, Vector());
        if(xi.second)
          xi.first->second = Vector::Zero(Ai->getDim(j));
        xi.first->second += Ai->getA(j).transpose() * *ei;
      }
      ++ ei;
    }
    return x;
  }

  /* ************************************************************************* */
  Errors GaussianFactorGraph::gaussianErrors(const VectorValues& x) const
  {
    Errors e;
    BOOST_FOREACH(const sharedFactor& Ai_G, *this) {
      JacobianFactor::shared_ptr Ai = convertToJacobianFactorPtr(Ai_G);
      e.push_back(Ai->error_vector(x));
    }
    return e;
  }

} // namespace gtsam
