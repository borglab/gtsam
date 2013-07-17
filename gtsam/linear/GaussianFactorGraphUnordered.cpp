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

#include <gtsam/linear/GaussianFactorGraphUnordered.h>
#include <gtsam/linear/VectorValuesUnordered.h>
#include <gtsam/linear/GaussianBayesTreeUnordered.h>
#include <gtsam/linear/GaussianEliminationTreeUnordered.h>
#include <gtsam/linear/GaussianJunctionTreeUnordered.h>
#include <gtsam/inference/FactorGraphUnordered-inst.h>
#include <gtsam/inference/EliminateableFactorGraph-inst.h>
#include <gtsam/base/debug.h>
#include <gtsam/base/timing.h>
#include <gtsam/base/cholesky.h>

using namespace std;
using namespace gtsam;

namespace gtsam {

  /* ************************************************************************* */
  bool GaussianFactorGraphUnordered::equals(const This& fg, double tol) const
  {
    return Base::equals(fg, tol);
  }

  /* ************************************************************************* */
  void GaussianFactorGraphUnordered::push_back_bayesTree(const GaussianBayesTreeUnordered& bayesTree)
  {
    Base::push_back_bayesTree(bayesTree);
  }

  /* ************************************************************************* */
  GaussianFactorGraphUnordered::Keys GaussianFactorGraphUnordered::keys() const {
    FastSet<Key> keys;
    BOOST_FOREACH(const sharedFactor& factor, *this)
    if (factor)
      keys.insert(factor->begin(), factor->end());
    return keys;
  }

  /* ************************************************************************* */
  std::vector<boost::tuple<size_t, size_t, double> > GaussianFactorGraphUnordered::sparseJacobian() const {
    // First find dimensions of each variable
    FastVector<size_t> dims;
    BOOST_FOREACH(const sharedFactor& factor, *this) {
      for(GaussianFactorUnordered::const_iterator pos = factor->begin(); pos != factor->end(); ++pos) {
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
      JacobianFactorUnordered::shared_ptr jacobianFactor(
          boost::dynamic_pointer_cast<JacobianFactorUnordered>(factor));
      if (!jacobianFactor) {
        //TODO Unordered: re-enable
        //HessianFactorUnordered::shared_ptr hessian(boost::dynamic_pointer_cast<HessianFactorUnordered>(factor));
        //if (hessian)
        //  jacobianFactor.reset(new JacobianFactorUnordered(*hessian));
        //else
          throw invalid_argument(
              "GaussianFactorGraph contains a factor that is neither a JacobianFactor nor a HessianFactor.");
      }

      // Whiten the factor and add entries for it
      // iterate over all variables in the factor
      const JacobianFactorUnordered whitened(jacobianFactor->whiten());
      for(JacobianFactorUnordered::const_iterator pos=whitened.begin(); pos<whitened.end(); ++pos) {
        JacobianFactorUnordered::constABlock whitenedA = whitened.getA(pos);
        // find first column index for this key
        size_t column_start = columnIndices[*pos];
        for (size_t i = 0; i < (size_t) whitenedA.rows(); i++)
          for (size_t j = 0; j < (size_t) whitenedA.cols(); j++) {
            double s = whitenedA(i,j);
            if (std::abs(s) > 1e-12) entries.push_back(
                boost::make_tuple(row+i, column_start+j, s));
          }
      }

      JacobianFactorUnordered::constBVector whitenedb(whitened.getb());
      size_t bcolumn = columnIndices.back();
      for (size_t i = 0; i < (size_t) whitenedb.size(); i++)
        entries.push_back(boost::make_tuple(row+i, bcolumn, whitenedb(i)));

      // Increment row index
      row += jacobianFactor->rows();
    }
    return vector<triplet>(entries.begin(), entries.end());
  }

  /* ************************************************************************* */
  Matrix GaussianFactorGraphUnordered::sparseJacobian_() const {

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
  Matrix GaussianFactorGraphUnordered::augmentedJacobian() const {
    // combine all factors
    JacobianFactorUnordered combined(*this);
    return combined.augmentedJacobian();
  }

  /* ************************************************************************* */
  std::pair<Matrix,Vector> GaussianFactorGraphUnordered::jacobian() const {
    Matrix augmented = augmentedJacobian();
    return make_pair(
      augmented.leftCols(augmented.cols()-1),
      augmented.col(augmented.cols()-1));
  }

  /* ************************************************************************* */
  //Matrix GaussianFactorGraphUnordered::augmentedHessian() const {
  //  // combine all factors and get upper-triangular part of Hessian
  //  HessianFactorUnordered combined(*this);
  //  Matrix result = combined.info();
  //  // Fill in lower-triangular part of Hessian
  //  result.triangularView<Eigen::StrictlyLower>() = result.transpose();
  //  return result;
  //}

  /* ************************************************************************* */
  //std::pair<Matrix,Vector> GaussianFactorGraphUnordered::hessian() const {
  //  Matrix augmented = augmentedHessian();
  //  return make_pair(
  //    augmented.topLeftCorner(augmented.rows()-1, augmented.rows()-1),
  //    augmented.col(augmented.rows()-1).head(augmented.rows()-1));
  //}

  /* ************************************************************************* */
  //GaussianFactorGraphUnordered::EliminationResult EliminateCholesky(const FactorGraph<
  //    GaussianFactorUnordered>& factors, size_t nrFrontals) {

  //  const bool debug = ISDEBUG("EliminateCholesky");

  //  // Form Ab' * Ab
  //  gttic(combine);
  //  HessianFactorUnordered::shared_ptr combinedFactor(new HessianFactorUnordered(factors));
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
  VectorValuesUnordered GaussianFactorGraphUnordered::optimize(const Eliminate& function) const
  {
    return BaseEliminateable::eliminateMultifrontal(boost::none, function)->optimize();
  }

  /* ************************************************************************* */
  VectorValuesUnordered GaussianFactorGraphUnordered::gradient(const VectorValuesUnordered& x0) const
  {
    VectorValuesUnordered g = VectorValuesUnordered::Zero(x0);
    Errors e = gaussianErrors(x0);
    transposeMultiplyAdd(1.0, e, g);
    return g;
  }

  /* ************************************************************************* */
  namespace {
    JacobianFactorUnordered::shared_ptr convertToJacobianFactorPtr(const GaussianFactorUnordered::shared_ptr &gf) {
      JacobianFactorUnordered::shared_ptr result = boost::dynamic_pointer_cast<JacobianFactorUnordered>(gf);
      if( !result ) {
        result = boost::make_shared<JacobianFactorUnordered>(*gf); // Convert any non-Jacobian factors to Jacobians (e.g. Hessian -> Jacobian with Cholesky)
      }
      return result;
    }
  }

  /* ************************************************************************* */
  VectorValuesUnordered GaussianFactorGraphUnordered::gradientAtZero() const
  {
    // Zero-out the gradient
    VectorValuesUnordered g;
    Errors e;
    BOOST_FOREACH(const sharedFactor& Ai_G, *this) {
      JacobianFactorUnordered::shared_ptr Ai = convertToJacobianFactorPtr(Ai_G);
      e.push_back(-Ai->getb());
    }
    transposeMultiplyAdd(1.0, e, g);
    return g;
  }

  /* ************************************************************************* */
  bool hasConstraints(const GaussianFactorGraphUnordered& factors) {
    typedef JacobianFactorUnordered J;
    BOOST_FOREACH(const GaussianFactorUnordered::shared_ptr& factor, factors) {
      J::shared_ptr jacobian(boost::dynamic_pointer_cast<J>(factor));
      if (jacobian && jacobian->get_model()->isConstrained()) {
        return true;
      }
    }
    return false;
  }

  /* ************************************************************************* */
  //GaussianFactorGraphUnordered::EliminationResult EliminatePreferCholesky(
  //    const FactorGraph<GaussianFactorUnordered>& factors, size_t nrFrontals) {

  //  // If any JacobianFactors have constrained noise models, we have to convert
  //  // all factors to JacobianFactors.  Otherwise, we can convert all factors
  //  // to HessianFactors.  This is because QR can handle constrained noise
  //  // models but Cholesky cannot.
  //  if (hasConstraints(factors))
  //    return EliminateQR(factors, nrFrontals);
  //  else {
  //    GaussianFactorGraphUnordered::EliminationResult ret;
  //    gttic(EliminateCholesky);
  //    ret = EliminateCholesky(factors, nrFrontals);
  //    gttoc(EliminateCholesky);
  //    return ret;
  //  }

  //} // \EliminatePreferCholesky



  ///* ************************************************************************* */
  //Errors operator*(const GaussianFactorGraphUnordered& fg, const VectorValuesUnordered& x) {
  //  Errors e;
  //  BOOST_FOREACH(const GaussianFactorUnordered::shared_ptr& Ai_G, fg) {
  //    JacobianFactorUnordered::shared_ptr Ai = convertToJacobianFactorPtr(Ai_G);
  //    e.push_back((*Ai)*x);
  //  }
  //  return e;
  //}

  ///* ************************************************************************* */
  //void multiplyInPlace(const GaussianFactorGraphUnordered& fg, const VectorValuesUnordered& x, Errors& e) {
  //  multiplyInPlace(fg,x,e.begin());
  //}

  ///* ************************************************************************* */
  //void multiplyInPlace(const GaussianFactorGraphUnordered& fg, const VectorValuesUnordered& x, const Errors::iterator& e) {
  //  Errors::iterator ei = e;
  //  BOOST_FOREACH(const GaussianFactorUnordered::shared_ptr& Ai_G, fg) {
  //    JacobianFactorUnordered::shared_ptr Ai = convertToJacobianFactorPtr(Ai_G);
  //    *ei = (*Ai)*x;
  //    ei++;
  //  }
  //}

  /* ************************************************************************* */
  // x += alpha*A'*e
  void GaussianFactorGraphUnordered::transposeMultiplyAdd(double alpha, const Errors& e, VectorValuesUnordered& x) const
  {
    // For each factor add the gradient contribution
    Errors::const_iterator ei = e.begin();
    BOOST_FOREACH(const sharedFactor& Ai_G, *this) {
      JacobianFactorUnordered::shared_ptr Ai = convertToJacobianFactorPtr(Ai_G);
      Ai->transposeMultiplyAdd(alpha, *(ei++), x);
    }
  }

  ///* ************************************************************************* */
  //void residual(const GaussianFactorGraphUnordered& fg, const VectorValuesUnordered &x, VectorValuesUnordered &r) {
  //  Key i = 0 ;
  //  BOOST_FOREACH(const GaussianFactorUnordered::shared_ptr& Ai_G, fg) {
  //    JacobianFactorUnordered::shared_ptr Ai = convertToJacobianFactorPtr(Ai_G);
  //    r[i] = Ai->getb();
  //    i++;
  //  }
  //  VectorValuesUnordered Ax = VectorValuesUnordered::SameStructure(r);
  //  multiply(fg,x,Ax);
  //  axpy(-1.0,Ax,r);
  //}

  ///* ************************************************************************* */
  //void multiply(const GaussianFactorGraphUnordered& fg, const VectorValuesUnordered &x, VectorValuesUnordered &r) {
  //  r.setZero();
  //  Key i = 0;
  //  BOOST_FOREACH(const GaussianFactorUnordered::shared_ptr& Ai_G, fg) {
  //    JacobianFactorUnordered::shared_ptr Ai = convertToJacobianFactorPtr(Ai_G);
  //    Vector &y = r[i];
  //    for(JacobianFactorUnordered::const_iterator j = Ai->begin(); j != Ai->end(); ++j) {
  //      y += Ai->getA(j) * x[*j];
  //    }
  //    ++i;
  //  }
  //}

  /* ************************************************************************* */
  VectorValuesUnordered GaussianFactorGraphUnordered::transposeMultiply(const Errors& e) const
  {
    VectorValuesUnordered x;
    Errors::const_iterator ei = e.begin();
    BOOST_FOREACH(const sharedFactor& Ai_G, *this) {
      JacobianFactorUnordered::shared_ptr Ai = convertToJacobianFactorPtr(Ai_G);
      for(JacobianFactorUnordered::const_iterator j = Ai->begin(); j != Ai->end(); ++j) {
        // Create the value as a zero vector if it does not exist.
        pair<VectorValuesUnordered::iterator, bool> xi = x.tryInsert(*j, Vector());
        if(xi.second)
          xi.first->second = Vector::Zero(Ai->getDim(j));
        xi.first->second += Ai->getA(j).transpose() * *ei;
      }
      ++ ei;
    }
    return x;
  }

  /* ************************************************************************* */
  Errors GaussianFactorGraphUnordered::gaussianErrors(const VectorValuesUnordered& x) const
  {
    Errors e;
    BOOST_FOREACH(const sharedFactor& Ai_G, *this) {
      JacobianFactorUnordered::shared_ptr Ai = convertToJacobianFactorPtr(Ai_G);
      e.push_back(Ai->error_vector(x));
    }
    return e;
  }

} // namespace gtsam
