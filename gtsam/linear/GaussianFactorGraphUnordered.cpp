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
#include <gtsam/base/debug.h>
#include <gtsam/base/timing.h>
#include <gtsam/base/cholesky.h>

using namespace std;
using namespace gtsam;

namespace gtsam {

  /* ************************************************************************* */
  GaussianFactorGraphUnordered::Keys GaussianFactorGraphUnordered::keys() const {
    FastSet<Index> keys;
    BOOST_FOREACH(const sharedFactor& factor, *this)
    if (factor)
      keys.insert(factor->begin(), factor->end());
    return keys;
  }

  /* ************************************************************************* */
  GaussianFactorGraphUnordered GaussianFactorGraphUnordered::combine2(
      const GaussianFactorGraphUnordered& lfg1, const GaussianFactorGraphUnordered& lfg2)
  {
    // Copy the first graph and add the second graph
    GaussianFactorGraphUnordered fg = lfg1;
    fg.push_back(lfg2);
    return fg;
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
    return combined.matrix_augmented();
  }

  /* ************************************************************************* */
  std::pair<Matrix,Vector> GaussianFactorGraphUnordered::jacobian() const {
    Matrix augmented = augmentedJacobian();
    return make_pair(
      augmented.leftCols(augmented.cols()-1),
      augmented.col(augmented.cols()-1));
  }
  
  /* ************************************************************************* */
  GaussianFactorGraphUnordered::EliminationResult EliminateJacobians(const FactorGraph<
      JacobianFactorUnordered>& factors, size_t nrFrontals) {
  }

  /* ************************************************************************* */
  Matrix GaussianFactorGraphUnordered::augmentedHessian() const {
    // combine all factors and get upper-triangular part of Hessian
    HessianFactorUnordered combined(*this);
    Matrix result = combined.info();
    // Fill in lower-triangular part of Hessian
    result.triangularView<Eigen::StrictlyLower>() = result.transpose();
    return result;
  }

  /* ************************************************************************* */
  std::pair<Matrix,Vector> GaussianFactorGraphUnordered::hessian() const {
    Matrix augmented = augmentedHessian();
    return make_pair(
      augmented.topLeftCorner(augmented.rows()-1, augmented.rows()-1),
      augmented.col(augmented.rows()-1).head(augmented.rows()-1));
  }

  /* ************************************************************************* */
  GaussianFactorGraphUnordered::EliminationResult EliminateCholesky(const FactorGraph<
      GaussianFactorUnordered>& factors, size_t nrFrontals) {

    const bool debug = ISDEBUG("EliminateCholesky");

    // Form Ab' * Ab
    gttic(combine);
    HessianFactorUnordered::shared_ptr combinedFactor(new HessianFactorUnordered(factors));
    gttoc(combine);

    // Do Cholesky, note that after this, the lower triangle still contains
    // some untouched non-zeros that should be zero.  We zero them while
    // extracting submatrices next.
    gttic(partial_Cholesky);
    combinedFactor->partialCholesky(nrFrontals);

    gttoc(partial_Cholesky);

    // Extract conditional and fill in details of the remaining factor
    gttic(split);
    GaussianConditional::shared_ptr conditional =
        combinedFactor->splitEliminatedFactor(nrFrontals);
    if (debug) {
      conditional->print("Extracted conditional: ");
      combinedFactor->print("Eliminated factor (L piece): ");
    }
    gttoc(split);

    combinedFactor->assertInvariants();
    return make_pair(conditional, combinedFactor);
  }

  /* ************************************************************************* */
  static FactorGraph<JacobianFactorUnordered> convertToJacobians(const FactorGraph<
      GaussianFactorUnordered>& factors) {

    typedef JacobianFactorUnordered J;
    typedef HessianFactorUnordered H;

    const bool debug = ISDEBUG("convertToJacobians");

    FactorGraph<J> jacobians;
    jacobians.reserve(factors.size());
    BOOST_FOREACH(const GaussianFactorUnordered::shared_ptr& factor, factors)
    if (factor) {
      J::shared_ptr jacobian(boost::dynamic_pointer_cast<J>(factor));
      if (jacobian) {
        jacobians.push_back(jacobian);
        if (debug) jacobian->print("Existing JacobianFactor: ");
      } else {
        H::shared_ptr hessian(boost::dynamic_pointer_cast<H>(factor));
        if (!hessian) throw std::invalid_argument(
            "convertToJacobians: factor is neither a JacobianFactor nor a HessianFactor.");
        J::shared_ptr converted(new J(*hessian));
        if (debug) {
          cout << "Converted HessianFactor to JacobianFactor:\n";
          hessian->print("HessianFactor: ");
          converted->print("JacobianFactor: ");
          if (!assert_equal(*hessian, HessianFactorUnordered(*converted), 1e-3)) throw runtime_error(
              "convertToJacobians: Conversion between Jacobian and Hessian incorrect");
        }
        jacobians.push_back(converted);
      }
    }
    return jacobians;
  }

  /* ************************************************************************* */
  GaussianFactorGraphUnordered::EliminationResult EliminateQR(
    const std::vector<GaussianFactorUnordered::shared_ptr>& factors, const std::vector<Key>& keys)
  {

    gttic(Combine);
    JacobianFactorUnordered jointFactor(factors);
    gttoc(Combine);
    gttic(eliminate);
    GaussianConditional::shared_ptr gbn = jointFactor->eliminate(nrFrontals);
    gttoc(eliminate);
    return make_pair(gbn, jointFactor);


    const bool debug = ISDEBUG("EliminateQR");

    // Convert all factors to the appropriate type and call the type-specific EliminateGaussian.
    if (debug) cout << "Using QR" << endl;

    gttic(convert_to_Jacobian);
    FactorGraph<JacobianFactorUnordered> jacobians = convertToJacobians(factors);
    gttoc(convert_to_Jacobian);

    gttic(Jacobian_EliminateGaussian);
    GaussianConditional::shared_ptr conditional;
    GaussianFactorUnordered::shared_ptr factor;
    boost::tie(conditional, factor) = EliminateJacobians(jacobians, nrFrontals);
    gttoc(Jacobian_EliminateGaussian);

    return make_pair(conditional, factor);
  } // \EliminateQR

  /* ************************************************************************* */
  bool hasConstraints(const FactorGraph<GaussianFactorUnordered>& factors) {
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
  GaussianFactorGraphUnordered::EliminationResult EliminatePreferCholesky(
      const FactorGraph<GaussianFactorUnordered>& factors, size_t nrFrontals) {

    // If any JacobianFactors have constrained noise models, we have to convert
    // all factors to JacobianFactors.  Otherwise, we can convert all factors
    // to HessianFactors.  This is because QR can handle constrained noise
    // models but Cholesky cannot.
    if (hasConstraints(factors))
      return EliminateQR(factors, nrFrontals);
    else {
      GaussianFactorGraphUnordered::EliminationResult ret;
      gttic(EliminateCholesky);
      ret = EliminateCholesky(factors, nrFrontals);
      gttoc(EliminateCholesky);
      return ret;
    }

  } // \EliminatePreferCholesky



  /* ************************************************************************* */
  static JacobianFactorUnordered::shared_ptr convertToJacobianFactorPtr(const GaussianFactorUnordered::shared_ptr &gf) {
    JacobianFactorUnordered::shared_ptr result = boost::dynamic_pointer_cast<JacobianFactorUnordered>(gf);
    if( !result ) {
      result = boost::make_shared<JacobianFactorUnordered>(*gf); // Convert any non-Jacobian factors to Jacobians (e.g. Hessian -> Jacobian with Cholesky)
    }
    return result;
  }

  /* ************************************************************************* */
  Errors operator*(const GaussianFactorGraphUnordered& fg, const VectorValues& x) {
    Errors e;
    BOOST_FOREACH(const GaussianFactorUnordered::shared_ptr& Ai_G, fg) {
      JacobianFactorUnordered::shared_ptr Ai = convertToJacobianFactorPtr(Ai_G);
      e.push_back((*Ai)*x);
    }
    return e;
  }

  /* ************************************************************************* */
  void multiplyInPlace(const GaussianFactorGraphUnordered& fg, const VectorValues& x, Errors& e) {
    multiplyInPlace(fg,x,e.begin());
  }

  /* ************************************************************************* */
  void multiplyInPlace(const GaussianFactorGraphUnordered& fg, const VectorValues& x, const Errors::iterator& e) {
    Errors::iterator ei = e;
    BOOST_FOREACH(const GaussianFactorUnordered::shared_ptr& Ai_G, fg) {
      JacobianFactorUnordered::shared_ptr Ai = convertToJacobianFactorPtr(Ai_G);
      *ei = (*Ai)*x;
      ei++;
    }
  }

  /* ************************************************************************* */
  // x += alpha*A'*e
  void transposeMultiplyAdd(const GaussianFactorGraphUnordered& fg, double alpha, const Errors& e, VectorValues& x) {
    // For each factor add the gradient contribution
    Errors::const_iterator ei = e.begin();
    BOOST_FOREACH(const GaussianFactorUnordered::shared_ptr& Ai_G, fg) {
      JacobianFactorUnordered::shared_ptr Ai = convertToJacobianFactorPtr(Ai_G);
      Ai->transposeMultiplyAdd(alpha,*(ei++),x);
    }
  }

  /* ************************************************************************* */
  VectorValues gradient(const GaussianFactorGraphUnordered& fg, const VectorValues& x0) {
    // It is crucial for performance to make a zero-valued clone of x
    VectorValues g = VectorValues::Zero(x0);
    Errors e;
    BOOST_FOREACH(const GaussianFactorUnordered::shared_ptr& Ai_G, fg) {
      JacobianFactorUnordered::shared_ptr Ai = convertToJacobianFactorPtr(Ai_G);
      e.push_back(Ai->error_vector(x0));
    }
    transposeMultiplyAdd(fg, 1.0, e, g);
    return g;
  }

  /* ************************************************************************* */
  void gradientAtZero(const GaussianFactorGraphUnordered& fg, VectorValues& g) {
    // Zero-out the gradient
    g.setZero();
    Errors e;
    BOOST_FOREACH(const GaussianFactorUnordered::shared_ptr& Ai_G, fg) {
      JacobianFactorUnordered::shared_ptr Ai = convertToJacobianFactorPtr(Ai_G);
      e.push_back(-Ai->getb());
    }
    transposeMultiplyAdd(fg, 1.0, e, g);
  }

  /* ************************************************************************* */
  void residual(const GaussianFactorGraphUnordered& fg, const VectorValues &x, VectorValues &r) {
    Index i = 0 ;
    BOOST_FOREACH(const GaussianFactorUnordered::shared_ptr& Ai_G, fg) {
      JacobianFactorUnordered::shared_ptr Ai = convertToJacobianFactorPtr(Ai_G);
      r[i] = Ai->getb();
      i++;
    }
    VectorValues Ax = VectorValues::SameStructure(r);
    multiply(fg,x,Ax);
    axpy(-1.0,Ax,r);
  }

  /* ************************************************************************* */
  void multiply(const GaussianFactorGraphUnordered& fg, const VectorValues &x, VectorValues &r) {
    r.setZero();
    Index i = 0;
    BOOST_FOREACH(const GaussianFactorUnordered::shared_ptr& Ai_G, fg) {
      JacobianFactorUnordered::shared_ptr Ai = convertToJacobianFactorPtr(Ai_G);
      Vector &y = r[i];
      for(JacobianFactorUnordered::const_iterator j = Ai->begin(); j != Ai->end(); ++j) {
        y += Ai->getA(j) * x[*j];
      }
      ++i;
    }
  }

  /* ************************************************************************* */
  void transposeMultiply(const GaussianFactorGraphUnordered& fg, const VectorValues &r, VectorValues &x) {
    x.setZero();
    Index i = 0;
    BOOST_FOREACH(const GaussianFactorUnordered::shared_ptr& Ai_G, fg) {
      JacobianFactorUnordered::shared_ptr Ai = convertToJacobianFactorPtr(Ai_G);
      for(JacobianFactorUnordered::const_iterator j = Ai->begin(); j != Ai->end(); ++j) {
        x[*j] += Ai->getA(j).transpose() * r[i];
      }
      ++i;
    }
  }

  /* ************************************************************************* */
  boost::shared_ptr<Errors> gaussianErrors_(const GaussianFactorGraphUnordered& fg, const VectorValues& x) {
    boost::shared_ptr<Errors> e(new Errors);
    BOOST_FOREACH(const GaussianFactorUnordered::shared_ptr& Ai_G, fg) {
      JacobianFactorUnordered::shared_ptr Ai = convertToJacobianFactorPtr(Ai_G);
      e->push_back(Ai->error_vector(x));
    }
    return e;
  }

} // namespace gtsam
