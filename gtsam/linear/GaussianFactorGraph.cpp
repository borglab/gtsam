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
#include <gtsam/linear/HessianFactor.h>
#include <gtsam/inference/FactorGraph-inst.h>
#include <gtsam/inference/EliminateableFactorGraph-inst.h>
#include <gtsam/base/debug.h>
#include <gtsam/base/timing.h>
#include <gtsam/base/cholesky.h>

using namespace std;
using namespace gtsam;

namespace gtsam {

  // Instantiate base classes
  template class FactorGraph<GaussianFactor>;
  template class EliminateableFactorGraph<GaussianFactorGraph>;

  /* ************************************************************************* */
  bool GaussianFactorGraph::equals(const This& fg, double tol) const
  {
    return Base::equals(fg, tol);
  }

  /* ************************************************************************* */
  GaussianFactorGraph::Keys GaussianFactorGraph::keys() const {
    KeySet keys;
    BOOST_FOREACH(const sharedFactor& factor, *this)
    if (factor)
      keys.insert(factor->begin(), factor->end());
    return keys;
  }

  /* ************************************************************************* */
  std::map<Key, size_t> GaussianFactorGraph::getKeyDimMap() const {
    map<Key, size_t> spec;
    BOOST_FOREACH ( const GaussianFactor::shared_ptr &gf, *this ) {
      for ( GaussianFactor::const_iterator it = gf->begin() ; it != gf->end() ; it++ ) {
        map<Key,size_t>::iterator it2 = spec.find(*it);
        if ( it2 == spec.end() ) {
          spec.insert(make_pair(*it, gf->getDim(it)));
        }
      }
    }
    return spec;
  }

  /* ************************************************************************* */
    vector<size_t> GaussianFactorGraph::getkeydim() const {
      // First find dimensions of each variable
      vector<size_t> dims;
      BOOST_FOREACH(const sharedFactor& factor, *this) {
        for (GaussianFactor::const_iterator pos = factor->begin();
            pos != factor->end(); ++pos) {
          if (dims.size() <= *pos)
            dims.resize(*pos + 1, 0);
          dims[*pos] = factor->getDim(pos);
        }
      }
      // Find accumulated dimensions for variables
      vector<size_t> dims_accumulated;
      dims_accumulated.resize(dims.size()+1,0);
      dims_accumulated[0]=0;
      for (size_t i=1; i<dims_accumulated.size(); i++)
        dims_accumulated[i] = dims_accumulated[i-1]+dims[i-1];
      return dims_accumulated;
    }

  /* ************************************************************************* */
  GaussianFactorGraph::shared_ptr GaussianFactorGraph::cloneToPtr() const {
    gtsam::GaussianFactorGraph::shared_ptr result(new GaussianFactorGraph());
    *result = *this;
    return result;
  }

  /* ************************************************************************* */
  GaussianFactorGraph GaussianFactorGraph::clone() const {
    GaussianFactorGraph result;
    BOOST_FOREACH(const sharedFactor& f, *this) {
      if (f)
        result.push_back(f->clone());
      else
        result.push_back(sharedFactor()); // Passes on null factors so indices remain valid
    }
    return result;
  }

  /* ************************************************************************* */
  GaussianFactorGraph GaussianFactorGraph::negate() const {
    GaussianFactorGraph result;
    BOOST_FOREACH(const sharedFactor& f, *this) {
      if (f)
        result.push_back(f->negate());
      else
        result.push_back(sharedFactor()); // Passes on null factors so indices remain valid
    }
    return result;
  }

  /* ************************************************************************* */
  vector<boost::tuple<size_t, size_t, double> > GaussianFactorGraph::sparseJacobian() const {
    // First find dimensions of each variable
    typedef std::map<Key, size_t> KeySizeMap;
    KeySizeMap dims;
    BOOST_FOREACH(const sharedFactor& factor, *this) {
      if (!static_cast<bool>(factor)) continue;

      for (GaussianFactor::const_iterator key = factor->begin();
          key != factor->end(); ++key) {
        dims[*key] = factor->getDim(key);
      }
    }

    // Compute first scalar column of each variable
    size_t currentColIndex = 0;
    KeySizeMap columnIndices = dims;
    BOOST_FOREACH(const KeySizeMap::value_type& col, dims) {
      columnIndices[col.first] = currentColIndex;
      currentColIndex += dims[col.first];
    }

    // Iterate over all factors, adding sparse scalar entries
    typedef boost::tuple<size_t, size_t, double> triplet;
    vector<triplet> entries;
    size_t row = 0;
    BOOST_FOREACH(const sharedFactor& factor, *this) {
      if (!static_cast<bool>(factor)) continue;

      // Convert to JacobianFactor if necessary
      JacobianFactor::shared_ptr jacobianFactor(
          boost::dynamic_pointer_cast<JacobianFactor>(factor));
      if (!jacobianFactor) {
        HessianFactor::shared_ptr hessian(
            boost::dynamic_pointer_cast<HessianFactor>(factor));
        if (hessian)
          jacobianFactor.reset(new JacobianFactor(*hessian));
        else
          throw invalid_argument(
              "GaussianFactorGraph contains a factor that is neither a JacobianFactor nor a HessianFactor.");
      }

      // Whiten the factor and add entries for it
      // iterate over all variables in the factor
      const JacobianFactor whitened(jacobianFactor->whiten());
      for (JacobianFactor::const_iterator key = whitened.begin();
          key < whitened.end(); ++key) {
        JacobianFactor::constABlock whitenedA = whitened.getA(key);
        // find first column index for this key
        size_t column_start = columnIndices[*key];
        for (size_t i = 0; i < (size_t) whitenedA.rows(); i++)
          for (size_t j = 0; j < (size_t) whitenedA.cols(); j++) {
            double s = whitenedA(i, j);
            if (std::abs(s) > 1e-12)
              entries.push_back(boost::make_tuple(row + i, column_start + j, s));
          }
      }

      JacobianFactor::constBVector whitenedb(whitened.getb());
      size_t bcolumn = currentColIndex;
      for (size_t i = 0; i < (size_t) whitenedb.size(); i++)
        entries.push_back(boost::make_tuple(row + i, bcolumn, whitenedb(i)));

      // Increment row index
      row += jacobianFactor->rows();
    }
    return vector<triplet>(entries.begin(), entries.end());
  }

  /* ************************************************************************* */
  Matrix GaussianFactorGraph::sparseJacobian_() const {

    // call sparseJacobian
    typedef boost::tuple<size_t, size_t, double> triplet;
    vector<triplet> result = sparseJacobian();

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
  Matrix GaussianFactorGraph::augmentedJacobian(
      boost::optional<const Ordering&> optionalOrdering) const {
    // combine all factors
    JacobianFactor combined(*this, optionalOrdering);
    return combined.augmentedJacobian();
  }

  /* ************************************************************************* */
  pair<Matrix, Vector> GaussianFactorGraph::jacobian(
      boost::optional<const Ordering&> optionalOrdering) const {
    Matrix augmented = augmentedJacobian(optionalOrdering);
    return make_pair(augmented.leftCols(augmented.cols() - 1),
        augmented.col(augmented.cols() - 1));
  }

  /* ************************************************************************* */
  Matrix GaussianFactorGraph::augmentedHessian(
      boost::optional<const Ordering&> optionalOrdering) const {
    // combine all factors and get upper-triangular part of Hessian
    HessianFactor combined(*this, Scatter(*this, optionalOrdering));
    Matrix result = combined.info();
    // Fill in lower-triangular part of Hessian
    result.triangularView<Eigen::StrictlyLower>() = result.transpose();
    return result;
  }

  /* ************************************************************************* */
  pair<Matrix, Vector> GaussianFactorGraph::hessian(
      boost::optional<const Ordering&> optionalOrdering) const {
    Matrix augmented = augmentedHessian(optionalOrdering);
    return make_pair(
        augmented.topLeftCorner(augmented.rows() - 1, augmented.rows() - 1),
        augmented.col(augmented.rows() - 1).head(augmented.rows() - 1));
  }

  /* ************************************************************************* */
  VectorValues GaussianFactorGraph::hessianDiagonal() const {
    VectorValues d;
    BOOST_FOREACH(const sharedFactor& factor, *this) {
      if(factor){
        VectorValues di = factor->hessianDiagonal();
        d.addInPlace_(di);
      }
    }
    return d;
  }

  /* ************************************************************************* */
  map<Key,Matrix> GaussianFactorGraph::hessianBlockDiagonal() const {
    map<Key,Matrix> blocks;
    BOOST_FOREACH(const sharedFactor& factor, *this) {
      if (!factor) continue;
      map<Key,Matrix> BD = factor->hessianBlockDiagonal();
      map<Key,Matrix>::const_iterator it = BD.begin();
      for(;it!=BD.end();it++) {
        Key j = it->first; // variable key for this block
        const Matrix& Bj = it->second;
        if (blocks.count(j))
          blocks[j] += Bj;
        else
          blocks.insert(make_pair(j,Bj));
      }
    }
    return blocks;
  }

  /* ************************************************************************* */
  VectorValues GaussianFactorGraph::optimize(OptionalOrdering ordering, const Eliminate& function) const
  {
    gttic(GaussianFactorGraph_optimize);
    return BaseEliminateable::eliminateMultifrontal(ordering, function)->optimize();
  }

  /* ************************************************************************* */
  namespace {
    JacobianFactor::shared_ptr convertToJacobianFactorPtr(const GaussianFactor::shared_ptr &gf) {
      JacobianFactor::shared_ptr result = boost::dynamic_pointer_cast<JacobianFactor>(gf);
      if( !result )
        // Convert any non-Jacobian factors to Jacobians (e.g. Hessian -> Jacobian with Cholesky)
        result = boost::make_shared<JacobianFactor>(*gf);
      return result;
    }
  }

  /* ************************************************************************* */
  VectorValues GaussianFactorGraph::gradient(const VectorValues& x0) const
  {
    VectorValues g = VectorValues::Zero(x0);
    BOOST_FOREACH(const sharedFactor& Ai_G, *this) {
      JacobianFactor::shared_ptr Ai = convertToJacobianFactorPtr(Ai_G);
      Vector e = Ai->error_vector(x0);
      Ai->transposeMultiplyAdd(1.0, e, g);
    }
    return g;
  }

  /* ************************************************************************* */
  VectorValues GaussianFactorGraph::gradientAtZero() const {
    // Zero-out the gradient
    VectorValues g;
    BOOST_FOREACH(const sharedFactor& factor, *this) {
      if (!factor) continue;
      VectorValues gi = factor->gradientAtZero();
      g.addInPlace_(gi);
    }
    return g;
  }

  /* ************************************************************************* */
  VectorValues GaussianFactorGraph::optimizeGradientSearch() const
  {
    gttic(GaussianFactorGraph_optimizeGradientSearch);

    gttic(Compute_Gradient);
    // Compute gradient (call gradientAtZero function, which is defined for various linear systems)
    VectorValues grad = gradientAtZero();
    double gradientSqNorm = grad.dot(grad);
    gttoc(Compute_Gradient);

    gttic(Compute_Rg);
    // Compute R * g
    Errors Rg = *this * grad;
    gttoc(Compute_Rg);

    gttic(Compute_minimizing_step_size);
    // Compute minimizing step size
    double step = -gradientSqNorm / dot(Rg, Rg);
    gttoc(Compute_minimizing_step_size);

    gttic(Compute_point);
    // Compute steepest descent point
    grad *= step;
    gttoc(Compute_point);

    return grad;
  }

  /* ************************************************************************* */
  Errors GaussianFactorGraph::operator*(const VectorValues& x) const {
    Errors e;
    BOOST_FOREACH(const GaussianFactor::shared_ptr& Ai_G, *this) {
      JacobianFactor::shared_ptr Ai = convertToJacobianFactorPtr(Ai_G);
      e.push_back((*Ai) * x);
    }
    return e;
  }

  /* ************************************************************************* */
  void GaussianFactorGraph::multiplyHessianAdd(double alpha,
      const VectorValues& x, VectorValues& y) const {
    BOOST_FOREACH(const GaussianFactor::shared_ptr& f, *this)
     f->multiplyHessianAdd(alpha, x, y);
  }

  /* ************************************************************************* */
  void GaussianFactorGraph::multiplyInPlace(const VectorValues& x, Errors& e) const {
    multiplyInPlace(x, e.begin());
  }

  /* ************************************************************************* */
  void GaussianFactorGraph::multiplyInPlace(const VectorValues& x, const Errors::iterator& e) const {
    Errors::iterator ei = e;
    BOOST_FOREACH(const GaussianFactor::shared_ptr& Ai_G, *this) {
      JacobianFactor::shared_ptr Ai = convertToJacobianFactorPtr(Ai_G);
      *ei = (*Ai)*x;
      ei++;
    }
  }

  /* ************************************************************************* */
  bool hasConstraints(const GaussianFactorGraph& factors) {
    typedef JacobianFactor J;
    BOOST_FOREACH(const GaussianFactor::shared_ptr& factor, factors) {
      J::shared_ptr jacobian(boost::dynamic_pointer_cast<J>(factor));
      if (jacobian && jacobian->get_model() && jacobian->get_model()->isConstrained()) {
        return true;
      }
    }
    return false;
  }

  /* ************************************************************************* */
  // x += alpha*A'*e
  void GaussianFactorGraph::transposeMultiplyAdd(double alpha, const Errors& e,
                                                 VectorValues& x) const {
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
