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

#include <algorithm>
#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/linear/GaussianBayesTree.h>
#include <gtsam/linear/GaussianEliminationTree.h>
#include <gtsam/linear/GaussianJunctionTree.h>
#include <gtsam/linear/HessianFactor.h>
#include <gtsam/inference/FactorGraph-inst.h>
#include <gtsam/inference/EliminateableFactorGraph-inst.h>
#include <gtsam/base/debug.h>
#include <gtsam/base/timing.h>
#include <gtsam/base/cholesky.h>
#include <iterator>

#include <Eigen/Sparse>

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
    for (const sharedFactor& factor: *this)
    if (factor)
      keys.insert(factor->begin(), factor->end());
    return keys;
  }

  /* ************************************************************************* */
  std::map<Key, size_t> GaussianFactorGraph::getKeyDimMap() const {
    map<Key, size_t> spec;
    for (const GaussianFactor::shared_ptr& gf : *this) {
      for (GaussianFactor::const_iterator it = gf->begin(); it != gf->end(); it++) {
        map<Key,size_t>::iterator it2 = spec.find(*it);
        if ( it2 == spec.end() ) {
          spec.emplace(*it, gf->getDim(it));
        }
      }
    }
    return spec;
  }

  /* ************************************************************************* */
  double GaussianFactorGraph::error(const VectorValues& x) const {
    double total_error = 0.;
    for(const sharedFactor& factor: *this){
      if(factor)
        total_error += factor->error(x);
    }
    return total_error;
  }

  /* ************************************************************************* */
  double GaussianFactorGraph::probPrime(const VectorValues& c) const {
    // NOTE the 0.5 constant is handled by the factor error.
    return exp(-error(c));
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
    for (const sharedFactor& f : *this) {
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
    for (const sharedFactor& factor: *this) {
      if (factor)
        result.push_back(factor->negate());
      else
        result.push_back(sharedFactor()); // Passes on null factors so indices remain valid
    }
    return result;
  }

  /* ************************************************************************* */
  using SparseTriplets = std::vector<std::tuple<int, int, double> >;
  SparseTriplets GaussianFactorGraph::sparseJacobian(const Ordering& ordering,
                                                     size_t& nrows,
                                                     size_t& ncols) const {
    gttic_(GaussianFactorGraph_sparseJacobian);
    // First find dimensions of each variable
    typedef std::map<Key, size_t> KeySizeMap;
    KeySizeMap dims;
    for (const auto& factor : *this) {
      if (!static_cast<bool>(factor)) continue;

      for (auto it = factor->begin(); it != factor->end(); ++it) {
        dims[*it] = factor->getDim(it);
      }
    }

    // Compute first scalar column of each variable
    ncols = 0;
    KeySizeMap columnIndices = dims;
    for (const auto key : ordering) {
      columnIndices[key] = ncols;
      ncols += dims[key];
    }

    // Iterate over all factors, adding sparse scalar entries
    SparseTriplets entries;
    entries.reserve(30 * size());
    nrows = 0;
    for (const auto& factor : *this) {
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
          throw std::invalid_argument(
              "GaussianFactorGraph contains a factor that is neither a "
              "JacobianFactor nor a HessianFactor.");
      }

      // Whiten the factor and add entries for it
      // iterate over all variables in the factor
      const JacobianFactor whitened(jacobianFactor->whiten());
      for (auto key = whitened.begin(); key < whitened.end(); ++key) {
        JacobianFactor::constABlock whitenedA = whitened.getA(key);
        // find first column index for this key
        size_t column_start = columnIndices[*key];
        for (size_t i = 0; i < (size_t)whitenedA.rows(); i++)
          for (size_t j = 0; j < (size_t)whitenedA.cols(); j++) {
            double s = whitenedA(i, j);
            if (std::abs(s) > 1e-12)
              entries.emplace_back(nrows + i, column_start + j, s);
          }
      }

      JacobianFactor::constBVector whitenedb(whitened.getb());
      for (size_t i = 0; i < (size_t)whitenedb.size(); i++) {
        double s = whitenedb(i);
        if (std::abs(s) > 1e-12) entries.emplace_back(nrows + i, ncols, s);
      }

      // Increment row index
      nrows += jacobianFactor->rows();
    }

    ncols++;  // +1 for b-column
    return entries;
  }

  /* ************************************************************************* */
  SparseTriplets GaussianFactorGraph::sparseJacobian() const {
    size_t nrows, ncols;
    return sparseJacobian(Ordering(this->keys()), nrows, ncols);
  }

  template <>
  std::tuple<size_t, size_t, SparseMatrixEigen>
  GaussianFactorGraph::sparseJacobian<SparseMatrixEigen>(
      const Ordering& ordering) const {
    gttic_(GaussianFactorGraph_sparseJacobian);
    gttic_(obtainSparseJacobian);
    size_t rows, cols;
    SparseTriplets entries_ = sparseJacobian(ordering, rows, cols);
    std::vector<Eigen::Triplet<double>> entries;
    for (auto& v : entries_) {
      entries.push_back(Eigen::Triplet<double>(std::get<0>(v), std::get<1>(v), std::get<2>(v)));
    }
    gttoc_(obtainSparseJacobian);
    gttic_(convertSparseJacobian);
    SparseMatrixEigen Ab(rows, cols);
    Ab.setFromTriplets(entries.begin(), entries.end());
    // Ab.reserve(entries.size());
    // for (auto entry : entries) {
    //   Ab.insert(std::get<0>(entry), std::get<1>(entry)) = std::get<2>(entry);
    // }
    Ab.makeCompressed();
    // TODO(gerry): benchmark to see if setFromTriplets is faster
    // Ab.setFromTriplets(entries.begin(), entries.end());
    return std::make_tuple(rows, cols, Ab);
  }
  // Eigen Matrix in "matlab" format (template specialized)
  template <>
  std::tuple<size_t, size_t, Matrix>
  GaussianFactorGraph::sparseJacobian<Matrix>(const Ordering& ordering) const {
    gttic_(GaussianFactorGraph_sparseJacobian);
    // call sparseJacobian
    size_t rows, cols;
    SparseTriplets result =
        sparseJacobian(ordering, rows, cols);

    // translate to base 1 matrix
    size_t nzmax = result.size();
    Matrix IJS(3, nzmax);
    for (size_t k = 0; k < result.size(); k++) {
      const auto& entry = result[k];
      IJS(0, k) = double(std::get<0>(entry) + 1);
      IJS(1, k) = double(std::get<1>(entry) + 1);
      IJS(2, k) = std::get<2>(entry);
    }
    return std::make_tuple(rows, cols, IJS);
  }

  /* ************************************************************************* */
  Matrix GaussianFactorGraph::sparseJacobian_() const {
    gttic_(GaussianFactorGraph_sparseJacobian_matrix);
    // call sparseJacobian
    auto result = sparseJacobian();

    // translate to base 1 matrix
    size_t nzmax = result.size();
    Matrix IJS(3, nzmax);
    for (size_t k = 0; k < result.size(); k++) {
      const auto& entry = result[k];
      IJS(0, k) = double(std::get<0>(entry) + 1);
      IJS(1, k) = double(std::get<1>(entry) + 1);
      IJS(2, k) = std::get<2>(entry);
    }
    return IJS;
  }

  /* ************************************************************************* */
  Matrix GaussianFactorGraph::augmentedJacobian(
      const Ordering& ordering) const {
    // combine all factors
    JacobianFactor combined(*this, ordering);
    return combined.augmentedJacobian();
  }

  /* ************************************************************************* */
  Matrix GaussianFactorGraph::augmentedJacobian() const {
    // combine all factors
    JacobianFactor combined(*this);
    return combined.augmentedJacobian();
  }

  /* ************************************************************************* */
  pair<Matrix, Vector> GaussianFactorGraph::jacobian(
      const Ordering& ordering) const {
    Matrix augmented = augmentedJacobian(ordering);
    return make_pair(augmented.leftCols(augmented.cols() - 1),
        augmented.col(augmented.cols() - 1));
  }

  /* ************************************************************************* */
  pair<Matrix, Vector> GaussianFactorGraph::jacobian() const {
    Matrix augmented = augmentedJacobian();
    return make_pair(augmented.leftCols(augmented.cols() - 1),
        augmented.col(augmented.cols() - 1));
  }

  /* ************************************************************************* */
  Matrix GaussianFactorGraph::augmentedHessian(
      const Ordering& ordering) const {
    // combine all factors and get upper-triangular part of Hessian
    Scatter scatter(*this, ordering);
    HessianFactor combined(*this, scatter);
    return combined.info().selfadjointView();;
  }

  /* ************************************************************************* */
  Matrix GaussianFactorGraph::augmentedHessian() const {
    // combine all factors and get upper-triangular part of Hessian
    Scatter scatter(*this);
    HessianFactor combined(*this, scatter);
    return combined.info().selfadjointView();;
  }

  /* ************************************************************************* */
  pair<Matrix, Vector> GaussianFactorGraph::hessian(
      const Ordering& ordering) const {
    Matrix augmented = augmentedHessian(ordering);
    size_t n = augmented.rows() - 1;
    return make_pair(augmented.topLeftCorner(n, n), augmented.topRightCorner(n, 1));
  }

  /* ************************************************************************* */
  pair<Matrix, Vector> GaussianFactorGraph::hessian() const {
    Matrix augmented = augmentedHessian();
    size_t n = augmented.rows() - 1;
    return make_pair(augmented.topLeftCorner(n, n), augmented.topRightCorner(n, 1));
  }

  /* ************************************************************************* */
  VectorValues GaussianFactorGraph::hessianDiagonal() const {
    VectorValues d;
    for (const sharedFactor& factor : *this) {
      if(factor){
        factor->hessianDiagonalAdd(d);
      }
    }
    return d;
  }

  /* ************************************************************************* */
  map<Key,Matrix> GaussianFactorGraph::hessianBlockDiagonal() const {
    map<Key,Matrix> blocks;
    for (const sharedFactor& factor : *this) {
      if (!factor) continue;
      map<Key,Matrix> BD = factor->hessianBlockDiagonal();
      map<Key,Matrix>::const_iterator it = BD.begin();
      for (;it!=BD.end();++it) {
        Key j = it->first; // variable key for this block
        const Matrix& Bj = it->second;
        if (blocks.count(j))
          blocks[j] += Bj;
        else
          blocks.emplace(j,Bj);
      }
    }
    return blocks;
  }

  /* ************************************************************************ */
  VectorValues GaussianFactorGraph::optimize(const Eliminate& function) const {
    gttic(GaussianFactorGraph_optimize);
    return BaseEliminateable::eliminateMultifrontal(Ordering::COLAMD, function)
        ->optimize();
  }

  /* ************************************************************************* */
  VectorValues GaussianFactorGraph::optimize(const Ordering& ordering, const Eliminate& function) const {
    gttic(GaussianFactorGraph_optimize);
    return BaseEliminateable::eliminateMultifrontal(ordering, function)->optimize();
  }

  /* ************************************************************************* */
  // TODO(frank): can we cache memory across invocations
  VectorValues GaussianFactorGraph::optimizeDensely() const {
    gttic(GaussianFactorGraph_optimizeDensely);

    // Combine all factors in a single HessianFactor (as done in augmentedHessian)
    Scatter scatter(*this);
    HessianFactor combined(*this, scatter);

    // TODO(frank): cast to large dynamic matrix :-(
    // NOTE(frank): info only valid (I think) in upper triangle. No problem for LLT...
    Matrix augmented = combined.info().selfadjointView();

    // Do Cholesky Factorization
    size_t n = augmented.rows() - 1;
    auto AtA = augmented.topLeftCorner(n, n);
    auto eta = augmented.topRightCorner(n, 1);
    Eigen::LLT<Matrix, Eigen::Upper> llt(AtA);

    // Solve and convert, re-using scatter data structure
    Vector solution = llt.solve(eta);
    return VectorValues(solution, scatter);
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
    for (const sharedFactor& factor: *this) {
      JacobianFactor::shared_ptr Ai = convertToJacobianFactorPtr(factor);
      Vector e = Ai->error_vector(x0);
      Ai->transposeMultiplyAdd(1.0, e, g);
    }
    return g;
  }

  /* ************************************************************************* */
  VectorValues GaussianFactorGraph::gradientAtZero() const {
    // Zero-out the gradient
    VectorValues g;
    for (const sharedFactor& factor: *this) {
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
    double step = -gradientSqNorm / gtsam::dot(Rg, Rg);
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
    for (const GaussianFactor::shared_ptr& factor: *this) {
      JacobianFactor::shared_ptr Ai = convertToJacobianFactorPtr(factor);
      e.push_back((*Ai) * x);
    }
    return e;
  }

  /* ************************************************************************* */
  void GaussianFactorGraph::multiplyHessianAdd(double alpha,
      const VectorValues& x, VectorValues& y) const {
    for (const GaussianFactor::shared_ptr& f: *this)
     f->multiplyHessianAdd(alpha, x, y);
  }

  /* ************************************************************************* */
  void GaussianFactorGraph::multiplyInPlace(const VectorValues& x, Errors& e) const {
    multiplyInPlace(x, e.begin());
  }

  /* ************************************************************************* */
  void GaussianFactorGraph::multiplyInPlace(const VectorValues& x, const Errors::iterator& e) const {
    Errors::iterator ei = e;
    for (const GaussianFactor::shared_ptr& factor: *this) {
      JacobianFactor::shared_ptr Ai = convertToJacobianFactorPtr(factor);
      *ei = (*Ai)*x;
      ei++;
    }
  }

  /* ************************************************************************* */
  bool hasConstraints(const GaussianFactorGraph& factors) {
    typedef JacobianFactor J;
    for (const GaussianFactor::shared_ptr& factor: factors) {
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
    for (const sharedFactor& factor: *this) {
      JacobianFactor::shared_ptr Ai = convertToJacobianFactorPtr(factor);
      Ai->transposeMultiplyAdd(alpha, *(ei++), x);
    }
  }

  ///* ************************************************************************* */
  //void residual(const GaussianFactorGraph& fg, const VectorValues &x, VectorValues &r) {
  //  Key i = 0 ;
  //  for (const GaussianFactor::shared_ptr& factor, fg) {
  //    JacobianFactor::shared_ptr Ai = convertToJacobianFactorPtr(factor);
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
  //  for (const GaussianFactor::shared_ptr& factor, fg) {
  //    JacobianFactor::shared_ptr Ai = convertToJacobianFactorPtr(factor);
  //    Vector &y = r[i];
  //    for (JacobianFactor::const_iterator j = Ai->begin(); j != Ai->end(); ++j) {
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
    for (const sharedFactor& factor: *this) {
      JacobianFactor::shared_ptr Ai = convertToJacobianFactorPtr(factor);
      for (JacobianFactor::const_iterator j = Ai->begin(); j != Ai->end(); ++j) {
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
    for (const sharedFactor& factor: *this) {
      JacobianFactor::shared_ptr Ai = convertToJacobianFactorPtr(factor);
      e.push_back(Ai->error_vector(x));
    }
    return e;
  }

  /* ************************************************************************* */
  void GaussianFactorGraph::printErrors(
      const VectorValues& values, const std::string& str,
      const KeyFormatter& keyFormatter,
      const std::function<bool(const Factor* /*factor*/,
                               double /*whitenedError*/, size_t /*index*/)>&
          printCondition) const {
    cout << str << "size: " << size() << endl << endl;
    for (size_t i = 0; i < (*this).size(); i++) {
      const sharedFactor& factor = (*this)[i];
      const double errorValue =
          (factor != nullptr ? (*this)[i]->error(values) : .0);
      if (!printCondition(factor.get(), errorValue, i))
        continue;  // User-provided filter did not pass

      stringstream ss;
      ss << "Factor " << i << ": ";
      if (factor == nullptr) {
        cout << "nullptr"
             << "\n";
      } else {
        factor->print(ss.str(), keyFormatter);
        cout << "error = " << errorValue << "\n";
      }
      cout << endl;  // only one "endl" at end might be faster, \n for each
                     // factor
    }
  }

} // namespace gtsam
