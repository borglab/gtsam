/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    HessianFactor.cpp
 * @brief   
 * @author  Richard Roberts
 * @created Dec 8, 2010
 */

#include <gtsam/base/timing.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/FastMap.h>
#include <gtsam/base/cholesky.h>
#include <gtsam/linear/GaussianConditional.h>
#include <gtsam/linear/HessianFactor.h>
#include <gtsam/linear/JacobianFactor.h>
#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/linear/GaussianBayesNet.h>

#include <boost/foreach.hpp>
#include <boost/format.hpp>
#include <boost/make_shared.hpp>
#include <boost/lambda/bind.hpp>
#include <boost/lambda/lambda.hpp>
#include <boost/tuple/tuple.hpp>

#include <boost/numeric/ublas/triangular.hpp>
#include <boost/numeric/ublas/symmetric.hpp>
#include <boost/numeric/ublas/io.hpp>
#include <boost/numeric/ublas/matrix_proxy.hpp>
#include <boost/numeric/ublas/vector_proxy.hpp>
#include <boost/numeric/ublas/blas.hpp>

#include <sstream>

using namespace std;

namespace ublas = boost::numeric::ublas;
using namespace boost::lambda;

namespace gtsam {

  /* ************************************************************************* */
  HessianFactor::HessianFactor(const HessianFactor& gf) :
    GaussianFactor(gf), info_(matrix_) {
    info_.assignNoalias(gf.info_);
  }

  /* ************************************************************************* */
  HessianFactor::HessianFactor() : info_(matrix_) {}

  /* ************************************************************************* */
  HessianFactor::HessianFactor(const Vector& b_in) : info_(matrix_) {
    JacobianFactor jf(b_in);
    ublas::noalias(matrix_) = ublas::prod(ublas::trans(jf.matrix_), jf.matrix_);
    info_.copyStructureFrom(jf.Ab_);
  }

  /* ************************************************************************* */
  HessianFactor::HessianFactor(Index i1, const Matrix& A1,
      const Vector& b, const SharedDiagonal& model) :
      GaussianFactor(i1), info_(matrix_) {
    JacobianFactor jf(i1, A1, b, model);
    ublas::noalias(matrix_) = ublas::prod(ublas::trans(jf.matrix_), jf.matrix_);
    info_.copyStructureFrom(jf.Ab_);
  }

  /* ************************************************************************* */
  HessianFactor::HessianFactor(Index i1, const Matrix& A1, Index i2, const Matrix& A2,
      const Vector& b, const SharedDiagonal& model) :
      GaussianFactor(i1,i2), info_(matrix_) {
    JacobianFactor jf(i1, A1, i2, A2, b, model);
    ublas::noalias(matrix_) = ublas::prod(ublas::trans(jf.matrix_), jf.matrix_);
    info_.copyStructureFrom(jf.Ab_);
  }

  /* ************************************************************************* */
  HessianFactor::HessianFactor(Index i1, const Matrix& A1, Index i2, const Matrix& A2,
      Index i3, const Matrix& A3, const Vector& b, const SharedDiagonal& model) :
      GaussianFactor(i1,i2,i3), info_(matrix_) {
    JacobianFactor jf(i1, A1, i2, A2, i3, A3, b, model);
    ublas::noalias(matrix_) = ublas::prod(ublas::trans(jf.matrix_), jf.matrix_);
    info_.copyStructureFrom(jf.Ab_);
  }

  /* ************************************************************************* */
  HessianFactor::HessianFactor(const std::vector<std::pair<Index, Matrix> > &terms,
      const Vector &b, const SharedDiagonal& model) : info_(matrix_) {
    JacobianFactor jf(terms, b, model);
    keys_ = jf.keys_;
    ublas::noalias(matrix_) = ublas::prod(ublas::trans(jf.matrix_), jf.matrix_);
    info_.copyStructureFrom(jf.Ab_);
  }

  /* ************************************************************************* */
  HessianFactor::HessianFactor(const std::list<std::pair<Index, Matrix> > &terms,
      const Vector &b, const SharedDiagonal& model) : info_(matrix_) {
    JacobianFactor jf(terms, b, model);
    keys_ = jf.keys_;
    ublas::noalias(matrix_) = ublas::prod(ublas::trans(jf.matrix_), jf.matrix_);
    info_.copyStructureFrom(jf.Ab_);
  }

  /* ************************************************************************* */
  HessianFactor::HessianFactor(const GaussianConditional& cg) : GaussianFactor(cg), info_(matrix_) {
    JacobianFactor jf(cg);
    ublas::noalias(ublas::symmetric_adaptor<MatrixColMajor,ublas::upper>(matrix_)) =
        ublas::prod(ublas::trans(jf.matrix_), jf.matrix_);
    info_.copyStructureFrom(jf.Ab_);
  }

  /* ************************************************************************* */
  HessianFactor::HessianFactor(const GaussianFactor& gf) : info_(matrix_) {
    // Copy the variable indices
    (GaussianFactor&)(*this) = gf;
    // Copy the matrix data depending on what type of factor we're copying from
    if(dynamic_cast<const JacobianFactor*>(&gf)) {
      const JacobianFactor& jf(static_cast<const JacobianFactor&>(gf));
      JacobianFactor whitened(jf.whiten());
      matrix_ = ublas::prod(ublas::trans(whitened.matrix_), whitened.matrix_);
      info_.copyStructureFrom(whitened.Ab_);
    } else if(dynamic_cast<const HessianFactor*>(&gf)) {
      const HessianFactor& hf(static_cast<const HessianFactor&>(gf));
      info_.assignNoalias(hf.info_);
    } else
      throw std::invalid_argument("In HessianFactor(const GaussianFactor& gf), gf is neither a JacobianFactor nor a HessianFactor");
  }

  /* ************************************************************************* */
  void HessianFactor::print(const std::string& s) const {
    cout << s << "\n";
    cout << " keys: ";
    for(const_iterator key=this->begin(); key!=this->end(); ++key)
      cout << *key << "(" << this->getDim(key) << ") ";
    cout << "\n";
    gtsam::print(ublas::symmetric_adaptor<const constBlock,ublas::upper>(
        info_.range(0,info_.nBlocks(), 0,info_.nBlocks())), "Ab^T * Ab: ");
  }

  /* ************************************************************************* */
  bool HessianFactor::equals(const GaussianFactor& lf, double tol) const {
    if(!dynamic_cast<const HessianFactor*>(&lf))
      return false;
    else {
      MatrixColMajor thisMatrix = ublas::symmetric_adaptor<const MatrixColMajor,ublas::upper>(this->info_.full());
      thisMatrix(thisMatrix.size1()-1, thisMatrix.size2()-1) = 0.0;
      MatrixColMajor rhsMatrix = ublas::symmetric_adaptor<const MatrixColMajor,ublas::upper>(static_cast<const HessianFactor&>(lf).info_.full());
      rhsMatrix(rhsMatrix.size1()-1, rhsMatrix.size2()-1) = 0.0;
      return equal_with_abs_tol(thisMatrix, rhsMatrix, tol);
    }
  }

  /* ************************************************************************* */
  double HessianFactor::error(const VectorValues& c) const {
    return ublas::inner_prod(c.vector(), ublas::prod(info_.range(0, this->size(), 0, this->size()), c.vector())) -
        2.0*ublas::inner_prod(c.vector(), info_.rangeColumn(0, this->size(), this->size(), 0));
  }

  /* ************************************************************************* */
  static FastMap<Index, SlotEntry> findScatterAndDims(const FactorGraph<HessianFactor>& factors) {

    static const bool debug = false;

    // The "scatter" is a map from global variable indices to slot indices in the
    // union of involved variables.  We also include the dimensionality of the
    // variable.

    Scatter scatter;

    // First do the set union.
    BOOST_FOREACH(const HessianFactor::shared_ptr& factor, factors) {
      for(HessianFactor::const_iterator variable = factor->begin(); variable != factor->end(); ++variable) {
        scatter.insert(make_pair(*variable, SlotEntry(0, factor->getDim(variable))));
      }
    }

    // Next fill in the slot indices (we can only get these after doing the set
    // union.
    size_t slot = 0;
    BOOST_FOREACH(Scatter::value_type& var_slot, scatter) {
      var_slot.second.slot = (slot ++);
      if(debug)
        cout << "scatter[" << var_slot.first << "] = (slot " << var_slot.second.slot << ", dim " << var_slot.second.dimension << ")" << endl;
    }

    return scatter;
  }

///* ************************************************************************* */
//static MatrixColMajor formAbTAb(const FactorGraph<HessianFactor>& factors, const Scatter& scatter) {
//
//  static const bool debug = false;
//
//  tic("CombineAndEliminate: 3.1 varStarts");
//  // Determine scalar indices of each variable
//  vector<size_t> varStarts;
//  varStarts.reserve(scatter.size() + 2);
//  varStarts.push_back(0);
//  BOOST_FOREACH(const Scatter::value_type& var_slot, scatter) {
//    varStarts.push_back(varStarts.back() + var_slot.second.dimension);
//  }
//  // This is for the r.h.s. vector
//  varStarts.push_back(varStarts.back() + 1);
//  toc("CombineAndEliminate: 3.1 varStarts");
//
//  // Allocate and zero matrix for Ab' * Ab
//  MatrixColMajor ATA(ublas::zero_matrix<double>(varStarts.back(), varStarts.back()));
//
//  tic("CombineAndEliminate: 3.2 updates");
//  // Do blockwise low-rank updates to Ab' * Ab for each factor.  Here, we
//  // only update the upper triangle because this is all that Cholesky uses.
//  BOOST_FOREACH(const HessianFactor::shared_ptr& factor, factors) {
//
//    // Whiten the factor first so it has a unit diagonal noise model
//    HessianFactor whitenedFactor(factor->whiten());
//
//    if(debug) whitenedFactor.print("whitened factor: ");
//
//    for(HessianFactor::const_iterator var2 = whitenedFactor.begin(); var2 != whitenedFactor.end(); ++var2) {
//      assert(scatter.find(*var2) != scatter.end());
//      size_t vj = scatter.find(*var2)->second.slot;
//      for(HessianFactor::const_iterator var1 = whitenedFactor.begin(); var1 <= var2; ++var1) {
//        assert(scatter.find(*var1) != scatter.end());
//        size_t vi = scatter.find(*var1)->second.slot;
//        if(debug) cout << "Updating block " << vi << ", " << vj << endl;
//        if(debug) cout << "Updating (" << varStarts[vi] << ":" << varStarts[vi+1] << ", " <<
//            varStarts[vj] << ":" << varStarts[vj+1] << ") from A" << *var1 << "' * A" << *var2 << endl;
//        ublas::noalias(ublas::project(ATA,
//            ublas::range(varStarts[vi], varStarts[vi+1]), ublas::range(varStarts[vj], varStarts[vj+1]))) +=
//                ublas::prod(ublas::trans(whitenedFactor.getA(var1)), whitenedFactor.getA(var2));
//      }
//    }
//
//    // Update r.h.s. vector
//    size_t vj = scatter.size();
//    for(HessianFactor::const_iterator var1 = whitenedFactor.begin(); var1 < whitenedFactor.end(); ++var1) {
//      assert(scatter.find(*var1) != scatter.end());
//      size_t vi = scatter.find(*var1)->second.slot;
//      if(debug) cout << "Updating block " << vi << ", " << vj << endl;
//      if(debug) cout << "Updating (" << varStarts[vi] << ":" << varStarts[vi+1] << ", " <<
//          varStarts[vj] << ":" << varStarts[vj+1] << ") from A" << *var1 << "' * b" << endl;
//      ublas::matrix_column<MatrixColMajor> col(ATA, varStarts[vj]);
//      ublas::noalias(ublas::subrange(col, varStarts[vi], varStarts[vi+1])) +=
//          ublas::prod(ublas::trans(whitenedFactor.getA(var1)), whitenedFactor.getb());
//    }
//
//    size_t vi = scatter.size();
//    if(debug) cout << "Updating block " << vi << ", " << vj << endl;
//    if(debug) cout << "Updating (" << varStarts[vi] << ":" << varStarts[vi+1] << ", " <<
//        varStarts[vj] << ":" << varStarts[vj+1] << ") from b" << "' * b" << endl;
//    ublas::noalias(ATA(varStarts[vi], varStarts[vj])) += ublas::inner_prod(whitenedFactor.getb(), whitenedFactor.getb());
//  }
//  toc("CombineAndEliminate: 3.2 updates");
//
//  return ATA;
//}

void HessianFactor::updateATA(const HessianFactor& update, const Scatter& scatter) {

  // This function updates 'combined' with the information in 'update'.
  // 'scatter' maps variables in the update factor to slots in the combined
  // factor.

  static const bool debug = false;

  // First build an array of slots
  tic(1, "slots");
  size_t slots[update.size()];
  size_t slot = 0;
  BOOST_FOREACH(Index j, update) {
    slots[slot] = scatter.find(j)->second.slot;
    ++ slot;
  }
  toc(1, "slots");

  // Apply updates to the upper triangle
  tic(2, "update");
  assert(this->info_.nBlocks() - 1 == scatter.size());
  for(size_t j2=0; j2<update.info_.nBlocks(); ++j2) {
    size_t slot2 = (j2 == update.size()) ? this->info_.nBlocks()-1 : slots[j2];
    for(size_t j1=0; j1<=j2; ++j1) {
      size_t slot1 = (j1 == update.size()) ? this->info_.nBlocks()-1 : slots[j1];
      if(debug)
        cout << "Updating (" << slot1 << "," << slot2 << ") from (" << j1 << "," << j2 << ")" << endl;
      ublas::noalias(this->info_(slot1, slot2)) += update.info_(j1,j2);
    }
  }
  toc(2, "update");
}

GaussianBayesNet::shared_ptr HessianFactor::splitEliminatedFactor(size_t nrFrontals, const vector<Index>& keys) {

  static const bool debug = false;

  // Extract conditionals
  tic(1, "extract conditionals");
  GaussianBayesNet::shared_ptr conditionals(new GaussianBayesNet());
  typedef VerticalBlockView<MatrixColMajor> BlockAb;
  BlockAb Ab(matrix_, info_);
  for(size_t j=0; j<nrFrontals; ++j) {
    // Temporarily restrict the matrix view to the conditional blocks of the
    // eliminated Ab_ matrix to create the GaussianConditional from it.
    size_t varDim = Ab(0).size2();
    Ab.rowEnd() = Ab.rowStart() + varDim;

    // Zero the entries below the diagonal (this relies on the matrix being
    // column-major).
    {
      tic(1, "zero");
      BlockAb::Block remainingMatrix(Ab.range(0, Ab.nBlocks()));
      if(remainingMatrix.size1() > 1)
        for(size_t j = 0; j < remainingMatrix.size1() - 1; ++j)
          memset(&remainingMatrix(j+1, j), 0, sizeof(remainingMatrix(0,0)) * (remainingMatrix.size1() - j - 1));
      toc(1, "zero");
    }

    tic(2, "construct cond");
    const ublas::scalar_vector<double> sigmas(varDim, 1.0);
    conditionals->push_back(boost::make_shared<Conditional>(keys.begin()+j, keys.end(), 1, Ab, sigmas));
    toc(2, "construct cond");
    if(debug) conditionals->back()->print("Extracted conditional: ");
    Ab.rowStart() += varDim;
    Ab.firstBlock() += 1;
    if(debug) cout << "rowStart = " << Ab.rowStart() << ", rowEnd = " << Ab.rowEnd() << endl;
  }
  toc(1, "extract conditionals");

  // Take lower-right block of Ab_ to get the new factor
  tic(2, "remaining factor");
  info_.blockStart() = nrFrontals;
  // Assign the keys
  keys_.assign(keys.begin() + nrFrontals, keys.end());
  toc(2, "remaining factor");

  return conditionals;
}

/* ************************************************************************* */
pair<GaussianBayesNet::shared_ptr, HessianFactor::shared_ptr> HessianFactor::CombineAndEliminate(
        const FactorGraph<HessianFactor>& factors, size_t nrFrontals) {

  static const bool debug = false;

  // Find the scatter and variable dimensions
  tic(1, "find scatter");
  Scatter scatter(findScatterAndDims(factors));
  toc(1, "find scatter");

  // Pull out keys and dimensions
  tic(2, "keys");
  vector<Index> keys(scatter.size());
  vector<size_t> dimensions(scatter.size() + 1);
  BOOST_FOREACH(const Scatter::value_type& var_slot, scatter) {
    keys[var_slot.second.slot] = var_slot.first;
    dimensions[var_slot.second.slot] = var_slot.second.dimension;
  }
  // This is for the r.h.s. vector
  dimensions.back() = 1;
  toc(2, "keys");

  // Form Ab' * Ab
  tic(3, "combine");
  tic(1, "allocate");
  HessianFactor::shared_ptr combinedFactor(new HessianFactor());
  combinedFactor->info_.resize(dimensions.begin(), dimensions.end(), false);
  toc(1, "allocate");
  tic(2, "zero");
  ublas::noalias(combinedFactor->matrix_) = ublas::zero_matrix<double>(combinedFactor->matrix_.size1(), combinedFactor->matrix_.size2());
  toc(2, "zero");
  tic(3, "update");
  BOOST_FOREACH(const HessianFactor::shared_ptr& factor, factors) {
    combinedFactor->updateATA(*factor, scatter);
  }
  toc(3, "update");
  if(debug) gtsam::print(combinedFactor->matrix_, "Ab' * Ab: ");
  toc(3, "combine");

  // Do Cholesky, note that after this, the lower triangle still contains
  // some untouched non-zeros that should be zero.  We zero them while
  // extracting submatrices next.
  tic(4, "partial Cholesky");
  choleskyPartial(combinedFactor->matrix_, combinedFactor->info_.offset(nrFrontals));
  if(debug) gtsam::print(combinedFactor->matrix_, "chol(Ab' * Ab): ");
  toc(4, "partial Cholesky");

  // Extract conditionals and fill in details of the remaining factor
  tic(5, "split");
  GaussianBayesNet::shared_ptr conditionals(combinedFactor->splitEliminatedFactor(nrFrontals, keys));
  if(debug) {
    conditionals->print("Extracted conditionals: ");
    combinedFactor->print("Eliminated factor (L piece): ");
  }
  toc(5, "split");

  return make_pair(conditionals, combinedFactor);
}

}
