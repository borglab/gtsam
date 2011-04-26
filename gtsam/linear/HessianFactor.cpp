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

#include <gtsam/base/debug.h>
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

#include <gtsam/3rdparty/Eigen/Core>
#include <gtsam/3rdparty/Eigen/Dense>

#include <sstream>

using namespace std;

namespace ublas = boost::numeric::ublas;
using namespace boost::lambda;

namespace gtsam {

  /* ************************************************************************* */
  void HessianFactor::assertInvariants() const {
#ifndef NDEBUG
    // Check for non-finite values
    for(size_t i=0; i<matrix_.size1(); ++i)
      for(size_t j=i; j<matrix_.size2(); ++j)
        if(!isfinite(matrix_(i,j)))
          throw invalid_argument("HessianFactor contains non-finite matrix entries.");
#endif
  }

  /* ************************************************************************* */
  HessianFactor::HessianFactor(const HessianFactor& gf) :
    GaussianFactor(gf), info_(matrix_) {
    info_.assignNoalias(gf.info_);
    assertInvariants();
  }

  /* ************************************************************************* */
  HessianFactor::HessianFactor() : info_(matrix_) {
    assertInvariants();
  }

  /* ************************************************************************* */
  HessianFactor::HessianFactor(Index j1, const Matrix& G, const Vector& g, double f) :
      GaussianFactor(j1), info_(matrix_) {
    if(G.size1() != G.size2() || G.size1() != g.size())
      throw invalid_argument("Inconsistent matrix and/or vector dimensions in HessianFactor constructor");
    size_t dims[] = { G.size1(), 1 };
    InfoMatrix fullMatrix(G.size1() + 1, G.size1() + 1);
    BlockInfo infoMatrix(fullMatrix, dims, dims+2);
    infoMatrix(0,0) = G;
    infoMatrix.column(0,1,0) = g;
    infoMatrix(1,1)(0,0) = f;
    infoMatrix.swap(info_);
    assertInvariants();
  }

  /* ************************************************************************* */
  HessianFactor::HessianFactor(Index j1, Index j2,
      const Matrix& G11, const Matrix& G12, const Vector& g1,
      const Matrix& G22, const Vector& g2, double f) :
      GaussianFactor(j1, j2), info_(matrix_) {
    if(G11.size1() != G11.size2() || G11.size1() != G12.size1() || G11.size1() != g1.size() ||
        G22.size2() != G12.size2() || G22.size2() != g2.size())
      throw invalid_argument("Inconsistent matrix and/or vector dimensions in HessianFactor constructor");
    size_t dims[] = { G11.size1(), G22.size1(), 1 };
    InfoMatrix fullMatrix(G11.size1() + G22.size1() + 1, G11.size1() + G22.size1() + 1);
    BlockInfo infoMatrix(fullMatrix, dims, dims+3);
    infoMatrix(0,0) = G11;
    infoMatrix(0,1) = G12;
    infoMatrix.column(0,2,0) = g1;
    infoMatrix(1,1) = G22;
    infoMatrix.column(1,2,0) = g2;
    infoMatrix(2,2)(0,0) = f;
    infoMatrix.swap(info_);
    assertInvariants();
  }

  /* ************************************************************************* */
  HessianFactor::HessianFactor(const GaussianConditional& cg) : GaussianFactor(cg), info_(matrix_) {
    JacobianFactor jf(cg);
    info_.copyStructureFrom(jf.Ab_);
    ublas::noalias(ublas::symmetric_adaptor<MatrixColMajor,ublas::upper>(matrix_)) =
        ublas::prod(ublas::trans(jf.matrix_), jf.matrix_);
    assertInvariants();
  }

  /* ************************************************************************* */
  HessianFactor::HessianFactor(const GaussianFactor& gf) : info_(matrix_) {
    // Copy the variable indices
    (GaussianFactor&)(*this) = gf;
    // Copy the matrix data depending on what type of factor we're copying from
    if(dynamic_cast<const JacobianFactor*>(&gf)) {
      const JacobianFactor& jf(static_cast<const JacobianFactor&>(gf));
      if(jf.model_->isConstrained())
        throw invalid_argument("Cannot construct HessianFactor from JacobianFactor with constrained noise model");
      else {
      	Vector invsigmas = jf.model_->invsigmas();
        typedef Eigen::Map<Eigen::MatrixXd> EigenMap;
        typedef typeof(EigenMap(&jf.matrix_(0,0),0,0).block(0,0,0,0)) EigenBlock;
        EigenBlock A(EigenMap(&jf.matrix_(0,0),jf.matrix_.size1(),jf.matrix_.size2()).block(
            jf.Ab_.rowStart(),jf.Ab_.offset(0), jf.Ab_.full().size1(), jf.Ab_.full().size2()));
        typedef typeof(Eigen::Map<Eigen::VectorXd>(&invsigmas(0),0).asDiagonal()) EigenDiagonal;
        EigenDiagonal R(Eigen::Map<Eigen::VectorXd>(&invsigmas(0),jf.model_->dim()).asDiagonal());
        info_.copyStructureFrom(jf.Ab_);
        EigenMap L(EigenMap(&matrix_(0,0), matrix_.size1(), matrix_.size2()));
        L.noalias() = A.transpose() * R * R * A;
      }
    } else if(dynamic_cast<const HessianFactor*>(&gf)) {
      const HessianFactor& hf(static_cast<const HessianFactor&>(gf));
      info_.assignNoalias(hf.info_);
    } else
      throw std::invalid_argument("In HessianFactor(const GaussianFactor& gf), gf is neither a JacobianFactor nor a HessianFactor");
    assertInvariants();
  }

  /* ************************************************************************* */
	HessianFactor::HessianFactor(const FactorGraph<GaussianFactor>& factors,
			const vector<size_t>& dimensions, const Scatter& scatter) :
		info_(matrix_) {

		const bool debug = ISDEBUG("EliminateCholesky");
		// Form Ab' * Ab
		tic(1, "allocate");
		info_.resize(dimensions.begin(), dimensions.end(), false);
		toc(1, "allocate");
		tic(2, "zero");
		ublas::noalias(matrix_) = ublas::zero_matrix<double>(matrix_.size1(),matrix_.size2());
		toc(2, "zero");
		tic(3, "update");
		BOOST_FOREACH(const GaussianFactor::shared_ptr& factor, factors)
		{
			shared_ptr hessian(boost::dynamic_pointer_cast<HessianFactor>(factor));
			if (hessian)
				updateATA(*hessian, scatter);
			else {
				JacobianFactor::shared_ptr jacobianFactor(boost::dynamic_pointer_cast<JacobianFactor>(factor));
				if (jacobianFactor)
					updateATA(*jacobianFactor, scatter);
				else
					throw invalid_argument("GaussianFactor is neither Hessian nor Jacobian");
			}
		}
		toc(3, "update");

		if (debug) gtsam::print(matrix_, "Ab' * Ab: ");

		assertInvariants();
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
    return 0.5 * (ublas::inner_prod(c.vector(),
        ublas::prod(
            ublas::symmetric_adaptor<const constBlock,ublas::upper>(info_.range(0, this->size(), 0, this->size())),
            c.vector())) -
        2.0*ublas::inner_prod(c.vector(), info_.rangeColumn(0, this->size(), this->size(), 0)) +
        info_(this->size(), this->size())(0,0));
  }

void HessianFactor::updateATA(const HessianFactor& update, const Scatter& scatter) {

  // This function updates 'combined' with the information in 'update'.
  // 'scatter' maps variables in the update factor to slots in the combined
  // factor.

  const bool debug = ISDEBUG("updateATA");

  // First build an array of slots
  tic(1, "slots");
  size_t slots[update.size()];
  size_t slot = 0;
  BOOST_FOREACH(Index j, update) {
    slots[slot] = scatter.find(j)->second.slot;
    ++ slot;
  }
  toc(1, "slots");

  if(debug) {
    this->print("Updating this: ");
    update.print("with: ");
  }

  Eigen::Map<Eigen::MatrixXd> information(&matrix_(0,0), matrix_.size1(), matrix_.size2());
  Eigen::Map<Eigen::MatrixXd> updateInform(&update.matrix_(0,0), update.matrix_.size1(), update.matrix_.size2());

  // Apply updates to the upper triangle
  tic(3, "update");
  assert(this->info_.nBlocks() - 1 == scatter.size());
  for(size_t j2=0; j2<update.info_.nBlocks(); ++j2) {
    size_t slot2 = (j2 == update.size()) ? this->info_.nBlocks()-1 : slots[j2];
    for(size_t j1=0; j1<=j2; ++j1) {
      size_t slot1 = (j1 == update.size()) ? this->info_.nBlocks()-1 : slots[j1];
      if(slot2 > slot1) {
        if(debug)
          cout << "Updating (" << slot1 << "," << slot2 << ") from (" << j1 << "," << j2 << ")" << endl;
        information.block(info_.offset(slot1), info_.offset(slot2), info_(slot1,slot2).size1(), info_(slot1,slot2).size2()).noalias() +=
                updateInform.block(update.info_.offset(j1), update.info_.offset(j2), update.info_(j1,j2).size1(), update.info_(j1,j2).size2());
      } else if(slot1 > slot2) {
        if(debug)
          cout << "Updating (" << slot2 << "," << slot1 << ") from (" << j1 << "," << j2 << ")" << endl;
        information.block(info_.offset(slot2), info_.offset(slot1), info_(slot2,slot1).size1(), info_(slot2,slot1).size2()).noalias() +=
                updateInform.block(update.info_.offset(j1), update.info_.offset(j2), update.info_(j1,j2).size1(), update.info_(j1,j2).size2()).transpose();
      } else {
        if(debug)
          cout << "Updating (" << slot1 << "," << slot2 << ") from (" << j1 << "," << j2 << ")" << endl;
        information.block(info_.offset(slot1), info_.offset(slot2), info_(slot1,slot2).size1(), info_(slot1,slot2).size2()).triangularView<Eigen::Upper>() +=
            updateInform.block(update.info_.offset(j1), update.info_.offset(j2), update.info_(j1,j2).size1(), update.info_(j1,j2).size2());
      }
      if(debug) cout << "Updating block " << slot1 << "," << slot2 << " from block " << j1 << "," << j2 << "\n";
      if(debug) this->print();
    }
  }
  toc(3, "update");
}

void HessianFactor::updateATA(const JacobianFactor& update, const Scatter& scatter) {

  // This function updates 'combined' with the information in 'update'.
  // 'scatter' maps variables in the update factor to slots in the combined
  // factor.

  const bool debug = ISDEBUG("updateATA");

  // First build an array of slots
  tic(1, "slots");
  size_t slots[update.size()];
  size_t slot = 0;
  BOOST_FOREACH(Index j, update) {
    slots[slot] = scatter.find(j)->second.slot;
    ++ slot;
  }
  toc(1, "slots");

  tic(2, "form A^T*A");
  if(update.model_->isConstrained())
    throw invalid_argument("Cannot update HessianFactor from JacobianFactor with constrained noise model");

  if(debug) {
    this->print("Updating this: ");
    update.print("with: ");
  }

  Eigen::Map<Eigen::MatrixXd> information(&matrix_(0,0), matrix_.size1(), matrix_.size2());
  Eigen::Map<Eigen::MatrixXd> updateAf(&update.matrix_(0,0), update.matrix_.size1(), update.matrix_.size2());
  Eigen::Block<typeof(updateAf)> updateA(updateAf.block(
              update.Ab_.rowStart(),update.Ab_.offset(0), update.Ab_.full().size1(), update.Ab_.full().size2()));

  Eigen::MatrixXd updateInform;
  if(boost::dynamic_pointer_cast<noiseModel::Unit>(update.model_)) {
    updateInform.noalias() = updateA.transpose() * updateA;
  } else {
    noiseModel::Diagonal::shared_ptr diagonal(boost::dynamic_pointer_cast<noiseModel::Diagonal>(update.model_));
    if(diagonal) {
      typeof(Eigen::Map<Eigen::VectorXd>(&update.model_->invsigmas()(0),0).asDiagonal()) R(
          Eigen::Map<Eigen::VectorXd>(&update.model_->invsigmas()(0),update.model_->dim()).asDiagonal());
      updateInform.noalias() = updateA.transpose() * R * R * updateA;
    } else
      throw invalid_argument("In HessianFactor::updateATA, JacobianFactor noise model is neither Unit nor Diagonal");
  }
  toc(2, "form A^T*A");

  // Apply updates to the upper triangle
  tic(3, "update");
  assert(this->info_.nBlocks() - 1 == scatter.size());
  for(size_t j2=0; j2<update.Ab_.nBlocks(); ++j2) {
    size_t slot2 = (j2 == update.size()) ? this->info_.nBlocks()-1 : slots[j2];
    for(size_t j1=0; j1<=j2; ++j1) {
      size_t slot1 = (j1 == update.size()) ? this->info_.nBlocks()-1 : slots[j1];
      size_t off0 = update.Ab_.offset(0);
      if(slot2 > slot1) {
        if(debug)
          cout << "Updating (" << slot1 << "," << slot2 << ") from (" << j1 << "," << j2 << ")" << endl;
        information.block(info_.offset(slot1), info_.offset(slot2), info_(slot1,slot2).size1(), info_(slot1,slot2).size2()).noalias() +=
                updateInform.block(update.Ab_.offset(j1)-off0, update.Ab_.offset(j2)-off0, update.Ab_(j1).size2(), update.Ab_(j2).size2());
      } else if(slot1 > slot2) {
        if(debug)
          cout << "Updating (" << slot2 << "," << slot1 << ") from (" << j1 << "," << j2 << ")" << endl;
        information.block(info_.offset(slot2), info_.offset(slot1), info_(slot2,slot1).size1(), info_(slot2,slot1).size2()).noalias() +=
                updateInform.block(update.Ab_.offset(j1)-off0, update.Ab_.offset(j2)-off0, update.Ab_(j1).size2(), update.Ab_(j2).size2()).transpose();
      } else {
        if(debug)
          cout << "Updating (" << slot1 << "," << slot2 << ") from (" << j1 << "," << j2 << ")" << endl;
        information.block(info_.offset(slot1), info_.offset(slot2), info_(slot1,slot2).size1(), info_(slot1,slot2).size2()).triangularView<Eigen::Upper>() +=
            updateInform.block(update.Ab_.offset(j1)-off0, update.Ab_.offset(j2)-off0, update.Ab_(j1).size2(), update.Ab_(j2).size2());
      }
      if(debug) cout << "Updating block " << slot1 << "," << slot2 << " from block " << j1 << "," << j2 << "\n";
      if(debug) this->print();
    }
  }
  toc(3, "update");

//  Eigen::Map<Eigen::MatrixXd> information(&matrix_(0,0), matrix_.size1(), matrix_.size2());
//  Eigen::Map<Eigen::MatrixXd> updateA(&update.matrix_(0,0), update.matrix_.size1(), update.matrix_.size2());
//
//  // Apply updates to the upper triangle
//  tic(2, "update");
//  assert(this->info_.nBlocks() - 1 == scatter.size());
//  for(size_t j2=0; j2<update.Ab_.nBlocks(); ++j2) {
//    size_t slot2 = (j2 == update.size()) ? this->info_.nBlocks()-1 : slots[j2];
//    for(size_t j1=0; j1<=j2; ++j1) {
//      size_t slot1 = (j1 == update.size()) ? this->info_.nBlocks()-1 : slots[j1];
//      typedef typeof(updateA.block(0,0,0,0)) ABlock;
//      ABlock A1(updateA.block(update.Ab_.rowStart(),update.Ab_.offset(j1), update.Ab_(j1).size1(),update.Ab_(j1).size2()));
//      ABlock A2(updateA.block(update.Ab_.rowStart(),update.Ab_.offset(j2), update.Ab_(j2).size1(),update.Ab_(j2).size2()));
//      if(slot2 > slot1) {
//        if(debug)
//          cout << "Updating (" << slot1 << "," << slot2 << ") from (" << j1 << "," << j2 << ")" << endl;
//        if(boost::dynamic_pointer_cast<noiseModel::Unit>(update.model_)) {
//          information.block(info_.offset(slot1), info_.offset(slot2), info_(slot1,slot2).size1(), info_(slot1,slot2).size2()) +=
//              A1.transpose() * A2;
//        } else {
//          noiseModel::Diagonal::shared_ptr diagonal(boost::dynamic_pointer_cast<noiseModel::Diagonal>(update.model_));
//          if(diagonal) {
//            typeof(Eigen::Map<Eigen::VectorXd>(&update.model_->invsigmas()(0),0).asDiagonal()) R(
//                Eigen::Map<Eigen::VectorXd>(&update.model_->invsigmas()(0),update.model_->dim()).asDiagonal());
//            information.block(info_.offset(slot1), info_.offset(slot2), info_(slot1,slot2).size1(), info_(slot1,slot2).size2()).noalias() +=
//                A1.transpose() * R * R * A2;
//          } else
//            throw invalid_argument("In HessianFactor::updateATA, JacobianFactor noise model is neither Unit nor Diagonal");
//        }
//      } else if(slot1 > slot2) {
//        if(debug)
//          cout << "Updating (" << slot2 << "," << slot1 << ") from (" << j1 << "," << j2 << ")" << endl;
//        if(boost::dynamic_pointer_cast<noiseModel::Unit>(update.model_)) {
//          information.block(info_.offset(slot2), info_.offset(slot1), info_(slot2,slot1).size1(), info_(slot2,slot1).size2()).noalias() +=
//              A2.transpose() * A1;
//        } else {
//          noiseModel::Diagonal::shared_ptr diagonal(boost::dynamic_pointer_cast<noiseModel::Diagonal>(update.model_));
//          if(diagonal) {
//            typeof(Eigen::Map<Eigen::VectorXd>(&update.model_->invsigmas()(0),0).asDiagonal()) R(
//                Eigen::Map<Eigen::VectorXd>(&update.model_->invsigmas()(0),update.model_->dim()).asDiagonal());
//            information.block(info_.offset(slot1), info_.offset(slot2), info_(slot1,slot2).size1(), info_(slot1,slot2).size2()).noalias() +=
//                A2.transpose() * R * R * A1;
//          } else
//            throw invalid_argument("In HessianFactor::updateATA, JacobianFactor noise model is neither Unit nor Diagonal");
//        }
//     } else {
//        if(debug)
//          cout << "Updating (" << slot1 << "," << slot2 << ") from (" << j1 << "," << j2 << ")" << endl;
//        if(boost::dynamic_pointer_cast<noiseModel::Unit>(update.model_)) {
//          information.block(info_.offset(slot2), info_.offset(slot1), info_(slot2,slot1).size1(), info_(slot2,slot1).size2()).triangularView<Eigen::Upper>() +=
//              A1.transpose() * A1;
//        } else {
//          noiseModel::Diagonal::shared_ptr diagonal(boost::dynamic_pointer_cast<noiseModel::Diagonal>(update.model_));
//          if(diagonal) {
//            typeof(Eigen::Map<Eigen::VectorXd>(&update.model_->invsigmas()(0),0).asDiagonal()) R(
//                Eigen::Map<Eigen::VectorXd>(&update.model_->invsigmas()(0),update.model_->dim()).asDiagonal());
//            information.block(info_.offset(slot2), info_.offset(slot1), info_(slot2,slot1).size1(), info_(slot2,slot1).size2()).triangularView<Eigen::Upper>() +=
//                A1.transpose() * R * R * A1;
//          } else
//            throw invalid_argument("In HessianFactor::updateATA, JacobianFactor noise model is neither Unit nor Diagonal");
//        }
//      }
//      if(debug) cout << "Updating block " << slot1 << "," << slot2 << " from block " << j1 << "," << j2 << "\n";
//      if(debug) this->print();
//    }
//  }
//  toc(2, "update");
}

/* ************************************************************************* */
void HessianFactor::partialCholesky(size_t nrFrontals) {
	choleskyPartial(matrix_, info_.offset(nrFrontals));
}

/* ************************************************************************* */
GaussianBayesNet::shared_ptr
HessianFactor::splitEliminatedFactor(size_t nrFrontals, const vector<Index>& keys) {

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
        for(size_t j = 0; j < remainingMatrix.size1() - 1; ++j) {
          ublas::matrix_column<BlockAb::Block> col(ublas::column(remainingMatrix, j));
          std::fill(col.begin() + j+1, col.end(), 0.0);
        }
      toc(1, "zero");
    }

    tic(2, "construct cond");
    const ublas::scalar_vector<double> sigmas(varDim, 1.0);
    conditionals->push_back(boost::make_shared<ConditionalType>(keys.begin()+j, keys.end(), 1, Ab, sigmas));
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

} // gtsam
