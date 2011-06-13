/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    HessianFactor.cpp
 * @author  Richard Roberts
 * @created Dec 8, 2010
 */

#include <sstream>

#include <boost/foreach.hpp>
#include <boost/format.hpp>
#include <boost/make_shared.hpp>
#include <boost/lambda/bind.hpp>
#include <boost/lambda/lambda.hpp>
#include <boost/tuple/tuple.hpp>

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

using namespace std;

using namespace boost::lambda;

namespace gtsam {

/* ************************************************************************* */
string SlotEntry::toString() const {
	ostringstream oss;
	oss << "SlotEntry: slot=" << slot << ", dim=" << dimension;
	return oss.str();
}

/* ************************************************************************* */
void HessianFactor::assertInvariants() const {
#ifndef NDEBUG
	// Check for non-finite values
	for(size_t i=0; i<(size_t) matrix_.rows(); ++i)
		for(size_t j=i; j<(size_t) matrix_.cols(); ++j)
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
	if(G.rows() != G.cols() || G.rows() != g.size())
		throw invalid_argument("Inconsistent matrix and/or vector dimensions in HessianFactor constructor");
	size_t dims[] = { G.rows(), 1 };
	InfoMatrix fullMatrix(G.rows() + 1, G.rows() + 1);
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
	if(G11.rows() != G11.cols() || G11.rows() != G12.rows() || G11.rows() != g1.size() ||
			G22.cols() != G12.cols() || G22.cols() != g2.size())
		throw invalid_argument("Inconsistent matrix and/or vector dimensions in HessianFactor constructor");
	size_t dims[] = { G11.rows(), G22.rows(), 1 };
	InfoMatrix fullMatrix(G11.rows() + G22.rows() + 1, G11.rows() + G22.rows() + 1);
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
	matrix_.noalias() = jf.matrix_.transpose() * jf.matrix_;
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
			Vector invsigmas = jf.model_->invsigmas().cwiseProduct(jf.model_->invsigmas());
			info_.copyStructureFrom(jf.Ab_);
			BlockInfo::constBlock A = jf.Ab_.full();
			matrix_.noalias() = A.transpose() * invsigmas.asDiagonal() * A;
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

	const bool debug = ISDEBUG("EliminateCholesky") || ISDEBUG("EliminateLDL");
	// Form Ab' * Ab
	tic(1, "allocate");
	info_.resize(dimensions.begin(), dimensions.end(), false);
	toc(1, "allocate");
	tic(2, "zero");
	matrix_.noalias() = Matrix::Zero(matrix_.rows(),matrix_.cols());
	toc(2, "zero");
	tic(3, "update");
	if (debug) cout << "Combining " << factors.size() << " factors" << endl;
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
	gtsam::print(Matrix(info_.range(0,info_.nBlocks(), 0,info_.nBlocks()).selfadjointView<Eigen::Upper>()), "Ab^T * Ab: ");
}

/* ************************************************************************* */
bool HessianFactor::equals(const GaussianFactor& lf, double tol) const {
	if(!dynamic_cast<const HessianFactor*>(&lf))
		return false;
	else {
		Matrix thisMatrix = this->info_.full().selfadjointView<Eigen::Upper>();
		thisMatrix(thisMatrix.rows()-1, thisMatrix.cols()-1) = 0.0;
		Matrix rhsMatrix = static_cast<const HessianFactor&>(lf).info_.full().selfadjointView<Eigen::Upper>();
		rhsMatrix(rhsMatrix.rows()-1, rhsMatrix.cols()-1) = 0.0;
		return equal_with_abs_tol(thisMatrix, rhsMatrix, tol);
	}
}

/* ************************************************************************* */
double HessianFactor::constant_term() const {
	return info_(this->size(), this->size())(0,0);
}

/* ************************************************************************* */
HessianFactor::constColumn HessianFactor::linear_term() const {
	return info_.rangeColumn(0, this->size(), this->size(), 0);
}

/* ************************************************************************* */
double HessianFactor::error(const VectorValues& c) const {
	// error 0.5*(f - 2*x'*g + x'*G*x)
	const double f = constant_term();
	const double xtg = c.vector().dot(linear_term());
	const double xGx = c.vector().transpose() * info_.range(0, this->size(), 0, this->size()).selfadjointView<Eigen::Upper>() *	c.vector();

	return 0.5 * (f - 2.0 * xtg +  xGx);
}

/* ************************************************************************* */
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
		update.print("with (Hessian): ");
	}

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
				matrix_.block(info_.offset(slot1), info_.offset(slot2), info_(slot1,slot2).rows(), info_(slot1,slot2).cols()).noalias() +=
						update.matrix_.block(update.info_.offset(j1), update.info_.offset(j2), update.info_(j1,j2).rows(), update.info_(j1,j2).cols());
			} else if(slot1 > slot2) {
				if(debug)
					cout << "Updating (" << slot2 << "," << slot1 << ") from (" << j1 << "," << j2 << ")" << endl;
				matrix_.block(info_.offset(slot2), info_.offset(slot1), info_(slot2,slot1).rows(), info_(slot2,slot1).cols()).noalias() +=
						update.matrix_.block(update.info_.offset(j1), update.info_.offset(j2), update.info_(j1,j2).rows(), update.info_(j1,j2).cols()).transpose();
			} else {
				if(debug)
					cout << "Updating (" << slot1 << "," << slot2 << ") from (" << j1 << "," << j2 << ")" << endl;
				matrix_.block(info_.offset(slot1), info_.offset(slot2), info_(slot1,slot2).rows(), info_(slot1,slot2).cols()).triangularView<Eigen::Upper>() +=
						update.matrix_.block(update.info_.offset(j1), update.info_.offset(j2), update.info_(j1,j2).rows(), update.info_(j1,j2).cols());
			}
			if(debug) cout << "Updating block " << slot1 << "," << slot2 << " from block " << j1 << "," << j2 << "\n";
			if(debug) this->print();
		}
	}
	toc(3, "update");
}

/* ************************************************************************* */
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
		update.print("with (Jacobian): ");
	}

	Eigen::Block<typeof(update.matrix_)> updateA(update.matrix_.block(
			update.Ab_.rowStart(),update.Ab_.offset(0), update.Ab_.full().rows(), update.Ab_.full().cols()));
	if (debug) cout << "updateA: \n" << updateA << endl;

	Matrix updateInform;
	if(boost::dynamic_pointer_cast<noiseModel::Unit>(update.model_)) {
		updateInform.noalias() = updateA.transpose() * updateA;
	} else {
		noiseModel::Diagonal::shared_ptr diagonal(boost::dynamic_pointer_cast<noiseModel::Diagonal>(update.model_));
		if(diagonal) {
			Vector invsigmas2 = update.model_->invsigmas().cwiseProduct(update.model_->invsigmas());
			updateInform.noalias() = updateA.transpose() * invsigmas2.asDiagonal() * updateA;
		} else
			throw invalid_argument("In HessianFactor::updateATA, JacobianFactor noise model is neither Unit nor Diagonal");
	}
	if (debug) cout << "updateInform: \n" << updateInform << endl;
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
				matrix_.block(info_.offset(slot1), info_.offset(slot2), info_(slot1,slot2).rows(), info_(slot1,slot2).cols()).noalias() +=
						updateInform.block(update.Ab_.offset(j1)-off0, update.Ab_.offset(j2)-off0, update.Ab_(j1).cols(), update.Ab_(j2).cols());
			} else if(slot1 > slot2) {
				if(debug)
					cout << "Updating (" << slot2 << "," << slot1 << ") from (" << j1 << "," << j2 << ")" << endl;
				matrix_.block(info_.offset(slot2), info_.offset(slot1), info_(slot2,slot1).rows(), info_(slot2,slot1).cols()).noalias() +=
						updateInform.block(update.Ab_.offset(j1)-off0, update.Ab_.offset(j2)-off0, update.Ab_(j1).cols(), update.Ab_(j2).cols()).transpose();
			} else {
				if(debug)
					cout << "Updating (" << slot1 << "," << slot2 << ") from (" << j1 << "," << j2 << ")" << endl;
				matrix_.block(info_.offset(slot1), info_.offset(slot2), info_(slot1,slot2).rows(), info_(slot1,slot2).cols()).triangularView<Eigen::Upper>() +=
						updateInform.block(update.Ab_.offset(j1)-off0, update.Ab_.offset(j2)-off0, update.Ab_(j1).cols(), update.Ab_(j2).cols());
			}
			if(debug) cout << "Updating block " << slot1 << "," << slot2 << " from block " << j1 << "," << j2 << "\n";
			if(debug) this->print();
		}
	}
	toc(3, "update");
}

/* ************************************************************************* */
void HessianFactor::partialCholesky(size_t nrFrontals) {
	choleskyPartial(matrix_, info_.offset(nrFrontals));
}

/* ************************************************************************* */
Eigen::LDLT<Matrix>::TranspositionType HessianFactor::partialLDL(size_t nrFrontals) {
  return ldlPartial(matrix_, info_.offset(nrFrontals));
}

/* ************************************************************************* */
GaussianConditional::shared_ptr
HessianFactor::splitEliminatedFactor(size_t nrFrontals, const vector<Index>& keys, const Eigen::LDLT<Matrix>::TranspositionType& permutation) {

  static const bool debug = false;

  // Extract conditionals
  tic(1, "extract conditionals");
  GaussianConditional::shared_ptr conditionals(new GaussianConditional());
  typedef VerticalBlockView<Matrix> BlockAb;
  BlockAb Ab(matrix_, info_);

  size_t varDim = info_.offset(nrFrontals);
  Ab.rowEnd() = Ab.rowStart() + varDim;

  // Create one big conditionals with many frontal variables.
  // Because of the pivoting permutation when using LDL, treating each variable separately doesn't make sense.
  tic(2, "construct cond");
  Vector sigmas = Vector::Ones(varDim);
  conditionals = boost::make_shared<ConditionalType>(keys.begin(), keys.end(), nrFrontals, Ab, sigmas, permutation);
  toc(2, "construct cond");
  if(debug) conditionals->print("Extracted conditional: ");

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
