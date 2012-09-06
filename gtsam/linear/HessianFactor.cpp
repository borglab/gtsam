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
 * @date    Dec 8, 2010
 */

#include <sstream>

#include <boost/foreach.hpp>
#include <boost/format.hpp>
#include <boost/make_shared.hpp>
#include <boost/tuple/tuple.hpp>
#include <boost/bind.hpp>

#include <gtsam/base/debug.h>
#include <gtsam/base/timing.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/FastMap.h>
#include <gtsam/base/cholesky.h>
#include <gtsam/linear/linearExceptions.h>
#include <gtsam/linear/GaussianConditional.h>
#include <gtsam/linear/HessianFactor.h>
#include <gtsam/linear/JacobianFactor.h>
#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/linear/GaussianBayesNet.h>

using namespace std;

namespace gtsam {

/* ************************************************************************* */
string SlotEntry::toString() const {
	ostringstream oss;
	oss << "SlotEntry: slot=" << slot << ", dim=" << dimension;
	return oss.str();
}

/* ************************************************************************* */
void HessianFactor::assertInvariants() const {
	GaussianFactor::assertInvariants(); // The base class checks for unique keys
}

/* ************************************************************************* */
HessianFactor::HessianFactor(const HessianFactor& gf) :
    		GaussianFactor(gf), info_(matrix_) {
	info_.assignNoalias(gf.info_);
	assertInvariants();
}

/* ************************************************************************* */
HessianFactor::HessianFactor() : info_(matrix_) {
  // The empty HessianFactor has only a constant error term of zero
  FastVector<size_t> dims;
  dims.push_back(1);
  info_.resize(dims.begin(), dims.end(), false);
  info_(0,0)(0,0) = 0.0;
	assertInvariants();
}

/* ************************************************************************* */
HessianFactor::HessianFactor(Index j, const Matrix& G, const Vector& g, double f) :
      		GaussianFactor(j), info_(matrix_) {
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
// error is 0.5*(x-mu)'*inv(Sigma)*(x-mu) = 0.5*(x'*G*x - 2*x'*G*mu + mu'*G*mu)
// where G = inv(Sigma), g = G*mu, f = mu'*G*mu = mu'*g
HessianFactor::HessianFactor(Index j, const Vector& mu, const Matrix& Sigma) :
		GaussianFactor(j), info_(matrix_) {
	if (Sigma.rows() != Sigma.cols() || Sigma.rows() != mu.size()) throw invalid_argument(
			"Inconsistent matrix and/or vector dimensions in HessianFactor constructor");
	Matrix G = inverse(Sigma);
	Vector g = G * mu;
	double f = dot(mu, g);
	size_t dims[] = { G.rows(), 1 };
	InfoMatrix fullMatrix(G.rows() + 1, G.rows() + 1);
	BlockInfo infoMatrix(fullMatrix, dims, dims + 2);
	infoMatrix(0, 0) = G;
	infoMatrix.column(0, 1, 0) = g;
	infoMatrix(1, 1)(0, 0) = f;
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
HessianFactor::HessianFactor(Index j1, Index j2, Index j3,
    const Matrix& G11, const Matrix& G12, const Matrix& G13, const Vector& g1,
    const Matrix& G22, const Matrix& G23, const Vector& g2,
    const Matrix& G33, const Vector& g3, double f) :
    GaussianFactor(j1, j2, j3), info_(matrix_) {
	if(G11.rows() != G11.cols() || G11.rows() != G12.rows() || G11.rows() != G13.rows()  || G11.rows() != g1.size() ||
			G22.cols() != G12.cols() || G33.cols() != G13.cols() ||  G22.cols() != g2.size() || G33.cols() != g3.size())
		throw invalid_argument("Inconsistent matrix and/or vector dimensions in HessianFactor constructor");
	size_t dims[] = { G11.rows(), G22.rows(), G33.rows(), 1 };
	InfoMatrix fullMatrix(G11.rows() + G22.rows() + G33.rows() + 1, G11.rows() + G22.rows() + G33.rows() + 1);
	BlockInfo infoMatrix(fullMatrix, dims, dims+4);
	infoMatrix(0,0) = G11;
	infoMatrix(0,1) = G12;
	infoMatrix(0,2) = G13;
	infoMatrix.column(0,3,0) = g1;
	infoMatrix(1,1) = G22;
	infoMatrix(1,2) = G23;
	infoMatrix.column(1,3,0) = g2;
	infoMatrix(2,2) = G33;
	infoMatrix.column(2,3,0) = g3;
	infoMatrix(3,3)(0,0) = f;
	infoMatrix.swap(info_);
	assertInvariants();
}

/* ************************************************************************* */
HessianFactor::HessianFactor(const std::vector<Index>& js, const std::vector<Matrix>& Gs,
        const std::vector<Vector>& gs, double f) : GaussianFactor(js), info_(matrix_) {

  // Get the number of variables
  size_t variable_count = js.size();

  // Verify the provided number of entries in the vectors are consistent
  if(gs.size() != variable_count || Gs.size() != (variable_count*(variable_count+1))/2)
    throw invalid_argument("Inconsistent number of entries between js, Gs, and gs in HessianFactor constructor.\nThe number of keys provided \
        in js must match the number of linear vector pieces in gs. The number of upper-diagonal blocks in Gs must be n*(n+1)/2");

  // Verify the dimensions of each provided matrix are consistent
  // Note: equations for calculating the indices derived from the "sum of an arithmetic sequence" formula
  for(size_t i = 0; i < variable_count; ++i){
    int block_size = gs[i].size();
    // Check rows
    for(size_t j = 0; j < variable_count-i; ++j){
      size_t index = i*(2*variable_count - i + 1)/2 + j;
      if(Gs[index].rows() != block_size){
        throw invalid_argument("Inconsistent matrix and/or vector dimensions in HessianFactor constructor");
      }
    }
    // Check cols
    for(size_t j = 0; j <= i; ++j){
      size_t index = j*(2*variable_count - j + 1)/2 + (i-j);
      if(Gs[index].cols() != block_size){
        throw invalid_argument("Inconsistent matrix and/or vector dimensions in HessianFactor constructor");
      }
    }
  }

  // Create the dims vector
  size_t* dims = (size_t*)alloca(sizeof(size_t)*(variable_count+1)); // FIXME: alloca is bad, just ask Google.
  size_t total_size = 0;
  for(unsigned int i = 0; i < variable_count; ++i){
    dims[i] = gs[i].size();
    total_size += gs[i].size();
  }
  dims[variable_count] = 1;
  total_size += 1;

  // Fill in the internal matrix with the supplied blocks
  InfoMatrix fullMatrix(total_size, total_size);
  BlockInfo infoMatrix(fullMatrix, dims, dims+variable_count+1);
  size_t index = 0;
  for(size_t i = 0; i < variable_count; ++i){
    for(size_t j = i; j < variable_count; ++j){
      infoMatrix(i,j) = Gs[index++];
    }
    infoMatrix.column(i,variable_count,0) = gs[i];
  }
  infoMatrix(variable_count,variable_count)(0,0) = f;

  // update the BlockView variable
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

	const bool debug = ISDEBUG("EliminateCholesky");
	// Form Ab' * Ab
	tic(1, "allocate");
	info_.resize(dimensions.begin(), dimensions.end(), false);
	// Fill in keys
	keys_.resize(scatter.size());
	std::transform(scatter.begin(), scatter.end(), keys_.begin(), boost::bind(&Scatter::value_type::first, ::_1));
	toc(1, "allocate");
	tic(2, "zero");
	matrix_.noalias() = Matrix::Zero(matrix_.rows(),matrix_.cols());
	toc(2, "zero");
	tic(3, "update");
	if (debug) cout << "Combining " << factors.size() << " factors" << endl;
	BOOST_FOREACH(const GaussianFactor::shared_ptr& factor, factors)
	{
		if(factor) {
			if(shared_ptr hessian = boost::dynamic_pointer_cast<HessianFactor>(factor))
				updateATA(*hessian, scatter);
			else if(JacobianFactor::shared_ptr jacobianFactor = boost::dynamic_pointer_cast<JacobianFactor>(factor))
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
HessianFactor& HessianFactor::operator=(const HessianFactor& rhs) {
  this->Base::operator=(rhs);     // Copy keys
  info_.assignNoalias(rhs.info_); // Copy matrix and block structure
  return *this;
}

/* ************************************************************************* */
void HessianFactor::print(const std::string& s, const IndexFormatter& formatter) const {
	cout << s << "\n";
	cout << " keys: ";
	for(const_iterator key=this->begin(); key!=this->end(); ++key)
		cout << formatter(*key) << "(" << this->getDim(key) << ") ";
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
Matrix HessianFactor::computeInformation() const {
  return info_.full().selfadjointView<Eigen::Upper>();
}

/* ************************************************************************* */
double HessianFactor::error(const VectorValues& c) const {
	// error 0.5*(f - 2*x'*g + x'*G*x)
	const double f = constantTerm();
	const double xtg = c.vector().dot(linearTerm());
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
	size_t* slots = (size_t*)alloca(sizeof(size_t)*update.size()); // FIXME: alloca is bad, just ask Google.
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
	size_t* slots = (size_t*)alloca(sizeof(size_t)*update.size()); // FIXME: alloca is bad, just ask Google.
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

	typedef Eigen::Block<const JacobianFactor::AbMatrix> BlockUpdateMatrix;
	BlockUpdateMatrix updateA(update.matrix_.block(
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
	if(!choleskyPartial(matrix_, info_.offset(nrFrontals)))
		throw IndeterminantLinearSystemException(this->keys().front());
}

/* ************************************************************************* */
GaussianConditional::shared_ptr HessianFactor::splitEliminatedFactor(size_t nrFrontals) {

  static const bool debug = false;

  // Extract conditionals
  tic(1, "extract conditionals");
  GaussianConditional::shared_ptr conditional(new GaussianConditional());
  typedef VerticalBlockView<Matrix> BlockAb;
  BlockAb Ab(matrix_, info_);

  size_t varDim = info_.offset(nrFrontals);
  Ab.rowEnd() = Ab.rowStart() + varDim;

  // Create one big conditionals with many frontal variables.
  tic(2, "construct cond");
  Vector sigmas = Vector::Ones(varDim);
  conditional = boost::make_shared<ConditionalType>(keys_.begin(), keys_.end(), nrFrontals, Ab, sigmas);
  toc(2, "construct cond");
  if(debug) conditional->print("Extracted conditional: ");

  toc(1, "extract conditionals");

  // Take lower-right block of Ab_ to get the new factor
  tic(2, "remaining factor");
  info_.blockStart() = nrFrontals;
  // Assign the keys
  vector<Index> remainingKeys(keys_.size() - nrFrontals);
  remainingKeys.assign(keys_.begin() + nrFrontals, keys_.end());
  keys_.swap(remainingKeys);
  toc(2, "remaining factor");

  return conditional;
}

/* ************************************************************************* */
GaussianFactor::shared_ptr HessianFactor::negate() const {
  // Copy Hessian Blocks from Hessian factor and invert
  std::vector<Index> js;
  std::vector<Matrix> Gs;
  std::vector<Vector> gs;
  double f;
  js.insert(js.end(), begin(), end());
  for(size_t i = 0; i < js.size(); ++i){
    for(size_t j = i; j < js.size(); ++j){
      Gs.push_back( -info(begin()+i, begin()+j) );
    }
    gs.push_back( -linearTerm(begin()+i) );
  }
  f = -constantTerm();

  // Create the Anti-Hessian Factor from the negated blocks
  return HessianFactor::shared_ptr(new HessianFactor(js, Gs, gs, f));
}

} // gtsam
