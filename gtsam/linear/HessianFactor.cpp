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

#include <gtsam/base/debug.h>
#include <gtsam/base/timing.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/FastMap.h>
#include <gtsam/base/cholesky.h>
#include <gtsam/linear/linearExceptions.h>
#include <gtsam/linear/GaussianConditional.h>
#include <gtsam/linear/GaussianFactor.h>
#include <gtsam/linear/HessianFactor.h>
#include <gtsam/linear/GaussianFactorGraph.h>

#include <boost/foreach.hpp>
#include <boost/format.hpp>
#include <boost/make_shared.hpp>
#include <boost/tuple/tuple.hpp>
#ifdef __GNUC__
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-variable"
#endif
#include <boost/bind.hpp>
#ifdef __GNUC__
#pragma GCC diagnostic pop
#endif
#include <boost/assign/list_of.hpp>
#include <boost/range/adaptor/transformed.hpp>
#include <boost/range/join.hpp>

#include <sstream>

using namespace std;
using namespace boost::assign;
using boost::adaptors::transformed;
namespace br = boost::range;

namespace gtsam {

/* ************************************************************************* */
string SlotEntry::toString() const {
  ostringstream oss;
  oss << "SlotEntry: slot=" << slot << ", dim=" << dimension;
  return oss.str();
}

/* ************************************************************************* */
Scatter::Scatter(const GaussianFactorGraph& gfg) {
  // First do the set union.
  BOOST_FOREACH(const GaussianFactor::shared_ptr& factor, gfg) {
    if(factor) {
      for(GaussianFactor::const_iterator variable = factor->begin(); variable != factor->end(); ++variable) {
        this->insert(make_pair(*variable, SlotEntry(0, factor->getDim(variable))));
      }
    }
  }

  // Next fill in the slot indices (we can only get these after doing the set
  // union.
  size_t slot = 0;
  BOOST_FOREACH(value_type& var_slot, *this) {
    var_slot.second.slot = (slot ++);
  }
}

/* ************************************************************************* */
HessianFactor::HessianFactor() :
  info_(cref_list_of<1>(1))
{
  linearTerm().setZero();
  constantTerm() = 0.0;
}

/* ************************************************************************* */
HessianFactor::HessianFactor(Key j, const Matrix& G, const Vector& g, double f) :
  GaussianFactor(cref_list_of<1>(j)), info_(cref_list_of<2>(G.cols())(1))
{
  if(G.rows() != G.cols() || G.rows() != g.size()) throw invalid_argument(
    "Attempting to construct HessianFactor with inconsistent matrix and/or vector dimensions");
  info_(0,0) = G;
  info_(0,1) = g;
  info_(1,1)(0,0) = f;
}

/* ************************************************************************* */
// error is 0.5*(x-mu)'*inv(Sigma)*(x-mu) = 0.5*(x'*G*x - 2*x'*G*mu + mu'*G*mu)
// where G = inv(Sigma), g = G*mu, f = mu'*G*mu = mu'*g
HessianFactor::HessianFactor(Key j, const Vector& mu, const Matrix& Sigma) :
    GaussianFactor(cref_list_of<1>(j)),
    info_(cref_list_of<2> (Sigma.cols()) (1) )
{
  if (Sigma.rows() != Sigma.cols() || Sigma.rows() != mu.size()) throw invalid_argument(
    "Attempting to construct HessianFactor with inconsistent matrix and/or vector dimensions");
  info_(0,0) = Sigma.inverse(); // G
  info_(0,1) = info_(0,0) * mu; // g
  info_(1,1)(0,0) = mu.dot(info_(0,1).col(0)); // f
}

/* ************************************************************************* */
HessianFactor::HessianFactor(Key j1, Key j2,
                             const Matrix& G11, const Matrix& G12, const Vector& g1,
                             const Matrix& G22, const Vector& g2, double f) :
GaussianFactor(cref_list_of<2>(j1)(j2)),
  info_(cref_list_of<3> (G11.cols()) (G22.cols()) (1) )
{
  info_(0,0) = G11;
  info_(0,1) = G12;
  info_(0,2) = g1;
  info_(1,1) = G22;
  info_(1,2) = g2;
  info_(2,2)(0,0) = f;
}

/* ************************************************************************* */
HessianFactor::HessianFactor(Key j1, Key j2, Key j3,
                             const Matrix& G11, const Matrix& G12, const Matrix& G13, const Vector& g1,
                             const Matrix& G22, const Matrix& G23, const Vector& g2,
                             const Matrix& G33, const Vector& g3, double f) :
GaussianFactor(cref_list_of<3>(j1)(j2)(j3)),
  info_(cref_list_of<4> (G11.cols()) (G22.cols()) (G33.cols()) (1) )
{
  if(G11.rows() != G11.cols() || G11.rows() != G12.rows() || G11.rows() != G13.rows()  || G11.rows() != g1.size() ||
    G22.cols() != G12.cols() || G33.cols() != G13.cols() ||  G22.cols() != g2.size() || G33.cols() != g3.size())
    throw invalid_argument("Inconsistent matrix and/or vector dimensions in HessianFactor constructor");
  info_(0,0) = G11;
  info_(0,1) = G12;
  info_(0,2) = G13;
  info_(0,3) = g1;
  info_(1,1) = G22;
  info_(1,2) = G23;
  info_(1,3) = g2;
  info_(2,2) = G33;
  info_(2,3) = g3;
  info_(3,3)(0,0) = f;
}

/* ************************************************************************* */
namespace { DenseIndex _getColsHF(const Matrix& m) { return m.cols(); } }

/* ************************************************************************* */
HessianFactor::HessianFactor(const std::vector<Key>& js, const std::vector<Matrix>& Gs,
        const std::vector<Vector>& gs, double f) :
GaussianFactor(js), info_(br::join(Gs | transformed(&_getColsHF), cref_list_of<1,DenseIndex>(1)))
{
  // Get the number of variables
  size_t variable_count = js.size();

  // Verify the provided number of entries in the vectors are consistent
  if(gs.size() != variable_count || Gs.size() != (variable_count*(variable_count+1))/2)
    throw invalid_argument("Inconsistent number of entries between js, Gs, and gs in HessianFactor constructor.\nThe number of keys provided \
        in js must match the number of linear vector pieces in gs. The number of upper-diagonal blocks in Gs must be n*(n+1)/2");

  // Verify the dimensions of each provided matrix are consistent
  // Note: equations for calculating the indices derived from the "sum of an arithmetic sequence" formula
  for(size_t i = 0; i < variable_count; ++i){
    DenseIndex block_size = gs[i].size();
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

  // Fill in the blocks
  size_t index = 0;
  for(size_t i = 0; i < variable_count; ++i){
    for(size_t j = i; j < variable_count; ++j){
      info_(i, j) = Gs[index++];
    }
    info_(i, variable_count) = gs[i];
  }
  info_(variable_count, variable_count)(0,0) = f;
}

/* ************************************************************************* */
HessianFactor::HessianFactor(const JacobianFactor& jf)
{
  throw std::runtime_error("Not implemented");
}

/* ************************************************************************* */
HessianFactor::HessianFactor(const GaussianFactor& gf)
{
  throw std::runtime_error("Not implemented");
  //// Copy the variable indices
  //(GaussianFactorOrdered&)(*this) = gf;
  //// Copy the matrix data depending on what type of factor we're copying from
  //if(dynamic_cast<const JacobianFactorOrdered*>(&gf)) {
  //  const JacobianFactorOrdered& jf(static_cast<const JacobianFactorOrdered&>(gf));
  //  if(jf.model_->isConstrained())
  //    throw invalid_argument("Cannot construct HessianFactor from JacobianFactor with constrained noise model");
  //  else {
  //    Vector invsigmas = jf.model_->invsigmas().cwiseProduct(jf.model_->invsigmas());
  //    info_.copyStructureFrom(jf.Ab_);
  //    BlockInfo::constBlock A = jf.Ab_.full();
  //    matrix_.noalias() = A.transpose() * invsigmas.asDiagonal() * A;
  //  }
  //} else if(dynamic_cast<const HessianFactorOrdered*>(&gf)) {
  //  const HessianFactorOrdered& hf(static_cast<const HessianFactorOrdered&>(gf));
  //  info_.assignNoalias(hf.info_);
  //} else
  //  throw std::invalid_argument("In HessianFactor(const GaussianFactor& gf), gf is neither a JacobianFactor nor a HessianFactor");
  //assertInvariants();
}

/* ************************************************************************* */
HessianFactor::HessianFactor(const GaussianFactorGraph& factors)
{
  throw std::runtime_error("Not implemented");

  //Scatter scatter(factors);

  //// Pull out keys and dimensions
  //gttic(keys);
  //vector<size_t> dimensions(scatter.size() + 1);
  //BOOST_FOREACH(const Scatter::value_type& var_slot, scatter) {
  //  dimensions[var_slot.second.slot] = var_slot.second.dimension;
  //}
  //// This is for the r.h.s. vector
  //dimensions.back() = 1;
  //gttoc(keys);

  //const bool debug = ISDEBUG("EliminateCholesky");
  //// Form Ab' * Ab
  //gttic(allocate);
  //info_.resize(dimensions.begin(), dimensions.end(), false);
  //// Fill in keys
  //keys_.resize(scatter.size());
  //std::transform(scatter.begin(), scatter.end(), keys_.begin(), boost::bind(&Scatter::value_type::first, ::_1));
  //gttoc(allocate);
  //gttic(zero);
  //matrix_.noalias() = Matrix::Zero(matrix_.rows(),matrix_.cols());
  //gttoc(zero);
  //gttic(update);
  //if (debug) cout << "Combining " << factors.size() << " factors" << endl;
  //BOOST_FOREACH(const GaussianFactorOrdered::shared_ptr& factor, factors)
  //{
  //  if(factor) {
  //    if(shared_ptr hessian = boost::dynamic_pointer_cast<HessianFactorOrdered>(factor))
  //      updateATA(*hessian, scatter);
  //    else if(JacobianFactorOrdered::shared_ptr jacobianFactor = boost::dynamic_pointer_cast<JacobianFactorOrdered>(factor))
  //      updateATA(*jacobianFactor, scatter);
  //    else
  //      throw invalid_argument("GaussianFactor is neither Hessian nor Jacobian");
  //  }
  //}
  //gttoc(update);

  //if (debug) gtsam::print(matrix_, "Ab' * Ab: ");

  //assertInvariants();
}

/* ************************************************************************* */
void HessianFactor::print(const std::string& s, const KeyFormatter& formatter) const {
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
Matrix HessianFactor::augmentedInformation() const {
  return info_.full().selfadjointView<Eigen::Upper>();
}

/* ************************************************************************* */
Matrix HessianFactor::information() const {
  return info_.range(0, this->size(), 0, this->size()).selfadjointView<Eigen::Upper>();
}

/* ************************************************************************* */
Matrix HessianFactor::augmentedJacobian() const
{
  throw std::runtime_error("Not implemented");
}

/* ************************************************************************* */
std::pair<Matrix, Vector> HessianFactor::jacobian() const
{
  throw std::runtime_error("Not implemented");
}

/* ************************************************************************* */
double HessianFactor::error(const VectorValues& c) const {
  // error 0.5*(f - 2*x'*g + x'*G*x)
  const double f = constantTerm();
  double xtg = 0, xGx = 0;
  // extract the relevant subset of the VectorValues
  // NOTE may not be as efficient
  const Vector x = c.vector(this->keys());
  xtg = x.dot(linearTerm());
  xGx = x.transpose() * info_.range(0, this->size(), 0, this->size()).selfadjointView<Eigen::Upper>() *  x;
  return 0.5 * (f - 2.0 * xtg +  xGx);
}

/* ************************************************************************* */
void HessianFactor::updateATA(const HessianFactor& update, const Scatter& scatter)
{

  throw std::runtime_error("Not implemented");

  //// This function updates 'combined' with the information in 'update'.
  //// 'scatter' maps variables in the update factor to slots in the combined
  //// factor.

  //const bool debug = ISDEBUG("updateATA");

  //// First build an array of slots
  //gttic(slots);
  //size_t* slots = (size_t*)alloca(sizeof(size_t)*update.size()); // FIXME: alloca is bad, just ask Google.
  //size_t slot = 0;
  //BOOST_FOREACH(Index j, update) {
  //  slots[slot] = scatter.find(j)->second.slot;
  //  ++ slot;
  //}
  //gttoc(slots);

  //if(debug) {
  //  this->print("Updating this: ");
  //  update.print("with (Hessian): ");
  //}

  //// Apply updates to the upper triangle
  //gttic(update);
  //for(size_t j2=0; j2<update.info_.nBlocks(); ++j2) {
  //  size_t slot2 = (j2 == update.size()) ? this->info_.nBlocks()-1 : slots[j2];
  //  for(size_t j1=0; j1<=j2; ++j1) {
  //    size_t slot1 = (j1 == update.size()) ? this->info_.nBlocks()-1 : slots[j1];
  //    if(slot2 > slot1) {
  //      if(debug)
  //        cout << "Updating (" << slot1 << "," << slot2 << ") from (" << j1 << "," << j2 << ")" << endl;
  //      matrix_.block(info_.offset(slot1), info_.offset(slot2), info_(slot1,slot2).rows(), info_(slot1,slot2).cols()).noalias() +=
  //          update.matrix_.block(update.info_.offset(j1), update.info_.offset(j2), update.info_(j1,j2).rows(), update.info_(j1,j2).cols());
  //    } else if(slot1 > slot2) {
  //      if(debug)
  //        cout << "Updating (" << slot2 << "," << slot1 << ") from (" << j1 << "," << j2 << ")" << endl;
  //      matrix_.block(info_.offset(slot2), info_.offset(slot1), info_(slot2,slot1).rows(), info_(slot2,slot1).cols()).noalias() +=
  //          update.matrix_.block(update.info_.offset(j1), update.info_.offset(j2), update.info_(j1,j2).rows(), update.info_(j1,j2).cols()).transpose();
  //    } else {
  //      if(debug)
  //        cout << "Updating (" << slot1 << "," << slot2 << ") from (" << j1 << "," << j2 << ")" << endl;
  //      matrix_.block(info_.offset(slot1), info_.offset(slot2), info_(slot1,slot2).rows(), info_(slot1,slot2).cols()).triangularView<Eigen::Upper>() +=
  //          update.matrix_.block(update.info_.offset(j1), update.info_.offset(j2), update.info_(j1,j2).rows(), update.info_(j1,j2).cols());
  //    }
  //    if(debug) cout << "Updating block " << slot1 << "," << slot2 << " from block " << j1 << "," << j2 << "\n";
  //    if(debug) this->print();
  //  }
  //}
  //gttoc(update);
}

/* ************************************************************************* */
void HessianFactor::updateATA(const JacobianFactor& update, const Scatter& scatter)
{
  throw std::runtime_error("Not implemented");

  //// This function updates 'combined' with the information in 'update'.
  //// 'scatter' maps variables in the update factor to slots in the combined
  //// factor.

  //const bool debug = ISDEBUG("updateATA");

  //// First build an array of slots
  //gttic(slots);
  //size_t* slots = (size_t*)alloca(sizeof(size_t)*update.size()); // FIXME: alloca is bad, just ask Google.
  //size_t slot = 0;
  //BOOST_FOREACH(Index j, update) {
  //  slots[slot] = scatter.find(j)->second.slot;
  //  ++ slot;
  //}
  //gttoc(slots);

  //gttic(form_ATA);
  //if(update.model_->isConstrained())
  //  throw invalid_argument("Cannot update HessianFactor from JacobianFactor with constrained noise model");

  //if(debug) {
  //  this->print("Updating this: ");
  //  update.print("with (Jacobian): ");
  //}

  //typedef Eigen::Block<const JacobianFactorOrdered::AbMatrix> BlockUpdateMatrix;
  //BlockUpdateMatrix updateA(update.matrix_.block(
  //    update.Ab_.rowStart(),update.Ab_.offset(0), update.Ab_.full().rows(), update.Ab_.full().cols()));
  //if (debug) cout << "updateA: \n" << updateA << endl;

  //Matrix updateInform;
  //if(boost::dynamic_pointer_cast<noiseModel::Unit>(update.model_)) {
  //  updateInform.noalias() = updateA.transpose() * updateA;
  //} else {
  //  noiseModel::Diagonal::shared_ptr diagonal(boost::dynamic_pointer_cast<noiseModel::Diagonal>(update.model_));
  //  if(diagonal) {
  //    Vector invsigmas2 = update.model_->invsigmas().cwiseProduct(update.model_->invsigmas());
  //    updateInform.noalias() = updateA.transpose() * invsigmas2.asDiagonal() * updateA;
  //  } else
  //    throw invalid_argument("In HessianFactor::updateATA, JacobianFactor noise model is neither Unit nor Diagonal");
  //}
  //if (debug) cout << "updateInform: \n" << updateInform << endl;
  // gttoc(form_ATA);

  //// Apply updates to the upper triangle
  //gttic(update);
  //for(size_t j2=0; j2<update.Ab_.nBlocks(); ++j2) {
  //  size_t slot2 = (j2 == update.size()) ? this->info_.nBlocks()-1 : slots[j2];
  //  for(size_t j1=0; j1<=j2; ++j1) {
  //    size_t slot1 = (j1 == update.size()) ? this->info_.nBlocks()-1 : slots[j1];
  //    size_t off0 = update.Ab_.offset(0);
  //    if(slot2 > slot1) {
  //      if(debug)
  //        cout << "Updating (" << slot1 << "," << slot2 << ") from (" << j1 << "," << j2 << ")" << endl;
  //      matrix_.block(info_.offset(slot1), info_.offset(slot2), info_(slot1,slot2).rows(), info_(slot1,slot2).cols()).noalias() +=
  //          updateInform.block(update.Ab_.offset(j1)-off0, update.Ab_.offset(j2)-off0, update.Ab_(j1).cols(), update.Ab_(j2).cols());
  //    } else if(slot1 > slot2) {
  //      if(debug)
  //        cout << "Updating (" << slot2 << "," << slot1 << ") from (" << j1 << "," << j2 << ")" << endl;
  //      matrix_.block(info_.offset(slot2), info_.offset(slot1), info_(slot2,slot1).rows(), info_(slot2,slot1).cols()).noalias() +=
  //          updateInform.block(update.Ab_.offset(j1)-off0, update.Ab_.offset(j2)-off0, update.Ab_(j1).cols(), update.Ab_(j2).cols()).transpose();
  //    } else {
  //      if(debug)
  //        cout << "Updating (" << slot1 << "," << slot2 << ") from (" << j1 << "," << j2 << ")" << endl;
  //      matrix_.block(info_.offset(slot1), info_.offset(slot2), info_(slot1,slot2).rows(), info_(slot1,slot2).cols()).triangularView<Eigen::Upper>() +=
  //          updateInform.block(update.Ab_.offset(j1)-off0, update.Ab_.offset(j2)-off0, update.Ab_(j1).cols(), update.Ab_(j2).cols());
  //    }
  //    if(debug) cout << "Updating block " << slot1 << "," << slot2 << " from block " << j1 << "," << j2 << "\n";
  //    if(debug) this->print();
  //  }
  //}
  //gttoc(update);
}

/* ************************************************************************* */
void HessianFactor::partialCholesky(size_t nrFrontals)
{
  throw std::runtime_error("Not implemented");

  //if(!choleskyPartial(matrix_, info_.offset(nrFrontals)))
  //  throw IndeterminantLinearSystemException(this->keys().front());
}

/* ************************************************************************* */
GaussianConditional::shared_ptr HessianFactor::splitEliminatedFactor(size_t nrFrontals)
{
  throw std::runtime_error("Not implemented");

  //static const bool debug = false;

  //// Extract conditionals
  //gttic(extract_conditionals);
  //GaussianConditionalOrdered::shared_ptr conditional(new GaussianConditionalOrdered());
  //typedef VerticalBlockView<Matrix> BlockAb;
  //BlockAb Ab(matrix_, info_);

  //size_t varDim = info_.offset(nrFrontals);
  //Ab.rowEnd() = Ab.rowStart() + varDim;

  //// Create one big conditionals with many frontal variables.
  //gttic(construct_cond);
  //Vector sigmas = Vector::Ones(varDim);
  //conditional = boost::make_shared<ConditionalType>(keys_.begin(), keys_.end(), nrFrontals, Ab, sigmas);
  //gttoc(construct_cond);
  //if(debug) conditional->print("Extracted conditional: ");

  //gttoc(extract_conditionals);

  //// Take lower-right block of Ab_ to get the new factor
  //gttic(remaining_factor);
  //info_.blockStart() = nrFrontals;
  //// Assign the keys
  //vector<Index> remainingKeys(keys_.size() - nrFrontals);
  //remainingKeys.assign(keys_.begin() + nrFrontals, keys_.end());
  //keys_.swap(remainingKeys);
  //gttoc(remaining_factor);

  //return conditional;
}

/* ************************************************************************* */
GaussianFactor::shared_ptr HessianFactor::negate() const
{
  throw std::runtime_error("Not implemented");

  //// Copy Hessian Blocks from Hessian factor and invert
  //std::vector<Index> js;
  //std::vector<Matrix> Gs;
  //std::vector<Vector> gs;
  //double f;
  //js.insert(js.end(), begin(), end());
  //for(size_t i = 0; i < js.size(); ++i){
  //  for(size_t j = i; j < js.size(); ++j){
  //    Gs.push_back( -info(begin()+i, begin()+j) );
  //  }
  //  gs.push_back( -linearTerm(begin()+i) );
  //}
  //f = -constantTerm();

  //// Create the Anti-Hessian Factor from the negated blocks
  //return HessianFactorOrdered::shared_ptr(new HessianFactorOrdered(js, Gs, gs, f));
}

} // gtsam
