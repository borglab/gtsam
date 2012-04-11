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

#include <gtsam/base/Testable.h>
#include <gtsam/base/debug.h>
#include <gtsam/base/timing.h>
#include <gtsam/base/cholesky.h>
#include <gtsam/base/FastVector.h>
#include <gtsam/linear/HessianFactor.h>
#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/inference/BayesTree-inl.h>
#include <gtsam/inference/VariableSlots.h>
#include <gtsam/base/debug.h>
#include <gtsam/base/timing.h>
#include <gtsam/base/cholesky.h>

using namespace std;
using namespace gtsam;

namespace gtsam {

	/* ************************************************************************* */
	GaussianFactorGraph::GaussianFactorGraph(const GaussianBayesNet& CBN) : Base(CBN) {}

	/* ************************************************************************* */
	GaussianFactorGraph::Keys GaussianFactorGraph::keys() const {
		FastSet<Index> keys;
		BOOST_FOREACH(const sharedFactor& factor, *this)
		if (factor) keys.insert(factor->begin(), factor->end());
		return keys;
	}

	/* ************************************************************************* */
	void GaussianFactorGraph::permuteWithInverse(
	    const Permutation& inversePermutation) {
	  BOOST_FOREACH(const sharedFactor& factor, factors_) {
	    if(factor)
	      factor->permuteWithInverse(inversePermutation);
    }
	}

	/* ************************************************************************* */
	void GaussianFactorGraph::combine(const GaussianFactorGraph &lfg) {
		for (const_iterator factor = lfg.factors_.begin(); factor
		!= lfg.factors_.end(); factor++) {
			push_back(*factor);
		}
	}

	/* ************************************************************************* */
	GaussianFactorGraph GaussianFactorGraph::combine2(
			const GaussianFactorGraph& lfg1, const GaussianFactorGraph& lfg2) {

		// create new linear factor graph equal to the first one
		GaussianFactorGraph fg = lfg1;

		// add the second factors_ in the graph
		for (const_iterator factor = lfg2.factors_.begin(); factor
		!= lfg2.factors_.end(); factor++) {
			fg.push_back(*factor);
		}
		return fg;
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
				HessianFactor::shared_ptr hessian(boost::dynamic_pointer_cast<HessianFactor>(factor));
				if (hessian)
					jacobianFactor.reset(new JacobianFactor(*hessian));
				else
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
			IJS(0,k) = entry.get<0>() + 1;
			IJS(1,k) = entry.get<1>() + 1;
			IJS(2,k) = entry.get<2>();
		}
		return IJS;
	}

	/* ************************************************************************* */
	Matrix GaussianFactorGraph::denseJacobian() const {
		// combine all factors
		JacobianFactor combined(*CombineJacobians(*convertCastFactors<FactorGraph<
				JacobianFactor> > (), VariableSlots(*this)));
		return combined.matrix_augmented();
	}

	/* ************************************************************************* */
	// Helper functions for Combine
	static boost::tuple<vector<size_t>, size_t, size_t> countDims(const std::vector<JacobianFactor::shared_ptr>& factors, const VariableSlots& variableSlots) {
#ifndef NDEBUG
		vector<size_t> varDims(variableSlots.size(), numeric_limits<size_t>::max());
#else
		vector<size_t> varDims(variableSlots.size());
#endif
		size_t m = 0;
		size_t n = 0;
		{
			Index jointVarpos = 0;
			BOOST_FOREACH(const VariableSlots::value_type& slots, variableSlots) {

				assert(slots.second.size() == factors.size());

				Index sourceFactorI = 0;
				BOOST_FOREACH(const Index sourceVarpos, slots.second) {
					if(sourceVarpos < numeric_limits<Index>::max()) {
						const JacobianFactor& sourceFactor = *factors[sourceFactorI];
						size_t vardim = sourceFactor.getDim(sourceFactor.begin() + sourceVarpos);
#ifndef NDEBUG
if(varDims[jointVarpos] == numeric_limits<size_t>::max()) {
	varDims[jointVarpos] = vardim;
	n += vardim;
} else
	assert(varDims[jointVarpos] == vardim);
#else
	varDims[jointVarpos] = vardim;
n += vardim;
break;
#endif
					}
					++ sourceFactorI;
				}
				++ jointVarpos;
			}
			BOOST_FOREACH(const JacobianFactor::shared_ptr& factor, factors) {
				m += factor->rows();
			}
		}
		return boost::make_tuple(varDims, m, n);
	}

	/* ************************************************************************* */
	JacobianFactor::shared_ptr CombineJacobians(
			const FactorGraph<JacobianFactor>& factors,
			const VariableSlots& variableSlots) {

		const bool debug = ISDEBUG("CombineJacobians");
		if (debug) factors.print("Combining factors: ");
		if (debug) variableSlots.print();

		if (debug) cout << "Determine dimensions" << endl;
		tic(1, "countDims");
		vector<size_t> varDims;
		size_t m, n;
		boost::tie(varDims, m, n) = countDims(factors, variableSlots);
		if (debug) {
			cout << "Dims: " << m << " x " << n << "\n";
			BOOST_FOREACH(const size_t dim, varDims) cout << dim << " ";
			cout << endl;
		}
		toc(1, "countDims");

		if (debug) cout << "Sort rows" << endl;
		tic(2, "sort rows");
		vector<JacobianFactor::_RowSource> rowSources;
		rowSources.reserve(m);
		bool anyConstrained = false;
		for (size_t sourceFactorI = 0; sourceFactorI < factors.size(); ++sourceFactorI) {
			const JacobianFactor& sourceFactor(*factors[sourceFactorI]);
			sourceFactor.collectInfo(sourceFactorI, rowSources);
			if (sourceFactor.isConstrained()) anyConstrained = true;
		}
		assert(rowSources.size() == m);
		std::sort(rowSources.begin(), rowSources.end());
		toc(2, "sort rows");

		if (debug) cout << "Allocate new factor" << endl;
		tic(3, "allocate");
		JacobianFactor::shared_ptr combined(new JacobianFactor());
		combined->allocate(variableSlots, varDims, m);
		Vector sigmas(m);
		toc(3, "allocate");

		if (debug) cout << "Copy rows" << endl;
		tic(4, "copy rows");
		Index combinedSlot = 0;
		BOOST_FOREACH(const VariableSlots::value_type& varslot, variableSlots) {
			for (size_t row = 0; row < m; ++row) {
				const JacobianFactor::_RowSource& info(rowSources[row]);
				const JacobianFactor& source(*factors[info.factorI]);
				size_t sourceRow = info.factorRowI;
				Index sourceSlot = varslot.second[info.factorI];
				combined->copyRow(source, sourceRow, sourceSlot, row, combinedSlot);
			}
			++combinedSlot;
		}
		toc(4, "copy rows");

		if (debug) cout << "Copy rhs (b), sigma, and firstNonzeroBlocks" << endl;
		tic(5, "copy vectors");
		for (size_t row = 0; row < m; ++row) {
			const JacobianFactor::_RowSource& info(rowSources[row]);
			const JacobianFactor& source(*factors[info.factorI]);
			const size_t sourceRow = info.factorRowI;
			combined->getb()(row) = source.getb()(sourceRow);
			sigmas(row) = source.get_model()->sigmas()(sourceRow);
		}
		combined->copyFNZ(m, variableSlots.size(),rowSources);
		toc(5, "copy vectors");

		if (debug) cout << "Create noise model from sigmas" << endl;
		tic(6, "noise model");
		combined->setModel( anyConstrained,sigmas);
		toc(6, "noise model");

		if (debug) cout << "Assert Invariants" << endl;
		combined->assertInvariants();

		return combined;
	}

	/* ************************************************************************* */
	GaussianFactorGraph::EliminationResult EliminateJacobians(const FactorGraph<
			JacobianFactor>& factors, size_t nrFrontals) {
		tic(1, "Combine");
		JacobianFactor::shared_ptr jointFactor =
				CombineJacobians(factors, VariableSlots(factors));
		toc(1, "Combine");
		tic(2, "eliminate");
		GaussianConditional::shared_ptr gbn = jointFactor->eliminate(nrFrontals);
		toc(2, "eliminate");
		return make_pair(gbn, jointFactor);
	}

	/* ************************************************************************* */
	static
	FastMap<Index, SlotEntry> findScatterAndDims
	(const FactorGraph<GaussianFactor>& factors) {

		const bool debug = ISDEBUG("findScatterAndDims");

		// The "scatter" is a map from global variable indices to slot indices in the
		// union of involved variables.  We also include the dimensionality of the
		// variable.

		Scatter scatter;

		// First do the set union.
		BOOST_FOREACH(const GaussianFactor::shared_ptr& factor, factors) {
			for(GaussianFactor::const_iterator variable = factor->begin(); variable != factor->end(); ++variable) {
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

	/* ************************************************************************* */
	Matrix GaussianFactorGraph::denseHessian() const {

		Scatter scatter = findScatterAndDims(*this);

		vector<size_t> dims; dims.reserve(scatter.size() + 1);
		BOOST_FOREACH(const Scatter::value_type& index_entry, scatter) {
			dims.push_back(index_entry.second.dimension);
		}
		dims.push_back(1);

		// combine all factors and get upper-triangular part of Hessian
		HessianFactor combined(*this, dims, scatter);
		Matrix result = combined.info();
		// Fill in lower-triangular part of Hessian
		result.triangularView<Eigen::StrictlyLower>() = result.transpose();
		return result;
	}

	/* ************************************************************************* */
	GaussianFactorGraph::EliminationResult EliminateCholesky(const FactorGraph<
			GaussianFactor>& factors, size_t nrFrontals) {

		const bool debug = ISDEBUG("EliminateCholesky");

		// Find the scatter and variable dimensions
		tic(1, "find scatter");
		Scatter scatter(findScatterAndDims(factors));
		toc(1, "find scatter");

		// Pull out keys and dimensions
		tic(2, "keys");
		vector<size_t> dimensions(scatter.size() + 1);
		BOOST_FOREACH(const Scatter::value_type& var_slot, scatter) {
			dimensions[var_slot.second.slot] = var_slot.second.dimension;
		}
		// This is for the r.h.s. vector
		dimensions.back() = 1;
		toc(2, "keys");

		// Form Ab' * Ab
		tic(3, "combine");
		HessianFactor::shared_ptr //
		combinedFactor(new HessianFactor(factors, dimensions, scatter));
		toc(3, "combine");

		// Do Cholesky, note that after this, the lower triangle still contains
		// some untouched non-zeros that should be zero.  We zero them while
		// extracting submatrices next.
		tic(4, "partial Cholesky");
		try {
			combinedFactor->partialCholesky(nrFrontals);
		} catch
		(std::exception &ex) { // catch exception from Cholesky
			combinedFactor->print("combinedFactor");
			string reason = "EliminateCholesky failed while trying to eliminate the combined factor";
			throw invalid_argument(reason);
		}

		toc(4, "partial Cholesky");

		// Extract conditional and fill in details of the remaining factor
		tic(5, "split");
		GaussianConditional::shared_ptr conditional =
				combinedFactor->splitEliminatedFactor(nrFrontals);
		if (debug) {
			conditional->print("Extracted conditional: ");
			combinedFactor->print("Eliminated factor (L piece): ");
		}
		toc(5, "split");

		combinedFactor->assertInvariants();
		return make_pair(conditional, combinedFactor);
	}

	/* ************************************************************************* */
	static FactorGraph<JacobianFactor> convertToJacobians(const FactorGraph<
			GaussianFactor>& factors) {

		typedef JacobianFactor J;
		typedef HessianFactor H;

		const bool debug = ISDEBUG("convertToJacobians");

		FactorGraph<J> jacobians;
		jacobians.reserve(factors.size());
		BOOST_FOREACH(const GaussianFactor::shared_ptr& factor, factors)
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
					if (!assert_equal(*hessian, HessianFactor(*converted), 1e-3)) throw runtime_error(
							"convertToJacobians: Conversion between Jacobian and Hessian incorrect");
				}
				jacobians.push_back(converted);
			}
		}
		return jacobians;
	}

	/* ************************************************************************* */
	GaussianFactorGraph::EliminationResult EliminateQR(const FactorGraph<
			GaussianFactor>& factors, size_t nrFrontals) {

		const bool debug = ISDEBUG("EliminateQR");

		// Convert all factors to the appropriate type and call the type-specific EliminateGaussian.
		if (debug) cout << "Using QR" << endl;

		tic(1, "convert to Jacobian");
		FactorGraph<JacobianFactor> jacobians = convertToJacobians(factors);
		toc(1, "convert to Jacobian");

		tic(2, "Jacobian EliminateGaussian");
		GaussianConditional::shared_ptr conditional;
		GaussianFactor::shared_ptr factor;
		boost::tie(conditional, factor) = EliminateJacobians(jacobians, nrFrontals);
		toc(2, "Jacobian EliminateGaussian");

		return make_pair(conditional, factor);
	} // \EliminateQR

	/* ************************************************************************* */
	bool hasConstraints(const FactorGraph<GaussianFactor>& factors) {
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
	GaussianFactorGraph::EliminationResult EliminatePreferCholesky(
			const FactorGraph<GaussianFactor>& factors, size_t nrFrontals) {

		typedef JacobianFactor J;
		typedef HessianFactor H;

		// If any JacobianFactors have constrained noise models, we have to convert
		// all factors to JacobianFactors.  Otherwise, we can convert all factors
		// to HessianFactors.  This is because QR can handle constrained noise
		// models but Cholesky cannot.
		if (hasConstraints(factors))
			return EliminateQR(factors, nrFrontals);
		else {
			GaussianFactorGraph::EliminationResult ret;
#ifdef NDEBUG
			static const bool diag = false;
#else
			static const bool diag = !ISDEBUG("NoCholeskyDiagnostics");
#endif
			if (!diag) {
				tic(2, "EliminateCholesky");
				ret = EliminateCholesky(factors, nrFrontals);
				toc(2, "EliminateCholesky");
			} else {
				try {
					tic(2, "EliminateCholesky");
					ret = EliminateCholesky(factors, nrFrontals);
					toc(2, "EliminateCholesky");
				} catch (const exception& e) {
					cout << "Exception in EliminateCholesky: " << e.what() << endl;
					SETDEBUG("EliminateCholesky", true);
					SETDEBUG("updateATA", true);
					SETDEBUG("JacobianFactor::eliminate", true);
					SETDEBUG("JacobianFactor::Combine", true);
					SETDEBUG("choleskyPartial", true);
					factors.print("Combining factors: ");
					EliminateCholesky(factors, nrFrontals);
					throw;
				}
			}

			const bool checkCholesky = ISDEBUG("EliminateGaussian Check Cholesky");
			if (checkCholesky) {
				GaussianFactorGraph::EliminationResult expected;
				FactorGraph<J> jacobians = convertToJacobians(factors);
				try {
					// Compare with QR
					expected = EliminateJacobians(jacobians, nrFrontals);
				} catch (...) {
					cout << "Exception in QR" << endl;
					throw;
				}

				H actual_factor(*ret.second);
				H expected_factor(*expected.second);
				if (!assert_equal(*expected.first, *ret.first, 100.0)
						|| !assert_equal(expected_factor, actual_factor, 1.0)) {
					cout << "Cholesky and QR do not agree" << endl;

					SETDEBUG("EliminateCholesky", true);
					SETDEBUG("updateATA", true);
					SETDEBUG("JacobianFactor::eliminate", true);
					SETDEBUG("JacobianFactor::Combine", true);
					jacobians.print("Jacobian Factors: ");
					EliminateJacobians(jacobians, nrFrontals);
					EliminateCholesky(factors, nrFrontals);
					factors.print("Combining factors: ");

					throw runtime_error("Cholesky and QR do not agree");
				}
			}

			return ret;
		}

	} // \EliminatePreferCholesky

	/* ************************************************************************* */
	GaussianFactorGraph::EliminationResult EliminateLDL(
			const FactorGraph<GaussianFactor>& factors, size_t nrFrontals) {
		const bool debug = ISDEBUG("EliminateLDL");

		// Find the scatter and variable dimensions
		tic(1, "find scatter");
		Scatter scatter(findScatterAndDims(factors));
		toc(1, "find scatter");

		// Pull out keys and dimensions
		tic(2, "keys");
		vector<size_t> dimensions(scatter.size() + 1);
		BOOST_FOREACH(const Scatter::value_type& var_slot, scatter) {
			dimensions[var_slot.second.slot] = var_slot.second.dimension;
		}
		// This is for the r.h.s. vector
		dimensions.back() = 1;
		toc(2, "keys");

		// Form Ab' * Ab
		tic(3, "combine");

		if (debug) {
			// print out everything before combine
			factors.print("Factors to be combined into hessian");
			cout << "Dimensions (" << dimensions.size() << "): ";
			BOOST_FOREACH(size_t d, dimensions) cout << d << " ";
			cout << "\nScatter:" << endl;
			BOOST_FOREACH(const Scatter::value_type& p, scatter)
			cout << "   Index: " << p.first << ", " << p.second.toString() << endl;
		}

		HessianFactor::shared_ptr //
		combinedFactor(new HessianFactor(factors, dimensions, scatter));
		toc(3, "combine");

		// Do LDL, note that after this, the lower triangle still contains
		// some untouched non-zeros that should be zero.  We zero them while
		// extracting submatrices next.
		tic(4, "partial LDL");
		Eigen::LDLT<Matrix>::TranspositionType permutation = combinedFactor->partialLDL(nrFrontals);
		toc(4, "partial LDL");

		// Extract conditional and fill in details of the remaining factor
		tic(5, "split");
		GaussianConditional::shared_ptr conditional =
				combinedFactor->splitEliminatedFactor(nrFrontals, permutation);
		if (debug) {
			conditional->print("Extracted conditional: ");
			combinedFactor->print("Eliminated factor (L piece): ");
		}
		toc(5, "split");

		combinedFactor->assertInvariants();
		return make_pair(conditional, combinedFactor);
	} // \EliminateLDL

	/* ************************************************************************* */
	GaussianFactorGraph::EliminationResult EliminatePreferLDL(
			const FactorGraph<GaussianFactor>& factors, size_t nrFrontals) {

		typedef JacobianFactor J;
		typedef HessianFactor H;

		// If any JacobianFactors have constrained noise models, we have to convert
		// all factors to JacobianFactors.  Otherwise, we can convert all factors
		// to HessianFactors.  This is because QR can handle constrained noise
		// models but LDL cannot.

		// Decide whether to use QR or LDL
		// Check if any JacobianFactors have constrained noise models.
		if (hasConstraints(factors))
			return EliminateQR(factors, nrFrontals);
		else {
			GaussianFactorGraph::EliminationResult ret;
#ifdef NDEBUG
			static const bool diag = false;
#else
			static const bool diag = !ISDEBUG("NoLDLDiagnostics");
#endif
			if(!diag) {
				tic(2, "EliminateLDL");
				ret = EliminateLDL(factors, nrFrontals);
				toc(2, "EliminateLDL");
			} else {
				try {
					tic(2, "EliminateLDL");
					ret = EliminateLDL(factors, nrFrontals);
					toc(2, "EliminateLDL");
				} catch (const NegativeMatrixException& e) {
					throw;
				} catch (const exception& e) {
					cout << "Exception in EliminateLDL: " << e.what() << endl;
					SETDEBUG("EliminateLDL", true);
					SETDEBUG("updateATA", true);
					SETDEBUG("JacobianFactor::eliminate", true);
					SETDEBUG("JacobianFactor::Combine", true);
					SETDEBUG("ldlPartial", true);
					SETDEBUG("findScatterAndDims", true);
					factors.print("Combining factors: ");
					EliminateLDL(factors, nrFrontals);
					throw;
				}
			}

			const bool checkLDL = ISDEBUG("EliminateGaussian Check LDL");
			if (checkLDL) {
				GaussianFactorGraph::EliminationResult expected;
				FactorGraph<J> jacobians = convertToJacobians(factors);
				try {
					// Compare with QR
					expected = EliminateJacobians(jacobians, nrFrontals);
				} catch (...) {
					cout << "Exception in QR" << endl;
					throw;
				}

				H actual_factor(*ret.second);
				H expected_factor(*expected.second);
				if (!assert_equal(*expected.first, *ret.first, 100.0) || !assert_equal(
						expected_factor, actual_factor, 1.0)) {
					cout << "LDL and QR do not agree" << endl;

					SETDEBUG("EliminateLDL", true);
					SETDEBUG("updateATA", true);
					SETDEBUG("JacobianFactor::eliminate", true);
					SETDEBUG("JacobianFactor::Combine", true);
					jacobians.print("Jacobian Factors: ");
					EliminateJacobians(jacobians, nrFrontals);
					EliminateLDL(factors, nrFrontals);
					factors.print("Combining factors: ");

					throw runtime_error("LDL and QR do not agree");
				}
			}

			return ret;
		}
	} // \EliminatePreferLDL

} // namespace gtsam
