/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 *  @file DiscreteConditional.cpp
 *  @date Feb 14, 2011
 *  @author Duy-Nguyen Ta
 *  @author Frank Dellaert
 */

#include <gtsam/discrete/DiscreteConditional.h>
#include <gtsam/discrete/Signature.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/debug.h>

#include <boost/make_shared.hpp>
#include <boost/random/mersenne_twister.hpp>
#include <boost/random/uniform_real.hpp>
#include <boost/random/variate_generator.hpp>

#include <vector>
#include <algorithm>
#include <stdexcept>

using namespace std;

namespace gtsam {

	/* ******************************************************************************** */
	DiscreteConditional::DiscreteConditional(const size_t nrFrontals,
			const DecisionTreeFactor& f) :
			IndexConditional(f.keys(), nrFrontals), Potentials(
					f / (*f.sum(nrFrontals))) {
	}

	/* ******************************************************************************** */
	DiscreteConditional::DiscreteConditional(const DecisionTreeFactor& joint,
			const DecisionTreeFactor& marginal) :
			IndexConditional(joint.keys(), joint.size() - marginal.size()), Potentials(
					ISDEBUG("DiscreteConditional::COUNT") ? joint : joint / marginal) {
//		assert(nrFrontals() == 1);
		if (ISDEBUG("DiscreteConditional::DiscreteConditional")) cout
				<< (firstFrontalKey()) << endl;	//TODO Print all keys
	}

	/* ******************************************************************************** */
	DiscreteConditional::DiscreteConditional(const Signature& signature) :
			IndexConditional(signature.indices(), 1), Potentials(
					signature.discreteKeysParentsFirst(), signature.cpt()) {
	}

	/* ******************************************************************************** */
	Potentials::ADT DiscreteConditional::choose(
			const Values& parentsValues) const {
		ADT pFS(*this);
		BOOST_FOREACH(Index key, parents())
			try {
				Index j = (key);
				size_t value = parentsValues.at(j);
				pFS = pFS.choose(j, value);
			} catch (exception&) {
				throw runtime_error(
						"DiscreteConditional::choose: parent value missing");
			};
		return pFS;
	}

	/* ******************************************************************************** */
	void DiscreteConditional::solveInPlace(Values& values) const {
//		OLD
//		assert(nrFrontals() == 1);
//		Index j = (firstFrontalKey());
//		size_t mpe = solve(values); // Solve for variable
//		values[j] = mpe; // store result in partial solution
//		OLD

		// TODO: is this really the fastest way? I think it is.

		//The following is to make make adjustment for nFrontals \neq 1
		ADT pFS = choose(values); // P(F|S=parentsValues)

		// Initialize
		Values mpe;
		double maxP = 0;

		DiscreteKeys keys;
		BOOST_FOREACH(Index idx, frontals()) {
			DiscreteKey dk(idx,cardinality(idx));
			keys & dk;
		}
		// Get all Possible Configurations
		vector<Values> allPosbValues = cartesianProduct(keys);

		// Find the MPE
		BOOST_FOREACH(Values& frontalVals, allPosbValues) {
			double pValueS = pFS(frontalVals); // P(F=value|S=parentsValues)
			// Update MPE solution if better
			if (pValueS > maxP) {
				maxP = pValueS;
				mpe = frontalVals;
			}
		}

		//set values (inPlace) to mpe
		BOOST_FOREACH(Index j, frontals()) {
			values[j] = mpe[j];
		}
	}

	/* ******************************************************************************** */
	void DiscreteConditional::sampleInPlace(Values& values) const {
		assert(nrFrontals() == 1);
		Index j = (firstFrontalKey());
		size_t sampled = sample(values); // Sample variable
		values[j] = sampled; // store result in partial solution
	}

	/* ******************************************************************************** */
	size_t DiscreteConditional::solve(const Values& parentsValues) const {

		// TODO: is this really the fastest way? I think it is.
		ADT pFS = choose(parentsValues); // P(F|S=parentsValues)

		// Then, find the max over all remaining
		// TODO, only works for one key now, seems horribly slow this way
		size_t mpe = 0;
		Values frontals;
		double maxP = 0;
		assert(nrFrontals() == 1);
		Index j = (firstFrontalKey());
		for (size_t value = 0; value < cardinality(j); value++) {
			frontals[j] = value;
			double pValueS = pFS(frontals); // P(F=value|S=parentsValues)
			// Update MPE solution if better
			if (pValueS > maxP) {
				maxP = pValueS;
				mpe = value;
			}
		}
		return mpe;
	}

	/* ******************************************************************************** */
	size_t DiscreteConditional::sample(const Values& parentsValues) const {

		using boost::uniform_real;
		static boost::mt19937 gen(2); // random number generator

		bool debug = ISDEBUG("DiscreteConditional::sample");

		// Get the correct conditional density
		ADT pFS = choose(parentsValues); // P(F|S=parentsValues)
		if (debug) GTSAM_PRINT(pFS);

		// get cumulative distribution function (cdf)
		// TODO, only works for one key now, seems horribly slow this way
		assert(nrFrontals() == 1);
		Index j = (firstFrontalKey());
		size_t nj = cardinality(j);
		vector<double> cdf(nj);
		Values frontals;
		double sum = 0;
		for (size_t value = 0; value < nj; value++) {
			frontals[j] = value;
			double pValueS = pFS(frontals); // P(F=value|S=parentsValues)
			sum += pValueS; // accumulate
			if (debug) cout << sum << " ";
			if (pValueS == 1) {
				if (debug) cout << "--> " << value << endl;
				return value; // shortcut exit
			}
			cdf[value] = sum;
		}

		// inspired by http://www.boost.org/doc/libs/1_46_1/doc/html/boost_random/tutorial.html
		uniform_real<> dist(0, cdf.back());
		boost::variate_generator<boost::mt19937&, uniform_real<> > die(gen, dist);
		size_t sampled = lower_bound(cdf.begin(), cdf.end(), die()) - cdf.begin();
		if (debug) cout << "-> " << sampled << endl;

		return sampled;

		return 0;
	}

	/* ******************************************************************************** */
	void DiscreteConditional::permuteWithInverse(const Permutation& inversePermutation){
		IndexConditional::permuteWithInverse(inversePermutation);
		Potentials::permuteWithInverse(inversePermutation);
	}


/* ******************************************************************************** */

} // namespace
