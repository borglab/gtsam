/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    IndexFactor.h
 * @author  Richard Roberts
 * @date    Oct 17, 2010
 */

#pragma once

#include <gtsam/inference/Factor.h>
#include <gtsam/inference/Permutation.h>

namespace gtsam {

	// Forward declaration of IndexConditional
	class IndexConditional;

	/**
	 * IndexFactor serves two purposes.  It is the base class for all linear
	 * factors (GaussianFactor, JacobianFactor, HessianFactor), and also
	 * functions as a symbolic factor with Index keys, used to do symbolic
	 * elimination by JunctionTree.
	 *
	 * It derives from Factor with a key type of Index, an unsigned integer.
	 * \nosubgrouping
	 */
	class IndexFactor: public Factor<Index> {

	protected:

		/// @name Advanced Interface
		/// @{

		/// Internal function for checking class invariants (unique keys for this factor)
		void assertInvariants() const;

		/// @}

	public:

		typedef IndexFactor This;
		typedef Factor<Index> Base;

		/** Elimination produces an IndexConditional */
		typedef IndexConditional ConditionalType;

		/** Overriding the shared_ptr typedef */
		typedef boost::shared_ptr<IndexFactor> shared_ptr;

		/// @name Standard Interface
		/// @{

		/** Copy constructor */
		IndexFactor(const This& f) :
			Base(f) {
			assertInvariants();
		}

		/** Construct from derived type */
		IndexFactor(const IndexConditional& c);

		/** Default constructor for I/O */
		IndexFactor() {
			assertInvariants();
		}

		/** Construct unary factor */
		IndexFactor(Index j) :
			Base(j) {
			assertInvariants();
		}

		/** Construct binary factor */
		IndexFactor(Index j1, Index j2) :
			Base(j1, j2) {
			assertInvariants();
		}

		/** Construct ternary factor */
		IndexFactor(Index j1, Index j2, Index j3) :
			Base(j1, j2, j3) {
			assertInvariants();
		}

		/** Construct 4-way factor */
		IndexFactor(Index j1, Index j2, Index j3, Index j4) :
			Base(j1, j2, j3, j4) {
			assertInvariants();
		}

    /** Construct 5-way factor */
    IndexFactor(Index j1, Index j2, Index j3, Index j4, Index j5) :
      Base(j1, j2, j3, j4, j5) {
      assertInvariants();
    }

    /** Construct 6-way factor */
    IndexFactor(Index j1, Index j2, Index j3, Index j4, Index j5, Index j6) :
      Base(j1, j2, j3, j4, j5, j6) {
      assertInvariants();
    }

		/// @}
		/// @name Advanced Constructors
		/// @{

		/** Construct n-way factor */
		IndexFactor(const std::set<Index>& js) :
			Base(js) {
			assertInvariants();
		}

		/** Construct n-way factor */
		IndexFactor(const std::vector<Index>& js) :
			Base(js) {
			assertInvariants();
		}

		/** Constructor from a collection of keys */
		template<class KeyIterator> IndexFactor(KeyIterator beginKey,
				KeyIterator endKey) :
			Base(beginKey, endKey) {
			assertInvariants();
		}

		/// @}

#ifdef TRACK_ELIMINATE
		/**
		 * eliminate the first variable involved in this factor
		 * @return a conditional on the eliminated variable
		 */
		boost::shared_ptr<ConditionalType> eliminateFirst();

		/** eliminate the first nrFrontals frontal variables.*/
		boost::shared_ptr<BayesNet<ConditionalType> > eliminate(size_t nrFrontals =
				1);
#endif

		/// @name Advanced Interface
		/// @{

	  /**
	   * Permutes the factor, but for efficiency requires the permutation
	   * to already be inverted.
	   */
	  void permuteWithInverse(const Permutation& inversePermutation);

		virtual ~IndexFactor() {
		}

  private:
    /** Serialization function */
    friend class boost::serialization::access;
    template<class ARCHIVE>
    void serialize(ARCHIVE & ar, const unsigned int version) {
    	ar & BOOST_SERIALIZATION_BASE_OBJECT_NVP(Base);
    }

		/// @}

	}; // IndexFactor

}
