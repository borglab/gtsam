/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file LinearApproxFactor.h
 * @brief A dummy factor that allows a linear factor to act as a nonlinear factor
 * @author Alex Cunningham
 */

#pragma once

#include <vector>
#include <gtsam/nonlinear/NonlinearFactor.h>

namespace gtsam {

/**
 * A dummy factor that takes a linearized factor and inserts it into
 * a nonlinear graph.  This version uses exactly one type of variable.
 */
template <class Values, class Key>
class LinearApproxFactor : public NonlinearFactor<Values> {

public:
	/** type for the variable */
	typedef typename Key::Value X;

	/** base type */
	typedef NonlinearFactor<Values> Base;

	/** shared pointer for convenience */
	typedef boost::shared_ptr<LinearApproxFactor<Values,Key> > shared_ptr;

	/** typedefs for key vectors */
	typedef std::vector<Key> KeyVector;

protected:
	/** hold onto the factor itself */
//	GaussianFactor::shared_ptr lin_factor_;
	// store components of a linear factor that can be reordered
	typedef std::map<Symbol, Matrix> SymbolMatrixMap;
	SymbolMatrixMap matrices_;
	Vector b_;
	SharedDiagonal model_;

	/** linearization points for error calculation */
	Values lin_points_;

	/** keep keys for the factor */
	KeyVector nonlinearKeys_;

	/**
	 * use this for derived classes with keys that don't copy easily
	 */
	LinearApproxFactor(size_t dim, const Values& lin_points)
		: Base(noiseModel::Unit::Create(dim)), lin_points_(lin_points) {}

public:

	/**
	 * use this constructor when starting with nonlinear keys
	 *
	 * Note that you need to have the ordering used to construct the factor
	 * initially in order to find the actual keys
	 */
	LinearApproxFactor(GaussianFactor::shared_ptr lin_factor,
			const Ordering& ordering, const Values& lin_points);

	virtual ~LinearApproxFactor() {}

	/** Vector of errors, unwhitened ! */
	virtual Vector unwhitenedError(const Values& c) const;

	/**
	 * linearize to a GaussianFactor
	 * Reconstructs the linear factor from components to ensure that
	 * the ordering is correct
	 */
	virtual boost::shared_ptr<GaussianFactor> linearize(
			const Values& c, const Ordering& ordering) const;

    /**
     * Create a symbolic factor using the given ordering to determine the
     * variable indices.
     */
    Factor::shared_ptr symbolic(const Ordering& ordering) const;

	/** get access to nonlinear keys */
	KeyVector nonlinearKeys() const { return nonlinearKeys_; }

	/** override print function */
	virtual void print(const std::string& s="") const;

	/** access to b vector of gaussian */
	Vector get_b() const { return b_; }
};

} // \namespace gtsam
