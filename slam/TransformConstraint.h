/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/*
 * @file TransformConstraint.h
 * @brief A constraint for combining graphs by common landmarks and a transform node
 * @author Alex Cunningham
 */

#pragma once

#include <gtsam/nonlinear/NonlinearConstraint.h>

namespace gtsam {

/**
 * A constraint between two landmarks in separate maps
 * Templated on:
 *   Values    : The overall config
 *	 PKey      : Key of landmark being constrained
 *	 Point     : Type of landmark
 *	 TKey      : Key of transform used
 *	 Transform : Transform variable class
 *
 * The Point and Transform concepts must be Lie types, and the transform
 * relationship "Point = transform_from(Transform, Point)" must exist.
 *
 * This base class should be specialized to implement the cost function for
 * specific classes of landmarks
 */
template<class Values, class PKey, class TKey>
class TransformConstraint : public NonlinearEqualityConstraint3<Values, PKey, TKey, PKey> {

public:
	typedef typename PKey::Value Point;
	typedef typename TKey::Value Transform;

	typedef NonlinearEqualityConstraint3<Values, PKey, TKey, PKey> Base;
	typedef TransformConstraint<Values, PKey, TKey> This;

	/**
	 * General constructor with all of the keys to variables in the map
	 * Extracts everything necessary from the key types
	 */
	TransformConstraint(const PKey& foreignKey, const TKey& transKey, const PKey& localKey, double mu = 1000.0)
	: Base(foreignKey, transKey, localKey, Point().dim(), mu) {}

	virtual ~TransformConstraint(){}

	/** Combined cost and derivative function using boost::optional */
	virtual Vector evaluateError(const Point& foreign, const Transform& trans, const Point& local,
				boost::optional<Matrix&> Dforeign = boost::none,
				boost::optional<Matrix&> Dtrans = boost::none,
				boost::optional<Matrix&> Dlocal = boost::none) const  {
		Point newlocal = trans.transform_from(foreign, Dtrans, Dforeign);
		if (Dlocal) {
			Point dummy;
			*Dlocal = -1* eye(dummy.dim());
		}
		return local.logmap(newlocal);
	}
};
} // \namespace gtsam
