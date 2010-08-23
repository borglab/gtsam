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
 *   Config    : The overall config
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
template<class Config, class PKey, class TKey>
class TransformConstraint : public NonlinearEqualityConstraint3<Config, PKey, TKey, PKey> {

public:
	typedef typename PKey::Value_t Point;
	typedef typename TKey::Value_t Transform;

	typedef NonlinearEqualityConstraint3<Config, PKey, TKey, PKey> Base;
	typedef TransformConstraint<Config, PKey, TKey> This;

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
		if (Dforeign) {
			Matrix Af;
			trans.transform_from(foreign, boost::none, Af);
			*Dforeign = Af;
		}
		if (Dtrans) {
			Matrix At;
			trans.transform_from(foreign, At, boost::none);
			*Dtrans = At;
		}
		if (Dlocal) {
			Point dummy;
			*Dlocal = -1* eye(dummy.dim());
		}
		return gtsam::logmap(gtsam::between<Point>(local, trans.transform_from(foreign)));
	}
};
} // \namespace gtsam
