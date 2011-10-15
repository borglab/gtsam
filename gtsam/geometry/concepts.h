/**
 * @file concepts.h
 *
 * @brief Concept-checking macros for geometric objects
 * Each macro instantiates a concept check structure, which
 * includes a static function that will fail to compile
 * if the concept does not pass. Functions are made static
 * to ensure they get instantiated.
 *
 * @date Oct 6, 2011
 * @author Alex Cunningham
 */

#pragma once

namespace gtsam {

/**
 * Pose Concept
 * A must contain a translation and a rotation, with
 * each structure accessable directly and a type provided
 * for each.
 */
template<class POSE>
struct PoseConcept {
	typedef typename POSE::Translation Translation;
	typedef typename POSE::Rotation Rotation;

	static Rotation checkRotationMemberAccess(const POSE& p) {
		return p.rotation();
	}

	static Translation checkTranslationMemberAccess(const POSE& p) {
		return p.translation();
	}
};

/** Instantiation macro */
#define GTSAM_CONCEPT_POSE(T) template class PoseConcept<T>;

/**
 * Range measurement concept
 * Given a pair of Lie variables, there must exist a function to calculate
 * range with derivatives.
 */
template<class V1, class V2>
struct RangeMeasurementConcept {
	static double checkRangeMeasurement(const V1& x, const V2& p) {
		return x.range(p);
	}

	static double checkRangeMeasurementDerivatives(const V1& x, const V2& p) {
		boost::optional<Matrix&> H1, H2;
		// FIXME: verify the dimensions of the derivative functions
		return x.range(p, H1, H2);
	}
};

/** Instantiation macro */
#define GTSAM_CONCEPT_RANGE_MEASUREMENT(V1,V2) template class RangeMeasurementConcept<V1,V2>;

} // \namespace gtsam
