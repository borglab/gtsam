/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

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

#include <gtsam/base/Matrix.h>
#include <boost/optional.hpp>

namespace gtsam {

/**
 * Pose Concept
 * A must contain a translation and a rotation, with
 * each structure accessable directly and a type provided
 * for each.
 */
template<class POSE>
class PoseConcept {
public:
	typedef typename POSE::Translation Translation;
	typedef typename POSE::Rotation Rotation;

private:
	static Rotation checkRotationMemberAccess(const POSE& p) {
		return p.rotation();
	}

	static Translation checkTranslationMemberAccess(const POSE& p) {
		return p.translation();
	}

	static std::pair<size_t, size_t> checkTranslationInterval() {
		return POSE::translationInterval();
	}

	static std::pair<size_t, size_t> checkRotationInterval() {
		return POSE::rotationInterval();
	}
};

/**
 * Range measurement concept
 * Given a pair of Lie variables, there must exist a function to calculate
 * range with derivatives.
 */
template<class V1, class V2>
class RangeMeasurementConcept {
private:
	static double checkRangeMeasurement(const V1& x, const V2& p) {
		return x.range(p);
	}

	static double checkRangeMeasurementDerivatives(const V1& x, const V2& p) {
		boost::optional<Matrix&> H1, H2;
		return x.range(p, H1, H2);
	}
};

} // \namespace gtsam

/**
 * Macros to use geometry concepts:
 *  - _INST macro: instantiates a struct: use in unit tests, but not in headers.
 *  - _TYPE macro: use in generic structures to validate the template arguments.
 *
 * NOTE: intentionally not in the gtsam namespace to avoid namespace complications
 * when using with objects not inside gtsam namespace.
 */

/** Pose Concept macros */
#define GTSAM_CONCEPT_POSE_INST(T) template class gtsam::PoseConcept<T>;
#define GTSAM_CONCEPT_POSE_TYPE(T) typedef gtsam::PoseConcept<T> _gtsam_PoseConcept##T;

/** Range Measurement macros */
#define GTSAM_CONCEPT_RANGE_MEASUREMENT_INST(V1,V2) template class gtsam::RangeMeasurementConcept<V1,V2>;
#define GTSAM_CONCEPT_RANGE_MEASUREMENT_TYPE(V1,V2) typedef gtsam::RangeMeasurementConcept<V1,V2> _gtsam_RangeMeasurementConcept##V1##V2;

