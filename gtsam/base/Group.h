/**
 * @file Group.h
 *
 * @brief Concept check class for variable types with Group properties
 * @date Nov 5, 2011
 * @author Alex Cunningham
 */

#pragma once

namespace gtsam {

/**
 * This concept check enforces a Group structure on a variable type,
 * in which we require the existence of basic algebraic operations.
 */
template<class T>
class GroupConcept {
private:
	static void concept_check(const T& t) {
		/** assignment */
		T t2 = t;

		/** compose with another object */
		T compose_ret = t.compose(t2);

		/** invert the object and yield a new one */
		T inverse_ret = t.inverse();

		/** identity */
		T identity = T::identity();
	}
};

} // \namespace gtsam

/**
 * Macros for using the GroupConcept
 *  - An instantiation for use inside unit tests
 *  - A typedef for use inside generic algorithms
 *
 * NOTE: intentionally not in the gtsam namespace to allow for classes not in
 * the gtsam namespace to be more easily enforced as testable
 */
#define GTSAM_CONCEPT_GROUP_INST(T) template class gtsam::GroupConcept<T>;
#define GTSAM_CONCEPT_GROUP_TYPE(T) typedef gtsam::GroupConcept<T> _gtsam_GroupConcept_##T;
