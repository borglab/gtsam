/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file Group.h
 *
 * @brief Concept check class for variable types with Group properties
 * @date November, 2011
 * @author Alex Cunningham
 * @author Frank Dellaert
 */

#pragma once

#include <boost/concept_check.hpp>
#include <boost/concept/requires.hpp>
#include <boost/type_traits/is_base_of.hpp>
#include <boost/static_assert.hpp>

namespace gtsam {

/// tag to assert a type is a group
struct group_tag {};

/// Group operator syntax flavors
struct multiplicative_group_tag {};
struct additive_group_tag {};

template <typename T> struct traits;

/**
 * Group Concept
 */
template<typename G>
class IsGroup {
public:
  typedef typename traits<G>::structure_category structure_category_tag;
  typedef typename traits<G>::group_flavor flavor_tag;
  //typedef typename traits<G>::identity::value_type identity_value_type;

  BOOST_CONCEPT_USAGE(IsGroup) {
    BOOST_STATIC_ASSERT_MSG(
        (boost::is_base_of<group_tag, structure_category_tag>::value),
        "This type's structure_category trait does not assert it as a group (or derived)");
    e = traits<G>::Identity();
    e = traits<G>::Compose(g, h);
    e = traits<G>::Between(g, h);
    e = traits<G>::Inverse(g);
    operator_usage(flavor);
    // todo: how do we test the act concept? or do we even need to?
  }

private:
  void operator_usage(multiplicative_group_tag) {
    e = g * h;
    //e = -g; // todo this should work, but it is failing for Quaternions
  }
  void operator_usage(additive_group_tag) {
    e = g + h;
    e = h - g;
    e = -g;
  }

  flavor_tag flavor;
  G e, g, h;
  bool b;
};

/// Check invariants
template<typename G>
BOOST_CONCEPT_REQUIRES(((IsGroup<G>)),(bool)) //
check_group_invariants(const G& a, const G& b, double tol = 1e-9) {
  G e = traits<G>::Identity();
  return traits<G>::Equals(traits<G>::Compose(a, traits<G>::Inverse(a)), e, tol)
      && traits<G>::Equals(traits<G>::Between(a, b), traits<G>::Compose(traits<G>::Inverse(a), b), tol)
      && traits<G>::Equals(traits<G>::Compose(a, traits<G>::Between(a, b)), b, tol);
}

namespace internal {

/// A helper class that implements the traits interface for groups.
template<class Class>
struct GroupTraits {
  typedef group_tag structure_category;
  static Class Identity() { return Class::Identity(); }
};

/// A helper class that implements the traits interface for additive groups.
template<class Class>
struct AdditiveGroupTraits : GroupTraits<Class> {
    typedef additive_group_tag group_flavor; \
    static Class Compose(const Class &g, const Class & h) { return g + h;} \
    static Class Between(const Class &g, const Class & h) { return h - g;} \
    static Class Inverse(const Class &g) { return -g;}
};

/// A helper class that implements the traits interface for multiplicative groups.
template<class Class>
struct MultiplicativeGroupTraits : GroupTraits<Class> {
    typedef additive_group_tag group_flavor; \
    static Class Compose(const Class &g, const Class & h) { return g * h;} \
    static Class Between(const Class &g, const Class & h) { return g.inverse() * h;} \
    static Class Inverse(const Class &g) { return g.inverse();}
};
}  // namespace internal
}  // namespace gtsam

/**
 * Macros for using the IsGroup
 *  - An instantiation for use inside unit tests
 *  - A typedef for use inside generic algorithms
 *
 * NOTE: intentionally not in the gtsam namespace to allow for classes not in
 * the gtsam namespace to be more easily enforced as testable
 */
#define GTSAM_CONCEPT_GROUP_INST(T) template class gtsam::IsGroup<T>;
#define GTSAM_CONCEPT_GROUP_TYPE(T) typedef gtsam::IsGroup<T> _gtsam_IsGroup_##T;
