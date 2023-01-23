/* ----------------------------------------------------------------------------
* Use, modification and distribution is subject to the Boost Software
* License, Version 1.0. (See accompanying file LICENSE_1_0.txt or copy at
* http://www.boost.org/LICENSE_1_0.txt)

* See http://www.boost.org for updates, documentation, and revision history.

* Functionality to serialize std::optional<T> to boost::archive
* Inspired from this PR: https://github.com/boostorg/serialization/pull/163
* ---------------------------------------------------------------------------- */

#pragma once
#include <optional>
#include <boost/config.hpp>

#include <boost/archive/detail/basic_iarchive.hpp>
#include <boost/move/utility_core.hpp>

#include <boost/serialization/item_version_type.hpp>
#include <boost/serialization/split_free.hpp>
#include <boost/serialization/level.hpp>
#include <boost/serialization/nvp.hpp>
#include <boost/serialization/version.hpp>
#include <boost/type_traits/is_pointer.hpp>
#include <boost/serialization/detail/stack_constructor.hpp>
#include <boost/serialization/detail/is_default_constructible.hpp>

/** A bunch of declarations to deal with gcc bug
 * The compiler has a difficult time distinguisihing between:
 *
 * template<template <Archive, class U> class SPT> void load(Archive, SPT<U>&, const unsigned int) : <boost/serialization/shared_ptr.hpp>
 *
 * and
 *
 * template<T> void load(Archive, std::optional<T>&, const unsigned int) : <std_optional_serialization.h>
 *
 * The compiler will try to instantiate an object of the type of std::optional<boost::serialization::U> which is not valid since U is not a type and
 * thus leading to a whole series of errros.
 *
 * This is a well known bug in gcc documented here: https://gcc.gnu.org/bugzilla/show_bug.cgi?id=84075
 * A minimal reproducible example here: https://godbolt.org/z/anj9YjnPY
 *
 * For std::optional the workaround is just provide the traits needed to make std::optional<boost::serialization::U> possible
 * This is not an issue since there is no actual type boost::serialization::U and we are not falsely providing a specialization
 * for std::optional<boost::serialization::U>
 */
#ifdef __GNUC__
#if __GNUC__ >= 7 && __cplusplus >= 201703L
namespace boost { namespace serialization { struct U; } }
namespace std { template<> struct is_trivially_default_constructible<boost::serialization::U> : std::false_type {}; }
namespace std { template<> struct is_trivially_copy_constructible<boost::serialization::U> : std::false_type {}; }
namespace std { template<> struct is_trivially_move_constructible<boost::serialization::U> : std::false_type {}; }
#endif
#endif


// function specializations must be defined in the appropriate
// namespace - boost::serialization
namespace boost {
namespace serialization {

template <class Archive, class T>
void save(Archive& ar, const std::optional<T>& t, const unsigned int /*version*/
) {
  // It is an inherent limitation to the serialization of optional.hpp
  // that the underlying type must be either a pointer or must have a
  // default constructor.
  BOOST_STATIC_ASSERT(boost::serialization::detail::is_default_constructible<T>::value || boost::is_pointer<T>::value);
  const bool tflag = t.has_value();
  ar << boost::serialization::make_nvp("initialized", tflag);
  if (tflag) {
    ar << boost::serialization::make_nvp("value", *t);
  }
}

template <class Archive, class T>
void load(Archive& ar, std::optional<T>& t, const unsigned int /*version*/
) {
  bool tflag;
  ar >> boost::serialization::make_nvp("initialized", tflag);
  if (!tflag) {
    t.reset();
    return;
  }

  if (!t.has_value()) {
    // Need to be default constructible
    t.emplace();
  }
  ar >> boost::serialization::make_nvp("value", *t);
}

template <class Archive, class T>
void serialize(Archive& ar, std::optional<T>& t, const unsigned int version) {
  boost::serialization::split_free(ar, t, version);
}

}  // namespace serialization
}  // namespace boost

