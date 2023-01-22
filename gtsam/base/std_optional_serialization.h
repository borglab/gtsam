// Functionality to serialize std::optional<T> to boost::archive
// Following this:
// PR: https://github.com/boostorg/serialization/pull/148/files#

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

//!!!!!!!!! I don't completely understand or know if this is correct but compilation works!!!!!!!!!!!
#ifdef __GNUC__
#if __GNUC__ >= 7 && __cplusplus >= 201703L
namespace boost { namespace serialization { struct U; } }
namespace std { template<> struct is_trivially_default_constructible<boost::serialization::U> : std::false_type {}; }
namespace std { template<> struct is_trivially_copy_constructible<boost::serialization::U> : std::false_type {}; }
namespace std { template<> struct is_trivially_move_constructible<boost::serialization::U> : std::false_type {}; }
#endif
#endif
//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!


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

// derive boost::xml_archive_impl for archiving std::optional<T> with xml

}  // namespace serialization
}  // namespace boost

