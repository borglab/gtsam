#pragma once

#include <string>
#include <iostream>
#include <sstream>

#include <gtsam/dllexport.h>

namespace gtsam {
/**
 * For Python __str__().
 * Redirect std cout to a string stream so we can return a string representation
 * of an object when it prints to cout.
 * https://stackoverflow.com/questions/5419356/redirect-stdout-stderr-to-a-string
 */
struct GTSAM_EXPORT RedirectCout {
  /// constructor -- redirect stdout buffer to a stringstream buffer
  RedirectCout() : ssBuffer_(), coutBuffer_(std::cout.rdbuf(ssBuffer_.rdbuf())) {}

  /// return the string
  std::string str() const;

  /// destructor -- redirect stdout buffer to its original buffer
  ~RedirectCout();

private:
  std::stringstream ssBuffer_;
  std::streambuf* coutBuffer_;
};

}

namespace gtsam {
// Adapted from https://stackoverflow.com/a/32223343/9151520
// An adaptation of boost::mp11::index_sequence
template <size_t... Ints>
struct index_sequence {
  using type = index_sequence;
  using value_type = size_t;
  static constexpr std::size_t size() noexcept { return sizeof...(Ints); }
};
namespace detail {
template <class Sequence1, class Sequence2>
struct _merge_and_renumber;

template <size_t... I1, size_t... I2>
struct _merge_and_renumber<index_sequence<I1...>, index_sequence<I2...>>
    : index_sequence<I1..., (sizeof...(I1) + I2)...> {};
}  // namespace detail
template <size_t N>
struct make_index_sequence : detail::_merge_and_renumber<typename make_index_sequence<N / 2>::type,
                                                         typename make_index_sequence<N - N / 2>::type> {};
template <>
struct make_index_sequence<0> : index_sequence<> {};
template <>
struct make_index_sequence<1> : index_sequence<0> {};
template <class... T>
using index_sequence_for = make_index_sequence<sizeof...(T)>;
}  // namespace gtsam
