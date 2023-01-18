/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file Symbol.h
 * @date Jan 12, 2010
 * @author: Frank Dellaert
 * @author: Richard Roberts
 */

#pragma once

#include <gtsam/base/Testable.h>
#include <gtsam/inference/Key.h>

#include <boost/serialization/nvp.hpp>
#include <cstdint>
#include <functional>

namespace gtsam {

/**
 * Character and index key used to refer to variables. Will simply cast to a Key,
 * i.e., a large integer. Keys are used to retrieve values from Values,
 * specify what variables factors depend on, etc.
 */
class GTSAM_EXPORT Symbol {
protected:
  unsigned char c_;
  std::uint64_t j_;

public:

  /** Default constructor */
  Symbol() :
    c_(0), j_(0) {
  }

  /** Copy constructor */
  Symbol(const Symbol& key) :
    c_(key.c_), j_(key.j_) {
  }

  /** Constructor */
  Symbol(unsigned char c, std::uint64_t j) :
    c_(c), j_(j) {
  }

  /** Constructor that decodes an integer Key */
  Symbol(Key key);

  /** return Key (integer) representation */
  Key key() const;

  /** Cast to integer */
  operator Key() const { return key(); }

  /// Print
  void print(const std::string& s = "") const;

  /// Check equality
  bool equals(const Symbol& expected, double tol = 0.0) const;

  /** Retrieve key character */
  unsigned char chr() const {
    return c_;
  }

  /** Retrieve key index */
  std::uint64_t index() const {
    return j_;
  }

  /** Create a string from the key */
  operator std::string() const;

  /// Return string representation of the key
  std::string string() const { return std::string(*this); };

  /** Comparison for use in maps */
  bool operator<(const Symbol& comp) const {
    return c_ < comp.c_ || (comp.c_ == c_ && j_ < comp.j_);
  }

  /** Comparison for use in maps */
  bool operator==(const Symbol& comp) const {
    return comp.c_ == c_ && comp.j_ == j_;
  }

  /** Comparison for use in maps */
  bool operator==(Key comp) const {
    return comp == (Key)(*this);
  }

  /** Comparison for use in maps */
  bool operator!=(const Symbol& comp) const {
    return comp.c_ != c_ || comp.j_ != j_;
  }

  /** Comparison for use in maps */
  bool operator!=(Key comp) const {
    return comp != (Key)(*this);
  }

  /** Return a filter function that returns true when evaluated on a Key whose
   * character (when converted to a Symbol) matches \c c.  Use this with the
   * Values::filter() function to retrieve all key-value pairs with the
   * requested character.
   */
  static std::function<bool(Key)> ChrTest(unsigned char c);

  /// Output stream operator that can be used with key_formatter (see Key.h).
  GTSAM_EXPORT friend std::ostream &operator<<(std::ostream &, const Symbol &);

private:

  /** Serialization function */
#ifdef GTSAM_ENABLE_BOOST_SERIALIZATION
  friend class boost::serialization::access;
  template<class ARCHIVE>
  void serialize(ARCHIVE & ar, const unsigned int /*version*/) {
    ar & BOOST_SERIALIZATION_NVP(c_);
    ar & BOOST_SERIALIZATION_NVP(j_);
  }
#endif
};

/** Create a symbol key from a character and index, i.e. x5. */
inline Key symbol(unsigned char c, std::uint64_t j) { return (Key)Symbol(c,j); }

/** Return the character portion of a symbol key. */
inline unsigned char symbolChr(Key key) { return Symbol(key).chr(); }

/** Return the index portion of a symbol key. */
inline std::uint64_t symbolIndex(Key key) { return Symbol(key).index(); }

namespace symbol_shorthand {
inline Key A(std::uint64_t j) { return Symbol('a', j); }
inline Key B(std::uint64_t j) { return Symbol('b', j); }
inline Key C(std::uint64_t j) { return Symbol('c', j); }
inline Key D(std::uint64_t j) { return Symbol('d', j); }
inline Key E(std::uint64_t j) { return Symbol('e', j); }
inline Key F(std::uint64_t j) { return Symbol('f', j); }
inline Key G(std::uint64_t j) { return Symbol('g', j); }
inline Key H(std::uint64_t j) { return Symbol('h', j); }
inline Key I(std::uint64_t j) { return Symbol('i', j); }
inline Key J(std::uint64_t j) { return Symbol('j', j); }
inline Key K(std::uint64_t j) { return Symbol('k', j); }
inline Key L(std::uint64_t j) { return Symbol('l', j); }
inline Key M(std::uint64_t j) { return Symbol('m', j); }
inline Key N(std::uint64_t j) { return Symbol('n', j); }
inline Key O(std::uint64_t j) { return Symbol('o', j); }
inline Key P(std::uint64_t j) { return Symbol('p', j); }
inline Key Q(std::uint64_t j) { return Symbol('q', j); }
inline Key R(std::uint64_t j) { return Symbol('r', j); }
inline Key S(std::uint64_t j) { return Symbol('s', j); }
inline Key T(std::uint64_t j) { return Symbol('t', j); }
inline Key U(std::uint64_t j) { return Symbol('u', j); }
inline Key V(std::uint64_t j) { return Symbol('v', j); }
inline Key W(std::uint64_t j) { return Symbol('w', j); }
inline Key X(std::uint64_t j) { return Symbol('x', j); }
inline Key Y(std::uint64_t j) { return Symbol('y', j); }
inline Key Z(std::uint64_t j) { return Symbol('z', j); }
}

/** Generates symbol shorthands with alternative names different than the
 * one-letter predefined ones. */
class SymbolGenerator {
  const unsigned char c_;
public:
  constexpr SymbolGenerator(const unsigned char c) : c_(c) {}
  Symbol operator()(const std::uint64_t j) const { return Symbol(c_, j); }
  constexpr unsigned char chr() const { return c_; }
};

/// traits
template<> struct traits<Symbol> : public Testable<Symbol> {};

} // \ namespace gtsam
