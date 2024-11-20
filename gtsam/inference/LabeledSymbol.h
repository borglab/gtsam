/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file LabeledSymbol.h
 * @date Jan 12, 2010
 * @author: Alex Cunningham
 * @author: Frank Dellaert
 * @author: Richard Roberts
 */

#pragma once

#include <gtsam/base/Testable.h>
#include <gtsam/inference/Symbol.h>

#include <functional>

namespace gtsam {

/**
 * Customized version of gtsam::Symbol for multi-robot use.
 *
 * This variation of Symbol stores two char values in addition to
 * an integer key, which is useful for encoding a group for a
 * variable. This was originally designed for multi-robot systems,
 * which allows expressing "Pose 7 from robot B" as "xB7".
 */
class GTSAM_EXPORT LabeledSymbol {
 protected:
  unsigned char c_, label_;
  std::uint64_t j_;

 public:
  /// @name Constructors
  /// @{

  /// Default constructor
  LabeledSymbol();

  /// Copy constructor
  LabeledSymbol(const LabeledSymbol& key);

  /// Constructor fro characters c and label, and integer j
  LabeledSymbol(unsigned char c, unsigned char label, std::uint64_t j);

  /// Constructor that decodes an integer Key
  LabeledSymbol(Key key);

  /// @}
  /// @name Testable
  /// @{

  /// Prints the LabeledSymbol with an optional prefix string.
  void print(const std::string& s = "") const;

  /// Checks if this LabeledSymbol is equal to another, tolerance is ignored.
  bool equals(const LabeledSymbol& expected, double tol = 0.0) const {
    return (*this) == expected;
  }

  /// @}
  /// @name Standard API
  /// @{

  /// Cast to Key
  operator Key() const;

  /// return the integer version
  Key key() const { return (Key) * this; }

  /// Retrieve label character
  inline unsigned char label() const { return label_; }

  /// Retrieve key character
  inline unsigned char chr() const { return c_; }

  /// Retrieve key index
  inline size_t index() const { return j_; }

  /// Create a string from the key
  operator std::string() const;

  /// Output stream operator that can be used with key_formatter (see Key.h).
  friend GTSAM_EXPORT std::ostream& operator<<(std::ostream&,
                                               const LabeledSymbol&);

  /// @}
  /// @name Comparison
  /// @{

  bool operator<(const LabeledSymbol& comp) const;
  bool operator==(const LabeledSymbol& comp) const;
  bool operator==(Key comp) const;
  bool operator!=(const LabeledSymbol& comp) const;
  bool operator!=(Key comp) const;

  /// @}
  /// @name Filtering
  /// @{
  /// Return a filter function that returns true when evaluated on a Key whose
  /// character (when converted to a LabeledSymbol) matches \c c.  Use this with
  /// the Values::filter() function to retrieve all key-value pairs with the
  /// requested character.

  /// Checks only the type
  static std::function<bool(Key)> TypeTest(unsigned char c);

  /// Checks only the robot ID (label_)
  static std::function<bool(Key)> LabelTest(unsigned char label);

  /// Checks both type and the robot ID
  static std::function<bool(Key)> TypeLabelTest(unsigned char c,
                                                unsigned char label);

  /// @}
  /// @name Advanced API
  /// @{

  /// Converts to upper/lower versions of labels
  LabeledSymbol upper() const { return LabeledSymbol(c_, toupper(label_), j_); }
  LabeledSymbol lower() const { return LabeledSymbol(c_, tolower(label_), j_); }

  /// Create a new symbol with a different character.
  LabeledSymbol newChr(unsigned char c) const {
    return LabeledSymbol(c, label_, j_);
  }

  /// Create a new symbol with a different label.
  LabeledSymbol newLabel(unsigned char label) const {
    return LabeledSymbol(c_, label, j_);
  }

  /// @}

 private:
#ifdef GTSAM_ENABLE_BOOST_SERIALIZATION
  /// Serialization function
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE& ar, const unsigned int /*version*/) {
    ar& BOOST_SERIALIZATION_NVP(c_);
    ar& BOOST_SERIALIZATION_NVP(label_);
    ar& BOOST_SERIALIZATION_NVP(j_);
  }
#endif
};  // \class LabeledSymbol

/// Create a symbol key from a character, label and index, i.e. xA5.
inline Key mrsymbol(unsigned char c, unsigned char label, size_t j) {
  return (Key)LabeledSymbol(c, label, j);
}

/// Return the character portion of a symbol key.
inline unsigned char mrsymbolChr(Key key) { return LabeledSymbol(key).chr(); }

/// Return the label portion of a symbol key.
inline unsigned char mrsymbolLabel(Key key) {
  return LabeledSymbol(key).label();
}

/// Return the index portion of a symbol key.
inline size_t mrsymbolIndex(Key key) { return LabeledSymbol(key).index(); }

/// traits
template <>
struct traits<LabeledSymbol> : public Testable<LabeledSymbol> {};

}  // namespace gtsam
