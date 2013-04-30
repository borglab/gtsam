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

#include <gtsam/nonlinear/Symbol.h>
#include <gtsam/nonlinear/Ordering.h>

namespace gtsam {

/**
 * Customized version of gtsam::Symbol for multi-robot use
 */
class LabeledSymbol {
protected:
  unsigned char c_, label_;
  size_t j_;

public:
  /** Default constructor */
  LabeledSymbol();

  /** Copy constructor */
  LabeledSymbol(const LabeledSymbol& key);

  /** Constructor */
  LabeledSymbol(unsigned char c, unsigned char label, size_t j);

  /** Constructor that decodes an integer gtsam::Key */
  LabeledSymbol(gtsam::Key key);

  /** Cast to integer */
  operator gtsam::Key() const;

  // Testable Requirements
  void print(const std::string& s = "") const;

  bool equals(const LabeledSymbol& expected, double tol = 0.0) const {
    return (*this) == expected;
  }

  /** return the integer version */
  gtsam::Key key() const { return (gtsam::Key) *this; }

  /** Retrieve label character */
  inline unsigned char label() const { return label_; }

  /** Retrieve key character */
  inline unsigned char chr() const { return c_; }

  /** Retrieve key index */
  inline size_t index() const { return j_; }

  /** Create a string from the key */
  operator std::string() const;

  /** Comparison for use in maps */
  bool operator<(const LabeledSymbol& comp) const;
  bool operator==(const LabeledSymbol& comp) const;
  bool operator==(gtsam::Key comp) const;
  bool operator!=(const LabeledSymbol& comp) const;
  bool operator!=(gtsam::Key comp) const;

  /** Return a filter function that returns true when evaluated on a gtsam::Key whose
   * character (when converted to a LabeledSymbol) matches \c c.  Use this with the
   * Values::filter() function to retrieve all key-value pairs with the
   * requested character.
   */

  // Checks only the type
  static boost::function<bool(gtsam::Key)> TypeTest(unsigned char c);

  // Checks only the robot ID (label_)
  static boost::function<bool(gtsam::Key)> LabelTest(unsigned char label);

  // Checks both type and the robot ID
  static boost::function<bool(gtsam::Key)> TypeLabelTest(unsigned char c, unsigned char label);

  // Converts to upper/lower versions of labels
  LabeledSymbol upper() const { return LabeledSymbol(c_, toupper(label_), j_); }
  LabeledSymbol lower() const { return LabeledSymbol(c_, tolower(label_), j_); }

  // Create a new symbol with a different value
  LabeledSymbol newChr(unsigned char c) const { return LabeledSymbol(c, label_, j_); }
  LabeledSymbol newLabel(unsigned char label) const { return LabeledSymbol(c_, label, j_); }

private:

  /** Serialization function */
  friend class boost::serialization::access;
  template<class ARCHIVE>
  void serialize(ARCHIVE & ar, const unsigned int version) {
    ar & BOOST_SERIALIZATION_NVP(c_);
    ar & BOOST_SERIALIZATION_NVP(label_);
    ar & BOOST_SERIALIZATION_NVP(j_);
  }
}; // \class LabeledSymbol

// Helper function for Multi-robot Key Formatter
std::string _multirobotKeyFormatter(gtsam::Key key);

/**
 * A KeyFormatter that will check for LabeledSymbol keys, as well as Symbol and plain
 * integer keys.  This keyformatter will need to be passed in to override the default
 * formatter in print functions.
 *
 * Checks for LabeledSymbol, Symbol and then plain keys, in order.
 */
static const gtsam::KeyFormatter MultiRobotKeyFormatter = &_multirobotKeyFormatter;

/// Version of orderingIndexFormatter using multi-robot formatter
struct MultiRobotLinearFormatter : gtsam::OrderingIndexFormatter {
  MultiRobotLinearFormatter(const gtsam::Ordering& ordering)
  : gtsam::OrderingIndexFormatter(ordering, MultiRobotKeyFormatter) {}
};

/// Utility function to print sets of keys with optional prefix
void printKeySet(const KeySet& keys, const std::string& s = "",
    const KeyFormatter& keyFormatter = DefaultKeyFormatter);

/// Computes the intersection between two sets
gtsam::KeySet keyIntersection(const gtsam::KeySet& keysA, const gtsam::KeySet& keysB);

/// Checks if an intersection exists - faster checking size of above
bool hasKeyIntersection(const gtsam::KeySet& keysA, const gtsam::KeySet& keysB);

/// Computes a difference between sets, so result is those that are in A, but not B
gtsam::KeySet keyDifference(const gtsam::KeySet& keysA, const gtsam::KeySet& keysB);

} // \namespace gtsam

