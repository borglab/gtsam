/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file Key.h
 * @brief
 * @author Richard Roberts
 * @date Feb 20, 2012
 */
#pragma once

#include <gtsam/base/FastList.h>
#include <gtsam/base/FastMap.h>
#include <gtsam/base/FastSet.h>
#include <gtsam/base/FastVector.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/types.h>
#include <gtsam/dllexport.h>

#include <boost/function.hpp>

namespace gtsam {

/// Typedef for a function to format a key, i.e. to convert it to a string
typedef boost::function<std::string(Key)> KeyFormatter;

// Helper function for DefaultKeyFormatter
GTSAM_EXPORT std::string _defaultKeyFormatter(Key key);

/// The default KeyFormatter, which is used if no KeyFormatter is passed to
/// a nonlinear 'print' function.  Automatically detects plain integer keys
/// and Symbol keys.
static const KeyFormatter DefaultKeyFormatter = &_defaultKeyFormatter;

// Helper function for Multi-robot Key Formatter
GTSAM_EXPORT std::string _multirobotKeyFormatter(gtsam::Key key);

///
/// A KeyFormatter that will check for LabeledSymbol keys, as well as Symbol and plain
/// integer keys.  This keyformatter will need to be passed in to override the default
/// formatter in print functions.
///
/// Checks for LabeledSymbol, Symbol and then plain keys, in order.
static const gtsam::KeyFormatter MultiRobotKeyFormatter =
    &_multirobotKeyFormatter;

/// Useful typedef for operations with Values - allows for matlab interface
typedef FastVector<Key> KeyVector;

// TODO(frank): Nothing fast about these :-(
typedef FastList<Key> KeyList;
typedef FastSet<Key> KeySet;
typedef FastMap<Key, int> KeyGroupMap;

/// Utility function to print one key with optional prefix
GTSAM_EXPORT void PrintKey(Key key, const std::string& s = "",
    const KeyFormatter& keyFormatter = DefaultKeyFormatter);

/// Utility function to print sets of keys with optional prefix
GTSAM_EXPORT void PrintKeyList(const KeyList& keys, const std::string& s = "",
    const KeyFormatter& keyFormatter = DefaultKeyFormatter);

/// Utility function to print sets of keys with optional prefix
GTSAM_EXPORT void PrintKeyVector(const KeyVector& keys, const std::string& s =
    "", const KeyFormatter& keyFormatter = DefaultKeyFormatter);

/// Utility function to print sets of keys with optional prefix
GTSAM_EXPORT void PrintKeySet(const KeySet& keys, const std::string& s = "",
    const KeyFormatter& keyFormatter = DefaultKeyFormatter);

// Define Key to be Testable by specializing gtsam::traits
template<typename T> struct traits;

template <>
struct traits<Key> {
  static void Print(const Key& val, const std::string& str = "") {
    PrintKey(val, str);
  }
  static bool Equals(const Key& val1, const Key& val2, double tol = 1e-8) {
    return val1 == val2;
  }
};

} // namespace gtsam




