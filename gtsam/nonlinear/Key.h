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

#include <boost/function.hpp>
#include <string>

#include <gtsam/global_includes.h>
#include <gtsam/base/FastVector.h>
#include <gtsam/base/FastList.h>
#include <gtsam/base/FastSet.h>

namespace gtsam {

  /// Integer nonlinear key type
  typedef size_t Key;

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
  static const gtsam::KeyFormatter MultiRobotKeyFormatter = &_multirobotKeyFormatter;

  /// Useful typedefs for operations with Values - allow for matlab interfaces
  typedef FastList<Key> KeyList;
  typedef FastVector<Key> KeyVector;
  typedef FastSet<Key> KeySet;

  /// Utility function to print sets of keys with optional prefix
  GTSAM_EXPORT void printKeySet(const KeySet& keys, const std::string& s = "",
      const KeyFormatter& keyFormatter = DefaultKeyFormatter);
}

