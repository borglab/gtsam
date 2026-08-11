/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    QpsParser.h
 * @brief   Parser for QPS quadratic programming files.
 * @author  Frank Dellaert
 */

#pragma once

#include <gtsam/constrained/QpProblem.h>
#include <gtsam/inference/Key.h>

#include <iosfwd>
#include <map>
#include <stdexcept>
#include <string>

namespace gtsam {

/** Parsed QPS problem and metadata. */
struct GTSAM_EXPORT QpsParserResult {
  QpProblem problem;                        ///< Parsed quadratic program.
  std::map<std::string, Key> variableKeys;  ///< QPS variable names to keys.
  std::string name;                         ///< Problem name from the NAME row.
  std::string objectiveName;                ///< Objective row name.
};

/** Exception thrown when QPS input cannot be parsed. */
class GTSAM_EXPORT QpsParserException : public std::runtime_error {
 public:
  /** Construct with a source/line-aware parse error description. */
  explicit QpsParserException(const std::string& description);
};

/** Parser for the QPS subset used by GTSAM examples and tests. */
class GTSAM_EXPORT QpsParser {
 public:
  /** Parse QPS input from a stream. */
  static QpsParserResult parse(std::istream& input,
                               const std::string& sourceName = "");

  /** Parse QPS input from a file path. */
  static QpsParserResult parseFile(const std::string& path);
};

}  // namespace gtsam
