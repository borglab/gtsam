/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file SignatureParser.h
 * @brief Parser for conditional distribution signatures.
 * @author Kartik Arcot
 * @date January 2023
 */

#pragma once

#include <optional>
#include <string>
#include <vector>
#include <gtsam/dllexport.h>

#include <gtsam/dllexport.h>

namespace gtsam {
/**
 * @brief A simple parser that replaces the boost spirit parser.
 *
 * It is meant to parse strings like "1/1 2/3 1/4". Every word of the form
 * "a/b/c/..." is parsed as a row, and the rows are stored in a table.
 *
 * A `Row` is a vector of doubles, and a `Table` is a vector of rows.
 *
 * Examples: the parser is able to parse the following strings:
 *   "1/1 2/3 1/4":
 *      {{1,1},{2,3},{1,4}}
 *   "1/1 2/3 1/4 1/1 2/3 1/4" :
 *      {{1,1},{2,3},{1,4},{1,1},{2,3},{1,4}}
 *   "1/2/3 2/3/4 1/2/3 2/3/4 1/2/3 2/3/4 1/2/3 2/3/4 1/2/3" :
 *      {{1,2,3},{2,3,4},{1,2,3},{2,3,4},{1,2,3},{2,3,4},{1,2,3},{2,3,4},{1,2,3}}
 *
 * If the string has un-parsable elements the parser will fail with nullopt:
 *   "1/2 sdf" : nullopt !
 *
 * It also fails if the string is empty:
 *   "": nullopt !
 *
 * Also fails if the rows are not of the same size.
 */
struct GTSAM_EXPORT SignatureParser {
  using Row = std::vector<double>;
  using Table = std::vector<Row>;

  static std::optional<Table> Parse(const std::string& str);
};
}  // namespace gtsam
