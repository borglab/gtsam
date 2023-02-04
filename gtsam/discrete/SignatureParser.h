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
#include <iostream>
#include <string>
#include <vector>

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
 * If the string has un-parsable elements, should parse whatever it can:
 *   "1/2 sdf" : {{1,2}}
 *
 * It should return false if the string is empty:
 *   "": false
 *
 * We should return false if the rows are not of the same size.
 */
namespace SignatureParser {
typedef std::vector<double> Row;
typedef std::vector<Row> Table;

bool parse(const std::string& str, Table& table);
};  // namespace SignatureParser
}  // namespace gtsam
