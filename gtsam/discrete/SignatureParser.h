/**
 * This is a simple parser that replaces the boost spirit parser. It is
 * meant to parse strings like "1/1 2/3 1/4". Every word of the form "a/b/c/..."
 * should be parsed as a row, and the rows should be stored in a table.
 * The elements of the row will be doubles.
 * The table is a vector of rows. The row is a vector of doubles.
 * The parser should be able to parse the following strings:
 * "1/1 2/3 1/4": {{1,1},{2,3},{1,4}}
 * "1/1 2/3 1/4 1/1 2/3 1/4" : {{1,1},{2,3},{1,4},{1,1},{2,3},{1,4}}
 * "1/2/3 2/3/4 1/2/3 2/3/4 1/2/3 2/3/4 1/2/3 2/3/4 1/2/3" : {{1,2,3},{2,3,4},{1,2,3},{2,3,4},{1,2,3},{2,3,4},{1,2,3},{2,3,4},{1,2,3}}
 * If the string has unparseable elements, the parser should parse whatever it can
 * "1/2 sdf" : {{1,2}}
 * It should return false if the string is empty.
 * "": false
 * We should return false if the rows are not of the same size.
 */

#pragma once
#include <iostream>
#include <string>
#include <vector>

namespace gtsam {
namespace SignatureParser {
typedef std::vector<double> Row;
typedef std::vector<Row> Table;

bool parse(const std::string& str, Table& table);
};  // namespace SignatureParser
}  // namespace gtsam
