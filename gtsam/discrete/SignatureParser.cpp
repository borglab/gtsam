#include <gtsam/discrete/SignatureParser.h>

#include <algorithm>
#include <iterator>
#include <optional>
#include <sstream>
namespace gtsam {

using Row = std::vector<double>;
using Table = std::vector<Row>;

inline static Row ParseTrueRow() { return {0, 1}; }

inline static Row ParseFalseRow() { return {1, 0}; }

inline static Table ParseOr() {
  return {ParseFalseRow(), ParseTrueRow(), ParseTrueRow(), ParseTrueRow()};
}

inline static Table ParseAnd() {
  return {ParseFalseRow(), ParseFalseRow(), ParseFalseRow(), ParseTrueRow()};
}

std::optional<Row> static ParseConditional(const std::string& token) {
  // Expect something like a/b/c
  std::istringstream iss2(token);
  Row row;
  try {
    // if the string has no / then return std::nullopt
    if (std::count(token.begin(), token.end(), '/') == 0) return std::nullopt;
    // split the word on the '/' character
    for (std::string s; std::getline(iss2, s, '/');) {
      // can throw exception
      row.push_back(std::stod(s));
    }
  } catch (...) {
    return std::nullopt;
  }
  return row;
}

std::optional<Table> static ParseConditionalTable(
    const std::vector<std::string>& tokens) {
  Table table;
  // loop over the words
  // for each word, split it into doubles using a stringstream
  for (const auto& word : tokens) {
    // If the string word is F or T then the row is {0,1} or {1,0} respectively
    if (word == "F") {
      table.push_back(ParseFalseRow());
    } else if (word == "T") {
      table.push_back(ParseTrueRow());
    } else {
      // Expect something like a/b/c
      if (auto row = ParseConditional(word)) {
        table.push_back(*row);
      } else {
        // stop parsing if we encounter an error
        return std::nullopt;
      }
    }
  }
  return table;
}

std::vector<std::string> static Tokenize(const std::string& str) {
  std::istringstream iss(str);
  std::vector<std::string> tokens;
  for (std::string s; iss >> s;) {
    tokens.push_back(s);
  }
  return tokens;
}

std::optional<Table> SignatureParser::Parse(const std::string& str) {
  // check if string is just whitespace
  if (std::all_of(str.begin(), str.end(), isspace)) {
    return std::nullopt;
  }

  // return std::nullopt if the string is empty
  if (str.empty()) {
    return std::nullopt;
  }

  // tokenize the str on whitespace
  std::vector<std::string> tokens = Tokenize(str);

  // if the first token is "OR", return the OR table
  if (tokens[0] == "OR") {
    // if there are more tokens, return std::nullopt
    if (tokens.size() > 1) {
      return std::nullopt;
    }
    return ParseOr();
  }

  // if the first token is "AND", return the AND table
  if (tokens[0] == "AND") {
    // if there are more tokens, return std::nullopt
    if (tokens.size() > 1) {
      return std::nullopt;
    }
    return ParseAnd();
  }

  // otherwise then parse the conditional table
  auto table = ParseConditionalTable(tokens);
  // return std::nullopt if the table is empty
  if (!table || table->empty()) {
    return std::nullopt;
  }
  // the boost::phoenix parser did not return an error if we could not fully
  // parse a string it just returned whatever it could parse
  return table;
}
}  // namespace gtsam
