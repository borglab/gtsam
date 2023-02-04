#include <gtsam/discrete/SignatureParser.h>

#include <algorithm>
#include <iterator>
#include <sstream>

namespace gtsam {

inline static std::vector<double> ParseTrueRow() { return {0, 1}; }

inline static std::vector<double> ParseFalseRow() { return {1, 0}; }

inline static SignatureParser::Table ParseOr() {
  return {ParseFalseRow(), ParseTrueRow(), ParseTrueRow(), ParseTrueRow()};
}

inline static SignatureParser::Table ParseAnd() {
  return {ParseFalseRow(), ParseFalseRow(), ParseFalseRow(), ParseTrueRow()};
}

bool static ParseConditional(const std::string& token,
                             std::vector<double>& row) {
  // Expect something like a/b/c
  std::istringstream iss2(token);
  try {
    // if the string has no / then return false
    if (std::count(token.begin(), token.end(), '/') == 0) return false;
    // split the word on the '/' character
    for (std::string s; std::getline(iss2, s, '/');) {
      // can throw exception
      row.push_back(std::stod(s));
    }
  } catch (...) {
    return false;
  }
  return true;
}

void static ParseConditionalTable(const std::vector<std::string>& tokens,
                                  SignatureParser::Table& table) {
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
      std::vector<double> row;
      if (!ParseConditional(word, row)) {
        // stop parsing if we encounter an error
        return;
      }
      table.push_back(row);
    }
  }
}

std::vector<std::string> static Tokenize(const std::string& str) {
  std::istringstream iss(str);
  std::vector<std::string> tokens;
  for (std::string s; iss >> s;) {
    tokens.push_back(s);
  }
  return tokens;
}

bool SignatureParser::parse(const std::string& str, Table& table) {
  // check if string is just whitespace
  if (std::all_of(str.begin(), str.end(), isspace)) {
    return false;
  }

  // return false if the string is empty
  if (str.empty()) {
    return false;
  }

  // tokenize the str on whitespace
  std::vector<std::string> tokens = Tokenize(str);

  // if the first token is "OR", return the OR table
  if (tokens[0] == "OR") {
    // if there are more tokens, return false
    if (tokens.size() > 1) {
      return false;
    }
    table = ParseOr();
    return true;
  }

  // if the first token is "AND", return the AND table
  if (tokens[0] == "AND") {
    // if there are more tokens, return false
    if (tokens.size() > 1) {
      return false;
    }
    table = ParseAnd();
    return true;
  }

  // otherwise then parse the conditional table
  ParseConditionalTable(tokens, table);
  // return false if the table is empty
  if (table.empty()) {
    return false;
  }
  // the boost::phoenix parser did not return an error if we could not fully
  // parse a string it just returned whatever it could parse
  return true;
}
}  // namespace gtsam
