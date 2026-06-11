/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    QpsParser.cpp
 * @brief   Parser for QPS quadratic programming files.
 * @author  Frank Dellaert
 */

#include <gtsam/constrained/LinearConstraint.h>
#include <gtsam/constrained/QpsParser.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/HessianFactor.h>
#include <gtsam/linear/JacobianFactor.h>

#include <algorithm>
#include <cctype>
#include <fstream>
#include <map>
#include <optional>
#include <set>
#include <sstream>
#include <utility>
#include <vector>

namespace gtsam {
namespace {

enum class Section { None, Rows, Columns, Rhs, Ranges, Bounds, QuadObj, End };

enum class RowSense { Objective, Equal, LessEqual, GreaterEqual };

using ScalarTerms = std::map<Key, double>;

/* ************************************************************************* */
std::string Trim(const std::string& text) {
  const auto begin = std::find_if_not(text.begin(), text.end(), [](char c) {
    return std::isspace(static_cast<unsigned char>(c));
  });
  const auto end = std::find_if_not(text.rbegin(), text.rend(), [](char c) {
                     return std::isspace(static_cast<unsigned char>(c));
                   }).base();
  return begin < end ? std::string(begin, end) : std::string();
}

/* ************************************************************************* */
std::vector<std::string> Split(const std::string& line) {
  std::istringstream stream(line);
  std::vector<std::string> tokens;
  std::string token;
  while (stream >> token) {
    tokens.push_back(token);
  }
  return tokens;
}

/* ************************************************************************* */
std::string Location(const std::string& sourceName, size_t lineNumber) {
  std::ostringstream stream;
  if (!sourceName.empty()) {
    stream << sourceName << ":" << lineNumber;
  } else {
    stream << "line " << lineNumber;
  }
  return stream.str();
}

/* ************************************************************************* */
[[noreturn]] void ThrowParseError(const std::string& sourceName,
                                  size_t lineNumber,
                                  const std::string& message) {
  throw QpsParserException(Location(sourceName, lineNumber) + ": " + message);
}

/* ************************************************************************* */
double ParseDouble(const std::string& token, const std::string& sourceName,
                   size_t lineNumber) {
  size_t parsed = 0;
  try {
    const double value = std::stod(token, &parsed);
    if (parsed != token.size()) {
      ThrowParseError(sourceName, lineNumber,
                      "invalid numeric token '" + token + "'");
    }
    return value;
  } catch (const std::invalid_argument&) {
    ThrowParseError(sourceName, lineNumber,
                    "invalid numeric token '" + token + "'");
  } catch (const std::out_of_range&) {
    ThrowParseError(sourceName, lineNumber,
                    "numeric token out of range '" + token + "'");
  }
}

/* ************************************************************************* */
Matrix ScalarMatrix(double value) { return (Matrix(1, 1) << value).finished(); }

/* ************************************************************************* */
Vector ScalarVector(double value) { return (Vector(1) << value).finished(); }

/* ************************************************************************* */
bool IsSectionName(const std::string& token) {
  return token == "ROWS" || token == "COLUMNS" || token == "RHS" ||
         token == "RANGES" || token == "BOUNDS" || token == "QUADOBJ" ||
         token == "ENDATA";
}

/* ************************************************************************* */
Section SectionForName(const std::string& token) {
  if (token == "ROWS") return Section::Rows;
  if (token == "COLUMNS") return Section::Columns;
  if (token == "RHS") return Section::Rhs;
  if (token == "RANGES") return Section::Ranges;
  if (token == "BOUNDS") return Section::Bounds;
  if (token == "QUADOBJ") return Section::QuadObj;
  if (token == "ENDATA") return Section::End;
  return Section::None;
}

/* ************************************************************************* */
std::pair<Key, Key> OrderedPair(Key key1, Key key2) {
  return key1 <= key2 ? std::make_pair(key1, key2) : std::make_pair(key2, key1);
}

/* ************************************************************************* */
class QpsBuilder {
 public:
  QpsBuilder(std::string sourceName) : sourceName_(std::move(sourceName)) {}

  void setName(const std::string& line) {
    result_.name = Trim(line.substr(std::string("NAME").size()));
  }

  void addRow(const std::vector<std::string>& tokens, size_t lineNumber) {
    if (tokens.size() != 2 || tokens[0].size() != 1) {
      ThrowParseError(sourceName_, lineNumber,
                      "ROWS entries must contain a row type and row name");
    }

    const char type = tokens[0][0];
    const std::string& rowName = tokens[1];
    switch (type) {
      case 'N':
        result_.objectiveName = rowName;
        break;
      case 'E':
        rowSenses_[rowName] = RowSense::Equal;
        rowOrder_.push_back(rowName);
        break;
      case 'L':
        rowSenses_[rowName] = RowSense::LessEqual;
        rowOrder_.push_back(rowName);
        break;
      case 'G':
        rowSenses_[rowName] = RowSense::GreaterEqual;
        rowOrder_.push_back(rowName);
        break;
      default:
        ThrowParseError(sourceName_, lineNumber,
                        "unsupported ROWS type '" + tokens[0] + "'");
    }
  }

  void addColumn(const std::vector<std::string>& tokens, size_t lineNumber) {
    if (tokens.size() != 3 && tokens.size() != 5) {
      ThrowParseError(
          sourceName_, lineNumber,
          "COLUMNS entries must contain one or two row/value pairs");
    }
    for (size_t index = 1; index < tokens.size(); index += 2) {
      const double value =
          ParseDouble(tokens[index + 1], sourceName_, lineNumber);
      addColumnEntry(tokens[0], tokens[index], value, lineNumber);
    }
  }

  void addRhs(const std::vector<std::string>& tokens, size_t lineNumber) {
    if (tokens.size() != 3 && tokens.size() != 5) {
      ThrowParseError(sourceName_, lineNumber,
                      "RHS entries must contain one or two row/value pairs");
    }
    for (size_t index = 1; index < tokens.size(); index += 2) {
      const double value =
          ParseDouble(tokens[index + 1], sourceName_, lineNumber);
      if (tokens[index] == result_.objectiveName) {
        objectiveConstant_ = -value;
      } else {
        checkConstraintRow(tokens[index], lineNumber);
        rhs_[tokens[index]] = value;
      }
    }
  }

  void addRange(const std::vector<std::string>& tokens, size_t lineNumber) {
    if (tokens.size() != 3 && tokens.size() != 5) {
      ThrowParseError(sourceName_, lineNumber,
                      "RANGES entries must contain one or two row/value pairs");
    }
    for (size_t index = 1; index < tokens.size(); index += 2) {
      checkConstraintRow(tokens[index], lineNumber);
      ranges_[tokens[index]] =
          ParseDouble(tokens[index + 1], sourceName_, lineNumber);
    }
  }

  void addBound(const std::vector<std::string>& tokens, size_t lineNumber) {
    if (tokens.empty()) {
      return;
    }
    const std::string& type = tokens[0];
    if (type == "FR") {
      if (tokens.size() != 3) {
        ThrowParseError(sourceName_, lineNumber,
                        "FR bounds must contain a bound set and variable");
      }
      freeVariables_.insert(keyForVariable(tokens[2]));
      return;
    }

    if (tokens.size() != 4) {
      ThrowParseError(sourceName_, lineNumber,
                      "bounds must contain a type, bound set, variable, and "
                      "value");
    }

    const Key key = keyForVariable(tokens[2]);
    const double value = ParseDouble(tokens[3], sourceName_, lineNumber);
    if (type == "UP") {
      upperBounds_[key] = value;
    } else if (type == "LO") {
      lowerBounds_[key] = value;
    } else if (type == "FX") {
      fixedBounds_[key] = value;
    } else {
      ThrowParseError(sourceName_, lineNumber,
                      "unsupported BOUNDS type '" + type + "'");
    }
  }

  void addQuadObj(const std::vector<std::string>& tokens, size_t lineNumber) {
    if (tokens.size() != 3) {
      ThrowParseError(sourceName_, lineNumber,
                      "QUADOBJ entries must contain two variables and a value");
    }
    const Key key1 = keyForVariable(tokens[0]);
    const Key key2 = keyForVariable(tokens[1]);
    hessian_[OrderedPair(key1, key2)] +=
        ParseDouble(tokens[2], sourceName_, lineNumber);
  }

  QpsParserResult build(size_t lineNumber, bool sawEnd) {
    if (!sawEnd) {
      ThrowParseError(sourceName_, lineNumber, "missing ENDATA section");
    }
    if (result_.objectiveName.empty()) {
      ThrowParseError(sourceName_, lineNumber,
                      "missing objective row in ROWS section");
    }
    if (variableOrder_.empty()) {
      ThrowParseError(sourceName_, lineNumber, "QPS file has no variables");
    }

    addCost();
    addRows();
    addBounds();
    return std::move(result_);
  }

 private:
  std::string sourceName_;
  QpsParserResult result_;
  std::vector<std::string> variableOrder_;
  std::map<std::string, RowSense> rowSenses_;
  std::vector<std::string> rowOrder_;
  std::map<std::string, ScalarTerms> rowTerms_;
  std::map<std::string, double> rhs_;
  std::map<std::string, double> ranges_;
  std::map<Key, double> linearTerms_;
  std::map<std::pair<Key, Key>, double> hessian_;
  std::map<Key, double> upperBounds_;
  std::map<Key, double> lowerBounds_;
  std::map<Key, double> fixedBounds_;
  std::set<Key> freeVariables_;
  double objectiveConstant_ = 0.0;

  Key keyForVariable(const std::string& variableName) {
    const auto existing = result_.variableKeys.find(variableName);
    if (existing != result_.variableKeys.end()) {
      return existing->second;
    }

    const Key key = Symbol('X', result_.variableKeys.size() + 1);
    result_.variableKeys.emplace(variableName, key);
    variableOrder_.push_back(variableName);
    return key;
  }

  void checkConstraintRow(const std::string& rowName, size_t lineNumber) const {
    if (rowSenses_.find(rowName) == rowSenses_.end()) {
      ThrowParseError(sourceName_, lineNumber,
                      "unknown constraint row '" + rowName + "'");
    }
  }

  void addColumnEntry(const std::string& variableName,
                      const std::string& rowName, double value,
                      size_t lineNumber) {
    const Key key = keyForVariable(variableName);
    if (rowName == result_.objectiveName) {
      linearTerms_[key] += value;
      return;
    }

    checkConstraintRow(rowName, lineNumber);
    rowTerms_[rowName][key] += value;
  }

  void addCost() {
    KeyVector keys;
    keys.reserve(variableOrder_.size());
    for (const std::string& variableName : variableOrder_) {
      keys.push_back(result_.variableKeys.at(variableName));
    }

    std::vector<Matrix> hessianBlocks;
    hessianBlocks.reserve(keys.size() * (keys.size() + 1) / 2);
    for (size_t i = 0; i < keys.size(); ++i) {
      for (size_t j = i; j < keys.size(); ++j) {
        const auto found = hessian_.find(OrderedPair(keys[i], keys[j]));
        hessianBlocks.push_back(
            ScalarMatrix(found != hessian_.end() ? found->second : 0.0));
      }
    }

    std::vector<Vector> gradientBlocks;
    gradientBlocks.reserve(keys.size());
    for (const Key key : keys) {
      const auto found = linearTerms_.find(key);
      gradientBlocks.push_back(
          ScalarVector(found != linearTerms_.end() ? -found->second : 0.0));
    }

    result_.problem.addCost(HessianFactor(keys, hessianBlocks, gradientBlocks,
                                          2.0 * objectiveConstant_));
  }

  JacobianFactor makeFactor(const ScalarTerms& terms, double rhs) const {
    std::vector<std::pair<Key, Matrix>> factorTerms;
    factorTerms.reserve(terms.size());
    for (const auto& [key, value] : terms) {
      factorTerms.emplace_back(key, ScalarMatrix(value));
    }
    return JacobianFactor(factorTerms, ScalarVector(rhs));
  }

  void addConstraint(const ScalarTerms& terms, double rhs,
                     LinearConstraint::Sense sense) {
    if (terms.empty()) {
      return;
    }
    const JacobianFactor factor = makeFactor(terms, rhs);
    result_.problem.addConstraint(LinearConstraint(factor, sense));
  }

  void addRangeConstraints(const ScalarTerms& terms, double rhs, double range,
                           RowSense sense) {
    if (sense == RowSense::Equal) {
      if (range > 0.0) {
        addConstraint(terms, rhs, LinearConstraint::Sense::GreaterEqual);
        addConstraint(terms, rhs + range, LinearConstraint::Sense::LessEqual);
      } else if (range < 0.0) {
        addConstraint(terms, rhs, LinearConstraint::Sense::LessEqual);
        addConstraint(terms, rhs - range,
                      LinearConstraint::Sense::GreaterEqual);
      } else {
        addConstraint(terms, rhs, LinearConstraint::Sense::Equal);
      }
      return;
    }

    if (sense == RowSense::GreaterEqual) {
      addConstraint(terms, rhs, LinearConstraint::Sense::GreaterEqual);
      addConstraint(terms, rhs + range, LinearConstraint::Sense::LessEqual);
      return;
    }

    addConstraint(terms, rhs, LinearConstraint::Sense::LessEqual);
    addConstraint(terms, rhs - range, LinearConstraint::Sense::GreaterEqual);
  }

  void addRows() {
    for (const std::string& rowName : rowOrder_) {
      const ScalarTerms& terms = rowTerms_[rowName];
      const double rhs = rhs_.count(rowName) ? rhs_.at(rowName) : 0.0;
      const RowSense sense = rowSenses_.at(rowName);
      const auto range = ranges_.find(rowName);
      if (range != ranges_.end()) {
        addRangeConstraints(terms, rhs, range->second, sense);
      } else if (sense == RowSense::Equal) {
        addConstraint(terms, rhs, LinearConstraint::Sense::Equal);
      } else if (sense == RowSense::GreaterEqual) {
        addConstraint(terms, rhs, LinearConstraint::Sense::GreaterEqual);
      } else {
        addConstraint(terms, rhs, LinearConstraint::Sense::LessEqual);
      }
    }
  }

  void addScalarConstraint(Key key, double coefficient, double rhs,
                           LinearConstraint::Sense sense) {
    result_.problem.addConstraint(LinearConstraint(
        JacobianFactor(key, ScalarMatrix(coefficient), ScalarVector(rhs)),
        sense));
  }

  void addBounds() {
    for (const std::string& variableName : variableOrder_) {
      const Key key = result_.variableKeys.at(variableName);
      if (freeVariables_.count(key)) {
        continue;
      }
      if (fixedBounds_.count(key)) {
        addScalarConstraint(key, 1.0, fixedBounds_.at(key),
                            LinearConstraint::Sense::Equal);
      }
      if (upperBounds_.count(key)) {
        addScalarConstraint(key, 1.0, upperBounds_.at(key),
                            LinearConstraint::Sense::LessEqual);
      }
      if (lowerBounds_.count(key)) {
        addScalarConstraint(key, 1.0, lowerBounds_.at(key),
                            LinearConstraint::Sense::GreaterEqual);
      } else if (!fixedBounds_.count(key)) {
        addScalarConstraint(key, 1.0, 0.0,
                            LinearConstraint::Sense::GreaterEqual);
      }
    }
  }
};

/* ************************************************************************* */
void ParseSectionLine(QpsBuilder* builder, Section section,
                      const std::vector<std::string>& tokens,
                      const std::string& sourceName, size_t lineNumber) {
  switch (section) {
    case Section::Rows:
      builder->addRow(tokens, lineNumber);
      break;
    case Section::Columns:
      builder->addColumn(tokens, lineNumber);
      break;
    case Section::Rhs:
      builder->addRhs(tokens, lineNumber);
      break;
    case Section::Ranges:
      builder->addRange(tokens, lineNumber);
      break;
    case Section::Bounds:
      builder->addBound(tokens, lineNumber);
      break;
    case Section::QuadObj:
      builder->addQuadObj(tokens, lineNumber);
      break;
    default:
      ThrowParseError(sourceName, lineNumber,
                      "data line appears before a section header");
  }
}

}  // namespace

/* ************************************************************************* */
QpsParserException::QpsParserException(const std::string& description)
    : std::runtime_error(description) {}

/* ************************************************************************* */
QpsParserResult QpsParser::parse(std::istream& input,
                                 const std::string& sourceName) {
  QpsBuilder builder(sourceName);
  Section section = Section::None;
  bool sawEnd = false;
  size_t lineNumber = 0;
  std::string line;

  while (std::getline(input, line)) {
    ++lineNumber;
    if (!line.empty() && line.back() == '\r') {
      line.pop_back();
    }

    const std::string trimmed = Trim(line);
    if (trimmed.empty() || trimmed.front() == '*') {
      continue;
    }

    const std::vector<std::string> tokens = Split(trimmed);
    if (tokens.empty()) {
      continue;
    }

    if (tokens[0] == "NAME") {
      builder.setName(trimmed);
      continue;
    }

    if (tokens.size() == 1 && IsSectionName(tokens[0])) {
      section = SectionForName(tokens[0]);
      if (section == Section::End) {
        sawEnd = true;
        break;
      }
      continue;
    }

    ParseSectionLine(&builder, section, tokens, sourceName, lineNumber);
  }

  return builder.build(lineNumber, sawEnd);
}

/* ************************************************************************* */
QpsParserResult QpsParser::parseFile(const std::string& path) {
  std::ifstream input(path);
  if (!input) {
    throw QpsParserException("could not open QPS file '" + path + "'");
  }
  return parse(input, path);
}

}  // namespace gtsam
