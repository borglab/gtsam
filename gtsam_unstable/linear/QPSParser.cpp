/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file     QPSParser.cpp
 * @author   Ivan Dario Jimenez
 * @date     3/5/16
 */

#define BOOST_SPIRIT_USE_PHOENIX_V3 1

#include <gtsam/base/Matrix.h>
#include <gtsam/inference/Key.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam_unstable/linear/QP.h>
#include <gtsam_unstable/linear/QPSParser.h>
#include <gtsam_unstable/linear/QPSParserException.h>

#include <boost/fusion/include/vector.hpp>
#include <boost/fusion/sequence.hpp>
#include <boost/lambda/lambda.hpp>
#include <boost/phoenix/bind.hpp>
#include <boost/spirit/include/classic.hpp>
#include <boost/spirit/include/qi.hpp>

#include <algorithm>
#include <iostream>
#include <map>
#include <string>
#include <unordered_map>
#include <vector>

using boost::fusion::at_c;
using namespace std::placeholders;
using namespace std;

namespace bf = boost::fusion;
namespace qi = boost::spirit::qi;

using Chars = std::vector<char>;

// Get a string from a fusion vector of Chars
template <size_t I, class FusionVector>
static string fromChars(const FusionVector &vars) {
  const Chars &chars = at_c<I>(vars);
  return string(chars.begin(), chars.end());
}

namespace gtsam {

/**
 * As the parser reads a file, it call functions in this visitor. This visitor
 * in turn stores what the parser has read in a way that can be later used to
 * build the full QP problem in the file.
 */
class QPSVisitor {
 private:
  typedef std::unordered_map<Key, Matrix11> coefficient_v;
  typedef std::unordered_map<std::string, coefficient_v> constraint_v;

  std::unordered_map<std::string, constraint_v *>
      row_to_constraint_v;  // Maps QPS ROWS to Variable-Matrix pairs
  constraint_v E;           // Equalities
  constraint_v IG;          // Inequalities >=
  constraint_v IL;          // Inequalities <=
  unsigned int numVariables;
  std::unordered_map<std::string, double>
      b;  // maps from constraint name to b value for Ax = b equality
          // constraints
  std::unordered_map<std::string, double>
      ranges;  // Inequalities can be specified as ranges on a variable
  std::unordered_map<Key, Vector1> g;  // linear term of quadratic cost
  std::unordered_map<std::string, Key>
      varname_to_key;  // Variable QPS string name to key
  std::unordered_map<Key, std::unordered_map<Key, Matrix11>>
      H;                 // H from hessian
  double f = 0;          // Constant term of quadratic cost
  std::string obj_name;  // the objective function has a name in the QPS
  std::string name_;     // the quadratic program has a name in the QPS
  std::unordered_map<Key, double>
      up;  // Upper Bound constraints on variable where X < MAX
  std::unordered_map<Key, double>
      lo;  // Lower Bound constraints on variable where MIN < X
  std::unordered_map<Key, double>
      fx;          // Equalities specified as FX in BOUNDS part of QPS
  KeyVector free;  // Variables can be specified as free (to which no
                   // constraints apply)
  const bool debug = false;

 public:
  QPSVisitor() : numVariables(1) {}

  void setName(boost::fusion::vector<Chars, Chars, Chars> const &name) {
    name_ = fromChars<1>(name);
    if (debug) {
      cout << "Parsing file: " << name_ << endl;
    }
  }

  void addColumn(boost::fusion::vector<Chars, Chars, Chars, Chars, Chars,
                                       double, Chars> const &vars) {
    string var_ = fromChars<1>(vars);
    string row_ = fromChars<3>(vars);
    Matrix11 coefficient = at_c<5>(vars) * I_1x1;
    if (debug) {
      cout << "Added Column for Var: " << var_ << " Row: " << row_
           << " Coefficient: " << coefficient << endl;
    }
    if (!varname_to_key.count(var_))
      varname_to_key[var_] = Symbol('X', numVariables++);
    if (row_ == obj_name) {
      g[varname_to_key[var_]] = coefficient;
      return;
    }
    (*row_to_constraint_v[row_])[row_][varname_to_key[var_]] = coefficient;
  }

  void addColumnDouble(
      boost::fusion::vector<Chars, Chars, Chars, Chars, double, Chars, Chars,
                            Chars, double> const &vars) {
    string var_ = fromChars<0>(vars);
    string row1_ = fromChars<2>(vars);
    string row2_ = fromChars<6>(vars);
    Matrix11 coefficient1 = at_c<4>(vars) * I_1x1;
    Matrix11 coefficient2 = at_c<8>(vars) * I_1x1;
    if (!varname_to_key.count(var_))
      varname_to_key.insert({var_, Symbol('X', numVariables++)});
    if (row1_ == obj_name)
      g[varname_to_key[var_]] = coefficient1;
    else
      (*row_to_constraint_v[row1_])[row1_][varname_to_key[var_]] = coefficient1;
    if (row2_ == obj_name)
      g[varname_to_key[var_]] = coefficient2;
    else
      (*row_to_constraint_v[row2_])[row2_][varname_to_key[var_]] = coefficient2;
  }

  void addRangeSingle(boost::fusion::vector<Chars, Chars, Chars, Chars, Chars,
                                            double, Chars> const &vars) {
    string var_ = fromChars<1>(vars);
    string row_ = fromChars<3>(vars);
    double range = at_c<5>(vars);
    ranges[row_] = range;
    if (debug) {
      cout << "SINGLE RANGE ADDED" << endl;
      cout << "VAR:" << var_ << " ROW: " << row_ << " RANGE: " << range << endl;
    }
  }
  void addRangeDouble(
      boost::fusion::vector<Chars, Chars, Chars, Chars, Chars, double, Chars,
                            Chars, Chars, double> const &vars) {
    string var_ = fromChars<1>(vars);
    string row1_ = fromChars<3>(vars);
    string row2_ = fromChars<7>(vars);
    double range1 = at_c<5>(vars);
    double range2 = at_c<9>(vars);
    ranges[row1_] = range1;
    ranges[row2_] = range2;
    if (debug) {
      cout << "DOUBLE RANGE ADDED" << endl;
      cout << "VAR: " << var_ << " ROW1: " << row1_ << " RANGE1: " << range1
           << " ROW2: " << row2_ << " RANGE2: " << range2 << endl;
    }
  }

  void addRHS(boost::fusion::vector<Chars, Chars, Chars, Chars, Chars, double,
                                    Chars> const &vars) {
    string var_ = fromChars<1>(vars);
    string row_ = fromChars<3>(vars);
    double coefficient = at_c<5>(vars);
    if (row_ == obj_name) {
      f = -coefficient;
    } else {
      b[row_] = coefficient;
    }

    if (debug) {
      cout << "Added RHS for Var: " << var_ << " Row: " << row_
           << " Coefficient: " << coefficient << endl;
    }
  }

  void addRHSDouble(
      boost::fusion::vector<Chars, Chars, Chars, Chars, Chars, double, Chars,
                            Chars, Chars, double> const &vars) {
    string var_ = fromChars<1>(vars);
    string row1_ = fromChars<3>(vars);
    string row2_ = fromChars<7>(vars);
    double coefficient1 = at_c<5>(vars);
    double coefficient2 = at_c<9>(vars);
    if (row1_ == obj_name) {
      f = -coefficient1;
    } else {
      b[row1_] = coefficient1;
    }

    if (row2_ == obj_name) {
      f = -coefficient2;
    } else {
      b[row2_] = coefficient2;
    }

    if (debug) {
      cout << "Added RHS for Var: " << var_ << " Row: " << row1_
           << " Coefficient: " << coefficient1 << endl;
      cout << "                      "
           << "Row: " << row2_ << " Coefficient: " << coefficient2 << endl;
    }
  }

  void addRow(
      boost::fusion::vector<Chars, char, Chars, Chars, Chars> const &vars) {
    string name_ = fromChars<3>(vars);
    char type = at_c<1>(vars);
    switch (type) {
      case 'N':
        obj_name = name_;
        break;
      case 'L':
        row_to_constraint_v[name_] = &IL;
        break;
      case 'G':
        row_to_constraint_v[name_] = &IG;
        break;
      case 'E':
        row_to_constraint_v[name_] = &E;
        break;
      default:
        cout << "invalid type: " << type << endl;
        break;
    }
    if (debug) {
      cout << "Added Row Type: " << type << " Name: " << name_ << endl;
    }
  }

  void addBound(boost::fusion::vector<Chars, Chars, Chars, Chars, Chars, Chars,
                                      Chars, double> const &vars) {
    string type_ = fromChars<1>(vars);
    string var_ = fromChars<5>(vars);
    double number = at_c<7>(vars);
    if (type_.compare(string("UP")) == 0)
      up[varname_to_key[var_]] = number;
    else if (type_.compare(string("LO")) == 0)
      lo[varname_to_key[var_]] = number;
    else if (type_.compare(string("FX")) == 0)
      fx[varname_to_key[var_]] = number;
    else
      cout << "Invalid Bound Type: " << type_ << endl;

    if (debug) {
      cout << "Added Bound Type: " << type_ << " Var: " << var_
           << " Amount: " << number << endl;
    }
  }

  void addFreeBound(boost::fusion::vector<Chars, Chars, Chars, Chars, Chars,
                                          Chars, Chars> const &vars) {
    string type_ = fromChars<1>(vars);
    string var_ = fromChars<5>(vars);
    free.push_back(varname_to_key[var_]);
    if (debug) {
      cout << "Added Free Bound Type: " << type_ << " Var: " << var_
           << " Amount: " << endl;
    }
  }

  void addQuadTerm(boost::fusion::vector<Chars, Chars, Chars, Chars, Chars,
                                         double, Chars> const &vars) {
    string var1_ = fromChars<1>(vars);
    string var2_ = fromChars<3>(vars);
    Matrix11 coefficient = at_c<5>(vars) * I_1x1;

    H[varname_to_key[var1_]][varname_to_key[var2_]] = coefficient;
    H[varname_to_key[var2_]][varname_to_key[var1_]] = coefficient;
    if (debug) {
      cout << "Added QuadTerm for Var: " << var1_ << " Row: " << var2_
           << " Coefficient: " << coefficient << endl;
    }
  }

  QP makeQP() {
    // Create the keys from the variable names
    KeyVector keys;
    for (auto kv : varname_to_key) {
      keys.push_back(kv.second);
    }

    // Fill the G matrices and g vectors from
    vector<Matrix> Gs;
    vector<Vector> gs;
    sort(keys.begin(), keys.end());
    for (size_t i = 0; i < keys.size(); ++i) {
      for (size_t j = i; j < keys.size(); ++j) {
        if (H.count(keys[i]) > 0 && H[keys[i]].count(keys[j]) > 0) {
          Gs.emplace_back(H[keys[i]][keys[j]]);
        } else {
          Gs.emplace_back(Z_1x1);
        }
      }
    }
    for (Key key1 : keys) {
      if (g.count(key1) > 0) {
        gs.emplace_back(-g[key1]);
      } else {
        gs.emplace_back(Z_1x1);
      }
    }

    // Construct the quadratic program
    QP madeQP;
    auto obj = HessianFactor(keys, Gs, gs, 2 * f);
    madeQP.cost.push_back(obj);

    // Add equality and inequality constraints into the QP
    size_t dual_key_num = keys.size() + 1;
    for (auto kv : E) {
      map<Key, Matrix11> keyMatrixMapPos;
      map<Key, Matrix11> keyMatrixMapNeg;
      if (ranges.count(kv.first) == 1) {
        for (auto km : kv.second) {
          keyMatrixMapPos.insert(km);
          km.second = -km.second;
          keyMatrixMapNeg.insert(km);
        }
        if (ranges[kv.first] > 0) {
          madeQP.inequalities.push_back(
              LinearInequality(keyMatrixMapNeg, -b[kv.first], dual_key_num++));
          madeQP.inequalities.push_back(LinearInequality(
              keyMatrixMapPos, b[kv.first] + ranges[kv.first], dual_key_num++));
        } else if (ranges[kv.first] < 0) {
          madeQP.inequalities.push_back(
              LinearInequality(keyMatrixMapPos, b[kv.first], dual_key_num++));
          madeQP.inequalities.push_back(LinearInequality(
              keyMatrixMapNeg, ranges[kv.first] - b[kv.first], dual_key_num++));
        } else {
          cerr << "ERROR: CANNOT ADD A RANGE OF ZERO" << endl;
          throw;
        }
        continue;
      }
      map<Key, Matrix11> keyMatrixMap;
      for (auto km : kv.second) {
        keyMatrixMap.insert(km);
      }
      madeQP.equalities.push_back(
          LinearEquality(keyMatrixMap, b[kv.first] * I_1x1, dual_key_num++));
    }

    for (auto kv : IG) {
      map<Key, Matrix11> keyMatrixMapNeg;
      map<Key, Matrix11> keyMatrixMapPos;
      for (auto km : kv.second) {
        keyMatrixMapPos.insert(km);
        km.second = -km.second;
        keyMatrixMapNeg.insert(km);
      }
      madeQP.inequalities.push_back(
          LinearInequality(keyMatrixMapNeg, -b[kv.first], dual_key_num++));
      if (ranges.count(kv.first) == 1) {
        madeQP.inequalities.push_back(LinearInequality(
            keyMatrixMapPos, b[kv.first] + ranges[kv.first], dual_key_num++));
      }
    }

    for (auto kv : IL) {
      map<Key, Matrix11> keyMatrixMapPos;
      map<Key, Matrix11> keyMatrixMapNeg;
      for (auto km : kv.second) {
        keyMatrixMapPos.insert(km);
        km.second = -km.second;
        keyMatrixMapNeg.insert(km);
      }
      madeQP.inequalities.push_back(
          LinearInequality(keyMatrixMapPos, b[kv.first], dual_key_num++));
      if (ranges.count(kv.first) == 1) {
        madeQP.inequalities.push_back(LinearInequality(
            keyMatrixMapNeg, ranges[kv.first] - b[kv.first], dual_key_num++));
      }
    }

    for (Key k : keys) {
      if (find(free.begin(), free.end(), k) != free.end()) continue;
      if (fx.count(k) == 1)
        madeQP.equalities.push_back(
            LinearEquality(k, I_1x1, fx[k] * I_1x1, dual_key_num++));
      if (up.count(k) == 1)
        madeQP.inequalities.push_back(
            LinearInequality(k, I_1x1, up[k], dual_key_num++));
      if (lo.count(k) == 1)
        madeQP.inequalities.push_back(
            LinearInequality(k, -I_1x1, -lo[k], dual_key_num++));
      else
        madeQP.inequalities.push_back(
            LinearInequality(k, -I_1x1, 0, dual_key_num++));
    }
    return madeQP;
  }
};

typedef qi::grammar<boost::spirit::basic_istream_iterator<char>> base_grammar;

struct QPSParser::MPSGrammar : base_grammar {
  typedef std::vector<char> Chars;
  QPSVisitor *rqp_;
  std::function<void(bf::vector<Chars, Chars, Chars> const &)> setName;
  std::function<void(bf::vector<Chars, char, Chars, Chars, Chars> const &)>
      addRow;
  std::function<void(
      bf::vector<Chars, Chars, Chars, Chars, Chars, double, Chars> const &)>
      rhsSingle;
  std::function<void(bf::vector<Chars, Chars, Chars, Chars, Chars, double,
                                  Chars, Chars, Chars, double>)>
      rhsDouble;
  std::function<void(
      bf::vector<Chars, Chars, Chars, Chars, Chars, double, Chars> const &)>
      rangeSingle;
  std::function<void(bf::vector<Chars, Chars, Chars, Chars, Chars, double,
                                  Chars, Chars, Chars, double>)>
      rangeDouble;
  std::function<void(
      bf::vector<Chars, Chars, Chars, Chars, Chars, double, Chars>)>
      colSingle;
  std::function<void(bf::vector<Chars, Chars, Chars, Chars, double, Chars,
                                  Chars, Chars, double> const &)>
      colDouble;
  std::function<void(
      bf::vector<Chars, Chars, Chars, Chars, Chars, double, Chars> const &)>
      addQuadTerm;
  std::function<void(bf::vector<Chars, Chars, Chars, Chars, Chars, Chars,
                                  Chars, double> const &)>
      addBound;
  std::function<void(
      bf::vector<Chars, Chars, Chars, Chars, Chars, Chars, Chars> const &)>
      addFreeBound;
  MPSGrammar(QPSVisitor *rqp)
      : base_grammar(start),
        rqp_(rqp),
        setName(std::bind(&QPSVisitor::setName, rqp, std::placeholders::_1)),
        addRow(std::bind(&QPSVisitor::addRow, rqp, std::placeholders::_1)),
        rhsSingle(std::bind(&QPSVisitor::addRHS, rqp, std::placeholders::_1)),
        rhsDouble(std::bind(&QPSVisitor::addRHSDouble, rqp, std::placeholders::_1)),
        rangeSingle(std::bind(&QPSVisitor::addRangeSingle, rqp, std::placeholders::_1)),
        rangeDouble(std::bind(&QPSVisitor::addRangeDouble, rqp, std::placeholders::_1)),
        colSingle(std::bind(&QPSVisitor::addColumn, rqp, std::placeholders::_1)),
        colDouble(std::bind(&QPSVisitor::addColumnDouble, rqp, std::placeholders::_1)),
        addQuadTerm(std::bind(&QPSVisitor::addQuadTerm, rqp, std::placeholders::_1)),
        addBound(std::bind(&QPSVisitor::addBound, rqp, std::placeholders::_1)),
        addFreeBound(std::bind(&QPSVisitor::addFreeBound, rqp, std::placeholders::_1)) {
    using namespace boost::spirit;
    using namespace boost::spirit::qi;
    character = lexeme[alnum | '_' | '-' | '.'];
    title = lexeme[character >> *(blank | character)];
    word = lexeme[+character];
    name = lexeme[lit("NAME") >> *blank >> title >> +space][setName];
    row = lexeme[*blank >> character >> +blank >> word >> *blank][addRow];
    rhs_single = lexeme[*blank >> word >> +blank >> word >> +blank >> double_ >>
                        *blank][rhsSingle];
    rhs_double =
        lexeme[(*blank >> word >> +blank >> word >> +blank >> double_ >>
                +blank >> word >> +blank >> double_)[rhsDouble] >>
               *blank];
    range_single = lexeme[*blank >> word >> +blank >> word >> +blank >>
                          double_ >> *blank][rangeSingle];
    range_double =
        lexeme[(*blank >> word >> +blank >> word >> +blank >> double_ >>
                +blank >> word >> +blank >> double_)[rangeDouble] >>
               *blank];
    col_single = lexeme[*blank >> word >> +blank >> word >> +blank >> double_ >>
                        *blank][colSingle];
    col_double =
        lexeme[*blank >> (word >> +blank >> word >> +blank >> double_ >>
                          +blank >> word >> +blank >> double_)[colDouble] >>
               *blank];
    quad_l = lexeme[*blank >> word >> +blank >> word >> +blank >> double_ >>
                    *blank][addQuadTerm];
    bound = lexeme[(*blank >> word >> +blank >> word >> +blank >> word >>
                    +blank >> double_)[addBound] >>
                   *blank];
    bound_fr = lexeme[*blank >> word >> +blank >> word >> +blank >> word >>
                      *blank][addFreeBound];
    rows = lexeme[lit("ROWS") >> *blank >> eol >> +(row >> eol)];
    rhs = lexeme[lit("RHS") >> *blank >> eol >>
                 +((rhs_double | rhs_single) >> eol)];
    cols = lexeme[lit("COLUMNS") >> *blank >> eol >>
                  +((col_double | col_single) >> eol)];
    quad = lexeme[lit("QUADOBJ") >> *blank >> eol >> +(quad_l >> eol)];
    bounds = lexeme[lit("BOUNDS") >> +space >> *((bound | bound_fr) >> eol)];
    ranges = lexeme[lit("RANGES") >> +space >>
                    *((range_double | range_single) >> eol)];
    end = lexeme[lit("ENDATA") >> *space];
    start =
        lexeme[name >> rows >> cols >> rhs >> -ranges >> bounds >> quad >> end];
  }

  qi::rule<boost::spirit::basic_istream_iterator<char>, char()> character;
  qi::rule<boost::spirit::basic_istream_iterator<char>, Chars()> word, title;
  qi::rule<boost::spirit::basic_istream_iterator<char>> row, end, col_single,
      col_double, rhs_single, rhs_double, range_single, range_double, ranges,
      bound, bound_fr, bounds, quad, quad_l, rows, cols, rhs, name, start;
};

QP QPSParser::Parse() {
  QPSVisitor rawData;
  std::fstream stream(fileName_.c_str());
  stream.unsetf(std::ios::skipws);
  boost::spirit::basic_istream_iterator<char> begin(stream);
  boost::spirit::basic_istream_iterator<char> last;

  if (!parse(begin, last, MPSGrammar(&rawData)) || begin != last) {
    throw QPSParserException();
  }

  return rawData.makeQP();
}

}  // namespace gtsam
