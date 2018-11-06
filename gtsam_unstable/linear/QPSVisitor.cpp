/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file     QPSVisitor.cpp
 * @brief    As the QPS parser reads a file, it call functions in QPSVistor.
 * This visitor in turn stores what the parser has read in a way that can be later used to build the Factor Graph for the
 * QP Constraints and cost. This intermediate representation is required because an expression in the QPS file doesn't
 * necessarily correspond to a single constraint or term in the cost function.
 * @author   Ivan Dario Jimenez
 * @date     3/5/16
 */

#include <gtsam_unstable/linear/QPSVisitor.h>
#include <iostream>

using boost::fusion::at_c;
using namespace std;

namespace gtsam {

void QPSVisitor::setName(
    boost::fusion::vector<vector<char>, vector<char>,
        vector<char>> const &name) {
  name_ = string(at_c < 1 > (name).begin(), at_c < 1 > (name).end());
  if (debug) {
    cout << "Parsing file: " << name_ << endl;
  }
}

void QPSVisitor::addColumn(
    boost::fusion::vector<vector<char>, vector<char>,
        vector<char>, vector<char>, vector<char>, double,
        vector<char>> const &vars) {

  string var_(at_c < 1 > (vars).begin(), at_c < 1 > (vars).end());
  string row_(at_c < 3 > (vars).begin(), at_c < 3 > (vars).end());
  Matrix11 coefficient = at_c < 5 > (vars) * I_1x1;
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

void QPSVisitor::addColumnDouble(
    boost::fusion::vector<vector<char>, vector<char>,
        vector<char>, vector<char>, double, vector<char>,
        vector<char>, vector<char>, double> const &vars) {

  string var_(at_c < 0 > (vars).begin(), at_c < 0 > (vars).end());
  string row1_(at_c < 2 > (vars).begin(), at_c < 2 > (vars).end());
  string row2_(at_c < 6 > (vars).begin(), at_c < 6 > (vars).end());
  Matrix11 coefficient1 = at_c < 4 > (vars) * I_1x1;
  Matrix11 coefficient2 = at_c < 8 > (vars) * I_1x1;
  if (!varname_to_key.count(var_))
    varname_to_key.insert( { var_, Symbol('X', numVariables++) });
  if (row1_ == obj_name)
    g[varname_to_key[var_]] = coefficient1;
  else
    (*row_to_constraint_v[row1_])[row1_][varname_to_key[var_]] = coefficient1;
  if (row2_ == obj_name)
    g[varname_to_key[var_]] = coefficient2;
  else
    (*row_to_constraint_v[row2_])[row2_][varname_to_key[var_]] = coefficient2;
}

void QPSVisitor::addRangeSingle(
    boost::fusion::vector<vector<char>, vector<char>,
        vector<char>, vector<char>, vector<char>, double,
        vector<char>> const & vars) {
  string var_(at_c < 1 > (vars).begin(), at_c < 1 > (vars).end());
  string row_(at_c < 3 > (vars).begin(), at_c < 3 > (vars).end());
  double range = at_c < 5 > (vars);
  ranges[row_] = range;
  if (debug) {
    cout << "SINGLE RANGE ADDED" << endl;
    cout << "VAR:" << var_ << " ROW: " << row_ << " RANGE: " << range
        << endl;
  }
  
}
void QPSVisitor::addRangeDouble(
    boost::fusion::vector<vector<char>, vector<char>,
        vector<char>, vector<char>, vector<char>, double,
        vector<char>, vector<char>, vector<char>, double> const & vars) {
  string var_(at_c < 1 > (vars).begin(), at_c < 1 > (vars).end());
  string row1_(at_c < 3 > (vars).begin(), at_c < 3 > (vars).end());
  string row2_(at_c < 7 > (vars).begin(), at_c < 7 > (vars).end());
  double range1 = at_c < 5 > (vars);
  double range2 = at_c < 9 > (vars);
  ranges[row1_] = range1;
  ranges[row2_] = range2;
  if (debug) {
    cout << "DOUBLE RANGE ADDED" << endl;
    cout << "VAR: " << var_ << " ROW1: " << row1_ << " RANGE1: " << range1
        << " ROW2: " << row2_ << " RANGE2: " << range2 << endl;
  }
  
}

void QPSVisitor::addRHS(
    boost::fusion::vector<vector<char>, vector<char>,
        vector<char>, vector<char>, vector<char>, double,
        vector<char>> const &vars) {

  string var_(at_c < 1 > (vars).begin(), at_c < 1 > (vars).end());
  string row_(at_c < 3 > (vars).begin(), at_c < 3 > (vars).end());
  double coefficient = at_c < 5 > (vars);
  if (row_ == obj_name)
    f = -coefficient;
  else
    b[row_] = coefficient;

  if (debug) {
    cout << "Added RHS for Var: " << var_ << " Row: " << row_
        << " Coefficient: " << coefficient << endl;
  }
}

void QPSVisitor::addRHSDouble(
    boost::fusion::vector<vector<char>, vector<char>,
        vector<char>, vector<char>, vector<char>, double,
        vector<char>, vector<char>, vector<char>, double> const &vars) {

  string var_(at_c < 1 > (vars).begin(), at_c < 1 > (vars).end());
  string row1_(at_c < 3 > (vars).begin(), at_c < 3 > (vars).end());
  string row2_(at_c < 7 > (vars).begin(), at_c < 7 > (vars).end());
  double coefficient1 = at_c < 5 > (vars);
  double coefficient2 = at_c < 9 > (vars);
  if (row1_ == obj_name)
    f = -coefficient1;
  else
    b[row1_] = coefficient1;

  if (row2_ == obj_name)
    f = -coefficient2;
  else
    b[row2_] = coefficient2;

  if (debug) {
    cout << "Added RHS for Var: " << var_ << " Row: " << row1_
        << " Coefficient: " << coefficient1 << endl;
    cout << "                      " << "Row: " << row2_
        << " Coefficient: " << coefficient2 << endl;
  }
}

void QPSVisitor::addRow(
    boost::fusion::vector<vector<char>, char, vector<char>,
        vector<char>, vector<char>> const &vars) {

  string name_(at_c < 3 > (vars).begin(), at_c < 3 > (vars).end());
  char type = at_c < 1 > (vars);
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

void QPSVisitor::addBound(
    boost::fusion::vector<vector<char>, vector<char>,
        vector<char>, vector<char>, vector<char>,
        vector<char>, vector<char>, double> const &vars) {

  string type_(at_c < 1 > (vars).begin(), at_c < 1 > (vars).end());
  string var_(at_c < 5 > (vars).begin(), at_c < 5 > (vars).end());
  double number = at_c < 7 > (vars);
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

void QPSVisitor::addFreeBound(
    boost::fusion::vector<vector<char>, vector<char>,
        vector<char>, vector<char>, vector<char>,
        vector<char>, vector<char>> const &vars) {
  string type_(at_c < 1 > (vars).begin(), at_c < 1 > (vars).end());
  string var_(at_c < 5 > (vars).begin(), at_c < 5 > (vars).end());
  Free.push_back(varname_to_key[var_]);
  if (debug) {
    cout << "Added Free Bound Type: " << type_ << " Var: " << var_
        << " Amount: " << endl;
  }
}

void QPSVisitor::addQuadTerm(
    boost::fusion::vector<vector<char>, vector<char>,
        vector<char>, vector<char>, vector<char>, double,
        vector<char>> const &vars) {
  string var1_(at_c < 1 > (vars).begin(), at_c < 1 > (vars).end());
  string var2_(at_c < 3 > (vars).begin(), at_c < 3 > (vars).end());
  Matrix11 coefficient = at_c < 5 > (vars) * I_1x1;

  H[varname_to_key[var1_]][varname_to_key[var2_]] = coefficient;
  H[varname_to_key[var2_]][varname_to_key[var1_]] = coefficient;
  if (debug) {
    cout << "Added QuadTerm for Var: " << var1_ << " Row: " << var2_
        << " Coefficient: " << coefficient << endl;
  }
}

QP QPSVisitor::makeQP() {
  // Create the keys from the variable names
  vector < Key > keys;
  for (auto kv : varname_to_key) {
    keys.push_back(kv.second);
  }

  // Fill the G matrices and g vectors from 
  vector < Matrix > Gs;
  vector < Vector > gs;
  sort(keys.begin(), keys.end());
  for (unsigned int i = 0; i < keys.size(); ++i) {
    for (unsigned int j = i; j < keys.size(); ++j) {
      if (H.count(keys[i]) > 0 and H[keys[i]].count(keys[j]) > 0){
        Gs.emplace_back(H[keys[i]][keys[j]]);
      }
      else{
        Gs.emplace_back(Z_1x1);
      }
    }
  }
  for (Key key1 : keys) {
    if(g.count(key1) > 0){
      gs.emplace_back(-g[key1]);
    }
    else{
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
    map < Key, Matrix11 > keyMatrixMapPos;
    map < Key, Matrix11 > keyMatrixMapNeg;
    if (ranges.count(kv.first) == 1) {
      for (auto km : kv.second) {
        keyMatrixMapPos.insert(km);
        km.second = -km.second;
        keyMatrixMapNeg.insert(km);
      }
      if (ranges[kv.first] > 0) {
        madeQP.inequalities.push_back(
            LinearInequality(keyMatrixMapNeg, -b[kv.first], dual_key_num++));
        madeQP.inequalities.push_back(
            LinearInequality(keyMatrixMapPos, b[kv.first] + ranges[kv.first],
                dual_key_num++));
      } else if (ranges[kv.first] < 0) {
        madeQP.inequalities.push_back(
            LinearInequality(keyMatrixMapPos, b[kv.first], dual_key_num++));
        madeQP.inequalities.push_back(
            LinearInequality(keyMatrixMapNeg, ranges[kv.first] - b[kv.first],
                dual_key_num++));
      } else {
        cerr << "ERROR: CANNOT ADD A RANGE OF ZERO" << endl;
        throw;
      }
      continue;
    }
    map < Key, Matrix11 > keyMatrixMap;
    for (auto km : kv.second) {
      keyMatrixMap.insert(km);
    }
    madeQP.equalities.push_back(
        LinearEquality(keyMatrixMap, b[kv.first] * I_1x1, dual_key_num++));
  }

  for (auto kv : IG) {
    map < Key, Matrix11 > keyMatrixMapNeg;
    map < Key, Matrix11 > keyMatrixMapPos;
    for (auto km : kv.second) {
      keyMatrixMapPos.insert(km);
      km.second = -km.second;
      keyMatrixMapNeg.insert(km);
    }
    madeQP.inequalities.push_back(
        LinearInequality(keyMatrixMapNeg, -b[kv.first], dual_key_num++));
    if (ranges.count(kv.first) == 1) {
      madeQP.inequalities.push_back(
          LinearInequality(keyMatrixMapPos, b[kv.first] + ranges[kv.first],
              dual_key_num++));
    }
  }

  for (auto kv : IL) {
    map < Key, Matrix11 > keyMatrixMapPos;
    map < Key, Matrix11 > keyMatrixMapNeg;
    for (auto km : kv.second) {
      keyMatrixMapPos.insert(km);
      km.second = -km.second;
      keyMatrixMapNeg.insert(km);
    }
    madeQP.inequalities.push_back(
        LinearInequality(keyMatrixMapPos, b[kv.first], dual_key_num++));
    if (ranges.count(kv.first) == 1) {
      madeQP.inequalities.push_back(
          LinearInequality(keyMatrixMapNeg, ranges[kv.first] - b[kv.first],
              dual_key_num++));
    }
  }

  for (Key k : keys) {
    if (find(Free.begin(), Free.end(), k) != Free.end())
      continue;
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
}

