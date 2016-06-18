/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file     RawQP.cpp
 * @brief    
 * @author   Ivan Dario Jimenez
 * @date     3/5/16
 */

#include <gtsam_unstable/linear/RawQP.h>
#include <iostream>

using boost::fusion::at_c;

namespace gtsam {

void RawQP::setName(
    boost::fusion::vector<std::vector<char>, std::vector<char>,
        std::vector<char>> const &name) {
  name_ = std::string(at_c < 1 > (name).begin(), at_c < 1 > (name).end());
  if (debug) {
    std::cout << "Parsing file: " << name_ << std::endl;
  }
}

void RawQP::addColumn(
    boost::fusion::vector<std::vector<char>, std::vector<char>,
        std::vector<char>, std::vector<char>, std::vector<char>, double,
        std::vector<char>> const &vars) {

  std::string var_(at_c < 1 > (vars).begin(), at_c < 1 > (vars).end());
  std::string row_(at_c < 3 > (vars).begin(), at_c < 3 > (vars).end());
  Matrix11 coefficient = at_c < 5 > (vars) * I_1x1;

  if (!varname_to_key.count(var_))
    varname_to_key[var_] = Symbol('X', varNumber++);
  if (row_ == obj_name) {
    g[varname_to_key[var_]] = coefficient;
    return;
  }
  (*row_to_constraint_v[row_])[row_][varname_to_key[var_]] = coefficient;
  if (debug) {
    std::cout << "Added Column for Var: " << var_ << " Row: " << row_
        << " Coefficient: " << coefficient << std::endl;
  }

}

void RawQP::addColumnDouble(
    boost::fusion::vector<std::vector<char>, std::vector<char>,
        std::vector<char>, std::vector<char>, double, std::vector<char>,
        std::vector<char>, std::vector<char>, double> const &vars) {

  std::string var_(at_c < 0 > (vars).begin(), at_c < 0 > (vars).end());
  std::string row1_(at_c < 2 > (vars).begin(), at_c < 2 > (vars).end());
  std::string row2_(at_c < 6 > (vars).begin(), at_c < 6 > (vars).end());
  Matrix11 coefficient1 = at_c < 4 > (vars) * I_1x1;
  Matrix11 coefficient2 = at_c < 8 > (vars) * I_1x1;
  if (!varname_to_key.count(var_))
    varname_to_key.insert( { var_, Symbol('X', varNumber++) });
  if (row1_ == obj_name)
    g[varname_to_key[var_]] = coefficient1;
  else
    (*row_to_constraint_v[row1_])[row1_][varname_to_key[var_]] = coefficient1;
  if (row2_ == obj_name)
    g[varname_to_key[var_]] = coefficient2;
  else
    (*row_to_constraint_v[row2_])[row2_][varname_to_key[var_]] = coefficient2;
}

void RawQP::addRHS(
    boost::fusion::vector<std::vector<char>, std::vector<char>,
        std::vector<char>, std::vector<char>, std::vector<char>, double,
        std::vector<char>> const &vars) {

  std::string var_(at_c < 1 > (vars).begin(), at_c < 1 > (vars).end());
  std::string row_(at_c < 3 > (vars).begin(), at_c < 3 > (vars).end());
  double coefficient = at_c < 5 > (vars);
  if (row_ == obj_name)
    f = -coefficient;
  else
    b[row_] = coefficient;

  if (debug) {
    std::cout << "Added RHS for Var: " << var_ << " Row: " << row_
        << " Coefficient: " << coefficient << std::endl;
  }
}

void RawQP::addRHSDouble(
    boost::fusion::vector<std::vector<char>, std::vector<char>,
        std::vector<char>, std::vector<char>, std::vector<char>, double,
        std::vector<char>, std::vector<char>, std::vector<char>, double> const &vars) {

  std::string var_(at_c < 1 > (vars).begin(), at_c < 1 > (vars).end());
  std::string row1_(at_c < 3 > (vars).begin(), at_c < 3 > (vars).end());
  std::string row2_(at_c < 7 > (vars).begin(), at_c < 7 > (vars).end());
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
    std::cout << "Added RHS for Var: " << var_ << " Row: " << row1_
        << " Coefficient: " << coefficient1 << std::endl;
    std::cout << "                      " << "Row: " << row2_
        << " Coefficient: " << coefficient2 << std::endl;
  }
}

void RawQP::addRow(
    boost::fusion::vector<std::vector<char>, char, std::vector<char>,
        std::vector<char>, std::vector<char>> const &vars) {

  std::string name_(at_c < 3 > (vars).begin(), at_c < 3 > (vars).end());
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
    std::cout << "invalid type: " << type << std::endl;
    break;
  }
  if (debug) {
    std::cout << "Added Row Type: " << type << " Name: " << name_ << std::endl;
  }
}

void RawQP::addBound(
    boost::fusion::vector<std::vector<char>, std::vector<char>,
        std::vector<char>, std::vector<char>, std::vector<char>,
        std::vector<char>, std::vector<char>, double> const &vars) {

  std::string type_(at_c < 1 > (vars).begin(), at_c < 1 > (vars).end());
  std::string var_(at_c < 5 > (vars).begin(), at_c < 5 > (vars).end());
  double number = at_c < 7 > (vars);
  if (type_.compare(std::string("UP")) == 0)
    up[varname_to_key[var_]] = number;
  else if (type_.compare(std::string("LO")) == 0)
    lo[varname_to_key[var_]] = number;
  else
    std::cout << "Invalid Bound Type: " << type_ << std::endl;

  if (debug) {
    std::cout << "Added Bound Type: " << type_ << " Var: " << var_
        << " Amount: " << number << std::endl;
  }
}

void RawQP::addBoundFr(
    boost::fusion::vector<std::vector<char>, std::vector<char>,
        std::vector<char>, std::vector<char>, std::vector<char>,
        std::vector<char>, std::vector<char>> const &vars) {
  std::string type_(at_c < 1 > (vars).begin(), at_c < 1 > (vars).end());
  std::string var_(at_c < 5 > (vars).begin(), at_c < 5 > (vars).end());
  Free.push_back(varname_to_key[var_]);
  if (debug) {
    std::cout << "Added Free Bound Type: " << type_ << " Var: " << var_
        << " Amount: " << std::endl;
  }
}

void RawQP::addQuadTerm(
    boost::fusion::vector<std::vector<char>, std::vector<char>,
        std::vector<char>, std::vector<char>, std::vector<char>, double,
        std::vector<char>> const &vars) {
  std::string var1_(at_c < 1 > (vars).begin(), at_c < 1 > (vars).end());
  std::string var2_(at_c < 3 > (vars).begin(), at_c < 3 > (vars).end());
  Matrix11 coefficient = at_c < 5 > (vars) * I_1x1;

  H[varname_to_key[var1_]][varname_to_key[var2_]] = coefficient;
  H[varname_to_key[var2_]][varname_to_key[var1_]] = coefficient;
  if (debug) {
    std::cout << "Added QuadTerm for Var: " << var1_ << " Row: " << var2_
        << " Coefficient: " << coefficient << std::endl;
  }
}

QP RawQP::makeQP() {
  std::vector < Key > keys;
  std::vector < Matrix > Gs;
  std::vector < Vector > gs;
  for (auto kv : varname_to_key) {
    keys.push_back(kv.second);
  }
  std::sort(keys.begin(), keys.end());
  for (unsigned int i = 0; i < keys.size(); ++i) {
    for (unsigned int j = i; j < keys.size(); ++j) {
      Gs.push_back(H[keys[i]][keys[j]]);
    }
  }
  for (Key key1 : keys) {
    gs.push_back(-g[key1]);
  }
  int dual_key_num = keys.size() + 1;
  QP madeQP;
  auto obj = HessianFactor(keys, Gs, gs, f);

  madeQP.cost.push_back(obj);

  for (auto kv : E) {
    std::map<Key, Matrix11> keyMatrixMap;
    for (auto km : kv.second) {
      keyMatrixMap.insert(km);
    }
    madeQP.equalities.push_back(
        LinearEquality(keyMatrixMap, b[kv.first] * I_1x1, dual_key_num++));
  }

  for (auto kv : IG) {
    std::map<Key, Matrix11> keyMatrixMap;
    for (auto km : kv.second) {
      km.second = -km.second;
      keyMatrixMap.insert(km);
    }
    madeQP.inequalities.push_back(
        LinearInequality(keyMatrixMap, -b[kv.first], dual_key_num++));
  }

  for (auto kv : IL) {
    std::map<Key, Matrix11> keyMatrixMap;
    for (auto km : kv.second) {
      keyMatrixMap.insert(km);
    }
    madeQP.inequalities.push_back(
        LinearInequality(keyMatrixMap, b[kv.first], dual_key_num++));
  }

  for (Key k : keys) {
    if (std::find(Free.begin(), Free.end(), k) != Free.end())
      continue;
    if (up.count(k) == 1)
      madeQP.inequalities.push_back(
          LinearInequality(k, I_1x1, up[k], dual_key_num++));
    if (lo.count(k) == 1)
      madeQP.inequalities.push_back(
          LinearInequality(k, -I_1x1, lo[k], dual_key_num++));
    else
      madeQP.inequalities.push_back(
          LinearInequality(k, -I_1x1, 0, dual_key_num++));
  }
  return madeQP;
}
}

