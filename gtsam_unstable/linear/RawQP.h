/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file     RawQP.h
 * @brief    
 * @author   Ivan Dario Jimenez
 * @date     3/5/16
 */

#pragma once

#include <gtsam_unstable/linear/QP.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/inference/Key.h>

#include <string>
#include <vector>
#include <unordered_map>
#include <gtsam/inference/Symbol.h>
#include <boost/fusion/sequence.hpp>
#include <boost/fusion/include/vector.hpp>

namespace gtsam {
class RawQP {
private:
  typedef std::unordered_map<Key, Matrix11> coefficient_v;
  typedef std::unordered_map<std::string, coefficient_v> constraint_v;

  std::unordered_map<std::string, constraint_v*> row_to_constraint_v;
  constraint_v E;
  constraint_v IG;
  constraint_v IL;
  unsigned int varNumber;
  std::unordered_map<std::string, double> b;
  std::unordered_map<Key, Vector1> g;
  std::unordered_map<std::string, Key> varname_to_key;
  std::unordered_map<Key, std::unordered_map<Key, Matrix11> > H;
  double f;
  std::string obj_name;
  std::string name_;
  std::unordered_map<Key, double> up;
  std::unordered_map<Key, double> lo;
  std::vector<Key> Free;
  const bool debug = false;

public:
  RawQP() :
      row_to_constraint_v(), E(), IG(), IL(), varNumber(1), b(), g(), varname_to_key(), H(), f(), obj_name(), name_(), up(), lo(), Free() {
  }

  void setName(
      boost::fusion::vector<std::vector<char>, std::vector<char>,
          std::vector<char>> const & name);

  void addColumn(
      boost::fusion::vector<std::vector<char>, std::vector<char>,
          std::vector<char>, std::vector<char>, std::vector<char>, double,
          std::vector<char>> const & vars);

  void addColumnDouble(
      boost::fusion::vector<std::vector<char>, std::vector<char>,
          std::vector<char>, std::vector<char>, double, std::vector<char>,
          std::vector<char>, std::vector<char>, double> const & vars);

  void addRHS(
      boost::fusion::vector<std::vector<char>, std::vector<char>,
          std::vector<char>, std::vector<char>, std::vector<char>, double,
          std::vector<char>> const & vars);

  void addRHSDouble(
      boost::fusion::vector<std::vector<char>, std::vector<char>,
          std::vector<char>, std::vector<char>, std::vector<char>, double,
          std::vector<char>, std::vector<char>, std::vector<char>, double> const & vars);

  void addRow(
      boost::fusion::vector<std::vector<char>, char, std::vector<char>,
          std::vector<char>, std::vector<char>> const & vars);

  void addBound(
      boost::fusion::vector<std::vector<char>, std::vector<char>,
          std::vector<char>, std::vector<char>, std::vector<char>,
          std::vector<char>, std::vector<char>, double> const & vars);

  void addBoundFr(
      boost::fusion::vector<std::vector<char>, std::vector<char>,
          std::vector<char>, std::vector<char>, std::vector<char>,
          std::vector<char>, std::vector<char>> const & vars);

  void addQuadTerm(
      boost::fusion::vector<std::vector<char>, std::vector<char>,
          std::vector<char>, std::vector<char>, std::vector<char>, double,
          std::vector<char>> const & vars);

  QP makeQP();
}
;
}
