/**
 * @file     LPSolver.cpp
 * @brief    QPS parser implementation
 * @author   Ivan Dario Jimenez
 * @date     1/26/16
 */

#pragma once

#include <gtsam_unstable/linear/QP.h>
#include <fstream>

namespace gtsam {

class QPSParser {

private:
  std::string fileName_;
  struct MPSGrammar;
public:

  QPSParser(const std::string& fileName) :
      fileName_(findExampleDataFile(fileName)) {
  }

  QP Parse();
};
}

