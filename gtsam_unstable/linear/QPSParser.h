/**
 * @file     LPSolver.cpp
 * @brief    QPS parser implementation
 * @author   Ivan Dario Jimenez
 * @date     1/26/16
 */

#pragma once

#include <gtsam_unstable/linear/QP.h>
#include <gtsam_unstable/linear/QPSParserException.h>
#include <gtsam_unstable/linear/RawQP.h>
#include <fstream>

namespace gtsam {

class QPSParser {

private:
  std::fstream stream;
  struct MPSGrammar;
public:

  QPSParser(const std::string& fileName) :
      stream(findExampleDataFile(fileName).c_str()) {
    stream.unsetf(std::ios::skipws);
  }

  QP Parse();
};
}

