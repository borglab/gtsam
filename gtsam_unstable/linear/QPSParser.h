/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file     QPSParser.h
 * @brief    QPS parser implementation
 * @author   Ivan Dario Jimenez
 * @date     3/5/16
 */

#pragma once

#include <gtsam_unstable/dllexport.h>
#include <gtsam_unstable/linear/QP.h>
#include <fstream>

namespace gtsam {

class GTSAM_UNSTABLE_EXPORT QPSParser {

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

