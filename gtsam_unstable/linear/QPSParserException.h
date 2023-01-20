/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file     QPSParserException.h
 * @brief    Exception thrown if there is an error parsing a QPS file
 * @author   Ivan Dario Jimenez
 * @date     3/5/16
 */

#pragma once

namespace gtsam {

class QPSParserException: public ThreadsafeException<QPSParserException> {
public:
  QPSParserException() {
  }

  ~QPSParserException() noexcept override {
  }

  const char *what() const noexcept override {
    if (description_.empty())
      description_ = "There is a problem parsing the QPS file.\n";
    return description_.c_str();
  }

private:
  mutable std::string description_;
};

}

