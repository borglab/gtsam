/**
 * @file QPSParserException
 * @brief Exception thrown if there is an error parsing a QPS file
 * @date jan 24, 2015
 * @author Duy-Nguyen Ta
 */

#pragma once

namespace gtsam {

class QPSParserException: public ThreadsafeException<QPSParserException> {
public:
  QPSParserException() {
  }

  virtual ~QPSParserException() throw () {
  }

  virtual const char *what() const throw () {
    if (description_.empty())
      description_ = "There is a problem parsing the QPS file.\n";
    return description_.c_str();
  }

private:
  mutable std::string description_;
};

}

