#pragma once

namespace gtsam {
/**
 * For Python __str__().
 * Redirect std cout to a string stream so we can return a string representation
 * of an object when it prints to cout.
 * https://stackoverflow.com/questions/5419356/redirect-stdout-stderr-to-a-string
 */
struct RedirectCout {
  /// constructor -- redirect stdout buffer to a stringstream buffer
  RedirectCout() : ssBuffer_(), coutBuffer_(std::cout.rdbuf(ssBuffer_.rdbuf())) {}

  /// return the string
  std::string str() const {
    return ssBuffer_.str();
  }

  /// destructor -- redirect stdout buffer to its original buffer
  ~RedirectCout() {
    std::cout.rdbuf(coutBuffer_);
  }

private:
  std::stringstream ssBuffer_;
  std::streambuf* coutBuffer_;
};

}
