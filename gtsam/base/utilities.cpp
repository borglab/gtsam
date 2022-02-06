#include <gtsam/base/utilities.h>

namespace gtsam {

std::string RedirectCout::str() const {
  return ssBuffer_.str();
}

RedirectCout::~RedirectCout() {
  std::cout.rdbuf(coutBuffer_);
}

}
