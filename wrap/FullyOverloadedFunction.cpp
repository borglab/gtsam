#include "FullyOverloadedFunction.h"

using namespace std;

namespace wrap {
const std::array<std::string, 2> FullyOverloadedFunction::pythonKeywords{
    {"print", "lambda"}};

/* ************************************************************************* */
std::string FullyOverloadedFunction::pyx_functionCall(
    const std::string& caller,
    const std::string& funcName, size_t iOverload) const {

  string ret;
  if (!returnVals_[iOverload].isPair && !returnVals_[iOverload].type1.isPtr &&
      returnVals_[iOverload].type1.isNonBasicType()) {
    ret = returnVals_[iOverload].type1.make_shared_pxd_class_in_pyx() + "(";
  }

  // actual function call ...
  if (!caller.empty()) ret += caller + ".";
  ret += funcName;
  if (templateArgValue_) ret += "[" + templateArgValue_->pxd_class_in_pyx() + "]";
  //... with argument list
  ret += "(" + argumentList(iOverload).pyx_asParams() + ")";

  if (!returnVals_[iOverload].isPair && !returnVals_[iOverload].type1.isPtr &&
      returnVals_[iOverload].type1.isNonBasicType())
    ret += ")";

  return ret;
}

}
