/**
 * @file ReturnValue.cpp
 *
 * @date Dec 1, 2011
 * @author Alex Cunningham
 */

#include "ReturnValue.h"
#include "utilities.h"

using namespace std;
using namespace wrap;

/* ************************************************************************* */
string ReturnValue::return_type(bool add_ptr, pairing p) {
  if (p==pair && returns_pair_) {
    string str = "pair< " +
    		wrap::maybe_shared_ptr(add_ptr && returns_ptr_, returns_) + ", " +
      wrap::maybe_shared_ptr(add_ptr && returns_ptr_, returns2_) + " >";
    return str;
  } else
    return wrap::maybe_shared_ptr(add_ptr && returns_ptr_, (p==arg2)? returns2_ : returns_);
}

/* ************************************************************************* */
void ReturnValue::wrap_result(std::ostream& ofs) {
  if (returns_pair_) {
  	// first return value in pair
    if (returns_ptr_)
      ofs << "  out[0] = wrap_shared_ptr(result.first,\"" << returns_ << "\");\n";
    else
      ofs << "  out[0] = wrap< " << return_type(true,arg1) << " >(result.first);\n";

    // second return value in pair
    if (returns_ptr2_)
      ofs << "  out[1] = wrap_shared_ptr(result.second,\"" << returns2_ << "\");\n";
    else
      ofs << "  out[1] = wrap< " << return_type(true,arg2) << " >(result.second);\n";
  }
  else if (returns_ptr_)
    ofs << "  out[0] = wrap_shared_ptr(result,\"" << returns_ << "\");\n";
  else if (returns_class_)
  	ofs << "  out[0] = wrap_shared_ptr(make_shared< " << returns_ << " >(result),\"" << returns_ << "\");\n";
  else if (returns_!="void")
    ofs << "  out[0] = wrap< " << return_type(true,arg1) << " >(result);\n";
}

/* ************************************************************************* */


