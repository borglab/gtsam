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
  if (p==pair && isPair) {
    string str = "pair< " +
    		wrap::maybe_shared_ptr(add_ptr && isPtr1, type1) + ", " +
      wrap::maybe_shared_ptr(add_ptr && isPtr2, type2) + " >";
    return str;
  } else
    return wrap::maybe_shared_ptr(add_ptr && isPtr1, (p==arg2)? type2 : type1);
}

/* ************************************************************************* */
void ReturnValue::wrap_result(std::ostream& ofs) {
  if (isPair) {
  	// first return value in pair
    if (isPtr1) // if we already have a pointer
      ofs << "  out[0] = wrap_shared_ptr(result.first,\"" << type1 << "\");\n";
    else if (category1 == ReturnValue::CLASS) // if we are going to make one
    	ofs << "  out[0] = wrap_shared_ptr(make_shared< " << type1 << " >(result.first),\"" << type1 << "\");\n";
    else // if basis type
      ofs << "  out[0] = wrap< " << return_type(true,arg1) << " >(result.first);\n";

    // second return value in pair
    if (isPtr2) // if we already have a pointer
      ofs << "  out[1] = wrap_shared_ptr(result.second,\"" << type2 << "\");\n";
    else if (category2 == ReturnValue::CLASS) // if we are going to make one
    	ofs << "  out[1] = wrap_shared_ptr(make_shared< " << type2 << " >(result.second),\"" << type2 << "\");\n";
    else
      ofs << "  out[1] = wrap< " << return_type(true,arg2) << " >(result.second);\n";
  }
  else if (isPtr1)
    ofs << "  out[0] = wrap_shared_ptr(result,\"" << type1 << "\");\n";
  else if (category1 == ReturnValue::CLASS)
  	ofs << "  out[0] = wrap_shared_ptr(make_shared< " << type1 << " >(result),\"" << type1 << "\");\n";
  else if (type1!="void")
    ofs << "  out[0] = wrap< " << return_type(true,arg1) << " >(result);\n";
}

/* ************************************************************************* */


