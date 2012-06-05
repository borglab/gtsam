/**
 * @file ReturnValue.cpp
 *
 * @date Dec 1, 2011
 * @author Alex Cunningham
 */

#include <boost/foreach.hpp>

#include "ReturnValue.h"
#include "utilities.h"

using namespace std;
using namespace wrap;

/* ************************************************************************* */
string ReturnValue::return_type(bool add_ptr, pairing p) const {
  if (p==pair && isPair) {
    string str = "pair< " +
    		maybe_shared_ptr(add_ptr && isPtr1, qualifiedType1("::")) + ", " +
      maybe_shared_ptr(add_ptr && isPtr2, qualifiedType2("::")) + " >";
    return str;
  } else
    return maybe_shared_ptr(add_ptr && isPtr1, (p==arg2)? qualifiedType2("::") : qualifiedType1("::"));
}

/* ************************************************************************* */
string ReturnValue::matlab_returnType() const {
	return isPair? "[first,second]" : "result";
}

/* ************************************************************************* */
string ReturnValue::qualifiedType1(const string& delim) const {
	string result;
	BOOST_FOREACH(const string& ns, namespaces1) result += ns + delim;
	return result + type1;
}

/* ************************************************************************* */
string ReturnValue::qualifiedType2(const string& delim) const {
	string result;
	BOOST_FOREACH(const string& ns, namespaces2) result += ns + delim;
	return result + type2;
}

/* ************************************************************************* */
void ReturnValue::wrap_result(FileWriter& file) const {
	string cppType1 = qualifiedType1("::"), matlabType1 = qualifiedType1();
	string cppType2 = qualifiedType2("::"), matlabType2 = qualifiedType2();

  if (isPair) {
  	// first return value in pair
    if (isPtr1) // if we already have a pointer
      file.oss << "  out[0] = wrap_shared_ptr(result.first,\"" << matlabType1 << "\");\n";
    else if (category1 == ReturnValue::CLASS) // if we are going to make one
    	file.oss << "  out[0] = wrap_shared_ptr(boost::make_shared< " << cppType1 << " >(result.first),\"" << matlabType1 << "\");\n";
    else // if basis type
      file.oss << "  out[0] = wrap< " << return_type(true,arg1) << " >(result.first);\n";

    // second return value in pair
    if (isPtr2) // if we already have a pointer
      file.oss << "  out[1] = wrap_shared_ptr(result.second,\"" << matlabType2 << "\");\n";
    else if (category2 == ReturnValue::CLASS) // if we are going to make one
    	file.oss << "  out[1] = wrap_shared_ptr(boost::make_shared< " << cppType2 << " >(result.second),\"" << matlabType2 << "\");\n";
    else
      file.oss << "  out[1] = wrap< " << return_type(true,arg2) << " >(result.second);\n";
  }
  else if (isPtr1)
    file.oss << "  out[0] = wrap_shared_ptr(result,\"" << matlabType1 << "\");\n";
  else if (category1 == ReturnValue::CLASS)
  	file.oss << "  out[0] = wrap_shared_ptr(boost::make_shared< " << cppType1 << " >(result),\"" << matlabType1 << "\");\n";
  else if (matlabType1!="void")
    file.oss << "  out[0] = wrap< " << return_type(true,arg1) << " >(result);\n";
}

/* ************************************************************************* */


