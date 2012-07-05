/**
 * @file ReturnValue.cpp
 *
 * @date Dec 1, 2011
 * @author Alex Cunningham
 * @author Andrew Melim
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
    		maybe_shared_ptr(add_ptr && isPtr1, qualifiedType1("::"), type1) + ", " +
      maybe_shared_ptr(add_ptr && isPtr2, qualifiedType2("::"), type2) + " >";
    return str;
  } else
    return maybe_shared_ptr(add_ptr && isPtr1, (p==arg2)? qualifiedType2("::") : qualifiedType1("::"), (p==arg2)? type2 : type1);
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
//TODO:Fix this
void ReturnValue::wrap_result(FileWriter& file) const {
	string cppType1 = qualifiedType1("::"), matlabType1 = qualifiedType1();
	string cppType2 = qualifiedType2("::"), matlabType2 = qualifiedType2();
	const string collectorName1 = "collector_" + matlabType1;
	const string collectorName2 = "collector_" + matlabType2;

  if (isPair) {
  	// first return value in pair
    if (isPtr1) {// if we already have a pointer
      file.oss << "  Shared" << type1 <<"* ret = new Shared" << type1 << "(result.first);" << endl;
			file.oss << "  " << collectorName1 << ".insert(ret);\n";
      file.oss << "  out[0] = wrap_shared_ptr(ret,\"" << matlabType1 << "\");\n";
    }
    else if (category1 == ReturnValue::CLASS) { // if we are going to make one
      file.oss << "  Shared" <<  type1 << "* ret = new Shared" <<  type1 << "(new " << cppType1 << "(result.first));\n";
			file.oss << "  " << collectorName1 << ".insert(ret);\n";
    	file.oss << "  out[0] = wrap_shared_ptr(ret,\"" << matlabType1 << "\");\n";
    }
    else // if basis type
      file.oss << "  out[0] = wrap< " << return_type(true,arg1) << " >(result.first);\n";

    // second return value in pair
    if (isPtr2) {// if we already have a pointer
      file.oss << "  Shared" << type2 <<"* ret = new Shared" << type2 << "(result.second);" << endl;
			file.oss << "  " << collectorName2 << ".insert(ret);\n";
      file.oss << "  out[1] = wrap_shared_ptr(ret,\"" << matlabType2 << "\");\n";
    }
    else if (category2 == ReturnValue::CLASS) { // if we are going to make one
      file.oss << "  Shared" <<  type2 << "* ret = new Shared" <<  type2 << "(new " << cppType2 << "(result.first));\n";
			file.oss << "  " << collectorName2 << ".insert(ret);\n";
    	file.oss << "  out[0] = wrap_shared_ptr(ret,\"" << matlabType2 << "\");\n";
    }
    else
      file.oss << "  out[1] = wrap< " << return_type(true,arg2) << " >(result.second);\n";
  }
  else if (isPtr1){
    file.oss << "  Shared" << type1 <<"* ret = new Shared" << type1 << "(result);" << endl;
		file.oss << "  " << collectorName1 << ".insert(ret);\n";
    file.oss << "  out[0] = wrap_shared_ptr(ret,\"" << matlabType1 << "\");\n";
  }
  else if (category1 == ReturnValue::CLASS){
    file.oss << "  Shared" <<  type1 << "* ret = new Shared" <<  type1 << "(new " << cppType1 << "(result));\n";
		file.oss << "  " << collectorName1 << ".insert(ret);\n";
		file.oss << "  out[0] = wrap_shared_ptr(ret,\"" << matlabType1 << "\");\n";
  }
  else if (matlabType1!="void")
    file.oss << "  out[0] = wrap< " << return_type(true,arg1) << " >(result);\n";
}

/* ************************************************************************* */


