/**
 * @file ReturnValue.cpp
 *
 * @date Dec 1, 2011
 * @author Alex Cunningham
 * @author Andrew Melim
 * @author Richard Roberts
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
void ReturnValue::wrap_result(const string& result, FileWriter& file, const TypeAttributesTable& typeAttributes) const {
	string cppType1 = qualifiedType1("::"), matlabType1 = qualifiedType1();
	string cppType2 = qualifiedType2("::"), matlabType2 = qualifiedType2();

  if (isPair) {
  	// first return value in pair
    if (category1 == ReturnValue::CLASS) { // if we are going to make one
			string objCopy, ptrType;
			ptrType = "Shared" + type1;
			const bool isVirtual = typeAttributes.at(cppType1).isVirtual;
			if(isVirtual) {
				if(isPtr1)
					objCopy = result + ".first";
				else
					objCopy = result + ".first.clone()";
			} else {
				if(isPtr1)
					objCopy = result + ".first";
				else
					objCopy = ptrType + "(new " + cppType1 + "(" + result + ".first))";
			}
    	file.oss << "  out[0] = wrap_shared_ptr(" << objCopy << ",\"" << matlabType1 << "\", " << (isVirtual ? "true" : "false") << ");\n";
    } else if(isPtr1) {
			file.oss << "  Shared" << type1 <<"* ret = new Shared" << type1 << "(" << result << ".first);" << endl;
			file.oss << "  out[0] = wrap_shared_ptr(ret,\"" << matlabType1 << "\", false);\n";
		} else // if basis type
      file.oss << "  out[0] = wrap< " << return_type(true,arg1) << " >(" << result << ".first);\n";

    // second return value in pair
    if (category2 == ReturnValue::CLASS) { // if we are going to make one
			string objCopy, ptrType;
			ptrType = "Shared" + type2;
			const bool isVirtual = typeAttributes.at(cppType2).isVirtual;
			if(isVirtual) {
				if(isPtr2)
					objCopy = result + ".second";
				else
					objCopy = result + ".second.clone()";
			} else {
				if(isPtr2)
					objCopy = result + ".second";
				else
					objCopy = ptrType + "(new " + cppType2 + "(" + result + ".second))";
			}
			file.oss << "  out[0] = wrap_shared_ptr(" << objCopy << ",\"" << matlabType2 << "\", " << (isVirtual ? "true" : "false") << ");\n";
    } else if(isPtr2) {
			file.oss << "  Shared" << type2 <<"* ret = new Shared" << type2 << "(" << result << ".second);" << endl;
			file.oss << "  out[1] = wrap_shared_ptr(ret,\"" << matlabType2 << "\");\n";
		} else
      file.oss << "  out[1] = wrap< " << return_type(true,arg2) << " >(" << result << ".second);\n";
  }
  else if (category1 == ReturnValue::CLASS){
		string objCopy, ptrType;
		ptrType = "Shared" + type1;
		const bool isVirtual = typeAttributes.at(cppType1).isVirtual;
		if(isVirtual) {
			if(isPtr1)
				objCopy = result;
			else
				objCopy = result + ".clone()";
		} else {
			if(isPtr1)
				objCopy = result;
			else
				objCopy = ptrType + "(new " + cppType1 + "(" + result + "))";
		}
		file.oss << "  out[0] = wrap_shared_ptr(" << objCopy << ",\"" << matlabType1 << "\", " << (isVirtual ? "true" : "false") << ");\n";
  } else if(isPtr1) {
		file.oss << "  Shared" << type1 <<"* ret = new Shared" << type1 << "(" << result << ");" << endl;
		file.oss << "  out[0] = wrap_shared_ptr(ret,\"" << matlabType1 << "\");\n";
	} else if (matlabType1!="void")
    file.oss << "  out[0] = wrap< " << return_type(true,arg1) << " >(" << result << ");\n";
}

/* ************************************************************************* */


