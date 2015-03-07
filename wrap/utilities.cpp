/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file utilities.ccp
 * @author Frank Dellaert
 * @author Andrew Melim
 * @author Richard Roberts
 **/

#include <iostream>
#include <cstdlib>

#include <boost/foreach.hpp>
#include <boost/filesystem.hpp>

#include "utilities.h"

namespace wrap {

using namespace std;

/* ************************************************************************* */
string file_contents(const string& filename, bool skipheader) {
  ifstream ifs(filename.c_str());
  if(!ifs) throw CantOpenFile(filename);

  // read file into stringstream
  stringstream ss;
  if (skipheader) ifs.ignore(256,'\n');
  ss << ifs.rdbuf();
  ifs.close();

  // return string
  return ss.str();
}

/* ************************************************************************* */
bool assert_equal(const string& expected, const string& actual) {
	if (expected == actual)
		return true;
	printf("Not equal:\n");
	cout << "expected: [" << expected << "]\n";
	cout << "actual: [" << actual << "]" << endl;
	return false;
}

/* ************************************************************************* */
bool assert_equal(const vector<string>& expected, const vector<string>& actual) {
  bool match = true;
  if (expected.size() != actual.size())
    match = false;
  vector<string>::const_iterator
  	itExp = expected.begin(),
  	itAct = actual.begin();
  if(match) {
  	for (; itExp!=expected.end() && itAct!=actual.end(); ++itExp, ++itAct) {
  		if (*itExp != *itAct) {
  			match = false;
  			break;
  		}
  	}
  }
	if(!match) {
	  cout << "expected: " << endl;
	  BOOST_FOREACH(const vector<string>::value_type& a, expected) { cout << "["  << a << "] "; }
	  cout << "\nactual: " << endl;
	  BOOST_FOREACH(const vector<string>::value_type& a, actual) { cout << "["  << a << "] "; }
	  cout << endl;
	  return false;
	}
	return true;

}

/* ************************************************************************* */
bool files_equal(const string& expected, const string& actual, bool skipheader) {
  try {
    string expected_contents = file_contents(expected, skipheader);
    string actual_contents   = file_contents(actual, skipheader);
    bool equal = actual_contents == expected_contents;
    if (!equal) {
      stringstream command;
      command << "diff " << actual << " " << expected << endl;
      system(command.str().c_str());
    }
    return equal;
  }
  catch (const string& reason) {
    cerr << "expection: " << reason << endl;
    return false;
  }
  catch (CantOpenFile& e) {
  	cerr << "file opening error: " << e.what() << endl;
  	return false;
  }
  return true;
}

/* ************************************************************************* */
string maybe_shared_ptr(bool add, const string& qtype, const string& type) {
  string str = add? "Shared" : "";
  if (add) str += type; 
  else str += qtype;

  //if (add) str += ">";
  return str;
}

/* ************************************************************************* */
string qualifiedName(const string& separator, const vector<string>& names, const string& finalName) {
	string result;
	if(!names.empty()) {
		for(size_t i = 0; i < names.size() - 1; ++i)
			result += (names[i] + separator);
		if(finalName.empty())
			result += names.back();
		else
			result += (names.back() + separator + finalName);
	} else if(!finalName.empty()) {
		result = finalName;
	}
	return result;
}

/* ************************************************************************* */
void createNamespaceStructure(const std::vector<std::string>& namespaces, const std::string& toolboxPath) {
	using namespace boost::filesystem;
	path curPath = toolboxPath;
	BOOST_FOREACH(const string& subdir, namespaces) {
		curPath /= "+" + subdir;
		if(!is_directory(curPath)) {
			if(exists("+" + subdir))
				throw OutputError("Need to write files to directory " + curPath.string() + ", which already exists as a file but is not a directory");
			else
				boost::filesystem::create_directory(curPath);
		}
	}
}

/* ************************************************************************* */

} // \namespace wrap
