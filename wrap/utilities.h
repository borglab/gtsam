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
 **/

#pragma once

#include <vector>
#include <exception>
#include <fstream>
#include <sstream>

#include "FileWriter.h"

namespace wrap {

class CantOpenFile : public std::exception {
 private:
  std::string filename_;
 public:
 CantOpenFile(const std::string& filename) : filename_(filename) {}
  ~CantOpenFile() throw() {}
  virtual const char* what() const throw() { 
    return ("Can't open file " + filename_).c_str(); 
  }
};

class ParseFailed : public std::exception {
 private:
  int length_;
 public:
 ParseFailed(int length) : length_(length) {}
  ~ParseFailed() throw() {}
  virtual const char* what() const throw() { 
    std::stringstream buf;
    int len = length_+1;
    buf << "Parse failed at character [" << len << "]";
    return buf.str().c_str(); 
  }
};

class DependencyMissing : public std::exception {
private:
	std::string dependency_;
	std::string location_;
public:
	DependencyMissing(const std::string& dep, const std::string& loc) {
		dependency_ = dep;
		location_ = loc;
	}
	~DependencyMissing() throw() {}
	virtual const char* what() const throw() {
		return ("Missing dependency " + dependency_ + " in " + location_).c_str();
	}
};


/**
 * read contents of a file into a std::string
 */
std::string file_contents(const std::string& filename, bool skipheader=false);

/**
 * Check whether two files are equal
 * By default, skips the first line of actual so header is not generated
 */
bool files_equal(const std::string& expected, const std::string& actual, bool skipheader=true);

/**
 * Compare strings for unit tests
 */
bool assert_equal(const std::string& expected, const std::string& actual);
bool assert_equal(const std::vector<std::string>& expected, const std::vector<std::string>& actual);
/**
 * emit a header at the top of generated files
 */
//void generateHeaderComment(FileWriter& file, const std::string& delimiter);

// auxiliary function to wrap an argument into a shared_ptr template
std::string maybe_shared_ptr(bool add, const std::string& type);

/**
 * Creates the "using namespace [name];" declarations
 */
void generateUsingNamespace(FileWriter& file, const std::vector<std::string>& using_namespaces);

/**
 * Creates the #include statements
 */
void generateIncludes(FileWriter& file, const std::string& class_name,
		const std::vector<std::string>& includes);

} // \namespace wrap
