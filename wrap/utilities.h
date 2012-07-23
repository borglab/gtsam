/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file utilities.h
 * @author Frank Dellaert
 * @author Andrew Melim
 * @author Richard Roberts
 **/

#pragma once

#include <vector>
#include <exception>
#include <fstream>
#include <sstream>
//#include <cstdint> // on Linux GCC: fails with error regarding needing C++0x std flags
//#include <cinttypes>  // same failure as above
#include <stdint.h> // works on Linux GCC
#include <string>
#include <boost/format.hpp>

#include "FileWriter.h"

namespace wrap {

class CantOpenFile : public std::exception {
 private:
	const std::string what_;
 public:
 CantOpenFile(const std::string& filename) : what_("Can't open file " + filename) {}
  ~CantOpenFile() throw() {}
	virtual const char* what() const throw() { return what_.c_str(); }
};

class OutputError : public std::exception {
private:
	const std::string what_;
public:
	OutputError(const std::string& what) : what_(what) {}
	~OutputError() throw() {}
	virtual const char* what() const throw() { return what_.c_str(); }
};

class ParseFailed : public std::exception {
 private:
  const std::string what_;
 public:
	 ParseFailed(int length) : what_((boost::format("Parse failed at character [%d]")%(length-1)).str()) {}
	 ~ParseFailed() throw() {}
	 virtual const char* what() const throw() { return what_.c_str(); }
};

class DependencyMissing : public std::exception {
private:
	const std::string what_;
public:
	DependencyMissing(const std::string& dep, const std::string& loc) :
		what_("Missing dependency " + dep + " in " + loc) {}
	~DependencyMissing() throw() {}
	virtual const char* what() const throw() { return what_.c_str(); }
};

class DuplicateDefinition : public std::exception {
private:
	const std::string what_;
public:
	DuplicateDefinition(const std::string& name) :
		what_("Duplicate definition of " + name) {}
	~DuplicateDefinition() throw() {}
	virtual const char* what() const throw() { return what_.c_str(); }
};

class AttributeError : public std::exception {
private:
	const std::string what_;
public:
	AttributeError(const std::string& name, const std::string& problem) :
		what_("Class " + name + ": " + problem) {}
	~AttributeError() throw() {}
	virtual const char* what() const throw() { return what_.c_str(); }
};
	
// "Unique" key to signal calling the matlab object constructor with a raw pointer
// to a shared pointer of the same C++ object type as the MATLAB type.
// Also present in matlab.h
static const uint64_t ptr_constructor_key =
	(uint64_t('G') << 56) |
	(uint64_t('T') << 48) |
	(uint64_t('S') << 40) |
	(uint64_t('A') << 32) |
	(uint64_t('M') << 24) |
	(uint64_t('p') << 16) |
	(uint64_t('t') << 8) |
	(uint64_t('r'));

/**
 * read contents of a file into a std::string
 */
std::string file_contents(const std::string& filename, bool skipheader=false);

/**
 * Check whether two files are equal
 * By default, skips the first line of actual so header is not generated
 */
bool files_equal(const std::string& expected, const std::string& actual, bool skipheader=false);

/**
 * Compare strings for unit tests
 */
bool assert_equal(const std::string& expected, const std::string& actual);
bool assert_equal(const std::vector<std::string>& expected, const std::vector<std::string>& actual);

// auxiliary function to wrap an argument into a shared_ptr template
std::string maybe_shared_ptr(bool add, const std::string& qtype, const std::string& type);

/**
 * Return a qualified name, if finalName is empty, only the names vector will
 * be used (i.e. there won't be a trailing separator on the qualified name).
 */
std::string qualifiedName(const std::string& separator, const std::vector<std::string>& names, const std::string& finalName = "");

/** creates the necessary folders for namespaces, as specified by a namespace stack */
void createNamespaceStructure(const std::vector<std::string>& namespaces, const std::string& toolboxPath);

} // \namespace wrap
