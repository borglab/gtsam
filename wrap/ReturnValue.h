/**
 * @file ReturnValue.h
 *
 * @brief Encapsulates a return value from a method
 *
 * @date Dec 1, 2011
 * @author Alex Cunningham
 * @author Richard Roberts
 */

#include <vector>
#include <map>

#include "FileWriter.h"
#include "TypeAttributesTable.h"

#pragma once

namespace wrap {

struct ReturnValue {

	typedef enum {
		CLASS,
		EIGEN,
		BASIS,
		VOID
	} return_category;

	ReturnValue(bool enable_verbosity = true)
	: verbose(enable_verbosity), isPtr1(false), isPtr2(false),
	  isPair(false), category1(VOID), category2(VOID)
	{}

	bool verbose;
	std::string type1, type2;
	bool isPtr1, isPtr2, isPair;
	std::vector<std::string> namespaces1, namespaces2;

	return_category category1, category2;

	typedef enum {
		arg1, arg2, pair
	} pairing;

	std::string return_type(bool add_ptr, pairing p) const;

	std::string qualifiedType1(const std::string& delim = "") const;
	std::string qualifiedType2(const std::string& delim = "") const;

	std::string matlab_returnType() const;

	void wrap_result(const std::string& result, FileWriter& file, const TypeAttributesTable& typeAttributes) const;

	void wrapTypeUnwrap(FileWriter& wrapperFile) const;

};

} // \namespace wrap
