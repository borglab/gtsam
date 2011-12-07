/**
 * @file ReturnValue.h
 *
 * @brief Encapsulates a return value from a method
 *
 * @date Dec 1, 2011
 * @author Alex Cunningham
 */

#include <ostream>

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

	return_category category1, category2;

	typedef enum {
		arg1, arg2, pair
	} pairing;

	std::string return_type(bool add_ptr, pairing p);

	std::string matlab_returnType() const { return isPair? "[first,second]" : "result"; }

	void wrap_result(std::ostream& ofs);

};

} // \namespace wrap
