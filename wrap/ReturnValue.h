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

	ReturnValue(bool verbose = true)
	: verbose_(verbose), returns_ptr_(false), returns_ptr2_(false), returns_pair_(false)
	{}

	bool verbose_;
	std::string returns_, returns2_;
	bool returns_ptr_, returns_ptr2_, returns_pair_;

	typedef enum {
		arg1, arg2, pair
	} pairing;

	std::string return_type(bool add_ptr, pairing p);

	std::string matlab_returnType() const { return returns_pair_? "[first,second]" : "result"; }

	void wrap_result(std::ostream& ofs);

};

} // \namespace wrap
