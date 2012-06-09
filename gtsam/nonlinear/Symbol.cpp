/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file Symbol.cpp
 * @date June 9, 2012
 * @author: Frank Dellaert
 * @author: Richard Roberts
 */

#include <gtsam/nonlinear/Symbol.h>

#include <boost/mpl/char.hpp>
#include <boost/format.hpp>
#include <boost/function.hpp>
#include <boost/lambda/bind.hpp>
#include <boost/lambda/construct.hpp>
#include <boost/lambda/lambda.hpp>

#include <limits.h>
#include <list>
#include <iostream>

namespace gtsam {

static const size_t keyBits = sizeof(Key) * 8;
static const size_t chrBits = sizeof(unsigned char) * 8;
static const size_t indexBits = keyBits - chrBits;
static const Key chrMask = Key(UCHAR_MAX)	<< indexBits; // For some reason, std::numeric_limits<unsigned char>::max() fails
static const Key indexMask = ~chrMask;

Symbol::Symbol(Key key) {
	c_ = (unsigned char) ((key & chrMask) >> indexBits);
	j_ = key & indexMask;
}

Key Symbol::key() const {
	if (j_ > indexMask) {
		boost::format msg("Symbol index is too large, j=%d, indexMask=%d");
		msg % j_ % indexMask;
		throw std::invalid_argument(msg.str());
	}
	Key key = (Key(c_) << indexBits) | j_;
	return key;
}

void Symbol::print(const std::string& s) const {
	std::cout << s << (std::string) (*this) << std::endl;
}

bool Symbol::equals(const Symbol& expected, double tol) const {
	return (*this) == expected;
}

Symbol::operator std::string() const {
	return str(boost::format("%c%d") % c_ % j_);
}

boost::function<bool(Key)> Symbol::ChrTest(unsigned char c) {
	namespace bl = boost::lambda;
	return bl::bind(&Symbol::chr, bl::bind(bl::constructor<Symbol>(), bl::_1))
	== c;
}

} // namespace gtsam

