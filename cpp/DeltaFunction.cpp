/*
 * DeltaFunction.cpp
 *
 *  Created on: Aug 11, 2009
 *      Author: alexgc
 */

#include <iostream>
#include "DeltaFunction.h"

namespace gtsam {

using namespace std;

DeltaFunction::DeltaFunction() {
	// TODO Auto-generated constructor stub

}

DeltaFunction::DeltaFunction(const Vector& v, const std::string& id)
: value_(v), key_(id)
{
}

DeltaFunction::DeltaFunction(const DeltaFunction& df)
: boost::noncopyable(), value_(df.value_), key_(df.key_)
{
}

DeltaFunction::~DeltaFunction() {
	// TODO Auto-generated destructor stub
}

bool DeltaFunction::equals(const DeltaFunction &df) const
{
	return equal_with_abs_tol(value_, df.value_) && key_ == df.key_;
}

void DeltaFunction::print() const
{
	cout << "DeltaFunction: [" << key_ << "]";
	gtsam::print(value_);
	cout << endl;
}

bool assert_equal(const DeltaFunction& actual, const DeltaFunction& expected, double tol)
{
	bool ret = actual.equals(expected);
	if (!ret)
	{
		cout << "Not Equal!" << endl;
		cout << "  Actual:" << endl;
		actual.print();
		cout << "  Expected:" << endl;
		expected.print();
	}
	return ret;
}

}
