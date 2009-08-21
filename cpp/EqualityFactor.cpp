/*
 * EqualityFactor.cpp
 *
 *  Created on: Aug 10, 2009
 *      Author: alexgc
 */

#include "EqualityFactor.h"
#include <iostream>

namespace gtsam {
using namespace std;

EqualityFactor::EqualityFactor()
: key(""), value(Vector(0))
{
}

EqualityFactor::EqualityFactor(const Vector& constraint, const std::string& id)
: key(id), value(constraint)
{
}

double EqualityFactor::error(const FGConfig& c) const
{
	return 0.0;
}

void EqualityFactor::print(const string& s) const
{
	cout << s << ": " << dump() << endl;
}

bool EqualityFactor::equals(const EqualityFactor& f, double tol) const
{
	return equal_with_abs_tol(value, f.get_value(), tol) && key == f.get_key();
}

bool EqualityFactor::equals(const Factor& f, double tol) const
{
	const EqualityFactor* p = dynamic_cast<const EqualityFactor*>(&f);
	  if (p == NULL) return false;
	  return equals(f, tol);
}

string EqualityFactor::dump() const
{
	string ret = "[" + key + "] " + gtsam::dump(value);
	return ret;
}

DeltaFunction::shared_ptr EqualityFactor::getDeltaFunction() const
{
	DeltaFunction::shared_ptr ret(new DeltaFunction(value, key));
	return ret;
}

EqualityFactor::shared_ptr EqualityFactor::linearize() const
{
	EqualityFactor::shared_ptr ret(new EqualityFactor(zero(value.size()), key));
	return ret;
}

bool assert_equal(const EqualityFactor& actual, const EqualityFactor& expected, double tol)
{
	bool ret = actual.equals(expected, tol);
	if (!ret)
	{
		cout << "Not Equal:" << endl;
		actual.print("Actual");
		expected.print("Expected");
	}
	return ret;
}

}
