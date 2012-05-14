/*
 * testTypedDiscreteVariable.cpp
 *
 *  @date March 2, 2011
 *  @author Duy-Nguyen Ta
 */

#include <CppUnitLite/TestHarness.h>

using namespace std;

/* ******************************************************************************** */

class DiscreteVariable {
	int v_;
public:
	DiscreteVariable(int v) :
		v_(v) {
	}
	bool operator ==(const DiscreteVariable& other) const {
		return v_ == other.v_;
	}
	bool operator !=(const DiscreteVariable& other) const {
		return v_ != other.v_;
	}
};

class Color: public DiscreteVariable {
public:
	enum Value {
		red, green, blue
	};
	Color(Value v) :
		DiscreteVariable(v) {
	}
};

class Flavor: public DiscreteVariable {
public:
	enum Value {
		sweet, sour
	};
	Flavor(Value v) :
		DiscreteVariable(v) {
	}
};

//TEST( TypedDiscreteFactorGraph, simple)
//{
//	Color v1(Color::red), v2(Color::green);
//	CHECK(v1!=v2);
//	CHECK(v1==v1);
//
//	// Declare a bunch of keys
//	DiscreteKey<Color> C("Color");
//	DiscreteKey<Flavor> F("Flavor");
//
//	// Create a factor saying red is associated with sweet,
//	// green with sour, blue with both
//	TypedDiscreteFactor factor(C, F, "1 0  0 1  1 1");
//}

/* ************************************************************************* */
int main() {
	TestResult tr;
	return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */

