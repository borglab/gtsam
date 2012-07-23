/**
 * This is a wrap header to verify permutations on namespaces
 */

#include <path/to/ns1.h>
namespace ns1 {

class ClassA {
	ClassA();
};

#include <path/to/ns1/ClassB.h>
class ClassB {
	ClassB();
};

// check namespace handling
Vector aGlobalFunction();

}

#include <path/to/ns2.h>
namespace ns2 {

#include <path/to/ns2/ClassA.h>
class ClassA {
	ClassA();
	static double afunction();
	double memberFunction();
	int nsArg(const ns1::ClassB& arg);
	ns2::ns3::ClassB nsReturn(double q);
};

#include <path/to/ns3.h>
namespace ns3 {

class ClassB {
	ClassB();
};

}

class ClassC {
	ClassC();
};

// separate namespace global function, same name
Vector aGlobalFunction();

} //\namespace ns2

class ClassD {
	ClassD();
};


