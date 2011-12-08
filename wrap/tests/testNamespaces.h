/**
 * This is a wrap header to verify permutations on namespaces
 */

namespace ns1 {

class ClassA {
	ClassA();
};

class ClassB {
	ClassB();
};

}///\namespace ns1

namespace ns2 {

class ClassA {
	ClassA();
	static double afunction();
	double memberFunction();
	int nsArg(const ns1::ClassB& arg);
};

namespace ns3 {

class ClassB {
	ClassB();
};

}///\namespace ns3

class ClassC {
	ClassC();
};

}///\namespace ns2

class ClassD {
	ClassD();
};


