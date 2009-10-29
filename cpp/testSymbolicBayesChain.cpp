/**
 * @file    testSymbolicBayesChain.cpp
 * @brief   Unit tests for a symbolic Bayes chain
 * @author  Frank Dellaert
 */

#include <CppUnitLite/TestHarness.h>

#include "smallExample.h"
#include "FactorGraph-inl.h"
#include "BayesChain-inl.h"
#include "SymbolicBayesChain-inl.h"

namespace gtsam {

	/** Symbolic Factor */
	class SymbolicFactor: public Testable<SymbolicFactor> {
	private:

		std::list<std::string> keys_;

	public:

		SymbolicFactor(std::list<std::string> keys) :
			keys_(keys) {
		}

		typedef boost::shared_ptr<SymbolicFactor> shared_ptr;

		/** print */
		void print(const std::string& s = "SymbolicFactor") const {
			cout << s << " ";
			BOOST_FOREACH(string key, keys_) cout << key << " ";
			cout << endl;
		}

		/** check equality */
		bool equals(const SymbolicFactor& other, double tol = 1e-9) const {
			return keys_ == other.keys_;
		}

		/**
		 * Find all variables
		 * @return The set of all variable keys
		 */
		std::list<std::string> keys() const {
			return keys_;
		}
	};

	/** Symbolic Factor Graph */
	class SymbolicFactorGraph: public FactorGraph<SymbolicFactor> {
	public:

		SymbolicFactorGraph() {}

		template<class Factor>
		SymbolicFactorGraph(const FactorGraph<Factor>& fg) {
			for (size_t i = 0; i < fg.size(); i++) {
				boost::shared_ptr<Factor> f = fg[i];
				std::list<std::string> keys = f->keys();
				SymbolicFactor::shared_ptr factor(new SymbolicFactor(keys));
				push_back(factor);
			}
		}

	};

}

using namespace std;
using namespace gtsam;

/* ************************************************************************* */
TEST( SymbolicBayesChain, symbolicFactorGraph )
{
	// construct expected symbolic graph
	SymbolicFactorGraph expected;

	list<string> f1_keys; f1_keys.push_back("x1");
	SymbolicFactor::shared_ptr f1(new SymbolicFactor(f1_keys));
	expected.push_back(f1);

	list<string> f2_keys; f2_keys.push_back("x1"); f2_keys.push_back("x2");
	SymbolicFactor::shared_ptr f2(new SymbolicFactor(f2_keys));
	expected.push_back(f2);

	list<string> f3_keys; f3_keys.push_back("l1"); f3_keys.push_back("x1");
	SymbolicFactor::shared_ptr f3(new SymbolicFactor(f3_keys));
	expected.push_back(f3);

	list<string> f4_keys; f4_keys.push_back("l1"); f4_keys.push_back("x2");
	SymbolicFactor::shared_ptr f4(new SymbolicFactor(f4_keys));
	expected.push_back(f4);

	// construct it from the factor graph graph
	LinearFactorGraph factorGraph = createLinearFactorGraph();
	SymbolicFactorGraph actual(factorGraph);

	CHECK(assert_equal(expected, actual));

	//symbolicGraph.find_factors_and_remove("x");
}

/* ************************************************************************* */
TEST( SymbolicBayesChain, constructor )
{
	// Create manually
	SymbolicConditional::shared_ptr x2(new SymbolicConditional("x1", "l1"));
	SymbolicConditional::shared_ptr l1(new SymbolicConditional("x1"));
	SymbolicConditional::shared_ptr x1(new SymbolicConditional());
	map<string, SymbolicConditional::shared_ptr> nodes;
	nodes.insert(make_pair("x2", x2));
	nodes.insert(make_pair("l1", l1));
	nodes.insert(make_pair("x1", x1));
	SymbolicBayesChain expected(nodes);

	// Create from a factor graph
	Ordering ordering;
	ordering.push_back("x2");
	ordering.push_back("l1");
	ordering.push_back("x1");
	LinearFactorGraph factorGraph = createLinearFactorGraph();
	SymbolicBayesChain actual(factorGraph, ordering);
	//CHECK(assert_equal(expected, actual));
}

/* ************************************************************************* */
int main() {
	TestResult tr;
	return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
