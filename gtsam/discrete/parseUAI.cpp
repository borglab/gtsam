/*
 * parseUAI.cpp
 * @brief: parse UAI 2008 format
 * @date March 5, 2011
 * @author Duy-Nguyen Ta
 * @author Frank Dellaert
 */

//#define PARSE
#ifdef PARSE
#include <fstream>
#include <boost/spirit/include/qi.hpp> // for parsing
#include <boost/spirit/include/phoenix.hpp> // for ref
#include <boost/spirit/home/phoenix/bind.hpp>
#include <boost/spirit/home/phoenix/object.hpp>
#include <boost/spirit/home/phoenix/operator.hpp>
#include <boost/spirit/include/support_istream_iterator.hpp>

#include <gtsam2/discrete/parseUAI.h>

using namespace std;
namespace qi = boost::spirit::qi;

namespace gtsam {

	/* ************************************************************************* */
	// Keys are the vars of variables connected to a factor
	// subclass of Indices with special constructor
	struct Keys: public Indices {
		Keys() {
		}
		// Pick correct vars based on indices
		Keys(const Indices& vars, const vector<int>& indices) {
			BOOST_FOREACH(int i, indices)
							push_back(vars[i]);
		}
	};

	/* ************************************************************************* */
	// The UAI grammar is defined in a class
	// Spirit local variables are used, see
	// http://boost-spirit.com/home/2010/01/21/what-are-rule-bound-semantic-actions
	/* ************************************************************************* */
	struct Grammar {

		// declare all parsers as instance variables
		typedef vector<double> Table;
		typedef boost::spirit::istream_iterator It;
		qi::rule<It, qi::space_type> uai, preamble, type, vars, factors, tables;
		qi::rule<It, qi::space_type, Keys(), qi::locals<size_t> > keys;
		qi::rule<It, qi::space_type, Table(), qi::locals<size_t> > table;

		// Variables filled by preamble parser
		size_t nrVars_, nrFactors_;
		Indices vars_;
		vector<Keys> factors_;

		// Variables filled by tables parser
		vector<Table> tables_;

		// The constructor defines the parser rules (declared below)
		// To debug, just say debug(rule) after defining the rule
		Grammar() {
			using boost::phoenix::val;
			using boost::phoenix::ref;
			using boost::phoenix::construct;
			using namespace boost::spirit::qi;

			//--------------- high level parsers with side-effects :-( -----------------

			// A uai file consists of preamble followed by tables
			uai = preamble >> tables;

			// The preamble defines the variables and factors
			// The parser fills in the first set of variables above,
			// including the vector of factor "Neighborhoods"
			preamble = type >> vars >> int_[ref(nrFactors_) = _1] >> factors;

			// type string, does not seem to matter
			type = lit("BAYES") | lit("MARKOV");

			// vars parses "3 2 2 3" and synthesizes a Keys class, in this case
			// containing Indices {v0,2}, {v1,2}, and {v2,3}
			vars = int_[ref(nrVars_) = _1] >> (repeat(ref(nrVars_))[int_]) //
					[ref(vars_) = construct<Indices> (_1)];

			// Parse a list of Neighborhoods and fill factors_
			factors = (repeat(ref(nrFactors_))[keys])//
					[ref(factors_) = _1];

			// The tables parser fills in the tables_
			tables = (repeat(ref(nrFactors_))[table])//
					[ref(tables_) = _1];

			//----------- basic parsers with synthesized attributes :-) -----------------

			// keys parses strings like "2 1 2", indicating
			// a binary factor (2) on variables v1 and v2.
			// It returns a Keys class as attribute
			keys = int_[_a = _1] >> repeat(_a)[int_] //
					[_val = construct<Keys> (ref(vars_), _1)];

			// The tables are a list of doubles preceded by a count, e.g. "4 1.0 2.0 3.0 4.0"
			// The table parser returns a PotentialTable::Table attribute
			table = int_[_a = _1] >> repeat(_a)[double_] //
					[_val = construct<Table> (_1)];
		}

		// Add the factors to the graph
		void addFactorsToGraph(TypedDiscreteFactorGraph& graph) {
			assert(factors_.size()==nrFactors_);
			assert(tables_.size()==nrFactors_);
			for (size_t i = 0; i < nrFactors_; i++)
				graph.add(factors_[i], tables_[i]);
		}

	};

	/* ************************************************************************* */
	bool parseUAI(const std::string& filename, TypedDiscreteFactorGraph& graph) {

		// open file, disable skipping of whitespace
		std::ifstream in(filename.c_str());
		if (!in) {
			cerr << "Could not open " << filename << endl;
			return false;
		}

		in.unsetf(std::ios::skipws);

		// wrap istream into iterator
		boost::spirit::istream_iterator first(in);
		boost::spirit::istream_iterator last;

		// Parse and add factors into the graph
		Grammar grammar;
		bool success = qi::phrase_parse(first, last, grammar.uai, qi::space);
		if (success) grammar.addFactorsToGraph(graph);

		return success;
	}
/* ************************************************************************* */

}// gtsam
#else

#include <gtsam2/discrete/parseUAI.h>

namespace gtsam {

/** Dummy version of function - otherwise, missing symbol */
bool parseUAI(const std::string& filename, TypedDiscreteFactorGraph& graph) {
	return false;
}

} // \namespace gtsam
#endif
