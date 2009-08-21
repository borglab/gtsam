/*
 * ConstrainedChordalBayesNet.cpp
 *
 *  Created on: Aug 11, 2009
 *      Author: alexgc
 */

#include <iostream>
#include <boost/foreach.hpp>
#include "ConstrainedChordalBayesNet.h"

// trick from some reading group
#define FOREACH_PAIR( KEY, VAL, COL) BOOST_FOREACH (boost::tie(KEY,VAL),COL)

namespace gtsam {

using namespace std;

ConstrainedChordalBayesNet::ConstrainedChordalBayesNet()
: ChordalBayesNet()
{
}

ConstrainedChordalBayesNet::ConstrainedChordalBayesNet(const ChordalBayesNet& cbn)
: ChordalBayesNet(cbn)
{
}

ConstrainedChordalBayesNet::~ConstrainedChordalBayesNet() {
	// TODO Auto-generated destructor stub
}

void ConstrainedChordalBayesNet::insert_df(const string& key, DeltaFunction::shared_ptr node)
{
	keys.push_front(key);
	delta_nodes.insert(make_pair(key,node));
}

void ConstrainedChordalBayesNet::insert(const string& key, ConditionalGaussian::shared_ptr node)
{
	keys.push_front(key);
	nodes.insert(make_pair(key,node));
}

bool ConstrainedChordalBayesNet::equals(const ConstrainedChordalBayesNet& cbn) const
{
	// check delta function nodes
	if (delta_nodes.size() != cbn.delta_nodes.size()) return false;
	const_delta_iterator it1 = delta_nodes.begin(), it2 = cbn.delta_nodes.begin();
	for(; it1 != delta_nodes.end(); it1++, it2++){
		const string& j1 = it1->first, j2 = it2->first;
		DeltaFunction::shared_ptr node1 = it1->second, node2 = it2->second;
		if (j1 != j2) return false;
		if (!node1->equals(*node2)) return false;
	}

	// use default equals
	return convert().equals(cbn.convert());
}

bool assert_equal(const ConstrainedChordalBayesNet& expected, const ConstrainedChordalBayesNet& actual, double tol)
{
	bool ret = expected.equals(actual);
	if (!ret)
	{
		cout << "Not Equal!" << endl;
		expected.print("Expected");
		actual.print("Actual");
	}
	return ret;
}

void ConstrainedChordalBayesNet::print(const std::string& s) const
{
	cout << s << ":" << endl;
	pair<string, DeltaFunction::shared_ptr> p1;
	BOOST_FOREACH(p1, delta_nodes)
	{
		cout << "  " << p1.first << endl;
		p1.second->print();
	}
	pair<string, ConditionalGaussian::shared_ptr> p2;
	BOOST_FOREACH(p2, nodes)
	{
		cout << "  " << p2.first << endl;
		p2.second->print();
	}
}

boost::shared_ptr<FGConfig> ConstrainedChordalBayesNet::optimize()
{
	boost::shared_ptr<FGConfig> empty(new FGConfig);
	return optimize(empty);
}

boost::shared_ptr<FGConfig> ConstrainedChordalBayesNet::optimize(const boost::shared_ptr<FGConfig> &c)
{
	// check if it is necessary to handle delta functions
	if (delta_nodes.size() == 0)
	{
		return ChordalBayesNet::optimize(c);
	}

	// verifying that there are no incorrect values for variables with delta functions
	vector<string> keys = c->get_names();
	BOOST_FOREACH(string k, keys)
	{
		if (delta_nodes.count(k) && !(delta_nodes[k]->get_value() == c->get(k)))
			throw(std::invalid_argument("ConstrainedChordalBayesNet:: Passed incorrect value for " + k + " to optimize()"));
	}

	// create a config with the delta functions
	pair<string, DeltaFunction::shared_ptr> p;
	BOOST_FOREACH(p, delta_nodes)
	{
		Vector v = p.second->get_value();
		string key = p.first;
		c->insert(key, v);
	}
	return convert().optimize(c);
}

ChordalBayesNet ConstrainedChordalBayesNet::convert() const
{
	ChordalBayesNet ret;
	pair<string, ConditionalGaussian::shared_ptr> p;
	BOOST_FOREACH(p, nodes)
	{
		ret.insert(p.first, p.second);
	}
	return ret;
}

}
