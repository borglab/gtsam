/*
 * ConstrainedLinearFactorGraph.cpp
 *
 *  Created on: Aug 10, 2009
 *      Author: alexgc
 */

#include <iostream>
#include "ConstrainedLinearFactorGraph.h"

using namespace std;

// trick from some reading group
#define FOREACH_PAIR( KEY, VAL, COL) BOOST_FOREACH (boost::tie(KEY,VAL),COL)

namespace gtsam {

ConstrainedLinearFactorGraph::ConstrainedLinearFactorGraph() {

}

ConstrainedLinearFactorGraph::ConstrainedLinearFactorGraph(const LinearFactorGraph& lfg)
{
	BOOST_FOREACH(LinearFactor::shared_ptr s, lfg)
	{
		push_back(s);
	}
}

ConstrainedLinearFactorGraph::~ConstrainedLinearFactorGraph() {
}

void ConstrainedLinearFactorGraph::push_back_eq(EqualityFactor::shared_ptr factor)
{
	eq_factors.push_back(factor);
}

bool ConstrainedLinearFactorGraph::involves_equality(const std::string& key) const
{
	BOOST_FOREACH(EqualityFactor::shared_ptr e, eq_factors)
	{
		if (e->get_key() == key) return true;
	}
	return false;
}

bool ConstrainedLinearFactorGraph::equals(const LinearFactorGraph& fg, double tol) const
{
	const ConstrainedLinearFactorGraph* p = (const ConstrainedLinearFactorGraph *) &fg;
	if (p == NULL) return false;

	/** check equality factors */
	if (eq_factors.size() != p->eq_factors.size()) return false;
	BOOST_FOREACH(EqualityFactor::shared_ptr ef1, eq_factors)
	{
		bool contains = false;
		BOOST_FOREACH(EqualityFactor::shared_ptr ef2, p->eq_factors)
		if (ef1->equals(*ef2))
			contains = true;
		if (!contains) return false;
	}

	/** default equality check */
	return LinearFactorGraph::equals(fg, tol);
}

ConstrainedChordalBayesNet::shared_ptr ConstrainedLinearFactorGraph::eliminate(const Ordering& ordering){
	ConstrainedChordalBayesNet::shared_ptr cbn (new ConstrainedChordalBayesNet());

	BOOST_FOREACH(string key, ordering) {
		if (involves_equality(key)) // check whether this is an existing equality factor
		{
			// check if eliminating an equality factor
			DeltaFunction::shared_ptr d = eliminate_one_eq(key);
			cbn->insert_df(key,d);
		}
		else
		{
			ConditionalGaussian::shared_ptr cg = eliminate_one(key);
			cbn->insert(key,cg);
		}
	}

	return cbn;
}

DeltaFunction::shared_ptr ConstrainedLinearFactorGraph::eliminate_one_eq(const string& key)
{
	// extract the equality factor - also removes from graph
	EqualityFactor::shared_ptr eqf = extract_eq(key);

	// remove all unary linear factors on this node
	vector<LinearFactor::shared_ptr> newfactors;
	BOOST_FOREACH(LinearFactor::shared_ptr f, factors)
	{
		if (f->size() != 1 || !f->involves(key))
		{
			newfactors.push_back(f);
		}
	}
	factors = newfactors;

	// combine the linear factors connected to equality node
	boost::shared_ptr<MutableLinearFactor> joint_factor = combine_factors(key);

	// combine the joint factor with the equality factor and add factors to graph
	if (joint_factor->size() > 0)
		eq_combine_and_eliminate(*eqf, *joint_factor);

	// create the delta function - all delta function information contained in the equality factor
	DeltaFunction::shared_ptr d = eqf->getDeltaFunction();

	return d;
}

EqualityFactor::shared_ptr ConstrainedLinearFactorGraph::extract_eq(const string& key)
{
	EqualityFactor::shared_ptr ret;
	vector<EqualityFactor::shared_ptr> new_vec;
	BOOST_FOREACH(EqualityFactor::shared_ptr eq, eq_factors)
	{
		if (eq->get_key() == key)
			ret = eq;
		else
			new_vec.push_back(eq);
	}
	eq_factors = new_vec;
	return ret;
}

FGConfig ConstrainedLinearFactorGraph::optimize(const Ordering& ordering){
	if (eq_factors.size() == 0)
	{
		// use default optimization
		return LinearFactorGraph::optimize(ordering);
	}

	// eliminate all nodes in the given ordering -> chordal Bayes net
	ConstrainedChordalBayesNet::shared_ptr cbn = eliminate(ordering);

	// calculate new configuration (using backsubstitution)
	boost::shared_ptr<FGConfig> newConfig = cbn->optimize();
	return *newConfig;
}

void ConstrainedLinearFactorGraph::print(const std::string& s) const
{
	cout << "ConstrainedFactorGraph: " << s << endl;
	BOOST_FOREACH(LinearFactor::shared_ptr f, factors)
	{
		f->print();
	}
	BOOST_FOREACH(EqualityFactor::shared_ptr f, eq_factors)
	{
		f->print();
	}
}

void ConstrainedLinearFactorGraph::eq_combine_and_eliminate(
		const EqualityFactor& eqf, const MutableLinearFactor& joint_factor)
{
	// start empty remaining factor to be returned
	boost::shared_ptr<MutableLinearFactor> lf(new MutableLinearFactor);

	// get the value of the target variable (x)
	Vector x = eqf.get_value();

	// get the RHS vector
	Vector b = joint_factor.get_b();

	// get key
	string key = eqf.get_key();

	// get the Ax matrix
	Matrix Ax = joint_factor.get_A(key);

	// calculate new b
	b -= Ax * x;

	// reassemble new factor
	lf->set_b(b);
	string j; Matrix A;
	LinearFactor::const_iterator it = joint_factor.begin();
	for (; it != joint_factor.end(); it++)
	{
		j = it->first;
		A = it->second;
		if (j != key)
		{
			lf->insert(j, A);
		}
	}

	// insert factor
	push_back(lf);
}

Ordering ConstrainedLinearFactorGraph::getOrdering() const
{
	Ordering ord = LinearFactorGraph::getOrdering();
	BOOST_FOREACH(EqualityFactor::shared_ptr e, eq_factors)
	{
		ord.push_back(e->get_key());
	}
	return ord;
}

LinearFactorGraph ConstrainedLinearFactorGraph::convert() const
{
	LinearFactorGraph ret;
	BOOST_FOREACH(LinearFactor::shared_ptr f, factors)
	{
		ret.push_back(f);
	}
	return ret;
}



}
