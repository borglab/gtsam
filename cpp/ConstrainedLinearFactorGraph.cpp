/**
 * @file ConstrainedLinearFactorGraph.cpp
 * @author Alex Cunningham
 */

#include <iostream>
#include "ConstrainedLinearFactorGraph.h"
#include "FactorGraph-inl.h" // for getOrdering
using namespace std;

// trick from some reading group
#define FOREACH_PAIR( KEY, VAL, COL) BOOST_FOREACH (boost::tie(KEY,VAL),COL)

namespace gtsam {

ConstrainedLinearFactorGraph::ConstrainedLinearFactorGraph() {

}

ConstrainedLinearFactorGraph::ConstrainedLinearFactorGraph(
		const LinearFactorGraph& lfg) {
	BOOST_FOREACH(LinearFactor::shared_ptr s, lfg)
	{	push_back(s);
	}
}

ConstrainedLinearFactorGraph::~ConstrainedLinearFactorGraph() {
}

void ConstrainedLinearFactorGraph::push_back_constraint(LinearConstraint::shared_ptr factor)
{
	constraints_.push_back(factor);
}

bool ConstrainedLinearFactorGraph::is_constrained(const std::string& key) const
{
	BOOST_FOREACH(LinearConstraint::shared_ptr e, constraints_)
	{
		if (e->involves(key)) return true;
	}
	return false;
}

bool ConstrainedLinearFactorGraph::equals(const LinearFactorGraph& fg, double tol) const
{
	const ConstrainedLinearFactorGraph* p = (const ConstrainedLinearFactorGraph *) &fg;
	if (p == NULL) return false;

	/** check equality factors */
	if (constraints_.size() != p->constraints_.size()) return false;
	BOOST_FOREACH(LinearConstraint::shared_ptr ef1, constraints_)
	{
		bool contains = false;
		BOOST_FOREACH(LinearConstraint::shared_ptr ef2, p->constraints_)
		if (ef1->equals(*ef2))
			contains = true;
		if (!contains) return false;
	}

	/** default equality check */
	return LinearFactorGraph::equals(fg, tol);
}

ChordalBayesNet::shared_ptr ConstrainedLinearFactorGraph::eliminate(const Ordering& ordering) {
	ChordalBayesNet::shared_ptr cbn (new ChordalBayesNet());

	BOOST_FOREACH(string key, ordering) {
		// constraints take higher priority in elimination, so check if
		// there are constraints first
		if (is_constrained(key))
		{
			ConditionalGaussian::shared_ptr ccg = eliminate_constraint(key);
			cbn->insert(key,ccg);
		}
		else
		{
			ConditionalGaussian::shared_ptr cg = eliminate_one(key);
			cbn->insert(key,cg);
		}
	}

	return cbn;
}

ConstrainedConditionalGaussian::shared_ptr ConstrainedLinearFactorGraph::eliminate_constraint(const string& key)
{
	// extract the constraint - in-place remove from graph
	LinearConstraint::shared_ptr constraint = extract_constraint(key);

	// perform elimination on the constraint itself to get the constrained conditional gaussian
	ConstrainedConditionalGaussian::shared_ptr ccg = constraint->eliminate(key);

	// perform a change of variables on the linear factors in the separator
	LinearFactorSet separator = find_factors_and_remove(key);
	BOOST_FOREACH(LinearFactor::shared_ptr factor, separator) {
		// reconstruct with a mutable factor
		boost::shared_ptr<MutableLinearFactor> new_factor(new MutableLinearFactor);

		// get T = A1*inv(C1) term
		Matrix A1 = factor->get_A(key);
		Matrix invC1 = inverse(constraint->get_A(key));
		Matrix T = A1*invC1;

		// loop over all nodes in separator of constraint
		list<string> constraint_keys = constraint->keys(key);
		BOOST_FOREACH(string cur_key, constraint_keys) {
			Matrix Ci = constraint->get_A(cur_key);
			if (cur_key != key && !factor->involves(cur_key)) {
				Matrix Ai = T*Ci;
				new_factor->insert(cur_key, -1 *Ai);
			} else if (cur_key != key) {
				Matrix Ai = factor->get_A(cur_key) - T*Ci;
				new_factor->insert(cur_key, Ai);
			}
		}

		// update RHS of updated factor
		Vector new_b = factor->get_b() - T*constraint->get_b();
		new_factor->set_b(new_b);

		// insert the new factor into the graph
		push_back(new_factor);
	}

	return ccg;
}

LinearConstraint::shared_ptr ConstrainedLinearFactorGraph::extract_constraint(const string& key)
{
	LinearConstraint::shared_ptr ret;
	bool found_key = false;
	vector<LinearConstraint::shared_ptr> new_vec;
	BOOST_FOREACH(LinearConstraint::shared_ptr constraint, constraints_)
	{
		if (constraint->involves(key)) {
			ret = constraint;
			found_key = true;
		}
		else
			new_vec.push_back(constraint);
	}
	constraints_ = new_vec;
	if (!found_key)
		throw invalid_argument("No constraint connected to node: " + key);
	return ret;
}

FGConfig ConstrainedLinearFactorGraph::optimize(const Ordering& ordering) {
	ChordalBayesNet::shared_ptr cbn = eliminate(ordering);
	boost::shared_ptr<FGConfig> newConfig = cbn->optimize();
	return *newConfig;
}

void ConstrainedLinearFactorGraph::print(const std::string& s) const
{
	cout << "ConstrainedFactorGraph: " << s << endl;
	BOOST_FOREACH(LinearFactor::shared_ptr f, factors_)
	{
		f->print();
	}
	BOOST_FOREACH(LinearConstraint::shared_ptr f, constraints_)
	{
		f->print();
	}
}

Ordering ConstrainedLinearFactorGraph::getOrdering() const
{
	Ordering ord = LinearFactorGraph::getOrdering();
	//	BOOST_FOREACH(LinearConstraint::shared_ptr e, constraints_)
	//		ord.push_back(e->get_key());
	return ord;
}

}
