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

/* ************************************************************************* */
ConstrainedLinearFactorGraph::ConstrainedLinearFactorGraph() {

}

/* ************************************************************************* */
ConstrainedLinearFactorGraph::ConstrainedLinearFactorGraph(
		const LinearFactorGraph& lfg) {
	BOOST_FOREACH(LinearFactor::shared_ptr s, lfg)
	{	push_back(s);
	}
}

/* ************************************************************************* */
ConstrainedLinearFactorGraph::~ConstrainedLinearFactorGraph() {
}

/* ************************************************************************* */
void ConstrainedLinearFactorGraph::push_back_constraint(LinearConstraint::shared_ptr factor)
{
	constraints_.push_back(factor);
}

/* ************************************************************************* */
bool ConstrainedLinearFactorGraph::is_constrained(const std::string& key) const
{
	BOOST_FOREACH(LinearConstraint::shared_ptr e, constraints_)
	{
		if (e->involves(key)) return true;
	}
	return false;
}

/* ************************************************************************* */
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

/* ************************************************************************* */
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

/* ************************************************************************* */
ConstrainedConditionalGaussian::shared_ptr ConstrainedLinearFactorGraph::eliminate_constraint(const string& key)
{
	// extract all adjacent constraints
	vector<LinearConstraint::shared_ptr> constraint_separator = find_constraints_and_remove(key);
	if (constraint_separator.size() == 0)
		throw invalid_argument("No constraints on node " + key);

	// split out a constraint to apply
	LinearConstraint::shared_ptr constraint = pick_constraint(constraint_separator);

	// perform change of variables on the remaining constraints and reinsert
	update_constraints(key, constraint_separator, constraint);

	// perform elimination on the constraint itself to get the constrained conditional gaussian
	ConstrainedConditionalGaussian::shared_ptr ccg = constraint->eliminate(key);

	// perform a change of variables on the linear factors in the separator
	LinearFactorSet separator = find_factors_and_remove(key);
	BOOST_FOREACH(LinearFactor::shared_ptr factor, separator) {
		// store the block matrices
		map<string, Matrix> blocks;

		// get the A1 term and reconstruct new_factor without the eliminated block
		Matrix A1 = factor->get_A(key);
		list<string> factor_keys = factor->keys();
		BOOST_FOREACH(string cur_key, factor_keys) {
			if (cur_key != key)
				blocks.insert(make_pair(cur_key, factor->get_A(cur_key)));
		}

		// get T = A1*inv(C1) term
		Matrix invC1 = inverse(constraint->get_A(key));
		Matrix T = A1*invC1;

		// loop over all nodes in separator of constraint
		list<string> constraint_keys = constraint->keys(key);
		BOOST_FOREACH(string cur_key, constraint_keys) {
			Matrix Ci = constraint->get_A(cur_key);
			if (cur_key != key && !factor->involves(cur_key)) {
				Matrix Ai = T*Ci;
				blocks.insert(make_pair(cur_key, -1 *Ai));
			} else if (cur_key != key) {
				Matrix Ai = factor->get_A(cur_key) - T*Ci;
				blocks.insert(make_pair(cur_key, Ai));
			}
		}

		// construct the updated factor
		boost::shared_ptr<LinearFactor> new_factor(new LinearFactor);
		string cur_key; Matrix M;
		FOREACH_PAIR(cur_key, M, blocks) {
			new_factor->insert(cur_key, M);
		}

		// update RHS of updated factor
		Vector new_b(A1.size1());
		if (factor->get_b().size() == 0)
			new_b = -1 * (T * constraint->get_b());
		else
			new_b = factor->get_b() - T * constraint->get_b();
		new_factor->set_b(new_b);

		// insert the new factor into the graph
		push_back(new_factor);
	}

	return ccg;
}

/* ************************************************************************* */
LinearConstraint::shared_ptr ConstrainedLinearFactorGraph::pick_constraint(
		const std::vector<LinearConstraint::shared_ptr>& constraints) const {
	if (constraints.size() == 0)
		throw invalid_argument("Constraint set is empty!");
	return constraints[0];
}

/* ************************************************************************* */
void ConstrainedLinearFactorGraph::update_constraints(const std::string& key,
		const std::vector<LinearConstraint::shared_ptr>& separator,
		const LinearConstraint::shared_ptr& primary) {
	// Implements update for each constraint, where primary is
	//    C1*x1 = d - C2*x2 - ...
	// and each constraint is
	//    A1*x1 + A2*x2 + ... = b;

	// extract components from primary
	Matrix invC1 = inverse(primary->get_A(key));
	Vector d = primary->get_b();

	// perform transform on each constraint
	// constraint c is the one being updated
	BOOST_FOREACH(LinearConstraint::shared_ptr updatee, separator) {
		if (!updatee->equals(*primary)) {
			// build transform matrix
			Matrix A1 = updatee->get_A(key);
			Matrix T = A1 * invC1;

			// copy existing nodes into new factor without the eliminated variable
			list<string> updatee_keys = updatee->keys(key);
			map<string, Matrix> blocks;
			BOOST_FOREACH(string cur_key, updatee_keys) {
				blocks[cur_key] = updatee->get_A(cur_key);
			}

			// loop over all nodes in separator of constraint
			list<string> primary_keys = primary->keys(key); // keys that are not key
			BOOST_FOREACH(string cur_key, primary_keys) {
				Matrix Ci = primary->get_A(cur_key);
				if (cur_key != key && !updatee->involves(cur_key)) {
					Matrix Ai = T*Ci;
					blocks[cur_key] = -1 * Ai;
				} else if (cur_key != key) {
					Matrix Ai = updatee->get_A(cur_key) - T*Ci;
					blocks[cur_key] = Ai;
				}
			}
			Vector rhs = updatee->get_b() - T * d;
			LinearConstraint::shared_ptr new_constraint(new LinearConstraint(blocks, rhs));

			// reinsert into graph
			push_back_constraint(new_constraint);
		}
	}
}

/* ************************************************************************* */
VectorConfig ConstrainedLinearFactorGraph::optimize(const Ordering& ordering) {
	ChordalBayesNet::shared_ptr cbn = eliminate(ordering);
	boost::shared_ptr<VectorConfig> newConfig = cbn->optimize();
	return *newConfig;
}

/* ************************************************************************* */
std::vector<LinearConstraint::shared_ptr>
ConstrainedLinearFactorGraph::find_constraints_and_remove(const string& key)
{
	vector<LinearConstraint::shared_ptr> found, uninvolved;
	BOOST_FOREACH(LinearConstraint::shared_ptr constraint, constraints_) {
		if (constraint->involves(key)) {
			found.push_back(constraint);
		} else {
			uninvolved.push_back(constraint);
		}
	}
	constraints_ = uninvolved;
	return found;
}

/* ************************************************************************* */
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

/* ************************************************************************* */
Ordering ConstrainedLinearFactorGraph::getOrdering() const
{
	Ordering ord = LinearFactorGraph::getOrdering();
	cout << "ConstrainedLinearFactorGraph::getOrdering() - Not Implemented!" << endl;
	//	BOOST_FOREACH(LinearConstraint::shared_ptr e, constraints_)
	//		ord.push_back(e->get_key());
	return ord;
}

}
