/**
 * @file    LinearFactorGraph.cpp
 * @brief   Linear Factor Graph where all factors are Gaussians
 * @author  Kai Ni
 * @author  Christian Potthast
 */

#include <boost/foreach.hpp>
#include <boost/tuple/tuple.hpp>
#include <boost/numeric/ublas/lu.hpp>
#include <boost/numeric/ublas/io.hpp>

#include <colamd/colamd.h>

#include "LinearFactorGraph.h"

using namespace std;
using namespace gtsam;

/* ************************************************************************* */
LinearFactorGraph::LinearFactorGraph(const ChordalBayesNet& CBN)
{
	setCBN(CBN);
}

/* ************************************************************************* */
void LinearFactorGraph::setCBN(const ChordalBayesNet& CBN)
{
	clear();
	ChordalBayesNet::const_iterator it = CBN.begin();
	for(; it != CBN.end(); it++) {
		LinearFactor::shared_ptr lf(new LinearFactor(it->first, it->second));
		push_back(lf);
	}
}

/* ************************************************************************* */
/* find the separators                                                       */
/* ************************************************************************* */
set<string> LinearFactorGraph::find_separator(const string& key) const
{
	set<string> separator;
	BOOST_FOREACH(shared_factor factor,factors)
	factor->tally_separator(key,separator);

	return separator;
}

/* ************************************************************************* */
/** O(n)                                                                     */
/* ************************************************************************* */ 
LinearFactorSet
LinearFactorGraph::find_factors_and_remove(const string& key)
{
	LinearFactorSet found;

	for(iterator factor=factors.begin(); factor!=factors.end(); )
		if ((*factor)->involves(key)) {
			found.insert(*factor);
			factor = factors.erase(factor);
		} else {
			factor++; // important, erase will have effect of ++
		}

	return found;
}

/* ************************************************************************* */
/* find factors and remove them from the factor graph: O(n)                  */
/* ************************************************************************* */ 
boost::shared_ptr<MutableLinearFactor> 
LinearFactorGraph::combine_factors(const string& key)
{
	LinearFactorSet found = find_factors_and_remove(key);
	boost::shared_ptr<MutableLinearFactor> lf(new MutableLinearFactor(found));
	return lf;
}

/* ************************************************************************* */
/* eliminate one node from the linear factor graph                           */ 
/* ************************************************************************* */
ConditionalGaussian::shared_ptr LinearFactorGraph::eliminate_one(const string& key)
{
	// combine the factors of all nodes connected to the variable to be eliminated
	// if no factors are connected to key, returns an empty factor
	boost::shared_ptr<MutableLinearFactor> joint_factor = combine_factors(key);

	// eliminate that joint factor
	try {
		ConditionalGaussian::shared_ptr conditional;
		LinearFactor::shared_ptr factor;
		boost::tie(conditional,factor) = joint_factor->eliminate(key);

		if (!factor->empty())
			push_back(factor);

		// return the conditional Gaussian
		return conditional;
	}
	catch (domain_error&) {
		throw(domain_error("LinearFactorGraph::eliminate: singular graph"));
	}
}

/* ************************************************************************* */
// eliminate factor graph using the given (not necessarily complete)
// ordering, yielding a chordal Bayes net and partially eliminated FG
/* ************************************************************************* */
ChordalBayesNet::shared_ptr
LinearFactorGraph::eliminate_partially(const Ordering& ordering)
{
	ChordalBayesNet::shared_ptr chordalBayesNet (new ChordalBayesNet()); // empty

	BOOST_FOREACH(string key, ordering) {
		ConditionalGaussian::shared_ptr cg = eliminate_one(key);
		chordalBayesNet->insert(key,cg);
	}

	return chordalBayesNet;
}

/* ************************************************************************* */
/** eliminate factor graph in the given order, yielding a chordal Bayes net  */ 
/* ************************************************************************* */
ChordalBayesNet::shared_ptr
LinearFactorGraph::eliminate(const Ordering& ordering)
{
	ChordalBayesNet::shared_ptr chordalBayesNet = eliminate_partially(ordering);

	// after eliminate, only one zero indegree factor should remain
	// TODO: this check needs to exist - verify that unit tests work when this check is in place
	/*
  if (factors.size() != 1) {
    print();
    throw(invalid_argument("LinearFactorGraph::eliminate: graph not empty after eliminate, ordering incomplete?"));
  }
	 */
	return chordalBayesNet;
}

/* ************************************************************************* */
/** optimize the linear factor graph                                          */ 
/* ************************************************************************* */
FGConfig LinearFactorGraph::optimize(const Ordering& ordering)
{
	// eliminate all nodes in the given ordering -> chordal Bayes net
	ChordalBayesNet::shared_ptr chordalBayesNet = eliminate(ordering);

	// calculate new configuration (using backsubstitution)
	boost::shared_ptr<FGConfig> newConfig = chordalBayesNet->optimize();

	return *newConfig;
}

/* ************************************************************************* */
/** combine two factor graphs                                                 */ 
/* ************************************************************************* */
void LinearFactorGraph::combine(LinearFactorGraph &lfg){
	for(const_iterator factor=lfg.factors.begin(); factor!=lfg.factors.end(); factor++){
		push_back(*factor);
	}
}

/* ************************************************************************* */
/** combine two factor graphs                                                */ 
/* ************************************************************************* */

const LinearFactorGraph LinearFactorGraph::combine2(const LinearFactorGraph& lfg1,
		const LinearFactorGraph& lfg2)  {
	// create new linear factor graph
	LinearFactorGraph fg;
	// set the first linear factor graph
	fg = lfg1;

	// add the second factors in the graph
	for(const_iterator factor=lfg2.factors.begin(); factor!=lfg2.factors.end(); factor++){
		fg.push_back(*factor);
	}

	return fg;
}


/* ************************************************************************* */  
// find all variables and their dimensions
VariableSet LinearFactorGraph::variables() const {
	VariableSet result;
	BOOST_FOREACH(shared_factor factor,factors) {
		VariableSet vs = factor->variables();
		BOOST_FOREACH(Variable v,vs) result.insert(v);
	}
	return result;
}

/* ************************************************************************* */  
LinearFactorGraph LinearFactorGraph::add_priors(double sigma) const {

	// start with this factor graph
	LinearFactorGraph result = *this;

	// find all variables and their dimensions
	VariableSet vs = variables();

	// for each of the variables, add a prior
	BOOST_FOREACH(Variable v,vs) {
		size_t n = v.dim();
		const string& key = v.key();
		Matrix A = sigma*eye(n);
		Vector b = zero(n);
		shared_factor prior(new LinearFactor(key,A,b));
		result.push_back(prior);
	}
	return result;
}

/* ************************************************************************* */  
pair<Matrix,Vector> LinearFactorGraph::matrix(const Ordering& ordering) const {

	// get all factors
	LinearFactorSet found;
	BOOST_FOREACH(shared_factor factor,factors)
	found.insert(factor);

	// combine them
	MutableLinearFactor lf(found);

	// Return Matrix and Vector
	return lf.matrix(ordering);
}

/* ************************************************************************* */  
Ordering LinearFactorGraph::getOrdering() const {
	int * _symbolicMatrix;
	int * _symbolicColumns;
	int _symbolicLength;
	int _symbolicColLength;
	std::vector<int> _symbolicElimintationOrder;

	Ordering result;

	map<string, vector<int> > symbolToMatrixElement;
	_symbolicLength = 0;
	_symbolicColLength = 0;
	int matrixRow = 0;
	int symNRows = 0;
	int symNCols = 0;
	for(vector<LinearFactor::shared_ptr>::const_iterator it = begin(); it != end(); it++)
	{
		symNRows++;
		for(map<string, Matrix>::const_iterator lit = (*it)->begin(); lit != (*it)->end(); lit++)
		{
			_symbolicLength++;
			symbolToMatrixElement[lit->first].push_back(matrixRow);
		}
		matrixRow++;
	}


	symNCols =  (int)(symbolToMatrixElement.size());
	_symbolicColLength = symNCols + 1;

	if(symNCols == 0) {result.clear(); return result;}
	//printf("%d %d\n", _symbolicLength, _symbolicColLength);

	_symbolicMatrix = new int[_symbolicLength];
	_symbolicColumns = new int[_symbolicColLength];

	int count = 0;
	_symbolicColumns[0] = 0;
	int colCount = 1;
	for(map<string, vector<int> >::iterator it = symbolToMatrixElement.begin(); it != symbolToMatrixElement.end(); it++)
	{
		for(vector<int>::iterator rit = it->second.begin(); rit != it->second.end(); rit++)
			_symbolicMatrix[count++] = (*rit);
		_symbolicColumns[colCount++] = count;
	}

	vector<string> initialOrder;
	for(map<string, vector<int> >::iterator it = symbolToMatrixElement.begin(); it != symbolToMatrixElement.end(); it++)
		initialOrder.push_back(it->first);

	int * tempColumnOrdering = new int[_symbolicColLength];
	for(int i = 0; i < _symbolicColLength; i++) tempColumnOrdering[i] = _symbolicColumns[i];
	int * tempSymbolicMatrix = new int[_symbolicLength*30];
	for(int i = 0; i < _symbolicLength; i++) tempSymbolicMatrix[i] = _symbolicMatrix[i];
	int stats [COLAMD_STATS] ;
	//for(int i = 0; i < _symbolicColLength; i++) printf("!%d\n", tempColumnOrdering[i]);
	colamd(symNRows, symNCols, _symbolicLength*30, tempSymbolicMatrix, tempColumnOrdering, (double *) NULL, stats) ;
	_symbolicElimintationOrder.clear();
	for(int i = 0; i < _symbolicColLength; i++)
		_symbolicElimintationOrder.push_back(tempColumnOrdering[i]);
	//for(int i = 0; i < _symbolicColLength; i++) printf("!%d\n", tempColumnOrdering[i]);
	delete [] tempColumnOrdering;
	delete [] tempSymbolicMatrix;

	result.clear();
	for(vector<int>::const_iterator it = _symbolicElimintationOrder.begin(); it != _symbolicElimintationOrder.end(); it++)
	{
		//printf("%d\n", (*it));
		if((*it) == -1) continue;
		result.push_back(initialOrder[(*it)]);
	}
	delete [] _symbolicMatrix;
	delete [] _symbolicColumns;

	return result;
}

