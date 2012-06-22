/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file	DiscreteBayesNet_FG.cpp
 * @brief 	Discrete Bayes Net example using Factor Graphs
 * @author	Abhijit
 * @date	Jun 4, 2012
 *
 * We use the famous Rain/Cloudy/Sprinkler Example of [Russell & Norvig, 2009, p529]
 * You may be familiar with other graphical model packages like BNT (available
 * at http://bnt.googlecode.com/svn/trunk/docs/usage.html) where this is used as an
 * example. The following demo is same as that in the above link, except that
 * everything is using GTSAM.
 */

#include <gtsam/discrete/DiscreteFactorGraph.h>
#include <gtsam/discrete/DiscreteSequentialSolver.h>
#include <iomanip>

using namespace std;
using namespace gtsam;

int main(int argc, char **argv) {

	// We assume binary state variables
	// we have 0 == "False" and 1 == "True"
	const size_t nrStates = 2;

	// define variables
	DiscreteKey Cloudy(1, nrStates), Sprinkler(2, nrStates), Rain(3, nrStates),
			WetGrass(4, nrStates);

	// create Factor Graph of the bayes net
	DiscreteFactorGraph graph;

	// add factors
	graph.add(Cloudy, "0.5 0.5"); //P(Cloudy)
	graph.add(Cloudy & Sprinkler, "0.5 0.5 0.9 0.1"); //P(Sprinkler | Cloudy)
	graph.add(Cloudy & Rain, "0.8 0.2 0.2 0.8"); //P(Rain | Cloudy)
	graph.add(Sprinkler & Rain & WetGrass,
			"1 0 0.1 0.9 0.1 0.9 0.001 0.99"); //P(WetGrass | Sprinkler, Rain)

	// Alternatively we can also create a DiscreteBayesNet, add DiscreteConditional
	// factors and create a FactorGraph from it. (See testDiscreteBayesNet.cpp)

	// Since this is a relatively small distribution, we can as well print
	// the whole distribution..
	cout << "Distribution of Example: " << endl;
	cout << setw(11) << "Cloudy(C)" << setw(14) << "Sprinkler(S)" << setw(10)
			<< "Rain(R)" << setw(14) << "WetGrass(W)" << setw(15) << "P(C,S,R,W)"
			<< endl;
	for (size_t a = 0; a < nrStates; a++)
		for (size_t m = 0; m < nrStates; m++)
			for (size_t h = 0; h < nrStates; h++)
				for (size_t c = 0; c < nrStates; c++) {
					DiscreteFactor::Values values;
					values[Cloudy.first] = c;
					values[Sprinkler.first] = h;
					values[Rain.first] = m;
					values[WetGrass.first] = a;
					double prodPot = graph(values);
					cout << boolalpha << setw(8) << (bool) c << setw(14)
							<< (bool) h << setw(12) << (bool) m << setw(13)
							<< (bool) a << setw(16) << prodPot << endl;
				}


	// "Most Probable Explanation", i.e., configuration with largest value
	DiscreteSequentialSolver solver(graph);
	DiscreteFactor::sharedValues optimalDecoding = solver.optimize();
	cout <<"\nMost Probable Explanation (MPE):" << endl;
	cout << boolalpha << "Cloudy = " << (bool)(*optimalDecoding)[Cloudy.first]
	                << "  Sprinkler = " << (bool)(*optimalDecoding)[Sprinkler.first]
	                << "  Rain = " << boolalpha << (bool)(*optimalDecoding)[Rain.first]
	                << "  WetGrass = " << (bool)(*optimalDecoding)[WetGrass.first]<< endl;


	// "Inference" We show an inference query like: probability that the Sprinkler was on;
	// given that the grass is wet i.e. P( S | W=1) =?
	cout << "\nInference Query: Probability of Sprinkler being on given Grass is Wet" << endl;

	// Method 1: we can compute the joint marginal P(S,W) and from that we can compute
	// P(S | W=1) = P(S,W=1)/P(W=1) We do this in following three steps..

	//Step1: Compute P(S,W)
	DiscreteFactorGraph jointFG;
	jointFG = *solver.jointFactorGraph(DiscreteKeys(Sprinkler & WetGrass).indices());
	DecisionTreeFactor probSW = jointFG.product();

	//Step2: Compute P(W)
	DiscreteFactor::shared_ptr probW = solver.marginalFactor(WetGrass.first);

	//Step3: Computer P(S | W=1) = P(S,W=1)/P(W=1)
	DiscreteFactor::Values values;
	values[WetGrass.first] = 1;

	//print P(S=0|W=1)
	values[Sprinkler.first] = 0;
	cout << "P(S=0|W=1) = " << probSW(values)/(*probW)(values) << endl;

	//print P(S=1|W=1)
	values[Sprinkler.first] = 1;
	cout << "P(S=1|W=1) = " << probSW(values)/(*probW)(values) << endl;

	// TODO: Method 2 : One way is to modify the factor graph to
	// incorporate the evidence node and compute the marginal
	// TODO: graph.addEvidence(Cloudy,0);

	return 0;
}
