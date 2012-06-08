///* ----------------------------------------------------------------------------
//
// * GTSAM Copyright 2010, Georgia Tech Research Corporation,
// * Atlanta, Georgia 30332-0415
// * All Rights Reserved
// * Authors: Frank Dellaert, et al. (see THANKS for the full author list)
//
// * See LICENSE for the license information
//
// * -------------------------------------------------------------------------- */
//
//#include <gtsam/linear/SubgraphSolver.h>
//
//using namespace std;
//
//namespace gtsam {
//
///* split the gaussian factor graph Ab into Ab1 and Ab2 according to the map */
//bool split(const std::map<Index, Index> &M,
//		const GaussianFactorGraph &Ab,
//		GaussianFactorGraph &Ab1,
//		GaussianFactorGraph &Ab2) {
//
//	Ab1 = GaussianFactorGraph();
//	Ab2 = GaussianFactorGraph();
//
//	for ( size_t i = 0 ; i < Ab.size() ; ++i ) {
//
//		boost::shared_ptr<GaussianFactor> factor = Ab[i] ;
//
//		if (factor->keys().size() > 2)
//			throw(invalid_argument("split: only support factors with at most two keys"));
//		if (factor->keys().size() == 1) {
//			Ab1.push_back(factor);
//			Ab2.push_back(factor);
//			continue;
//		}
//		Index key1 = factor->keys()[0];
//		Index key2 = factor->keys()[1];
//
//		if ((M.find(key1) != M.end() && M.find(key1)->second == key2) ||
//				(M.find(key2) != M.end() && M.find(key2)->second == key1))
//			Ab1.push_back(factor);
//		else
//			Ab2.push_back(factor);
//	}
//	return true ;
//}
//
//} // \namespace gtsam
