/**
 * @file   SymbolicBayesNet.cpp
 * @brief  Chordal Bayes Net, the result of eliminating a factor graph
 * @author Frank Dellaert
 */

#include <gtsam/inference/SymbolicBayesNet.h>
#include <gtsam/inference/BayesNet-inl.h>

using namespace std;

namespace gtsam {

	// Explicitly instantiate so we don't have to include everywhere
	template class BayesNet<SymbolicConditional>;

/* ************************************************************************* */

} // namespace gtsam
