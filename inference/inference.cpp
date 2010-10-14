/**
 * @file   inference-inl.h
 * @brief  inference definitions
 * @author Frank Dellaert, Richard Roberts
 */

#include <gtsam/inference/inference-inl.h>

#include <gtsam/inference/SymbolicFactorGraph.h>

namespace gtsam {


/* ************************************************************************* */
Conditional::shared_ptr
Inference::EliminateOneSymbolic(FactorGraph<Factor>& factorGraph, VariableIndex<>& variableIndex, Index var) {

  tic("EliminateOne");

  // Get the factors involving the eliminated variable
  VariableIndex<>::mapped_type& varIndexEntry(variableIndex[var]);
  typedef VariableIndex<>::mapped_factor_type mapped_factor_type;

  if(!varIndexEntry.empty()) {

    vector<size_t> removedFactors(varIndexEntry.size());
    transform(varIndexEntry.begin(), varIndexEntry.end(), removedFactors.begin(),
        boost::lambda::bind(&VariableIndex<>::mapped_factor_type::factorIndex, boost::lambda::_1));

    // The new joint factor will be the last one in the factor graph
    size_t jointFactorIndex = factorGraph.size();

    static const bool debug = false;

    if(debug) {
      cout << "Eliminating " << var;
      factorGraph.print(" from graph: ");
      cout << removedFactors.size() << " factors to remove" << endl;
    }

    // Compute the involved keys, uses the variableIndex to mark whether each
    // key has been added yet, but the positions stored in the variableIndex are
    // from the unsorted positions and will be fixed later.
    tic("EliminateOne: Find involved vars");
    typedef set<Index, std::less<Index>, boost::fast_pool_allocator<Index> > InvolvedKeys;
    InvolvedKeys involvedKeys;
    BOOST_FOREACH(size_t removedFactorI, removedFactors) {
      if(debug) cout << removedFactorI << " is involved" << endl;
      // If the factor has not previously been removed
      if(removedFactorI < factorGraph.size() && factorGraph[removedFactorI]) {
        // Loop over the variables involved in the removed factor to update the
        // variable index and joint factor positions of each variable.
        BOOST_FOREACH(Index involvedVariable, factorGraph[removedFactorI]->keys()) {
          if(debug) cout << "  pulls in variable " << involvedVariable << endl;
          // Mark the new joint factor as involving each variable in the removed factor.
          assert(!variableIndex[involvedVariable].empty());
          involvedKeys.insert(involvedVariable);
        }
      }

      // Remove the original factor
      factorGraph.remove(removedFactorI);
    }

    // We need only mark the next variable to be eliminated as involved with the joint factor
    if(involvedKeys.size() > 1) {
      InvolvedKeys::const_iterator next = involvedKeys.begin(); ++ next;
      variableIndex[*next].push_back(mapped_factor_type(jointFactorIndex,0));
    }
    toc("EliminateOne: Find involved vars");
    if(debug) cout << removedFactors.size() << " factors to remove" << endl;

    // Join the factors and eliminate the variable from the joint factor
    tic("EliminateOne: Combine");
    Conditional::shared_ptr conditional = Conditional::FromRange(involvedKeys.begin(), involvedKeys.end(), 1);
    Factor::shared_ptr eliminated(new Factor(conditional->beginParents(), conditional->endParents()));
    toc("EliminateOne: Combine");

    tic("EliminateOne: store eliminated");
    factorGraph.push_back(eliminated);  // Put the eliminated factor into the factor graph
    toc("EliminateOne: store eliminated");

    toc("EliminateOne");

    return conditional;

  } else { // varIndexEntry.empty()
    toc("EliminateOne");
    return Conditional::shared_ptr();
  }
}

}
