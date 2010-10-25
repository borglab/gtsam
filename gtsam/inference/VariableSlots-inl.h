/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    VariableSlots-inl.h
 * @brief   
 * @author  Richard Roberts
 * @created Oct 5, 2010
 */

#pragma once

#include <gtsam/inference/VariableSlots.h>

#include <iostream>

#include <boost/foreach.hpp>
#include <boost/tuple/tuple.hpp>

namespace gtsam {

using namespace std;

/* ************************************************************************* */
template<class FG>
VariableSlots::VariableSlots(const FG& factorGraph) {
  static const bool debug = false;

  // Compute a mapping (called variableSlots) *from* each involved
  // variable that will be in the new joint factor *to* the slot in each
  // removed factor in which that variable appears.  For each variable,
  // this is stored as a vector of slot numbers, stored in order of the
  // removed factors.  The slot number is the max integer value if the
  // factor does not involve that variable.
  size_t jointFactorPos = 0;
  BOOST_FOREACH(const typename FG::sharedFactor& factor, factorGraph) {
    assert(factor);
    Index factorVarSlot = 0;
    BOOST_FOREACH(const Index involvedVariable, *factor) {
      // Set the slot in this factor for this variable.  If the
      // variable was not already discovered, create an array for it
      // that we'll fill with the slot indices for each factor that
      // we're combining.  Initially we put the max integer value in
      // the array entry for each factor that will indicate the factor
      // does not involve the variable.
      iterator thisVarSlots; bool inserted;
      boost::tie(thisVarSlots, inserted) = this->insert(make_pair(involvedVariable, vector<Index>()));
      if(inserted)
        thisVarSlots->second.resize(factorGraph.size(), numeric_limits<Index>::max());
      thisVarSlots->second[jointFactorPos] = factorVarSlot;
      if(debug) cout << "  var " << involvedVariable << " rowblock " << jointFactorPos << " comes from factor's slot " << factorVarSlot << endl;
      ++ factorVarSlot;
    }
    ++ jointFactorPos;
  }
}

}
