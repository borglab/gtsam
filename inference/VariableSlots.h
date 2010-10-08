/**
 * @file    VariableSlots.h
 * @brief   VariableSlots describes the structure of a combined factor in terms of where each block comes from in the source factors.
 * @author  Richard Roberts
 * @created Oct 4, 2010
 */

#pragma once

#include <gtsam/base/types.h>
#include <gtsam/base/Testable.h>

#include <map>
#include <vector>
#include <string>
#include <boost/pool/pool_alloc.hpp>

namespace gtsam {

/**
 * A combined factor is assembled as one block of rows for each component
 * factor.  In each row-block (factor), some of the column-blocks (variables)
 * may be empty since factors involving different sets of variables are
 * interleaved.
 *
 * VariableSlots describes the 2D block structure of the combined factor.  It
 * is essentially a map<varid_t, vector<size_t> >.  The varid_t is the real
 * variable index of the combined factor slot.  The vector<size_t> tells, for
 * each row-block (factor), which column-block (variable slot) from the
 * component factor appears in this block of the combined factor.
 *
 * As an example, if the combined factor contains variables 1, 3, and 5, then
 * "variableSlots[3][2] == 0" indicates that column-block 1 (corresponding to
 * variable index 3), row-block 2 (also meaning component factor 2), comes from
 * column-block 0 of component factor 2.
 *
 * Note that in the case of GaussianFactor, the rows of the combined factor are
 * further sorted so that the column-block position of the first structural
 * non-zero increases monotonically through the rows.  This additional process
 * is not performed by this class.
 */

// Internal-use-only typedef for the VariableSlots map base class because this is such a long type name
typedef std::map<varid_t, std::vector<varid_t>, std::less<varid_t>,
    boost::fast_pool_allocator<std::pair<const varid_t, std::vector<varid_t> > > > _VariableSlots_map;

class VariableSlots : public _VariableSlots_map, public Testable<VariableSlots> {

public:

  typedef _VariableSlots_map Base;

  /**
   * Constructor from a set of factors to be combined.  Sorts the variables
   * and keeps track of which variable from each factor ends up in each slot
   * of the combined factor, as described in the class comment.
   */
  template<class FG>
  VariableSlots(const FG& factorGraph);

  /** print */
  void print(const std::string& str = "VariableSlots: ") const;

  /** equals */
  bool equals(const VariableSlots& rhs, double tol = 0.0) const;
};

}
