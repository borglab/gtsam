/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/*
 * Potentials.cpp
 *
 *  @date Feb 21, 2011
 *  @author Duy-Nguyen Ta
 */

#include <math.h>
#include <sstream>
#include <iostream>
#include <algorithm>
#include <boost/format.hpp>
#include <boost/foreach.hpp>
#include <gtsam/discrete/PotentialTable.h>

using namespace std;

namespace gtsam {

  /* ************************************************************************* */
	void PotentialTable::Iterator::operator++() {
		// note size_t is unsigned and i>=0 is always true, so strange-looking loop:
		for (size_t i = size(); i--; ) {
			if (++at(i) < cardinalities_[i])
				return;
			else
				at(i) = 0;
		}
	}

	/* ************************************************************************* */
	size_t PotentialTable::computeTableSize(
			const std::vector<size_t>& cardinalities) {
		size_t tableSize = 1;
		BOOST_FOREACH(const size_t c, cardinalities)
						tableSize *= c;
		return tableSize;
	}

  /* ************************************************************************* */
  PotentialTable::PotentialTable(const std::vector<size_t>& cs) :
		cardinalities_(cs), table_(computeTableSize(cs)) {
		generateKeyFactors();
	}

  /* ************************************************************************* */
  PotentialTable::PotentialTable(const std::vector<size_t>& cardinalities,
      const Table& table) : cardinalities_(cardinalities),table_(table) {
    generateKeyFactors();
  }

  /* ************************************************************************* */
  PotentialTable::PotentialTable(const std::vector<size_t>& cardinalities,
      const std::string& tableString) : cardinalities_(cardinalities) {
    parse(tableString);
    generateKeyFactors();
  }

  /* ************************************************************************* */
  bool PotentialTable::equals(const PotentialTable& other, double tol) const {
    //TODO: compare potentials in a more general sense with arbitrary order of keys???
    if ((cardinalities_ == other.cardinalities_) && (table_.size()
        == other.table_.size()) && (keyFactors_ == other.keyFactors_)) {
      for (size_t i = 0; i < table_.size(); i++) {
        if (fabs(table_[i] - other.table_[i]) > tol) {
          return false;
        }
        return true;
      }
    }
    return false;
  }

  /* ************************************************************************* */
  void PotentialTable::print(const std::string& s) const {
    cout << s << endl;
    for (size_t i = 0; i < cardinalities_.size(); i++)
      cout << boost::format("[%d,%d]") % cardinalities_[i] % keyFactors_[i] << " ";
    cout << endl;
    Iterator assignment(cardinalities_);
    for (size_t idx = 0; idx < table_.size(); ++idx, ++assignment) {
      for (size_t k = 0; k < assignment.size(); k++)
        cout << assignment[k] << "\t\t";
      cout << table_[idx] << endl;
    }
  }

  /* ************************************************************************* */
  const double&  PotentialTable::operator()(const Assignment& var) const  {
    return table_[tableIndexFromAssignment(var)];
  }

  /* ************************************************************************* */
  const double& PotentialTable::operator[](const size_t index) const {
    return table_.at(index);
  }


  /* ************************************************************************* */
  void PotentialTable::setPotential(const PotentialTable::Assignment& asg, const double potential) {
    size_t idx = tableIndexFromAssignment(asg);
    assert(idx<table_.size());
    table_[idx] = potential;
  }

  /* ************************************************************************* */
  void PotentialTable::setPotential(const size_t tableIndex, const double potential) {
    assert(tableIndex<table_.size());
    table_[tableIndex] = potential;
  }


	/* ************************************************************************* */
	size_t PotentialTable::tableIndexFromAssignment(
			const PotentialTable::Assignment& values) const {
		size_t index = 0;
		for (size_t i = 0; i < keyFactors_.size(); i++) {
			index += keyFactors_[i] * values[i];
		}
		return index;
	}

	/* ************************************************************************* */
	PotentialTable::Assignment PotentialTable::assignmentFromTableIndex(
			const size_t idx) const {
		assert(idx < table_.size());
		Assignment values(cardinalities_);
		Index processedIndex = idx;
		for (size_t i = 0; i < keyFactors_.size(); i++) {
      values[i] = processedIndex / keyFactors_[i];
      processedIndex %= keyFactors_[i];
		}
		return values;
	}

	/* ************************************************************************* */
	void PotentialTable::generateKeyFactors() {
	  size_t accumIndex = table_.size();
	  BOOST_FOREACH(size_t card, cardinalities_) {
	    accumIndex /= card;
	    keyFactors_.push_back(accumIndex);
	  }
	}

	/* ************************************************************************* */
	void PotentialTable::parse(const std::string& tableString) {
		// parse doubles
		istringstream iss(tableString);
		copy(istream_iterator<double> (iss), istream_iterator<double> (),
				back_inserter(table_));

#ifndef NDEBUG
		size_t expectedSize = computeTableSize(cardinalities_);
		if (table_.size() != expectedSize) throw invalid_argument(
				boost::str(
						boost::format(
								"String specification \"%s\" for table only contains %d doubles instead of %d")
								% tableString % table_.size() % expectedSize));
#endif
	}

} // namespace
