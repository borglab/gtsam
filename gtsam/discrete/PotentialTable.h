/*
 * Potentials.h
 *
 *  @date Feb 21, 2011
 *  @author Duy-Nguyen Ta
 */

#ifndef POTENTIALS_H_
#define POTENTIALS_H_

#include <vector>
#include <set>
#include <map>
#include <string>
#include <boost/shared_ptr.hpp>
#include <gtsam/base/types.h>

namespace gtsam
{
/**
 * PotentialTable holds the real-valued potentials for Factors or Conditionals
 */
class PotentialTable {
public:
  typedef std::vector<double> Table;         // container type for potentials f(x1,x2,..)
  typedef std::vector<size_t> Cardinalities; // just a typedef
  typedef std::vector<size_t> Assignment; // just a typedef

	/**
	 * An assignment that can be incemented
	 */
	struct Iterator: std::vector<size_t> {
		Cardinalities cardinalities_;
		Iterator(const Cardinalities& cs):cardinalities_(cs) {
				for(size_t i=0;i<cs.size();i++) push_back(0);
			}
		void operator++();
		};

private:
  std::vector<size_t> cardinalities_; // cardinalities of variables
  Table table_; // Potential values of all instantiations of the variables, following the variables' order in vector Keys.
  std::vector<size_t> keyFactors_;  // factors to multiply a key's assignment with, to access the potential table

  void generateKeyFactors();
  void parse(const std::string& tableString);

public:

  /** compute table size from variable cardinalities */
  static size_t computeTableSize(const std::vector<size_t>& cardinalities);

  /** construct an empty potential */
  PotentialTable() {}

  /** Dangerous empty n-ary potential. */
  PotentialTable(const std::vector<size_t>& cardinalities);

  /** n-ary potential. */
  PotentialTable(const std::vector<size_t>& cardinalities,
      const Table& table);

  /** n-ary potential. */
  PotentialTable(const std::vector<size_t>& cardinalities,
      const std::string& tableString);

  /** return iterator to first element */
  Iterator begin() const { return Iterator(cardinalities_);}

  /** equality */
  bool equals(const PotentialTable& other, double tol = 1e-9) const;

  /** print */
  void print(const std::string& s = "Potential Table: ") const;

  /** return cardinality of a variable */
  size_t cardinality(size_t var) const { return cardinalities_[var]; }
  size_t tableSize() const { return table_.size(); }

  /** accessors to potential values in the table given the assignment */
  const double& operator()(const Assignment& var) const;
  const double& operator[](const size_t index) const;

  void setPotential(const Assignment& asg, const double potential);
  void setPotential(const size_t tableIndex, const double potential);

  /** convert between assignment and where it is in the table */
  size_t tableIndexFromAssignment(const Assignment& var) const;
  Assignment assignmentFromTableIndex(const size_t i) const;
};


} // namespace

#endif /* POTENTIALS_H_ */
