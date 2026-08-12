/*
 * testSudoku.cpp
 * @brief develop code for Sudoku CSP solver
 * @date Jan 29, 2012
 * @author Frank Dellaert
 */

#include <CppUnitLite/TestHarness.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam_unstable/discrete/CSP.h>

#include <stdarg.h>

#include <iostream>
#include <sstream>

using namespace std;
using namespace gtsam;

#define PRINT false

/// A class that encodes Sudoku's as a CSP problem
class Sudoku : public CSP {
  size_t n_;  ///< Side of Sudoku, e.g. 4 or 9

  /// Mapping from base i,j coordinates to discrete keys:
  using IJ = std::pair<size_t, size_t>;
  std::map<IJ, DiscreteKey> dkeys_;

 public:
  /// return DiscreteKey for cell(i,j)
  const DiscreteKey& dkey(size_t i, size_t j) const {
    return dkeys_.at(IJ(i, j));
  }

  /// return Key for cell(i,j)
  Key key(size_t i, size_t j) const { return dkey(i, j).first; }

  /// Constructor
  Sudoku(size_t n, ...) : n_(n) {
    // Create variables, ordering, and unary constraints
    va_list ap;
    va_start(ap, n);
    for (size_t i = 0; i < n; ++i) {
      for (size_t j = 0; j < n; ++j) {
        // create the key
        IJ ij(i, j);
        Symbol key('1' + i, j + 1);
        dkeys_[ij] = DiscreteKey(key, n);
        // get the unary constraint, if any
        int value = va_arg(ap, int);
        if (value != 0) addSingleValue(dkeys_[ij], value - 1);
      }
      // cout << endl;
    }
    va_end(ap);

    // add row constraints
    for (size_t i = 0; i < n; i++) {
      DiscreteKeys dkeys;
      for (size_t j = 0; j < n; j++) dkeys.push_back(dkey(i, j));
      addAllDiff(dkeys);
    }

    // add col constraints
    for (size_t j = 0; j < n; j++) {
      DiscreteKeys dkeys;
      for (size_t i = 0; i < n; i++) dkeys.push_back(dkey(i, j));
      addAllDiff(dkeys);
    }

    // add box constraints
    size_t N = (size_t)sqrt(double(n)), i0 = 0;
    for (size_t I = 0; I < N; I++) {
      size_t j0 = 0;
      for (size_t J = 0; J < N; J++) {
        // Box I,J
        DiscreteKeys dkeys;
        for (size_t i = i0; i < i0 + N; i++)
          for (size_t j = j0; j < j0 + N; j++) dkeys.push_back(dkey(i, j));
        addAllDiff(dkeys);
        j0 += N;
      }
      i0 += N;
    }
  }

  /// Print readable form of assignment
  void printAssignment(const DiscreteValues& assignment) const {
    for (size_t i = 0; i < n_; i++) {
      for (size_t j = 0; j < n_; j++) {
        Key k = key(i, j);
        cout << 1 + assignment.at(k) << " ";
      }
      cout << endl;
    }
  }

  /// solve and print solution
  void printSolution() const {
    auto MPE = optimize();
    printAssignment(MPE);
  }

  // Print domain
  void printDomains(const Domains& domains) {
    for (size_t i = 0; i < n_; i++) {
      for (size_t j = 0; j < n_; j++) {
        Key k = key(i, j);
        cout << domains.at(k).base1Str();
        cout << "\t";
      }  // i
      cout << endl;
    }  // j
  }
};

/* ************************************************************************* */
TEST(Sudoku, small) {
  Sudoku csp(4,           //
             1, 0, 0, 4,  //
             0, 0, 0, 0,  //
             4, 0, 2, 0,  //
             0, 1, 0, 0);

  // optimize and check
  auto solution = csp.optimize();
  DiscreteValues expected;
  expected = {{csp.key(0, 0), 0}, {csp.key(0, 1), 1},
              {csp.key(0, 2), 2}, {csp.key(0, 3), 3},  //
              {csp.key(1, 0), 2}, {csp.key(1, 1), 3},
              {csp.key(1, 2), 0}, {csp.key(1, 3), 1},  //
              {csp.key(2, 0), 3}, {csp.key(2, 1), 2},
              {csp.key(2, 2), 1}, {csp.key(2, 3), 0},  //
              {csp.key(3, 0), 1}, {csp.key(3, 1), 0},
              {csp.key(3, 2), 3}, {csp.key(3, 3), 2}};
  EXPECT(assert_equal(expected, solution));
  // csp.printAssignment(solution);

  // Do BP (AC1)
  auto domains = csp.runArcConsistency(4, 3);
  // csp.printDomains(domains);
  Domain domain44 = domains.at(Symbol('4', 4));
  EXPECT_LONGS_EQUAL(1, domain44.nrValues());

  // Test Creation of a new, simpler CSP
  CSP new_csp = csp.partiallyApply(domains);
  // Should only be 16 new Domains
  EXPECT_LONGS_EQUAL(16, new_csp.size());

  // Check that solution
  auto new_solution = new_csp.optimize();
  // csp.printAssignment(new_solution);
  EXPECT(assert_equal(expected, new_solution));
}

/* ************************************************************************* */
TEST(Sudoku, easy) {
  Sudoku csp(9,                          //
             0, 0, 5, 0, 9, 0, 0, 0, 1,  //
             0, 0, 0, 0, 0, 2, 0, 7, 3,  //
             7, 6, 0, 0, 0, 8, 2, 0, 0,  //

             0, 1, 2, 0, 0, 9, 0, 0, 4,  //
             0, 0, 0, 2, 0, 3, 0, 0, 0,  //
             3, 0, 0, 1, 0, 0, 9, 6, 0,  //

             0, 0, 1, 9, 0, 0, 0, 5, 8,  //
             9, 7, 0, 5, 0, 0, 0, 0, 0,  //
             5, 0, 0, 0, 3, 0, 7, 0, 0);

  // csp.printSolution(); // don't do it

  // Do BP (AC1)
  auto domains = csp.runArcConsistency(9, 10);
  // csp.printDomains(domains);
  Key key99 = Symbol('9', 9);
  Domain domain99 = domains.at(key99);
  EXPECT_LONGS_EQUAL(1, domain99.nrValues());

  // Test Creation of a new, simpler CSP
  CSP new_csp = csp.partiallyApply(domains);
  // 81 new Domains, and still 26 all-diff constraints
  EXPECT_LONGS_EQUAL(81 + 26, new_csp.size());

  // csp.printSolution(); // still don't do it ! :-(
}

/* ************************************************************************* */
TEST(Sudoku, extreme) {
  Sudoku csp(9,                             //
             0, 0, 9, 7, 4, 8, 0, 0, 0, 7,  //
             0, 0, 0, 0, 0, 0, 0, 0, 0, 2,  //
             0, 1, 0, 9, 0, 0, 0, 0, 0, 7,  //
             0, 0, 0, 2, 4, 0, 0, 6, 4, 0,  //
             1, 0, 5, 9, 0, 0, 9, 8, 0, 0,  //
             0, 3, 0, 0, 0, 0, 0, 8, 0, 3,  //
             0, 2, 0, 0, 0, 0, 0, 0, 0, 0,  //
             0, 6, 0, 0, 0, 2, 7, 5, 9, 0, 0);

  // Do BP
  csp.runArcConsistency(9, 10);

#ifdef METIS
  VariableIndexOrdered index(csp);
  index.print("index");
  ofstream os("/Users/dellaert/src/hmetis-1.5-osx-i686/extreme-dual.txt");
  index.outputMetisFormat(os);
#endif

  // Do BP (AC1)
  auto domains = csp.runArcConsistency(9, 10);
  // csp.printDomains(domains);
  Key key99 = Symbol('9', 9);
  Domain domain99 = domains.at(key99);
  EXPECT_LONGS_EQUAL(2, domain99.nrValues());

  // Test Creation of a new, simpler CSP
  CSP new_csp = csp.partiallyApply(domains);
  // 81 new Domains, and still 20 all-diff constraints
  EXPECT_LONGS_EQUAL(81 + 20, new_csp.size());

  // csp.printSolution(); // still don't do it ! :-(
}

/* ************************************************************************* */
TEST(Sudoku, AJC_3star_Feb8_2012) {
  Sudoku csp(9,                          //
             9, 5, 0, 0, 0, 6, 0, 0, 0,  //
             0, 8, 4, 0, 7, 0, 0, 0, 0,  //
             6, 2, 0, 5, 0, 0, 4, 0, 0,  //

             0, 0, 0, 2, 9, 0, 6, 0, 0,  //
             0, 9, 0, 0, 0, 0, 0, 2, 0,  //
             0, 0, 2, 0, 6, 3, 0, 0, 0,  //

             0, 0, 9, 0, 0, 7, 0, 6, 8,  //
             0, 0, 0, 0, 3, 0, 2, 9, 0,  //
             0, 0, 0, 1, 0, 0, 0, 3, 7);

  // Do BP (AC1)
  auto domains = csp.runArcConsistency(9, 10);
  // csp.printDomains(domains);
  Key key99 = Symbol('9', 9);
  Domain domain99 = domains.at(key99);
  EXPECT_LONGS_EQUAL(1, domain99.nrValues());

  // Test Creation of a new, simpler CSP
  CSP new_csp = csp.partiallyApply(domains);
  // Just the 81 new Domains
  EXPECT_LONGS_EQUAL(81, new_csp.size());

  // Check that solution
  auto solution = new_csp.optimize();
  // csp.printAssignment(solution);
  EXPECT_LONGS_EQUAL(6, solution.at(key99));
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
