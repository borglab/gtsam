/*
 * testSudoku.cpp
 * @brief develop code for Sudoku CSP solver
 * @date Jan 29, 2012
 * @author Frank Dellaert
 */

#include <CppUnitLite/TestHarness.h>
#include <gtsam_unstable/discrete/CSP.h>

#include <boost/assign/std/map.hpp>
using boost::assign::insert;
#include <stdarg.h>

#include <iostream>
#include <sstream>

using namespace std;
using namespace gtsam;

#define PRINT false

class Sudoku : public CSP {
  /// sudoku size
  size_t n_;

  /// discrete keys
  typedef std::pair<size_t, size_t> IJ;
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
    Key k = 0;
    for (size_t i = 0; i < n; ++i) {
      for (size_t j = 0; j < n; ++j, ++k) {
        // create the key
        IJ ij(i, j);
        dkeys_[ij] = DiscreteKey(k, n);
        // get the unary constraint, if any
        int value = va_arg(ap, int);
        // cout << value << " ";
        if (value != 0) addSingleValue(dkeys_[ij], value - 1);
      }
      // cout << endl;
    }
    va_end(ap);

    // add row constraints
    for (size_t i = 0; i < n; i++) {
      DiscreteKeys dkeys;
      for (size_t j = 0; j < n; j++) dkeys += dkey(i, j);
      addAllDiff(dkeys);
    }

    // add col constraints
    for (size_t j = 0; j < n; j++) {
      DiscreteKeys dkeys;
      for (size_t i = 0; i < n; i++) dkeys += dkey(i, j);
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
          for (size_t j = j0; j < j0 + N; j++) dkeys += dkey(i, j);
        addAllDiff(dkeys);
        j0 += N;
      }
      i0 += N;
    }
  }

  /// Print readable form of assignment
  void printAssignment(DiscreteFactor::sharedValues assignment) const {
    for (size_t i = 0; i < n_; i++) {
      for (size_t j = 0; j < n_; j++) {
        Key k = key(i, j);
        cout << 1 + assignment->at(k) << " ";
      }
      cout << endl;
    }
  }

  /// solve and print solution
  void printSolution() {
    DiscreteFactor::sharedValues MPE = optimalAssignment();
    printAssignment(MPE);
  }
};

/* ************************************************************************* */
TEST_UNSAFE(Sudoku, small) {
  Sudoku csp(4,           //
             1, 0, 0, 4,  //
             0, 0, 0, 0,  //
             4, 0, 2, 0,  //
             0, 1, 0, 0);

  // Do BP
  csp.runArcConsistency(4, 10, PRINT);

  // optimize and check
  CSP::sharedValues solution = csp.optimalAssignment();
  CSP::Values expected;
  insert(expected)(csp.key(0, 0), 0)(csp.key(0, 1), 1)(csp.key(0, 2), 2)(
      csp.key(0, 3), 3)(csp.key(1, 0), 2)(csp.key(1, 1), 3)(csp.key(1, 2), 0)(
      csp.key(1, 3), 1)(csp.key(2, 0), 3)(csp.key(2, 1), 2)(csp.key(2, 2), 1)(
      csp.key(2, 3), 0)(csp.key(3, 0), 1)(csp.key(3, 1), 0)(csp.key(3, 2), 3)(
      csp.key(3, 3), 2);
  EXPECT(assert_equal(expected, *solution));
  // csp.printAssignment(solution);
}

/* ************************************************************************* */
TEST_UNSAFE(Sudoku, easy) {
  Sudoku sudoku(9,                          //
                0, 0, 5, 0, 9, 0, 0, 0, 1,  //
                0, 0, 0, 0, 0, 2, 0, 7, 3,  //
                7, 6, 0, 0, 0, 8, 2, 0, 0,  //

                0, 1, 2, 0, 0, 9, 0, 0, 4,  //
                0, 0, 0, 2, 0, 3, 0, 0, 0,  //
                3, 0, 0, 1, 0, 0, 9, 6, 0,  //

                0, 0, 1, 9, 0, 0, 0, 5, 8,  //
                9, 7, 0, 5, 0, 0, 0, 0, 0,  //
                5, 0, 0, 0, 3, 0, 7, 0, 0);

  // Do BP
  sudoku.runArcConsistency(4, 10, PRINT);

  // sudoku.printSolution(); // don't do it
}

/* ************************************************************************* */
TEST_UNSAFE(Sudoku, extreme) {
  Sudoku sudoku(9,                             //
                0, 0, 9, 7, 4, 8, 0, 0, 0, 7,  //
                0, 0, 0, 0, 0, 0, 0, 0, 0, 2,  //
                0, 1, 0, 9, 0, 0, 0, 0, 0, 7,  //
                0, 0, 0, 2, 4, 0, 0, 6, 4, 0,  //
                1, 0, 5, 9, 0, 0, 9, 8, 0, 0,  //
                0, 3, 0, 0, 0, 0, 0, 8, 0, 3,  //
                0, 2, 0, 0, 0, 0, 0, 0, 0, 0,  //
                0, 6, 0, 0, 0, 2, 7, 5, 9, 0, 0);

  // Do BP
  sudoku.runArcConsistency(9, 10, PRINT);

#ifdef METIS
  VariableIndexOrdered index(sudoku);
  index.print("index");
  ofstream os("/Users/dellaert/src/hmetis-1.5-osx-i686/extreme-dual.txt");
  index.outputMetisFormat(os);
#endif

  // sudoku.printSolution(); // don't do it
}

/* ************************************************************************* */
TEST_UNSAFE(Sudoku, AJC_3star_Feb8_2012) {
  Sudoku sudoku(9,                          //
                9, 5, 0, 0, 0, 6, 0, 0, 0,  //
                0, 8, 4, 0, 7, 0, 0, 0, 0,  //
                6, 2, 0, 5, 0, 0, 4, 0, 0,  //

                0, 0, 0, 2, 9, 0, 6, 0, 0,  //
                0, 9, 0, 0, 0, 0, 0, 2, 0,  //
                0, 0, 2, 0, 6, 3, 0, 0, 0,  //

                0, 0, 9, 0, 0, 7, 0, 6, 8,  //
                0, 0, 0, 0, 3, 0, 2, 9, 0,  //
                0, 0, 0, 1, 0, 0, 0, 3, 7);

  // Do BP
  sudoku.runArcConsistency(9, 10, PRINT);

  // sudoku.printSolution(); // don't do it
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
