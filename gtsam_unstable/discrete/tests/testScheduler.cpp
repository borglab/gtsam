/*
 * testScheduler.cpp
 * @date March 25, 2011
 * @author Frank Dellaert
 */

//#define ENABLE_TIMING
#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/timing.h>
#include <gtsam_unstable/discrete/Scheduler.h>

#include <boost/optional.hpp>

using namespace std;
using namespace gtsam;

/* ************************************************************************* */
// Create the expected graph of constraints
DiscreteFactorGraph createExpected() {
  // Start building
  size_t nrFaculty = 4, nrTimeSlots = 3;

  // variables assigning a time to a student:
  // Akansel and Jake
  DiscreteKey A(6, nrTimeSlots), J(7, nrTimeSlots);

  // variables assigning a faculty member to a student area
  // Akansel:AI,ME,PC and Jake:HR,CT,AI
  DiscreteKey A1(0, nrFaculty), J1(3, nrFaculty);
  DiscreteKey A2(1, nrFaculty), J2(4, nrFaculty);
  DiscreteKey A3(2, nrFaculty), J3(5, nrFaculty);

  CSP expected;

  // Area constraints
  string faculty_in_A = "1 0 0 1";
  string faculty_in_C = "0 0 1 0";
  string faculty_in_H = "0 0 0 1";
  string faculty_in_M = "0 1 0 0";
  string faculty_in_P = "1 0 1 0";
  string available = "1 1 1 0   1 1 1 1   0 1 1 1";

  // Akansel
  expected.add(A1, faculty_in_A);  // Area 1
  expected.add(A1, "1 1 1 0");     // Advisor
  expected.add(A & A1, available);
  expected.add(A2, faculty_in_M);  // Area 2
  expected.add(A2, "1 1 1 0");     // Advisor
  expected.add(A & A2, available);
  expected.add(A3, faculty_in_P);  // Area 3
  expected.add(A3, "1 1 1 0");     // Advisor
  expected.add(A & A3, available);
  // Mutual exclusion for faculty
  expected.addAllDiff(A1 & A2 & A3);

  // Jake
  expected.add(J1, faculty_in_H);  // Area 1
  expected.add(J1, "1 0 1 1");     // Advisor
  expected.add(J & J1, available);
  expected.add(J2, faculty_in_C);  // Area 2
  expected.add(J2, "1 0 1 1");     // Advisor
  expected.add(J & J2, available);
  expected.add(J3, faculty_in_A);  // Area 3
  expected.add(J3, "1 0 1 1");     // Advisor
  expected.add(J & J3, available);
  // Mutual exclusion for faculty
  expected.addAllDiff(J1 & J2 & J3);

  // Mutual exclusion for students
  expected.addAllDiff(A, J);

  return std::move(expected);
}

/* ************************************************************************* */
TEST(schedulingExample, test) {
  Scheduler s(2);

  // add faculty
  s.addFaculty("Frank");
  s.addFaculty("Harvey");
  s.addFaculty("Magnus");
  s.addFaculty("Andrea");

  // add time slots
  s.addSlot("Mon");
  s.addSlot("Wed");
  s.addSlot("Fri");

  // add areas
  s.addArea("Frank", "AI");
  s.addArea("Frank", "PC");
  s.addArea("Harvey", "ME");
  s.addArea("Magnus", "CT");
  s.addArea("Magnus", "PC");
  s.addArea("Andrea", "AI");
  s.addArea("Andrea", "HR");

  // add availability, nrTimeSlots * nrFaculty
  string available = "1 1 1 0  1 1 1 1  0 1 1 1";
  s.setAvailability(available);

  // add students
  s.addStudent("Akansel", "AI", "ME", "PC", "Andrea");
  s.addStudent("Jake", "HR", "CT", "AI", "Harvey");

  // BUILD THE GRAPH !
  s.buildGraph();
  //  s.print();

  // Check graph
  DiscreteFactorGraph expected = createExpected();
  EXPECT(assert_equal(expected, (DiscreteFactorGraph)s));

  // Do brute force product and output that to file
  DecisionTreeFactor product = s.product();
  // product.dot("scheduling", false);

  // Do exact inference
  gttic(small);
  auto MPE = s.optimize();
  gttoc(small);

  // print MPE, commented out as unit tests don't print
  //  s.printAssignment(MPE);

  // Commented out as does not work yet
  // s.runArcConsistency(8,10,true);

  // find the assignment of students to slots with most possible committees
  // Commented out as not implemented yet
  //  auto bestSchedule = s.bestSchedule();
  //  GTSAM_PRINT(bestSchedule);

  //  find the corresponding most desirable committee assignment
  // Commented out as not implemented yet
  //  auto bestAssignment = s.bestAssignment(bestSchedule);
  //  GTSAM_PRINT(bestAssignment);
}

/* ************************************************************************* */
TEST(schedulingExample, smallFromFile) {
  string path(TOPSRCDIR "/gtsam_unstable/discrete/examples/");
  Scheduler s(2, path + "small.csv");

  // add areas
  s.addArea("Frank", "AI");
  s.addArea("Frank", "PC");
  s.addArea("Harvey", "ME");
  s.addArea("Magnus", "CT");
  s.addArea("Magnus", "PC");
  s.addArea("Andrea", "AI");
  s.addArea("Andrea", "HR");

  //   add students
  s.addStudent("Akansel", "AI", "ME", "PC", "Andrea");
  s.addStudent("Jake", "HR", "CT", "AI", "Harvey");
  //  s.print();

  // BUILD THE GRAPH !
  s.buildGraph();

  // Check graph
  DiscreteFactorGraph expected = createExpected();
  EXPECT(assert_equal(expected, (DiscreteFactorGraph)s));
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
