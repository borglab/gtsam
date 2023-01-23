/*
 * schedulingExample.cpp
 * @brief hard scheduling example
 * @date March 25, 2011
 * @author Frank Dellaert
 */

//#define ENABLE_TIMING
#define ADD_NO_CACHING
#define ADD_NO_PRUNING
#include <gtsam_unstable/discrete/Scheduler.h>
#include <gtsam/base/debug.h>
#include <gtsam/base/timing.h>

#include <boost/format.hpp>

#include <algorithm>

using namespace std;
using namespace gtsam;

/* ************************************************************************* */
void addStudent(Scheduler& s, size_t i) {
  switch (i) {
  case 0:
    s.addStudent("Michael N", "AI", "Autonomy", "Perception", "Tucker Balch");
    break;
  case 1:
    s.addStudent("Tucker H", "Controls", "AI", "Perception", "Jim Rehg");
    break;
  case 2:
    s.addStudent("Jake H", "Controls", "AI", "Perception", "Henrik Christensen");
    break;
  case 3:
    s.addStudent("Tobias K", "Controls", "AI", "Autonomy", "Mike Stilman");
    break;
  case 4:
    s.addStudent("Shu J", "Controls", "AI", "HRI", "N/A 1");
    break;
  case 5:
    s.addStudent("Akansel C", "AI", "Autonomy", "Mechanics",
        "Henrik Christensen");
    break;
  case 6:
    s.addStudent("Tiffany C", "Controls", "N/A 1", "N/A 2", "Charlie Kemp");
    break;
  }
}
/* ************************************************************************* */
Scheduler largeExample(size_t nrStudents = 7) {
//  char cCurrentPath[FILENAME_MAX];
//  if (!getcwd(cCurrentPath, sizeof(cCurrentPath))) return errno;
//  cCurrentPath[sizeof(cCurrentPath) - 1] = '\0'; /* not really required */
//  printf ("The current working directory is %s", cCurrentPath);

  string path("../../../gtsam_unstable/discrete/examples/");
  Scheduler s(nrStudents, path + "Doodle.csv");

  s.addArea("Harvey Lipkin", "Mechanics");
  s.addArea("Wayne Book", "Mechanics");
  s.addArea("Jun Ueda", "Mechanics");

  //  s.addArea("Wayne Book", "Controls");
  s.addArea("Patricio Vela", "Controls");
  s.addArea("Magnus Egerstedt", "Controls");
  s.addArea("Jun Ueda", "Controls");

  //  s.addArea("Frank Dellaert", "Perception");
  s.addArea("Jim Rehg", "Perception");
  s.addArea("Irfan Essa", "Perception");
  s.addArea("Aaron Bobick", "Perception");
  s.addArea("Henrik Christensen", "Perception");

  s.addArea("Mike Stilman", "AI");
  s.addArea("Henrik Christensen", "AI");
  s.addArea("Frank Dellaert", "AI");
  s.addArea("Ayanna Howard", "AI");
  //  s.addArea("Tucker Balch", "AI");

  s.addArea("Ayanna Howard", "Autonomy");
  //  s.addArea("Andrea Thomaz", "Autonomy");
  s.addArea("Charlie Kemp", "Autonomy");
  s.addArea("Tucker Balch", "Autonomy");
  s.addArea("Ron Arkin", "Autonomy");

  s.addArea("Andrea Thomaz", "HRI");
  s.addArea("Karen Feigh", "HRI");
  s.addArea("Charlie Kemp", "HRI");

  // Allow students not to take three areas
  s.addArea("N/A 1", "N/A 1");
  s.addArea("N/A 2", "N/A 2");

  // add students
  for (size_t i = 0; i < nrStudents; i++)
    addStudent(s, i);

  return s;
}

/* ************************************************************************* */
void runLargeExample() {

  Scheduler scheduler = largeExample();
  scheduler.print();

  // BUILD THE GRAPH !
  size_t addMutex = 2;
  scheduler.buildGraph(addMutex);

  // Do brute force product and output that to file
  if (scheduler.nrStudents() == 1) { // otherwise too slow
    DecisionTreeFactor product = scheduler.product();
    product.dot("scheduling-large", DefaultKeyFormatter, false);
  }

  // Do exact inference
  //  SETDEBUG("timing-verbose", true);
  SETDEBUG("DiscreteConditional::DiscreteConditional", true);
  gttic(large);
  auto MPE = scheduler.optimize();
  gttoc(large);
  tictoc_finishedIteration();
  tictoc_print();
  scheduler.printAssignment(MPE);
}

/* ************************************************************************* */
// Solve a series of relaxed problems for maximum flexibility solution
void solveStaged(size_t addMutex = 2) {

  // super-hack! just count...
  bool debug = false;
  SETDEBUG("DiscreteConditional::COUNT", true);
  SETDEBUG("DiscreteConditional::DiscreteConditional", debug); // progress

  // make a vector with slot availability, initially all 1
  // Reads file to get count :-)
  vector<double> slotsAvailable(largeExample(0).nrTimeSlots(), 1.0);

  // now, find optimal value for each student, using relaxed mutex constraints
  for (size_t s = 0; s < 7; s++) {
    // add all students first time, then drop last one second time, etc...
    Scheduler scheduler = largeExample(7 - s);
    //scheduler.print(str(boost::format("Scheduler %d") % (7-s)));

    // only allow slots not yet taken
    scheduler.setSlotsAvailable(slotsAvailable);

    // BUILD THE GRAPH !
    scheduler.buildGraph(addMutex);

    // Do EXACT INFERENCE
    gttic_(eliminate);
    DiscreteBayesNet::shared_ptr chordal = scheduler.eliminate();
    gttoc_(eliminate);

    // find root node
    DiscreteConditional::shared_ptr root = chordal->back();
    if (debug)
      root->print(""/*scheduler.studentName(s)*/);

    // solve root node only
    size_t bestSlot = root->argmax();

    // get corresponding count
    DiscreteKey dkey = scheduler.studentKey(6 - s);
    DiscreteValues values;
    values[dkey.first] = bestSlot;
    size_t count = (*root)(values);

    // remove this slot from consideration
    slotsAvailable[bestSlot] = 0.0;
    cout << boost::format("%s = %d (%d), count = %d") % scheduler.studentName(6-s)
        % scheduler.slotName(bestSlot) % bestSlot % count << endl;
  }
  tictoc_print_();

  // Solution with addMutex = 2: (20 secs)
  //  TC = Wed 2 (9), count = 96375041778
  //  AC = Tue 2 (5), count = 4076088090
  //  SJ = Mon 1 (0), count = 29596704
  //  TK = Mon 3 (2), count = 755370
  //  JH = Wed 4 (11), count = 12000
  //  TH = Fri 2 (17), count = 220
  //  MN = Fri 1 (16), count = 5
  //
  // Mutex does make a difference !!

}

/* ************************************************************************* */
// Sample from solution found above and evaluate cost function
bool NonZero(size_t i) {
  return i > 0;
}

DiscreteBayesNet::shared_ptr createSampler(size_t i,
    size_t slot, vector<Scheduler>& schedulers) {
  Scheduler scheduler = largeExample(0); // todo: wrong nr students
  addStudent(scheduler, i);
  SETDEBUG("Scheduler::buildGraph", false);
  scheduler.addStudentSpecificConstraints(0, slot);
  DiscreteBayesNet::shared_ptr chordal = scheduler.eliminate();
  // chordal->print(scheduler[i].studentKey(0).name()); // large !
  schedulers.push_back(scheduler);
  return chordal;
}

void sampleSolutions() {

  vector<Scheduler> schedulers;
  vector<DiscreteBayesNet::shared_ptr> samplers(7);

  // Given the time-slots, we can create 7 independent samplers
  vector<size_t> slots{16, 17, 11, 2, 0, 5, 9}; // given slots
  for (size_t i = 0; i < 7; i++)
    samplers[i] = createSampler(i, slots[i], schedulers);

  // now, sample schedules
  for (size_t n = 0; n < 500; n++) {
    vector<size_t> stats(19, 0);
    vector<DiscreteValues> samples;
    for (size_t i = 0; i < 7; i++) {
      samples.push_back(samplers[i]->sample());
      schedulers[i].accumulateStats(samples[i], stats);
    }
    size_t max = *max_element(stats.begin(), stats.end());
    size_t min = *min_element(stats.begin(), stats.end());
    size_t nz = count_if(stats.begin(), stats.end(), NonZero);
    if (nz >= 15 && max <= 2) {
      cout << boost::format(
          "Sampled schedule %d, min = %d, nz = %d, max = %d\n") % (n + 1) % min
          % nz % max;
      for (size_t i = 0; i < 7; i++) {
        cout << schedulers[i].studentName(0) << " : " << schedulers[i].slotName(
            slots[i]) << endl;
        schedulers[i].printSpecial(samples[i]);
      }
    }
  }
  // Output was
  // Sampled schedule 359, min = 0, nz = 15, max = 2
  //  Michael N : Fri 9:00-10.30
  //  Michael N AI: Frank Dellaert
  //  Michael N Autonomy: Charlie Kemp
  //  Michael N Perception: Henrik Christensen
  //
  //  Tucker H : Fri 10:30-12:00
  //   Tucker H AI: Ayanna Howard
  //  Tucker H Controls: Patricio Vela
  //  Tucker H Perception: Irfan Essa
  //
  //  Jake H : Wed 3:00-4:30
  //     Jake H AI: Mike Stilman
  //  Jake H Controls: Magnus Egerstedt
  //  Jake H Perception: Jim Rehg
  //
  //  Tobias K : Mon 1:30-3:00
  //   Tobias K AI: Ayanna Howard
  //  Tobias K Autonomy: Charlie Kemp
  //  Tobias K Controls: Magnus Egerstedt
  //
  //  Shu J : Mon 9:00-10.30
  //      Shu J AI: Mike Stilman
  //  Shu J Controls: Jun Ueda
  //     Shu J HRI: Andrea Thomaz
  //
  //  Akansel C : Tue 10:30-12:00
  //  Akansel C AI: Frank Dellaert
  //  Akansel C Autonomy: Tucker Balch
  //  Akansel C Mechanics: Harvey Lipkin
  //
  //  Tiffany C : Wed 10:30-12:00
  //  Tiffany C Controls: Patricio Vela
  //  Tiffany C N/A 1: N/A 1
  //  Tiffany C N/A 2: N/A 2

}

/* ************************************************************************* */
void accomodateStudent() {

  // super-hack! just count...
  bool debug = false;
  //  SETDEBUG("DiscreteConditional::COUNT",true);
  SETDEBUG("DiscreteConditional::DiscreteConditional", debug); // progress

  Scheduler scheduler = largeExample(0);
  //  scheduler.addStudent("Victor E", "Autonomy", "HRI", "AI",
  //      "Henrik Christensen");
  scheduler.addStudent("Carlos N", "Perception", "AI", "Autonomy",
      "Henrik Christensen");
  scheduler.print("scheduler");

  // rule out all occupied slots
  vector<size_t> slots{16, 17, 11, 2, 0, 5, 9, 14};
  vector<double> slotsAvailable(scheduler.nrTimeSlots(), 1.0);
  for(size_t s: slots)
  slotsAvailable[s] = 0;
  scheduler.setSlotsAvailable(slotsAvailable);

  // BUILD THE GRAPH !
  scheduler.buildGraph(1);

  // Do EXACT INFERENCE
  DiscreteBayesNet::shared_ptr chordal = scheduler.eliminate();

  // find root node
  DiscreteConditional::shared_ptr root = chordal->back();
  if (debug)
    root->print(""/*scheduler.studentName(s)*/);
  //  GTSAM_PRINT(*chordal);

  // solve root node only
  size_t bestSlot = root->argmax();

  // get corresponding count
  DiscreteKey dkey = scheduler.studentKey(0);
  DiscreteValues values;
  values[dkey.first] = bestSlot;
  size_t count = (*root)(values);
  cout << boost::format("%s = %d (%d), count = %d") % scheduler.studentName(0)
      % scheduler.slotName(bestSlot) % bestSlot % count << endl;

  // sample schedules
  for (size_t n = 0; n < 10; n++) {
    auto sample0 = chordal->sample();
    scheduler.printAssignment(sample0);
  }
}

/* ************************************************************************* */
int main() {
    runLargeExample();
  solveStaged(3);
//    sampleSolutions();
  //  accomodateStudent();
  return 0;
}
/* ************************************************************************* */

