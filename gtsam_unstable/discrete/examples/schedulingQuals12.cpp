/*
 * schedulingExample.cpp
 * @brief hard scheduling example
 * @date March 25, 2011
 * @author Frank Dellaert
 */

#define ENABLE_TIMING
#define ADD_NO_CACHING
#define ADD_NO_PRUNING
#include <gtsam_unstable/discrete/Scheduler.h>
#include <gtsam/base/debug.h>
#include <gtsam/base/timing.h>

#include <boost/assign/std/vector.hpp>
#include <boost/assign/std/map.hpp>
#include <boost/optional.hpp>
#include <boost/format.hpp>

#include <algorithm>

using namespace boost::assign;
using namespace std;
using namespace gtsam;

size_t NRSTUDENTS = 9;

bool NonZero(size_t i) {
  return i > 0;
}

/* ************************************************************************* */
void addStudent(Scheduler& s, size_t i) {
  switch (i) {
  case 0:
    s.addStudent("Pan, Yunpeng", "Controls", "Perception", "Mechanics", "Eric Johnson");
    break;
  case 1:
    s.addStudent("Sawhney, Rahul", "Controls", "AI", "Perception", "Henrik Christensen");
    break;
  case 2:
    s.addStudent("Akgun, Baris", "Controls", "AI", "HRI", "Andrea Thomaz");
    break;
  case 3:
    s.addStudent("Jiang, Shu", "Controls", "AI", "Perception", "Ron Arkin");
    break;
  case 4:
    s.addStudent("Grice, Phillip", "Controls", "Perception", "HRI", "Charlie Kemp");
    break;
  case 5:
    s.addStudent("Huaman, Ana", "Controls", "AI", "Perception", "Mike Stilman");
    break;
  case 6:
    s.addStudent("Levihn, Martin", "AI", "Autonomy", "Perception", "Mike Stilman");
    break;
  case 7:
    s.addStudent("Nieto, Carlos", "AI", "Autonomy", "Perception", "Henrik Christensen");
    break;
  case 8:
    s.addStudent("Robinette, Paul", "Controls", "AI", "HRI", "Ayanna Howard");
    break;
  }
}

/* ************************************************************************* */
Scheduler largeExample(size_t nrStudents = NRSTUDENTS) {
  string path("../../../gtsam_unstable/discrete/examples/");
  Scheduler s(nrStudents, path + "Doodle2012.csv");

  s.addArea("Harvey Lipkin", "Mechanics");
  s.addArea("Jun Ueda", "Mechanics");

  s.addArea("Patricio Vela", "Controls");
  s.addArea("Magnus Egerstedt", "Controls");
  s.addArea("Jun Ueda", "Controls");
  s.addArea("Panos Tsiotras", "Controls");
  s.addArea("Fumin Zhang", "Controls");

  s.addArea("Henrik Christensen", "Perception");
  s.addArea("Aaron Bobick", "Perception");

  s.addArea("Mike Stilman", "AI");
//  s.addArea("Henrik Christensen", "AI");
  s.addArea("Ayanna Howard", "AI");
  s.addArea("Charles Isbell", "AI");
  s.addArea("Tucker Balch", "AI");

  s.addArea("Ayanna Howard", "Autonomy");
  s.addArea("Charlie Kemp", "Autonomy");
  s.addArea("Tucker Balch", "Autonomy");
  s.addArea("Ron Arkin", "Autonomy");

  s.addArea("Andrea Thomaz", "HRI");
  s.addArea("Karen Feigh", "HRI");
  s.addArea("Charlie Kemp", "HRI");

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
  size_t addMutex = 3;
  // SETDEBUG("Scheduler::buildGraph", true);
  scheduler.buildGraph(addMutex);

  // Do brute force product and output that to file
  if (scheduler.nrStudents() == 1) { // otherwise too slow
    DecisionTreeFactor product = scheduler.product();
    product.dot("scheduling-large", false);
  }

  // Do exact inference
  //  SETDEBUG("timing-verbose", true);
  SETDEBUG("DiscreteConditional::DiscreteConditional", true);
#define SAMPLE
#ifdef SAMPLE
  gttic(large);
  DiscreteBayesNet::shared_ptr chordal = scheduler.eliminate();
  gttoc(large);
  tictoc_finishedIteration();
  tictoc_print();
  for (size_t i=0;i<100;i++) {
    DiscreteFactor::sharedValues assignment = chordal->sample();
    vector<size_t> stats(scheduler.nrFaculty());
    scheduler.accumulateStats(assignment, stats);
    size_t max = *max_element(stats.begin(), stats.end());
    size_t min = *min_element(stats.begin(), stats.end());
    size_t nz = count_if(stats.begin(), stats.end(), NonZero);
//    cout << min << ", " << max << ", "  << nz << endl;
    if (nz >= 13 && min >=1 && max <= 4) {
      cout << "======================================================\n";
      scheduler.printAssignment(assignment);
    }
  }
#else
  gttic(large);
  DiscreteFactor::sharedValues MPE = scheduler.optimalAssignment();
  gttoc(large);
  tictoc_finishedIteration();
  tictoc_print();
  scheduler.printAssignment(MPE);
#endif
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
  for (size_t s = 0; s < NRSTUDENTS; s++) {
    // add all students first time, then drop last one second time, etc...
    Scheduler scheduler = largeExample(NRSTUDENTS - s);
    //scheduler.print(str(boost::format("Scheduler %d") % (NRSTUDENTS-s)));

    // only allow slots not yet taken
    scheduler.setSlotsAvailable(slotsAvailable);

    // BUILD THE GRAPH !
    scheduler.buildGraph(addMutex);

    // Do EXACT INFERENCE
    gttic_(eliminate);
    DiscreteBayesNet::shared_ptr chordal = scheduler.eliminate();
    gttoc_(eliminate);

    // find root node
//    chordal->back()->print("back: ");
//    chordal->front()->print("front: ");
//    exit(0);
    DiscreteConditional::shared_ptr root = chordal->back();
    if (debug)
      root->print(""/*scheduler.studentName(s)*/);

    // solve root node only
    Scheduler::Values values;
    size_t bestSlot = root->solve(values);

    // get corresponding count
    DiscreteKey dkey = scheduler.studentKey(NRSTUDENTS - 1 - s);
    values[dkey.first] = bestSlot;
    size_t count = (*root)(values);

    // remove this slot from consideration
    slotsAvailable[bestSlot] = 0.0;
    cout << boost::format("%s = %d (%d), count = %d") % scheduler.studentName(NRSTUDENTS-1-s)
        % scheduler.slotName(bestSlot) % bestSlot % count << endl;
  }
  tictoc_print_();
}

/* ************************************************************************* */
// Sample from solution found above and evaluate cost function
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
  vector<DiscreteBayesNet::shared_ptr> samplers(NRSTUDENTS);

  // Given the time-slots, we can create NRSTUDENTS independent samplers
  vector<size_t> slots;
  slots += 3, 20, 2, 6, 5, 11, 1, 4; // given slots
  for (size_t i = 0; i < NRSTUDENTS; i++)
    samplers[i] = createSampler(i, slots[i], schedulers);

  // now, sample schedules
  for (size_t n = 0; n < 500; n++) {
    vector<size_t> stats(19, 0);
    vector<Scheduler::sharedValues> samples;
    for (size_t i = 0; i < NRSTUDENTS; i++) {
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
      for (size_t i = 0; i < NRSTUDENTS; i++) {
        cout << schedulers[i].studentName(0) << " : " << schedulers[i].slotName(
            slots[i]) << endl;
        schedulers[i].printSpecial(samples[i]);
      }
    }
  }
}

/* ************************************************************************* */
int main() {
//  runLargeExample();
  solveStaged(3);
//  sampleSolutions();
  return 0;
}
/* ************************************************************************* */

