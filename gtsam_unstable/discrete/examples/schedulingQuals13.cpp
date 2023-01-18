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

#include <boost/format.hpp>

#include <algorithm>

using namespace std;
using namespace gtsam;

size_t NRSTUDENTS = 12;

bool NonZero(size_t i) {
  return i > 0;
}

/* ************************************************************************* */
void addStudent(Scheduler& s, size_t i) {
  switch (i) {
  case 0:
    s.addStudent("Young, Carol", "Controls", "Autonomy", "Mechanics", "Fumin Zhang");
    break;
  case 1:
    s.addStudent("Erdogan, Can", "Controls", "AI", "Perception", "Mike Stilman");
    break;
  case 2:
    s.addStudent("Arslan, Oktay", "Controls", "AI", "Mechanics", "Panos Tsiotras");
    break;
  case 3:
    s.addStudent("Bhattacharjee, Tapomayukh", "Controls", "AI", "Mechanics", "Charlie Kemp");
    break;
  case 4:
    s.addStudent("Grey, Michael", "Controls", "AI", "Mechanics", "Wayne Book");
    break;
  case 5:
    s.addStudent("O'Flaherty, Rowland", "Controls", "AI", "Mechanics", "Magnus Egerstedt");
    break;
  case 6:
    s.addStudent("Pickem, Daniel", "Controls", "AI", "Mechanics", "Jeff Shamma");
    break;
  case 7:
    s.addStudent("Lee, Kimoon", "Controls", "Autonomy", "Mechanics", "Henrik Christensen");
    break;
  case 8:
    s.addStudent("Melim, Andrew Lyon", "Controls", "AI", "Perception", "Frank Dellaert");
    break;
  case 9:
    s.addStudent("Jensen, David", "Controls", "Autonomy", "HRI", "Andrea Thomaz");
    break;
  case 10:
    s.addStudent("Nisbett, Jared", "Controls", "Perception", "Mechanics", "Magnus Egerstedt");
    break;
  case 11:
    s.addStudent("Pan, Yunpeng", "Controls", "Perception", "Mechanics", "Wayne Book");
    break;
//    case 12:
//    s.addStudent("Grice, Phillip", "Controls", "None", "None", "Wayne Book");
//    break;
//  case 13:
//    s.addStudent("Robinette, Paul", "Controls", "None", "None", "Ayanna Howard");
//    break;
//  case 14:
//    s.addStudent("Huaman, Ana", "Autonomy", "None", "None", "Mike Stilman");
//    break;
  }
}

/* ************************************************************************* */
Scheduler largeExample(size_t nrStudents = NRSTUDENTS, bool addStudents=true) {
  string path("../../../gtsam_unstable/discrete/examples/");
  Scheduler s(nrStudents, path + "Doodle2013.csv");

  s.addArea("Harvey Lipkin", "Mechanics");
  s.addArea("Jun Ueda", "Mechanics");
  s.addArea("Mike Stilman", "Mechanics");
//  s.addArea("Frank Dellaert", "Mechanics");
  s.addArea("Wayne Book", "Mechanics");
//  s.addArea("Charlie Kemp", "Mechanics");

  s.addArea("Patricio Vela", "Controls");
  s.addArea("Magnus Egerstedt", "Controls");
  s.addArea("Jun Ueda", "Controls");
  s.addArea("Panos Tsiotras", "Controls");
  s.addArea("Fumin Zhang", "Controls");
  s.addArea("Ayanna Howard", "Controls");
  s.addArea("Jeff Shamma", "Controls");

  s.addArea("Frank Dellaert", "Perception");
  s.addArea("Henrik Christensen", "Perception");

  s.addArea("Mike Stilman", "AI");
//  s.addArea("Henrik Christensen", "AI");
//  s.addArea("Ayanna Howard", "AI");
  s.addArea("Charles Isbell", "AI");
//  s.addArea("Tucker Balch", "AI");
  s.addArea("Andrea Thomaz", "AI");

  s.addArea("Ayanna Howard", "Autonomy");
  s.addArea("Charlie Kemp", "Autonomy");

//  s.addArea("Andrea Thomaz", "HRI");
  s.addArea("Karen Feigh", "HRI");
//  s.addArea("Charlie Kemp", "HRI");

  // add students
  if (addStudents)
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
  SETDEBUG("Scheduler::buildGraph", true);
  scheduler.buildGraph(addMutex);

  // Do brute force product and output that to file
  if (scheduler.nrStudents() == 1) { // otherwise too slow
    DecisionTreeFactor product = scheduler.product();
    product.dot("scheduling-large", DefaultKeyFormatter, false);
  }

  // Do exact inference
  //  SETDEBUG("timing-verbose", true);
  SETDEBUG("DiscreteConditional::DiscreteConditional", true);
//#define SAMPLE
#ifdef SAMPLE
  gttic(large);
  DiscreteBayesNet::shared_ptr chordal = scheduler.eliminate();
  gttoc(large);
  tictoc_finishedIteration();
  tictoc_print();
  for (size_t i=0;i<100;i++) {
    auto assignment = sample(*chordal);
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
  auto MPE = scheduler.optimize();
  gttoc(large);
  tictoc_finishedIteration();
  tictoc_print();
  scheduler.printAssignment(MPE);
#endif
}

/* ************************************************************************* */
// Solve a series of relaxed problems for maximum flexibility solution
void solveStaged(size_t addMutex = 2) {

  bool debug = false;

  // super-hack! just count...
  SETDEBUG("DiscreteConditional::COUNT", true);
  SETDEBUG("DiscreteConditional::DiscreteConditional", debug); // progress

  // make a vector with slot availability, initially all 1
  // Reads file to get count :-)
  vector<double> slotsAvailable(largeExample(0).nrTimeSlots(), 1.0);

  // now, find optimal value for each student, using relaxed mutex constraints
  for (size_t s = 0; s < NRSTUDENTS; s++) {
    // add all students first time, then drop last one second time, etc...
    Scheduler scheduler = largeExample(NRSTUDENTS - s);
//    scheduler.print(str(boost::format("Scheduler %d") % (NRSTUDENTS-s)));

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
    DiscreteKey dkey = scheduler.studentKey(NRSTUDENTS - 1 - s);
    DiscreteValues values;
    values[dkey.first] = bestSlot;
    double count = (*root)(values);

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
  Scheduler scheduler = largeExample(1,false);
  addStudent(scheduler, i);
  cout << " creating sampler for " << scheduler.studentName(0) << endl;
  SETDEBUG("Scheduler::buildGraph", false);
//  scheduler.print();
  scheduler.addStudentSpecificConstraints(0, slot);
  DiscreteBayesNet::shared_ptr chordal = scheduler.eliminate();
  schedulers.push_back(scheduler);
  return chordal;
}

void sampleSolutions() {

  size_t nrFaculty = 17; // Change to correct number !

  vector<Scheduler> schedulers;
  vector<DiscreteBayesNet::shared_ptr> samplers(NRSTUDENTS);

  // Given the time-slots, we can create NRSTUDENTS independent samplers
  vector<size_t> slots{12,11,13, 21,16,1, 3,2,6, 7,22,4}; // given slots
  for (size_t i = 0; i < NRSTUDENTS; i++)
    samplers[i] = createSampler(i, slots[i], schedulers);

  // now, sample schedules
  for (size_t n = 0; n < 10000; n++) {
    vector<size_t> stats(nrFaculty, 0);
    vector<DiscreteValues> samples;
    for (size_t i = 0; i < NRSTUDENTS; i++) {
      samples.push_back(samplers[i]->sample());
      schedulers[i].accumulateStats(samples[i], stats);
    }
    size_t max = *max_element(stats.begin(), stats.end());
    size_t min = *min_element(stats.begin(), stats.end());
    size_t nz = count_if(stats.begin(), stats.end(), NonZero);
    if (nz >= 16 && max <= 3) {
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
//  solveStaged(3);
  sampleSolutions();
  return 0;
}
/* ************************************************************************* */

