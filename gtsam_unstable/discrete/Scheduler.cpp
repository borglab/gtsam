/*
 * Scheduler.h
 * @brief an example how inference can be used for scheduling qualifiers
 * @date Mar 26, 2011
 * @author Frank Dellaert
 */

#include <gtsam/base/debug.h>
#include <gtsam/base/timing.h>
#include <gtsam/discrete/DiscreteFactorGraph.h>
#include <gtsam_unstable/discrete/Scheduler.h>

#include <boost/tokenizer.hpp>
#include <cmath>
#include <fstream>
#include <iomanip>

namespace gtsam {

using namespace std;

Scheduler::Scheduler(size_t maxNrStudents, const string& filename)
    : maxNrStudents_(maxNrStudents) {
  typedef boost::tokenizer<boost::escaped_list_separator<char> > Tokenizer;

  // open file
  ifstream is(filename.c_str());
  if (!is) {
    cerr << "Scheduler: could not open file " << filename << endl;
    throw runtime_error("Scheduler: could not open file " + filename);
  }

  string line;  // buffer

  // process first line with faculty
  if (getline(is, line, '\r')) {
    Tokenizer tok(line);
    Tokenizer::iterator it = tok.begin();
    for (++it; it != tok.end(); ++it) addFaculty(*it);
  }

  // for all remaining lines
  size_t count = 0;
  while (getline(is, line, '\r')) {
    if (count++ > 100) throw runtime_error("reached 100 lines, exiting");
    Tokenizer tok(line);
    Tokenizer::iterator it = tok.begin();
    addSlot(*it++);  // add slot
    // add availability
    for (; it != tok.end(); ++it) available_ += (it->empty()) ? "0 " : "1 ";
    available_ += '\n';
  }
}  // constructor

/** addStudent has to be called after adding slots and faculty */
void Scheduler::addStudent(const string& studentName, const string& area1,
                           const string& area2, const string& area3,
                           const string& advisor) {
  assert(nrStudents() < maxNrStudents_);
  assert(facultyInArea_.count(area1));
  assert(facultyInArea_.count(area2));
  assert(facultyInArea_.count(area3));
  size_t advisorIndex = facultyIndex_[advisor];
  Student student(nrFaculty(), advisorIndex);
  student.name_ = studentName;
  // We fix the ordering by assigning a higher index to the student
  // and numbering the areas lower
  Key j = 3 * maxNrStudents_ + nrStudents();
  student.key_ = DiscreteKey(j, nrTimeSlots());
  Key base = 3 * nrStudents();
  student.keys_[0] = DiscreteKey(base + 0, nrFaculty());
  student.keys_[1] = DiscreteKey(base + 1, nrFaculty());
  student.keys_[2] = DiscreteKey(base + 2, nrFaculty());
  student.areaName_[0] = area1;
  student.areaName_[1] = area2;
  student.areaName_[2] = area3;
  students_.push_back(student);
}

/** get key for student and area, 0 is time slot itself */
const DiscreteKey& Scheduler::key(size_t s,
                                  boost::optional<size_t> area) const {
  return area ? students_[s].keys_[*area] : students_[s].key_;
}

const string& Scheduler::studentName(size_t i) const {
  assert(i < nrStudents());
  return students_[i].name_;
}

const DiscreteKey& Scheduler::studentKey(size_t i) const {
  assert(i < nrStudents());
  return students_[i].key_;
}

const string& Scheduler::studentArea(size_t i, size_t area) const {
  assert(i < nrStudents());
  return students_[i].areaName_[area];
}

/** Add student-specific constraints to the graph */
void Scheduler::addStudentSpecificConstraints(size_t i,
                                              boost::optional<size_t> slot) {
  bool debug = ISDEBUG("Scheduler::buildGraph");

  assert(i < nrStudents());
  const Student& s = students_[i];

  if (!slot && !slotsAvailable_.empty()) {
    if (debug) cout << "Adding availability of slots" << endl;
    assert(slotsAvailable_.size() == s.key_.second);
    CSP::add(s.key_, slotsAvailable_);
  }

  // For all areas
  for (size_t area = 0; area < 3; area++) {
    DiscreteKey areaKey = s.keys_[area];
    const string& areaName = s.areaName_[area];

    if (debug) cout << "Area constraints " << areaName << endl;
    assert(facultyInArea_[areaName].size() == areaKey.second);
    CSP::add(areaKey, facultyInArea_[areaName]);

    if (debug) cout << "Advisor constraint " << areaName << endl;
    assert(s.advisor_.size() == areaKey.second);
    CSP::add(areaKey, s.advisor_);

    if (debug) cout << "Availability of faculty " << areaName << endl;
    if (slot) {
      // get all constraints then specialize to slot
      size_t dummyIndex = maxNrStudents_ * 3 + maxNrStudents_;
      DiscreteKey dummy(dummyIndex, nrTimeSlots());
      AlgebraicDecisionTree<Key> p(dummy & areaKey,
                        available_);  // available_ is Doodle string
      auto q = p.choose(dummyIndex, *slot);
      CSP::add(areaKey, q);
    } else {
      DiscreteKeys keys {s.key_, areaKey};
      CSP::add(keys, available_);  // available_ is Doodle string
    }
  }

  // add mutex
  if (debug) cout << "Mutex for faculty" << endl;
  addAllDiff(s.keys_[0] & s.keys_[1] & s.keys_[2]);
}

/** Main routine that builds factor graph */
void Scheduler::buildGraph(size_t mutexBound) {
  bool debug = ISDEBUG("Scheduler::buildGraph");

  if (debug) cout << "Adding student-specific constraints" << endl;
  for (size_t i = 0; i < nrStudents(); i++) addStudentSpecificConstraints(i);

  // special constraint for MN
  if (studentName(0) == "Michael N")
    CSP::add(studentKey(0), "0 0 0 0  1 1 1 1  1 1 1 1  1 1 1 1  1 1 1 1");

  if (!mutexBound) {
    DiscreteKeys dkeys;
    for (const Student& s : students_) dkeys.push_back(s.key_);
    addAllDiff(dkeys);
  } else {
    if (debug) cout << "Mutex for Students" << endl;
    for (size_t i1 = 0; i1 < nrStudents(); i1++) {
      // if mutexBound=1, we only mutex with next student
      size_t bound = min((i1 + 1 + mutexBound), nrStudents());
      for (size_t i2 = i1 + 1; i2 < bound; i2++) {
        addAllDiff(studentKey(i1), studentKey(i2));
      }
    }
  }
}  // buildGraph

/** print */
void Scheduler::print(const string& s, const KeyFormatter& formatter) const {
  cout << s << " Faculty:" << endl;
  for (const string& name : facultyName_) cout << name << '\n';
  cout << endl;

  cout << s << " Slots:\n";
  size_t i = 0;
  for (const string& name : slotName_) cout << i++ << " " << name << endl;
  cout << endl;

  cout << "Availability:\n" << available_ << '\n';

  cout << s << " Area constraints:\n";
  for (const FacultyInArea::value_type& it : facultyInArea_) {
    cout << setw(12) << it.first << ": ";
    for (double v : it.second) cout << v << " ";
    cout << '\n';
  }
  cout << endl;

  cout << s << " Students:\n";
  for (const Student& student : students_) student.print();
  cout << endl;

  CSP::print(s + " Factor graph");
  cout << endl;
}  // print

/** Print readable form of assignment */
void Scheduler::printAssignment(const DiscreteValues& assignment) const {
  // Not intended to be general! Assumes very particular ordering !
  cout << endl;
  for (size_t s = 0; s < nrStudents(); s++) {
    Key j = 3 * maxNrStudents_ + s;
    size_t slot = assignment.at(j);
    cout << studentName(s) << " slot: " << slotName_[slot] << endl;
    Key base = 3 * s;
    for (size_t area = 0; area < 3; area++) {
      size_t faculty = assignment.at(base + area);
      cout << setw(12) << studentArea(s, area) << ": " << facultyName_[faculty]
           << endl;
    }
    cout << endl;
  }
}

/** Special print for single-student case */
void Scheduler::printSpecial(const DiscreteValues& assignment) const {
  DiscreteValues::const_iterator it = assignment.begin();
  for (size_t area = 0; area < 3; area++, it++) {
    size_t f = it->second;
    cout << setw(12) << studentArea(0, area) << ": " << facultyName_[f] << endl;
  }
  cout << endl;
}

/** Accumulate faculty stats */
void Scheduler::accumulateStats(const DiscreteValues& assignment,
                                vector<size_t>& stats) const {
  for (size_t s = 0; s < nrStudents(); s++) {
    Key base = 3 * s;
    for (size_t area = 0; area < 3; area++) {
      size_t f = assignment.at(base + area);
      assert(f < stats.size());
      stats[f]++;
    }  // area
  }    // s
}

/** Eliminate, return a Bayes net */
DiscreteBayesNet::shared_ptr Scheduler::eliminate() const {
  gttic(my_eliminate);
  // TODO: fix this!!
  size_t maxKey = keys().size();
  Ordering defaultKeyOrdering;
  for (size_t i = 0; i < maxKey; ++i) defaultKeyOrdering += Key(i);
  DiscreteBayesNet::shared_ptr chordal =
      this->eliminateSequential(defaultKeyOrdering);
  gttoc(my_eliminate);
  return chordal;
}

/** find the assignment of students to slots with most possible committees */
DiscreteValues Scheduler::bestSchedule() const {
  DiscreteValues best;
  throw runtime_error("bestSchedule not implemented");
  return best;
}

/** find the corresponding most desirable committee assignment */
DiscreteValues Scheduler::bestAssignment(const DiscreteValues& bestSchedule) const {
  DiscreteValues best;
  throw runtime_error("bestAssignment not implemented");
  return best;
}

}  // namespace gtsam
