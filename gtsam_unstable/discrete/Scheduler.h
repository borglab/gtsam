/*
 * Scheduler.h
 * @brief an example how inference can be used for scheduling qualifiers
 * @date Mar 26, 2011
 * @author Frank Dellaert
 */

#pragma once

#include <gtsam_unstable/discrete/CSP.h>

namespace gtsam {

/**
 * Scheduler class
 * Creates one variable for each student, and three variables for each
 * of the student's areas, for a total of 4*nrStudents variables.
 * The "student" variable will determine when the student takes the qual.
 * The "area" variables determine which faculty are on his/her committee.
 */
class GTSAM_UNSTABLE_EXPORT Scheduler : public CSP {
 private:
  /** Internal data structure for students */
  struct Student {
    std::string name_;
    DiscreteKey key_;                // key for student
    std::vector<DiscreteKey> keys_;  // key for areas
    std::vector<std::string> areaName_;
    std::vector<double> advisor_;
    Student(size_t nrFaculty, size_t advisorIndex)
        : keys_(3), areaName_(3), advisor_(nrFaculty, 1.0) {
      advisor_[advisorIndex] = 0.0;
    }
    void print() const {
      using std::cout;
      cout << name_ << ": ";
      for (size_t area = 0; area < 3; area++) cout << areaName_[area] << " ";
      cout << std::endl;
    }
  };

  /** Maximum number of students */
  size_t maxNrStudents_;

  /** discrete keys, indexed by student and area index */
  std::vector<Student> students_;

  /** faculty identifiers */
  std::map<std::string, size_t> facultyIndex_;
  std::vector<std::string> facultyName_, slotName_, areaName_;

  /** area constraints */
  typedef std::map<std::string, std::vector<double> > FacultyInArea;
  FacultyInArea facultyInArea_;

  /** nrTimeSlots * nrFaculty availability constraints */
  std::string available_;

  /** which slots are good */
  std::vector<double> slotsAvailable_;

 public:
  /**
   * Constructor
   * We need to know the number of students in advance for ordering keys.
   * then add faculty, slots, areas, availability, students, in that order
   */
  Scheduler(size_t maxNrStudents) : maxNrStudents_(maxNrStudents) {}

  /// Destructor
  virtual ~Scheduler() {}

  void addFaculty(const std::string& facultyName) {
    facultyIndex_[facultyName] = nrFaculty();
    facultyName_.push_back(facultyName);
  }

  size_t nrFaculty() const { return facultyName_.size(); }

  /** boolean std::string of nrTimeSlots * nrFaculty */
  void setAvailability(const std::string& available) { available_ = available; }

  void addSlot(const std::string& slotName) { slotName_.push_back(slotName); }

  size_t nrTimeSlots() const { return slotName_.size(); }

  const std::string& slotName(size_t s) const { return slotName_[s]; }

  /** slots available, boolean */
  void setSlotsAvailable(const std::vector<double>& slotsAvailable) {
    slotsAvailable_ = slotsAvailable;
  }

  void addArea(const std::string& facultyName, const std::string& areaName) {
    areaName_.push_back(areaName);
    std::vector<double>& table =
        facultyInArea_[areaName];  // will create if needed
    if (table.empty()) table.resize(nrFaculty(), 0);
    table[facultyIndex_[facultyName]] = 1;
  }

  /**
   * Constructor that reads in faculty, slots, availibility.
   * Still need to add areas and students after this
   */
  Scheduler(size_t maxNrStudents, const std::string& filename);

  /** get key for student and area, 0 is time slot itself */
  const DiscreteKey& key(size_t s,
                         boost::optional<size_t> area = boost::none) const;

  /** addStudent has to be called after adding slots and faculty */
  void addStudent(const std::string& studentName, const std::string& area1,
                  const std::string& area2, const std::string& area3,
                  const std::string& advisor);

  /// current number of students
  size_t nrStudents() const { return students_.size(); }

  const std::string& studentName(size_t i) const;
  const DiscreteKey& studentKey(size_t i) const;
  const std::string& studentArea(size_t i, size_t area) const;

  /** Add student-specific constraints to the graph */
  void addStudentSpecificConstraints(
      size_t i, boost::optional<size_t> slot = boost::none);

  /** Main routine that builds factor graph */
  void buildGraph(size_t mutexBound = 7);

  /** print */
  void print(
      const std::string& s = "Scheduler",
      const KeyFormatter& formatter = DefaultKeyFormatter) const override;

  /** Print readable form of assignment */
  void printAssignment(sharedValues assignment) const;

  /** Special print for single-student case */
  void printSpecial(sharedValues assignment) const;

  /** Accumulate faculty stats */
  void accumulateStats(sharedValues assignment,
                       std::vector<size_t>& stats) const;

  /** Eliminate, return a Bayes net */
  DiscreteBayesNet::shared_ptr eliminate() const;

  /** Find the best total assignment - can be expensive */
  sharedValues optimalAssignment() const;

  /** find the assignment of students to slots with most possible committees */
  sharedValues bestSchedule() const;

  /** find the corresponding most desirable committee assignment */
  sharedValues bestAssignment(sharedValues bestSchedule) const;

};  // Scheduler

}  // namespace gtsam
