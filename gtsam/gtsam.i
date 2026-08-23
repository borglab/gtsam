/**

 * GTSAM Wrap Module Definition
 *
 * These are the current classes available through the matlab and python
 wrappers,
 * add more functions/classes as they are available.
 *
 * Please refer to the wrapping docs:
 https://github.com/borglab/wrap/blob/master/README.md
 */

namespace gtsam {

#include <gtsam/inference/Key.h>

const KeyFormatter DefaultKeyFormatter;

// Actually a FastList<Key>
class KeyList {
  KeyList();
  KeyList(const gtsam::KeyList& other);

  // Note: no print function

  // common STL methods
  size_t size() const;
  bool empty() const;
  void clear();

  // structure specific methods
  const gtsam::Key& front() const;
  const gtsam::Key& back() const;
  void push_back(const gtsam::Key& key);
  void push_front(const gtsam::Key& key);
  void pop_back();
  void pop_front();
  void sort();
  @pybind_lambda
  void remove(const gtsam::Key& key);

  void serialize() const;

  // Special dunder methods for Python wrapping
  __len__();
  __contains__(gtsam::Key key);
  __iter__();
};

// Actually a FastSet<Key>
class KeySet {
  KeySet();
  KeySet(const gtsam::KeySet& set);
  KeySet(const gtsam::KeyVector& vector);
  KeySet(const gtsam::KeyList& list);

  // Testable
  void print(string s = "") const;
  bool equals(const gtsam::KeySet& other, double tol = 1e-9) const;

  // common STL methods
  size_t size() const;
  bool empty() const;
  void clear();

  // structure specific methods
  @pybind_lambda
  void insert(gtsam::Key key);
  void merge(const gtsam::KeySet& other);
  size_t erase(const gtsam::Key& key);
  size_t count(const gtsam::Key& key) const;

  void serialize() const;

  // Special dunder methods for Python wrapping
  __len__();
  __contains__(gtsam::Key key);
  __iter__();
};

// Actually a vector<Key>, needed for Matlab
class KeyVector {
  KeyVector();
  KeyVector(const gtsam::KeyVector& other);

  // Note: no print function

  // common STL methods
  size_t size() const;
  bool empty() const;
  void clear();

  // structure specific methods
  gtsam::Key at(size_t i) const;
  gtsam::Key front() const;
  gtsam::Key back() const;
  void push_back(gtsam::Key key) const;

  void serialize() const;

  // Special dunder methods for Python wrapping
  __len__();
  __contains__(gtsam::Key key);
  __iter__();
};

// Actually a FastMap<Key,int>
class KeyGroupMap {
  KeyGroupMap();

  // Note: no print function

  // common STL methods
  size_t size() const;
  bool empty() const;
  void clear();

  // structure specific methods
  const int& at(const gtsam::Key& key) const;
  size_t erase(const gtsam::Key& key);
  bool insert2(const gtsam::Key& key, const int& val);
};

// Actually a FastSet<FactorIndex>
// Used in Matlab wrapper
class FactorIndexSet {
  FactorIndexSet();
  FactorIndexSet(const gtsam::FactorIndexSet& set);

  // common STL methods
  size_t size() const;
  bool empty() const;
  void clear();

  // structure specific methods
  void insert(gtsam::FactorIndex factorIndex);
  bool erase(gtsam::FactorIndex factorIndex);        // returns true if value was removed
  bool count(gtsam::FactorIndex factorIndex) const;  // returns true if value exists
};

// Actually a vector<FactorIndex>
// Used in Matlab wrapper
class FactorIndices {
  FactorIndices();
  FactorIndices(const gtsam::FactorIndices& other);

  // common STL methods
  size_t size() const;
  bool empty() const;
  void clear();

  // structure specific methods
  gtsam::FactorIndex at(size_t i) const;
  gtsam::FactorIndex front() const;
  gtsam::FactorIndex back() const;
  void push_back(gtsam::FactorIndex factorIndex) const;
};

//*************************************************************************
// Utilities
//*************************************************************************

namespace utilities {

#include <gtsam/nonlinear/utilities.h>
gtsam::KeyList createKeyList(const gtsam::Vector& I);
gtsam::KeyList createKeyList(string s, const gtsam::Vector& I);
gtsam::KeyVector createKeyVector(const gtsam::Vector& I);
gtsam::KeyVector createKeyVector(string s, const gtsam::Vector& I);
gtsam::KeySet createKeySet(const gtsam::Vector& I);
gtsam::KeySet createKeySet(string s, const gtsam::Vector& I);
gtsam::Matrix extractPoint2(const gtsam::Values& values);
gtsam::Matrix extractPoint3(const gtsam::Values& values);
gtsam::Values allPose2s(const gtsam::Values& values);
gtsam::Matrix extractPose2(const gtsam::Values& values);
gtsam::Values allPose3s(const gtsam::Values& values);
gtsam::Matrix extractPose3(const gtsam::Values& values);
gtsam::Matrix extractVectors(const gtsam::Values& values, char c);
void perturbPoint2(gtsam::Values& values, double sigma, int seed = 42u);
void perturbPose2(gtsam::Values& values, double sigmaT, double sigmaR,
                  int seed = 42u);
void perturbPoint3(gtsam::Values& values, double sigma, int seed = 42u);
void perturbPose3(gtsam::Values& values, double sigmaT, double sigmaR,
                  int seed = 42u);
void insertBackprojections(gtsam::Values& values,
                           const gtsam::PinholeCamera<gtsam::Cal3_S2>& camera,
                           const gtsam::Vector& J, gtsam::ConstMatrixView Z,
                           double depth);
@pybind_lambda
void insertProjectionFactors(
    gtsam::NonlinearFactorGraph& graph, gtsam::Key i, const gtsam::Vector& J,
    gtsam::ConstMatrixView Z,
    const gtsam::noiseModel::Base* model,
    const gtsam::Cal3_S2* K,
    const gtsam::Pose3& body_P_sensor = gtsam::Pose3());
gtsam::Matrix reprojectionErrors(const gtsam::NonlinearFactorGraph& graph,
                          const gtsam::Values& values);
gtsam::Values localToWorld(
    const gtsam::Values& local, const gtsam::Pose2& base,
    const gtsam::KeyVector user_keys = gtsam::KeyVector());

}  // namespace utilities

class RedirectCout {
  RedirectCout();
  string str() const;
};

}  // namespace gtsam
