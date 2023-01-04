//*************************************************************************
// base
//*************************************************************************

namespace gtsam {

#include <gtsam/geometry/Cal3Bundler.h>
#include <gtsam/geometry/Cal3DS2.h>
#include <gtsam/geometry/Cal3Fisheye.h>
#include <gtsam/geometry/Cal3Unified.h>
#include <gtsam/geometry/Cal3_S2.h>
#include <gtsam/geometry/CalibratedCamera.h>
#include <gtsam/geometry/EssentialMatrix.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot2.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/StereoPoint2.h>
#include <gtsam/navigation/ImuBias.h>

// #####

#include <gtsam/base/debug.h>
bool isDebugVersion();

#include <gtsam/base/DSFMap.h>
class IndexPair {
  IndexPair();
  IndexPair(size_t i, size_t j);
  size_t i() const;
  size_t j() const;
};

template<KEY = {gtsam::IndexPair}>
class DSFMap {
  DSFMap();
  KEY find(const KEY& key) const;
  void merge(const KEY& x, const KEY& y);
  std::map<KEY, This::Set> sets();
};

class IndexPairSet {
  IndexPairSet();
  // common STL methods
  size_t size() const;
  bool empty() const;
  void clear();

  // structure specific methods
  void insert(gtsam::IndexPair key);
  bool erase(gtsam::IndexPair key);        // returns true if value was removed
  bool count(gtsam::IndexPair key) const;  // returns true if value exists
};

class IndexPairVector {
  IndexPairVector();
  IndexPairVector(const gtsam::IndexPairVector& other);

  // common STL methods
  size_t size() const;
  bool empty() const;
  void clear();

  // structure specific methods
  gtsam::IndexPair at(size_t i) const;
  void push_back(gtsam::IndexPair key) const;
};

gtsam::IndexPairVector IndexPairSetAsArray(gtsam::IndexPairSet& set);

class IndexPairSetMap {
  IndexPairSetMap();
  // common STL methods
  size_t size() const;
  bool empty() const;
  void clear();

  // structure specific methods
  gtsam::IndexPairSet at(gtsam::IndexPair& key);
};

#include <gtsam/base/Matrix.h>
#include <gtsam/base/MatrixSerialization.h>
bool linear_independent(Matrix A, Matrix B, double tol);

#include <gtsam/base/Value.h>
virtual class Value {
  // No constructors because this is an abstract class

  // Testable
  void print(string s = "") const;

  // Manifold
  size_t dim() const;
};

#include <gtsam/base/GenericValue.h>
template <T = {Vector, Matrix, gtsam::Point2, gtsam::Point3, gtsam::Rot2,
               gtsam::Rot3, gtsam::Pose2, gtsam::Pose3, gtsam::StereoPoint2,
               gtsam::Cal3_S2, gtsam::Cal3DS2, gtsam::Cal3Bundler,
               gtsam::Cal3Fisheye, gtsam::Cal3Unified, gtsam::EssentialMatrix,
               gtsam::CalibratedCamera, gtsam::imuBias::ConstantBias}>
virtual class GenericValue : gtsam::Value {
  void serializable() const;
};

}  // namespace gtsam