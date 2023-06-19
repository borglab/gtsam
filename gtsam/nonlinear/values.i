//*************************************************************************
// nonlinear but only Values
//*************************************************************************

namespace gtsam {

#include <gtsam/geometry/Cal3Bundler.h>
#include <gtsam/geometry/Cal3DS2.h>
#include <gtsam/geometry/Cal3Fisheye.h>
#include <gtsam/geometry/Cal3Unified.h>
#include <gtsam/geometry/Cal3_S2.h>
#include <gtsam/geometry/CalibratedCamera.h>
#include <gtsam/geometry/EssentialMatrix.h>
#include <gtsam/geometry/PinholeCamera.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot2.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/SO3.h>
#include <gtsam/geometry/SO4.h>
#include <gtsam/geometry/SOn.h>
#include <gtsam/geometry/StereoPoint2.h>
#include <gtsam/geometry/Unit3.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/navigation/NavState.h>
#include <gtsam/basis/ParameterMatrix.h>

#include <gtsam/linear/VectorValues.h>

#include <gtsam/nonlinear/Values.h>
class Values {
  Values();
  Values(const gtsam::Values& other);

  size_t size() const;
  bool empty() const;
  void clear();
  size_t dim() const;

  void print(string s = "", const gtsam::KeyFormatter& keyFormatter =
                                gtsam::DefaultKeyFormatter) const;
  bool equals(const gtsam::Values& other, double tol) const;

  void insert(const gtsam::Values& values);
  void update(const gtsam::Values& values);
  void insert_or_assign(const gtsam::Values& values);
  void erase(size_t j);
  void swap(gtsam::Values& values);

  bool exists(size_t j) const;
  gtsam::KeyVector keys() const;

  gtsam::VectorValues zeroVectors() const;

  gtsam::Values retract(const gtsam::VectorValues& delta) const;
  gtsam::VectorValues localCoordinates(const gtsam::Values& cp) const;

  // enabling serialization functionality
  void serialize() const;

  // New in 4.0, we have to specialize every insert/update/at to generate
  // wrappers Instead of the old: void insert(size_t j, const gtsam::Value&
  // value); void update(size_t j, const gtsam::Value& val); gtsam::Value
  // at(size_t j) const;

  // The order is important: Vector has to precede Point2/Point3 so `atVector`
  // can work for those fixed-size vectors.
  void insert(size_t j, Vector vector);
  void insert(size_t j, Matrix matrix);
  void insert(size_t j, const gtsam::Point2& point2);
  void insert(size_t j, const gtsam::Point3& point3);
  void insert(size_t j, const gtsam::Rot2& rot2);
  void insert(size_t j, const gtsam::Pose2& pose2);
  void insert(size_t j, const gtsam::SO3& R);
  void insert(size_t j, const gtsam::SO4& Q);
  void insert(size_t j, const gtsam::SOn& P);
  void insert(size_t j, const gtsam::Rot3& rot3);
  void insert(size_t j, const gtsam::Pose3& pose3);
  void insert(size_t j, const gtsam::Unit3& unit3);
  void insert(size_t j, const gtsam::Cal3_S2& cal3_s2);
  void insert(size_t j, const gtsam::Cal3DS2& cal3ds2);
  void insert(size_t j, const gtsam::Cal3Bundler& cal3bundler);
  void insert(size_t j, const gtsam::Cal3Fisheye& cal3fisheye);
  void insert(size_t j, const gtsam::Cal3Unified& cal3unified);
  void insert(size_t j, const gtsam::EssentialMatrix& essential_matrix);
  void insert(size_t j, const gtsam::PinholeCamera<gtsam::Cal3_S2>& camera);
  void insert(size_t j, const gtsam::PinholeCamera<gtsam::Cal3Bundler>& camera);
  void insert(size_t j, const gtsam::PinholeCamera<gtsam::Cal3Fisheye>& camera);
  void insert(size_t j, const gtsam::PinholeCamera<gtsam::Cal3Unified>& camera);
  void insert(size_t j, const gtsam::PinholePose<gtsam::Cal3_S2>& camera);
  void insert(size_t j, const gtsam::PinholePose<gtsam::Cal3Bundler>& camera);
  void insert(size_t j, const gtsam::PinholePose<gtsam::Cal3Fisheye>& camera);
  void insert(size_t j, const gtsam::PinholePose<gtsam::Cal3Unified>& camera);
  void insert(size_t j, const gtsam::imuBias::ConstantBias& constant_bias);
  void insert(size_t j, const gtsam::NavState& nav_state);
  void insert(size_t j, double c);
  void insert(size_t j, const gtsam::ParameterMatrix& X);

  template <T = {gtsam::Point2, gtsam::Point3}>
  void insert(size_t j, const T& val);

  void update(size_t j, const gtsam::Point2& point2);
  void update(size_t j, const gtsam::Point3& point3);
  void update(size_t j, const gtsam::Rot2& rot2);
  void update(size_t j, const gtsam::Pose2& pose2);
  void update(size_t j, const gtsam::SO3& R);
  void update(size_t j, const gtsam::SO4& Q);
  void update(size_t j, const gtsam::SOn& P);
  void update(size_t j, const gtsam::Rot3& rot3);
  void update(size_t j, const gtsam::Pose3& pose3);
  void update(size_t j, const gtsam::Unit3& unit3);
  void update(size_t j, const gtsam::Cal3_S2& cal3_s2);
  void update(size_t j, const gtsam::Cal3DS2& cal3ds2);
  void update(size_t j, const gtsam::Cal3Bundler& cal3bundler);
  void update(size_t j, const gtsam::Cal3Fisheye& cal3fisheye);
  void update(size_t j, const gtsam::Cal3Unified& cal3unified);
  void update(size_t j, const gtsam::EssentialMatrix& essential_matrix);
  void update(size_t j, const gtsam::PinholeCamera<gtsam::Cal3_S2>& camera);
  void update(size_t j, const gtsam::PinholeCamera<gtsam::Cal3Bundler>& camera);
  void update(size_t j, const gtsam::PinholeCamera<gtsam::Cal3Fisheye>& camera);
  void update(size_t j, const gtsam::PinholeCamera<gtsam::Cal3Unified>& camera);
  void update(size_t j, const gtsam::PinholePose<gtsam::Cal3_S2>& camera);
  void update(size_t j, const gtsam::PinholePose<gtsam::Cal3Bundler>& camera);
  void update(size_t j, const gtsam::PinholePose<gtsam::Cal3Fisheye>& camera);
  void update(size_t j, const gtsam::PinholePose<gtsam::Cal3Unified>& camera);
  void update(size_t j, const gtsam::imuBias::ConstantBias& constant_bias);
  void update(size_t j, const gtsam::NavState& nav_state);
  void update(size_t j, Vector vector);
  void update(size_t j, Matrix matrix);
  void update(size_t j, double c);
  void update(size_t j, const gtsam::ParameterMatrix& X);

  void insert_or_assign(size_t j, const gtsam::Point2& point2);
  void insert_or_assign(size_t j, const gtsam::Point3& point3);
  void insert_or_assign(size_t j, const gtsam::Rot2& rot2);
  void insert_or_assign(size_t j, const gtsam::Pose2& pose2);
  void insert_or_assign(size_t j, const gtsam::SO3& R);
  void insert_or_assign(size_t j, const gtsam::SO4& Q);
  void insert_or_assign(size_t j, const gtsam::SOn& P);
  void insert_or_assign(size_t j, const gtsam::Rot3& rot3);
  void insert_or_assign(size_t j, const gtsam::Pose3& pose3);
  void insert_or_assign(size_t j, const gtsam::Unit3& unit3);
  void insert_or_assign(size_t j, const gtsam::Cal3_S2& cal3_s2);
  void insert_or_assign(size_t j, const gtsam::Cal3DS2& cal3ds2);
  void insert_or_assign(size_t j, const gtsam::Cal3Bundler& cal3bundler);
  void insert_or_assign(size_t j, const gtsam::Cal3Fisheye& cal3fisheye);
  void insert_or_assign(size_t j, const gtsam::Cal3Unified& cal3unified);
  void insert_or_assign(size_t j,
                        const gtsam::EssentialMatrix& essential_matrix);
  void insert_or_assign(size_t j,
                        const gtsam::PinholeCamera<gtsam::Cal3_S2>& camera);
  void insert_or_assign(size_t j,
                        const gtsam::PinholeCamera<gtsam::Cal3Bundler>& camera);
  void insert_or_assign(size_t j,
                        const gtsam::PinholeCamera<gtsam::Cal3Fisheye>& camera);
  void insert_or_assign(size_t j,
                        const gtsam::PinholeCamera<gtsam::Cal3Unified>& camera);
  void insert_or_assign(size_t j,
                        const gtsam::PinholePose<gtsam::Cal3_S2>& camera);
  void insert_or_assign(size_t j,
                        const gtsam::PinholePose<gtsam::Cal3Bundler>& camera);
  void insert_or_assign(size_t j,
                        const gtsam::PinholePose<gtsam::Cal3Fisheye>& camera);
  void insert_or_assign(size_t j,
                        const gtsam::PinholePose<gtsam::Cal3Unified>& camera);
  void insert_or_assign(size_t j,
                        const gtsam::imuBias::ConstantBias& constant_bias);
  void insert_or_assign(size_t j, const gtsam::NavState& nav_state);
  void insert_or_assign(size_t j, Vector vector);
  void insert_or_assign(size_t j, Matrix matrix);
  void insert_or_assign(size_t j, double c);
  void insert_or_assign(size_t j, const gtsam::ParameterMatrix& X);

  template <T = {gtsam::Point2,
                 gtsam::Point3,
                 gtsam::Rot2,
                 gtsam::Pose2,
                 gtsam::SO3,
                 gtsam::SO4,
                 gtsam::SOn,
                 gtsam::Rot3,
                 gtsam::Pose3,
                 gtsam::Unit3,
                 gtsam::Cal3_S2,
                 gtsam::Cal3DS2,
                 gtsam::Cal3Bundler,
                 gtsam::Cal3Fisheye,
                 gtsam::Cal3Unified,
                 gtsam::EssentialMatrix,
                 gtsam::PinholeCamera<gtsam::Cal3_S2>,
                 gtsam::PinholeCamera<gtsam::Cal3Bundler>,
                 gtsam::PinholeCamera<gtsam::Cal3Fisheye>,
                 gtsam::PinholeCamera<gtsam::Cal3Unified>,
                 gtsam::PinholePose<gtsam::Cal3_S2>,
                 gtsam::PinholePose<gtsam::Cal3Bundler>,
                 gtsam::PinholePose<gtsam::Cal3Fisheye>,
                 gtsam::PinholePose<gtsam::Cal3Unified>,
                 gtsam::imuBias::ConstantBias,
                 gtsam::NavState,
                 Vector,
                 Matrix,
                 double,
                 gtsam::ParameterMatrix}>
  T at(size_t j);
};

}  // namespace gtsam
