//*************************************************************************
// nonlinear but only Values
//*************************************************************************

namespace gtsam {

#include <gtsam/geometry/Cal3_S2.h>
#include <gtsam/geometry/Cal3Bundler.h>
#include <gtsam/geometry/Cal3DS2.h>
#include <gtsam/geometry/Cal3f.h>
#include <gtsam/geometry/Cal3Fisheye.h>
#include <gtsam/geometry/Cal3Unified.h>
#include <gtsam/geometry/EssentialMatrix.h>
#include <gtsam/geometry/FundamentalMatrix.h>
#include <gtsam/geometry/Gal3.h>
#include <gtsam/geometry/OrientedPlane3.h>
#include <gtsam/geometry/PinholeCamera.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot2.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Similarity2.h>
#include <gtsam/geometry/Similarity3.h>
#include <gtsam/geometry/SL4.h>
#include <gtsam/geometry/SO3.h>
#include <gtsam/geometry/SO4.h>
#include <gtsam/geometry/SOn.h>
#include <gtsam/geometry/SL4.h>
#include <gtsam/geometry/StereoPoint2.h>
#include <gtsam/geometry/Unit3.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/navigation/NavState.h>

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
  void erase(gtsam::Key j);
  void swap(gtsam::Values& other);

  bool exists(gtsam::Key j) const;
  gtsam::KeyVector keys() const;

  gtsam::VectorValues zeroVectors() const;

  gtsam::Values retract(const gtsam::VectorValues& delta) const;
  gtsam::VectorValues localCoordinates(const gtsam::Values& cp) const;

  // enabling serialization functionality
  void serialize() const;

  // New in 4.0, we have to specialize every insert/update/at
  // to generate wrappers.
  // Instead of the old:
  // void insert(gtsam::Key j, const gtsam::Value& value);
  // void update(gtsam::Key j, const gtsam::Value& val);
  // gtsam::Value at(gtsam::Key j) const;

  // The order is important: gtsam::Vector has to precede Point2/Point3 so `atVector`
  // can work for those fixed-size vectors. Same apparently for Cal3Bundler and Cal3f.
  void insert(gtsam::Key j, gtsam::Vector vector);
  void insert(gtsam::Key j, gtsam::Matrix matrix);
  void insert(gtsam::Key j, const gtsam::Point2& point2);
  void insert(gtsam::Key j, const gtsam::Point3& point3);
  void insert(gtsam::Key j, const gtsam::Gal3& T);
  void insert(gtsam::Key j, const gtsam::Pose2& pose2);
  void insert(gtsam::Key j, const gtsam::Pose3& pose3);
  void insert(gtsam::Key j, const gtsam::Rot2& rot2);
  void insert(gtsam::Key j, const gtsam::Rot3& rot3);
  void insert(gtsam::Key j, const gtsam::Similarity2& similarity2);
  void insert(gtsam::Key j, const gtsam::Similarity3& similarity3);
  void insert(gtsam::Key j, const gtsam::SL4& H);
  void insert(gtsam::Key j, const gtsam::SO3& R);
  void insert(gtsam::Key j, const gtsam::SO4& Q);
  void insert(gtsam::Key j, const gtsam::SOn& P);
  void insert(gtsam::Key j, const gtsam::Unit3& unit3);
  void insert(gtsam::Key j, const gtsam::Cal3Bundler& cal3bundler);
  void insert(gtsam::Key j, const gtsam::Cal3f& cal3f);
  void insert(gtsam::Key j, const gtsam::Cal3_S2& cal3_s2);
  void insert(gtsam::Key j, const gtsam::Cal3DS2& cal3ds2);
  void insert(gtsam::Key j, const gtsam::Cal3Fisheye& cal3fisheye);
  void insert(gtsam::Key j, const gtsam::Cal3Unified& cal3unified);
  void insert(gtsam::Key j, const gtsam::EssentialMatrix& E);
  void insert(gtsam::Key j, const gtsam::FundamentalMatrix& F);
  void insert(gtsam::Key j, const gtsam::SimpleFundamentalMatrix& F);
  void insert(gtsam::Key j, const gtsam::OrientedPlane3& plane);
  void insert(gtsam::Key j, const gtsam::PinholeCamera<gtsam::Cal3Bundler>& camera);
  void insert(gtsam::Key j, const gtsam::PinholeCamera<gtsam::Cal3f>& camera);
  void insert(gtsam::Key j, const gtsam::PinholeCamera<gtsam::Cal3_S2>& camera);
  void insert(gtsam::Key j, const gtsam::PinholeCamera<gtsam::Cal3DS2>& camera);
  void insert(gtsam::Key j, const gtsam::PinholeCamera<gtsam::Cal3Fisheye>& camera);
  void insert(gtsam::Key j, const gtsam::PinholeCamera<gtsam::Cal3Unified>& camera);
  void insert(gtsam::Key j, const gtsam::PinholePose<gtsam::Cal3Bundler>& camera);
  void insert(gtsam::Key j, const gtsam::PinholePose<gtsam::Cal3f>& camera);
  void insert(gtsam::Key j, const gtsam::PinholePose<gtsam::Cal3_S2>& camera);
  void insert(gtsam::Key j, const gtsam::PinholePose<gtsam::Cal3DS2>& camera);
  void insert(gtsam::Key j, const gtsam::PinholePose<gtsam::Cal3Fisheye>& camera);
  void insert(gtsam::Key j, const gtsam::PinholePose<gtsam::Cal3Unified>& camera);
  void insert(gtsam::Key j, const gtsam::imuBias::ConstantBias& constant_bias);
  void insert(gtsam::Key j, const gtsam::NavState& nav_state);
  void insert(gtsam::Key j, double c);

  // The order is important: gtsam::Vector has to precede Point2/Point3 so `atVector`
  // can work for those fixed-size vectors.
  void update(gtsam::Key j, gtsam::Vector vector);
  void update(gtsam::Key j, gtsam::Matrix matrix);
  void update(gtsam::Key j, const gtsam::Point2& point2);
  void update(gtsam::Key j, const gtsam::Point3& point3);
  void update(gtsam::Key j, const gtsam::Gal3& T);
  void update(gtsam::Key j, const gtsam::Pose2& pose2);
  void update(gtsam::Key j, const gtsam::Pose3& pose3);
  void update(gtsam::Key j, const gtsam::Rot2& rot2);
  void update(gtsam::Key j, const gtsam::Rot3& rot3);
  void update(gtsam::Key j, const gtsam::Similarity2& similarity2);
  void update(gtsam::Key j, const gtsam::Similarity3& similarity3);
  void update(gtsam::Key j, const gtsam::SL4& H);
  void update(gtsam::Key j, const gtsam::SO3& R);
  void update(gtsam::Key j, const gtsam::SO4& Q);
  void update(gtsam::Key j, const gtsam::SOn& P);
  void update(gtsam::Key j, const gtsam::Unit3& unit3);
  void update(gtsam::Key j, const gtsam::Cal3Bundler& cal3bundler);
  void update(gtsam::Key j, const gtsam::Cal3f& cal3f);
  void update(gtsam::Key j, const gtsam::Cal3_S2& cal3_s2);
  void update(gtsam::Key j, const gtsam::Cal3DS2& cal3ds2);
  void update(gtsam::Key j, const gtsam::Cal3Fisheye& cal3fisheye);
  void update(gtsam::Key j, const gtsam::Cal3Unified& cal3unified);
  void update(gtsam::Key j, const gtsam::EssentialMatrix& E);
  void update(gtsam::Key j, const gtsam::FundamentalMatrix& F);
  void update(gtsam::Key j, const gtsam::SimpleFundamentalMatrix& F);
  void update(gtsam::Key j, const gtsam::OrientedPlane3& plane);
  void update(gtsam::Key j, const gtsam::PinholeCamera<gtsam::Cal3Bundler>& camera);
  void update(gtsam::Key j, const gtsam::PinholeCamera<gtsam::Cal3f>& camera);
  void update(gtsam::Key j, const gtsam::PinholeCamera<gtsam::Cal3_S2>& camera);
  void update(gtsam::Key j, const gtsam::PinholeCamera<gtsam::Cal3DS2>& camera);
  void update(gtsam::Key j, const gtsam::PinholeCamera<gtsam::Cal3Fisheye>& camera);
  void update(gtsam::Key j, const gtsam::PinholeCamera<gtsam::Cal3Unified>& camera);
  void update(gtsam::Key j, const gtsam::PinholePose<gtsam::Cal3Bundler>& camera);
  void update(gtsam::Key j, const gtsam::PinholePose<gtsam::Cal3f>& camera);
  void update(gtsam::Key j, const gtsam::PinholePose<gtsam::Cal3_S2>& camera);
  void update(gtsam::Key j, const gtsam::PinholePose<gtsam::Cal3DS2>& camera);
  void update(gtsam::Key j, const gtsam::PinholePose<gtsam::Cal3Fisheye>& camera);
  void update(gtsam::Key j, const gtsam::PinholePose<gtsam::Cal3Unified>& camera);
  void update(gtsam::Key j, const gtsam::imuBias::ConstantBias& constant_bias);
  void update(gtsam::Key j, const gtsam::NavState& nav_state);
  void update(gtsam::Key j, double c);

  // The order is important: gtsam::Vector has to precede Point2/Point3 so `atVector`
  // can work for those fixed-size vectors.
  void insert_or_assign(gtsam::Key j, gtsam::Vector vector);
  void insert_or_assign(gtsam::Key j, gtsam::Matrix matrix);
  void insert_or_assign(gtsam::Key j, const gtsam::Point2& point2);
  void insert_or_assign(gtsam::Key j, const gtsam::Point3& point3);
  void insert_or_assign(gtsam::Key j, const gtsam::Gal3& T);
  void insert_or_assign(gtsam::Key j, const gtsam::Pose2& pose2);
  void insert_or_assign(gtsam::Key j, const gtsam::Pose3& pose3);
  void insert_or_assign(gtsam::Key j, const gtsam::Rot2& rot2);
  void insert_or_assign(gtsam::Key j, const gtsam::Rot3& rot3);
  void insert_or_assign(gtsam::Key j, const gtsam::Similarity2& similarity2);
  void insert_or_assign(gtsam::Key j, const gtsam::Similarity3& similarity3);
  void insert_or_assign(gtsam::Key j, const gtsam::SL4& H);
  void insert_or_assign(gtsam::Key j, const gtsam::SO3& R);
  void insert_or_assign(gtsam::Key j, const gtsam::SO4& Q);
  void insert_or_assign(gtsam::Key j, const gtsam::SOn& P);
  void insert_or_assign(gtsam::Key j, const gtsam::Unit3& unit3);
  void insert_or_assign(gtsam::Key j, const gtsam::Cal3Bundler& cal3bundler);
  void insert_or_assign(gtsam::Key j, const gtsam::Cal3f& cal3f);
  void insert_or_assign(gtsam::Key j, const gtsam::Cal3_S2& cal3_s2);
  void insert_or_assign(gtsam::Key j, const gtsam::Cal3DS2& cal3ds2);
  void insert_or_assign(gtsam::Key j, const gtsam::Cal3Fisheye& cal3fisheye);
  void insert_or_assign(gtsam::Key j, const gtsam::Cal3Unified& cal3unified);
  void insert_or_assign(gtsam::Key j, const gtsam::EssentialMatrix& E);
  void insert_or_assign(gtsam::Key j, const gtsam::FundamentalMatrix& F);
  void insert_or_assign(gtsam::Key j, const gtsam::SimpleFundamentalMatrix& F);
  void insert_or_assign(gtsam::Key j, const gtsam::OrientedPlane3& plane);
  void insert_or_assign(gtsam::Key j, const gtsam::PinholeCamera<gtsam::Cal3Bundler>& camera);
  void insert_or_assign(gtsam::Key j, const gtsam::PinholeCamera<gtsam::Cal3f>& camera);
  void insert_or_assign(gtsam::Key j, const gtsam::PinholeCamera<gtsam::Cal3_S2>& camera);
  void insert_or_assign(gtsam::Key j, const gtsam::PinholeCamera<gtsam::Cal3DS2>& camera);
  void insert_or_assign(gtsam::Key j, const gtsam::PinholeCamera<gtsam::Cal3Fisheye>& camera);
  void insert_or_assign(gtsam::Key j, const gtsam::PinholeCamera<gtsam::Cal3Unified>& camera);
  void insert_or_assign(gtsam::Key j, const gtsam::PinholePose<gtsam::Cal3Bundler>& camera);
  void insert_or_assign(gtsam::Key j, const gtsam::PinholePose<gtsam::Cal3f>& camera);
  void insert_or_assign(gtsam::Key j, const gtsam::PinholePose<gtsam::Cal3_S2>& camera);
  void insert_or_assign(gtsam::Key j, const gtsam::PinholePose<gtsam::Cal3DS2>& camera);
  void insert_or_assign(gtsam::Key j, const gtsam::PinholePose<gtsam::Cal3Fisheye>& camera);
  void insert_or_assign(gtsam::Key j, const gtsam::PinholePose<gtsam::Cal3Unified>& camera);
  void insert_or_assign(gtsam::Key j, const gtsam::imuBias::ConstantBias& constant_bias);
  void insert_or_assign(gtsam::Key j, const gtsam::NavState& nav_state);
  void insert_or_assign(gtsam::Key j, double c);

  template <T = {gtsam::Vector,
                 gtsam::Matrix,
                 gtsam::Point2,
                 gtsam::Point3,
                 gtsam::Gal3,
                 gtsam::Pose2,
                 gtsam::Pose3,
                 gtsam::Rot2,
                 gtsam::Rot3,
                 gtsam::Similarity2,
                 gtsam::Similarity3,
                 gtsam::SL4,
                 gtsam::SO3,
                 gtsam::SO4,
                 gtsam::SOn,
                 gtsam::Unit3,
                 gtsam::Cal3Bundler,
                 gtsam::Cal3f, 
                 gtsam::Cal3_S2,
                 gtsam::Cal3DS2,
                 gtsam::Cal3Fisheye,
                 gtsam::Cal3Unified,
                 gtsam::EssentialMatrix, 
                 gtsam::FundamentalMatrix, 
                 gtsam::SimpleFundamentalMatrix,
                 gtsam::OrientedPlane3,
                 gtsam::PinholeCamera<gtsam::Cal3Bundler>,
                 gtsam::PinholeCamera<gtsam::Cal3f>,
                 gtsam::PinholeCamera<gtsam::Cal3_S2>,
                 gtsam::PinholeCamera<gtsam::Cal3DS2>,
                 gtsam::PinholeCamera<gtsam::Cal3Fisheye>,
                 gtsam::PinholeCamera<gtsam::Cal3Unified>,
                 gtsam::PinholePose<gtsam::Cal3Bundler>,
                 gtsam::PinholePose<gtsam::Cal3f>,
                 gtsam::PinholePose<gtsam::Cal3_S2>,
                 gtsam::PinholePose<gtsam::Cal3DS2>,
                 gtsam::PinholePose<gtsam::Cal3Fisheye>,
                 gtsam::PinholePose<gtsam::Cal3Unified>,
                 gtsam::imuBias::ConstantBias,
                 gtsam::NavState,
                 double}>
  T at(gtsam::Key j);
};

}  // namespace gtsam
