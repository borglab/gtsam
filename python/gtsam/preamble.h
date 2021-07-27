/* Please refer to: https://pybind11.readthedocs.io/en/stable/advanced/cast/stl.html
 * These are required to save one copy operation on Python calls.
 *
 * NOTES
 * =================
 *
 * `PYBIND11_MAKE_OPAQUE` will mark the type as "opaque" for the pybind11 automatic STL binding,
 * such that the raw objects can be accessed in Python. Without this they will be automatically
 * converted to a Python object, and all mutations on Python side will not be reflected on C++.
 */
#ifdef GTSAM_ALLOCATOR_TBB
PYBIND11_MAKE_OPAQUE(std::vector<gtsam::Key, tbb::tbb_allocator<gtsam::Key>>);
#else
PYBIND11_MAKE_OPAQUE(std::vector<gtsam::Key>);
#endif
PYBIND11_MAKE_OPAQUE(std::vector<gtsam::Point2, Eigen::aligned_allocator<gtsam::Point2> >);
PYBIND11_MAKE_OPAQUE(gtsam::Point3Pairs);
PYBIND11_MAKE_OPAQUE(gtsam::Pose3Pairs);
PYBIND11_MAKE_OPAQUE(std::vector<gtsam::Pose3>);
PYBIND11_MAKE_OPAQUE(std::vector<boost::shared_ptr<gtsam::BetweenFactor<gtsam::Pose3> > >);
PYBIND11_MAKE_OPAQUE(std::vector<boost::shared_ptr<gtsam::BetweenFactor<gtsam::Pose2> > >);
PYBIND11_MAKE_OPAQUE(std::vector<gtsam::IndexPair>);
PYBIND11_MAKE_OPAQUE(gtsam::CameraSet<gtsam::PinholeCamera<gtsam::Cal3Bundler> >);
PYBIND11_MAKE_OPAQUE(gtsam::CameraSet<gtsam::PinholeCamera<gtsam::Cal3_S2> >);
PYBIND11_MAKE_OPAQUE(std::vector<gtsam::Matrix>); // JacobianVector

// TODO(fan): This is to fix the Argument-dependent lookup (ADL) of std::pair. We should find a way to NOT do this.
namespace std {
  using gtsam::operator<<;
}
