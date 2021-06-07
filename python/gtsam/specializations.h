/* Please refer to: https://pybind11.readthedocs.io/en/stable/advanced/cast/stl.html
 * These are required to save one copy operation on Python calls.
 *
 * NOTES
 * =================
 *
 * `PYBIND11_MAKE_OPAQUE` will mark the type as "opaque" for the pybind11 automatic STL binding,
 * such that the raw objects can be accessed in Python. Without this they will be automatically
 * converted to a Python object, and all mutations on Python side will not be reflected on C++.
 *
 * `py::bind_vector` and similar machinery gives the std container a Python-like interface, but
 * without the `<pybind11/stl.h>` copying mechanism. Combined with `PYBIND11_MAKE_OPAQUE` this
 * allows the types to be modified with Python, and saves one copy operation.
 */
#ifdef GTSAM_ALLOCATOR_TBB
py::bind_vector<std::vector<gtsam::Key, tbb::tbb_allocator<gtsam::Key> > >(m_, "KeyVector");
py::implicitly_convertible<py::list, std::vector<gtsam::Key, tbb::tbb_allocator<gtsam::Key> > >();
#else
py::bind_vector<std::vector<gtsam::Key> >(m_, "KeyVector");
py::implicitly_convertible<py::list, std::vector<gtsam::Key> >();
#endif

py::bind_vector<std::vector<gtsam::Point2, Eigen::aligned_allocator<gtsam::Point2> > >(m_, "Point2Vector");
py::bind_vector<std::vector<gtsam::Point3Pair> >(m_, "Point3Pairs");
py::bind_vector<std::vector<gtsam::Pose3Pair> >(m_, "Pose3Pairs");
py::bind_vector<std::vector<gtsam::Pose3> >(m_, "Pose3Vector");
py::bind_vector<std::vector<boost::shared_ptr<gtsam::BetweenFactor<gtsam::Pose3> > > >(m_, "BetweenFactorPose3s");
py::bind_vector<std::vector<boost::shared_ptr<gtsam::BetweenFactor<gtsam::Pose2> > > >(m_, "BetweenFactorPose2s");
py::bind_vector<std::vector<gtsam::BinaryMeasurement<gtsam::Unit3> > >(m_, "BinaryMeasurementsUnit3");
py::bind_map<gtsam::IndexPairSetMap>(m_, "IndexPairSetMap");
py::bind_vector<gtsam::IndexPairVector>(m_, "IndexPairVector");
py::bind_map<gtsam::KeyPairDoubleMap>(m_, "KeyPairDoubleMap");
py::bind_vector<gtsam::CameraSet<gtsam::PinholeCamera<gtsam::Cal3_S2> > >(m_, "CameraSetCal3_S2");
py::bind_vector<gtsam::CameraSet<gtsam::PinholeCamera<gtsam::Cal3Bundler> > >(m_, "CameraSetCal3Bundler");
py::bind_vector<std::vector<gtsam::Matrix> >(m_, "JacobianVector");
