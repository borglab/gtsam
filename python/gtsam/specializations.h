// Please refer to: https://pybind11.readthedocs.io/en/stable/advanced/cast/stl.html
// These are required to save one copy operation on Python calls
#ifdef GTSAM_ALLOCATOR_TBB
py::bind_vector<std::vector<gtsam::Key, tbb::tbb_allocator<gtsam::Key> > >(m_, "KeyVector");
#else
py::bind_vector<std::vector<gtsam::Key> >(m_, "KeyVector");
#endif
py::bind_vector<std::vector<gtsam::Point2, Eigen::aligned_allocator<gtsam::Point2> > >(m_, "Point2Vector");
py::bind_vector<std::vector<gtsam::Pose3> >(m_, "Pose3Vector");
py::bind_vector<std::vector<boost::shared_ptr<gtsam::BetweenFactor<gtsam::Pose3> > > >(m_, "BetweenFactorPose3s");
py::bind_vector<std::vector<boost::shared_ptr<gtsam::BetweenFactor<gtsam::Pose2> > > >(m_, "BetweenFactorPose2s");
py::bind_vector<std::vector<gtsam::BinaryMeasurement<gtsam::Unit3> > >(m_, "BinaryMeasurementsUnit3");
py::bind_map<gtsam::IndexPairSetMap>(m_, "IndexPairSetMap");
py::bind_vector<gtsam::IndexPairVector>(m_, "IndexPairVector");
