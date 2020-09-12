// Please refer to: https://pybind11.readthedocs.io/en/stable/advanced/cast/stl.html
// These are required to save one copy operation on Python calls
#ifdef GTSAM_ALLOCATOR_TBB
PYBIND11_MAKE_OPAQUE(std::vector<gtsam::Key, tbb::tbb_allocator<gtsam::Key>>);
#else
PYBIND11_MAKE_OPAQUE(std::vector<gtsam::Key>);
#endif
PYBIND11_MAKE_OPAQUE(std::vector<gtsam::Point2, Eigen::aligned_allocator<gtsam::Point2> >);
PYBIND11_MAKE_OPAQUE(std::vector<gtsam::Pose3>);
PYBIND11_MAKE_OPAQUE(std::vector<boost::shared_ptr<gtsam::BetweenFactor<gtsam::Pose3> > >);
PYBIND11_MAKE_OPAQUE(std::vector<boost::shared_ptr<gtsam::BetweenFactor<gtsam::Pose2> > >);
PYBIND11_MAKE_OPAQUE(std::vector<gtsam::IndexPair>);
