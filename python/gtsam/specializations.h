// Please refer to: https://pybind11.readthedocs.io/en/stable/advanced/cast/stl.html
// These are required to save one copy operation on Python calls
py::bind_vector<std::vector<gtsam::Key> >(m_, "KeyVector");
py::bind_vector<std::vector<gtsam::Point2, Eigen::aligned_allocator<gtsam::Point2> > >(m_, "Point2Vector");
py::bind_vector<std::vector<gtsam::Pose3> >(m_, "Pose3Vector");
py::bind_vector<std::vector<boost::shared_ptr<gtsam::BetweenFactor<gtsam::Pose3> > > >(m_, "BetweenFactorPose3s");