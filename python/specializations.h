//PYBIND11_MAKE_OPAQUE(std::vector<gtsam::Key>);
//PYBIND11_MAKE_OPAQUE(std::vector<gtsam::Point2>);
//PYBIND11_MAKE_OPAQUE(std::vector<gtsam::Pose3>);
//PYBIND11_MAKE_OPAQUE(std::vector<gtsam::BetweenFactor<gtsam::Pose3>>);

// gtsam::BetweenFactorPose3s
// gtsam::Point2Vector
// gtsam::Pose3Vector
// gtsam::KeyVector

py::bind_vector<std::vector<gtsam::Key> >(m_, "KeyVector");
py::bind_vector<std::vector<gtsam::Point2> >(m_, "Point2Vector");
py::bind_vector<std::vector<gtsam::Pose3> >(m_, "Pose3Vector");
py::bind_vector<std::vector<boost::shared_ptr<gtsam::BetweenFactor<gtsam::Pose3> > > >(m_, "BetweenFactorPose3s");