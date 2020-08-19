// Please refer to: https://pybind11.readthedocs.io/en/stable/advanced/cast/stl.html
// These are required to save one copy operation on Python calls
PYBIND11_MAKE_OPAQUE(std::vector<gtsam::Key>);
PYBIND11_MAKE_OPAQUE(std::vector<gtsam::Point2, Eigen::aligned_allocator<gtsam::Point2> >);
PYBIND11_MAKE_OPAQUE(std::vector<gtsam::Pose3>);
PYBIND11_MAKE_OPAQUE(std::vector<gtsam::BetweenFactor<gtsam::Pose3>>);
