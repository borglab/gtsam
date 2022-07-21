
py::bind_vector<std::vector<gtsam::GaussianFactor::shared_ptr> >(m_, "GaussianFactorVector");

py::implicitly_convertible<py::list, std::vector<gtsam::GaussianFactor::shared_ptr> >();
