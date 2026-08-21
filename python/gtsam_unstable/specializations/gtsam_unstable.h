py::bind_vector<gtsam::StereoPoint2Vector>(m_, "StereoPoint2Vector");
py::bind_vector<gtsam::Cal3_S2StereoVector>(m_, "Cal3_S2StereoVector");

// Preserve the iterable inputs accepted by pybind11's automatic STL caster.
py::implicitly_convertible<py::iterable, gtsam::StereoPoint2Vector>();
py::implicitly_convertible<py::iterable, gtsam::Cal3_S2StereoVector>();
