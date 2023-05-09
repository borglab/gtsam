/* Please refer to:
 * https://pybind11.readthedocs.io/en/stable/advanced/cast/stl.html
 * These are required to save one copy operation on Python calls.
 *
 * NOTES
 * =================
 *
 * `py::bind_vector` and similar machinery gives the std container a Python-like
 * interface, but without the `<pybind11/stl.h>` copying mechanism. Combined
 * with `PYBIND11_MAKE_OPAQUE` this allows the types to be modified with Python,
 * and saves one copy operation.
 */

py::bind_vector<std::vector<gtsam::BinaryMeasurement<gtsam::Point3> > >(
    m_, "BinaryMeasurementsPoint3");
py::bind_vector<std::vector<gtsam::BinaryMeasurement<gtsam::Unit3> > >(
    m_, "BinaryMeasurementsUnit3");
py::bind_vector<std::vector<gtsam::BinaryMeasurement<gtsam::Rot3> > >(
    m_, "BinaryMeasurementsRot3");
py::bind_map<gtsam::KeyPairDoubleMap>(m_, "KeyPairDoubleMap");
py::bind_vector<std::vector<gtsam::SfmTrack2d>>(m_, "SfmTrack2dVector");
py::bind_vector<std::vector<gtsam::SfmTrack>>(m_, "SfmTracks");
py::bind_vector<std::vector<gtsam::SfmCamera>>(m_, "SfmCameras");
py::bind_vector<std::vector<std::pair<size_t, gtsam::Point2>>>(
    m_, "SfmMeasurementVector");

py::bind_map<gtsam::gtsfm::MatchIndicesMap>(m_, "MatchIndicesMap");
py::bind_vector<std::vector<gtsam::gtsfm::Keypoints>>(m_, "KeypointsVector");
