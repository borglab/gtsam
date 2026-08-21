/* Keep these vectors as bound C++ containers instead of allowing pybind11's
 * automatic STL conversion to copy them to and from Python lists. */
PYBIND11_MAKE_OPAQUE(gtsam::StereoPoint2Vector);
PYBIND11_MAKE_OPAQUE(gtsam::Cal3_S2StereoVector);
