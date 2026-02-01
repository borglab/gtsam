#include <pybind11/eigen.h>
#include <pybind11/stl_bind.h>
#include <pybind11/pybind11.h>
#include <pybind11/operators.h>
#include "gtsam/nonlinear/utilities.h"  // for RedirectCout.

#include "gtsam/geometry/Point2.h"
#include "gtsam/geometry/Point3.h"
#include <boost/serialization/export.hpp>

BOOST_CLASS_EXPORT(gtsam::Point2)
BOOST_CLASS_EXPORT(gtsam::Point3)


using namespace std;

namespace py = pybind11;

PYBIND11_MODULE(geometry_py, m_) {
    m_.doc() = "pybind11 wrapper of geometry_py";

    pybind11::module m_gtsam = m_.def_submodule("gtsam", "gtsam submodule");

    py::class_<gtsam::Point2, std::shared_ptr<gtsam::Point2>>(m_gtsam, "Point2")
        .def(py::init<>())
        .def(py::init<double, double>(), py::arg("x"), py::arg("y"))
        .def("x",[](gtsam::Point2* self){return self->x();})
        .def("y",[](gtsam::Point2* self){return self->y();})
        .def("dim",[](gtsam::Point2* self){return self->dim();})
        .def("returnChar",[](gtsam::Point2* self){return self->returnChar();})
        .def("argChar",[](gtsam::Point2* self, char a){ self->argChar(a);}, py::arg("a"))
        .def("argChar",[](gtsam::Point2* self, std::shared_ptr<char> a){ self->argChar(a);}, py::arg("a"))
        .def("argChar",[](gtsam::Point2* self, char& a){ self->argChar(a);}, py::arg("a"))
        .def("argChar",[](gtsam::Point2* self, char* a){ self->argChar(a);}, py::arg("a"))
        .def("argChar",[](gtsam::Point2* self, const std::shared_ptr<char> a){ self->argChar(a);}, py::arg("a"))
        .def("argChar",[](gtsam::Point2* self, const char& a){ self->argChar(a);}, py::arg("a"))
        .def("argChar",[](gtsam::Point2* self, const char* a){ self->argChar(a);}, py::arg("a"))
        .def("argUChar",[](gtsam::Point2* self, unsigned char a){ self->argUChar(a);}, py::arg("a"))
        .def("eigenArguments",[](gtsam::Point2* self, const gtsam::Vector& v, const gtsam::Matrix& m){ self->eigenArguments(v, m);}, py::arg("v"), py::arg("m"))
        .def("vectorConfusion",[](gtsam::Point2* self){return self->vectorConfusion();})
        .def("serialize", [](gtsam::Point2* self){ return gtsam::serialize(*self); })
        .def("deserialize", [](gtsam::Point2* self, string serialized){ gtsam::deserialize(serialized, *self); }, py::arg("serialized"))
        .def(py::pickle(
            [](const gtsam::Point2 &a){ /* __getstate__: Returns a string that encodes the state of the object */ return py::make_tuple(gtsam::serialize(a)); },
            [](py::tuple t){ /* __setstate__ */ gtsam::Point2 obj; gtsam::deserialize(t[0].cast<std::string>(), obj); return obj; }));

    py::class_<gtsam::Point3, std::shared_ptr<gtsam::Point3>>(m_gtsam, "Point3")
        .def(py::init<double, double, double>(), py::arg("x"), py::arg("y"), py::arg("z"))
        .def("norm",[](gtsam::Point3* self){return self->norm();})
        .def("serialize", [](gtsam::Point3* self){ return gtsam::serialize(*self); })
        .def("deserialize", [](gtsam::Point3* self, string serialized){ gtsam::deserialize(serialized, *self); }, py::arg("serialized"))
        .def(py::pickle(
            [](const gtsam::Point3 &a){ /* __getstate__: Returns a string that encodes the state of the object */ return py::make_tuple(gtsam::serialize(a)); },
            [](py::tuple t){ /* __setstate__ */ gtsam::Point3 obj; gtsam::deserialize(t[0].cast<std::string>(), obj); return obj; }))
        .def_static("staticFunction",[](){return gtsam::Point3::staticFunction();})
        .def_static("StaticFunctionRet",[](double z){return gtsam::Point3::StaticFunctionRet(z);}, py::arg("z"));


#include "python/specializations.h"

}

