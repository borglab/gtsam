#include <boost/python.hpp>

#include <numpy_eigen/NumpyEigenConverter.hpp>

#include <gtsam/linear/NoiseModel.h>

using namespace boost::python;
using namespace gtsam;
using namespace gtsam::noiseModel;

// Wrap around pure virtual class Base
// See: http://www.boost.org/doc/libs/1_59_0/libs/python/doc/tutorial/doc/html/python/exposing.html#python.class_virtual_functions
struct BaseWrap : Base, wrapper<Base>
{
  void print (const std::string & name="") const {
    this->get_override("print")();
  }
  bool equals (const Base & expected, double tol=1e-9) const {
    return this->get_override("equals")();
  }
  Vector whiten (const Vector & v) const {
    return this->get_override("whiten")();
  }
  Matrix Whiten (const Matrix & v) const {
    return this->get_override("Whiten")();
  }
  Vector unwhiten (const Vector & v) const {
    return this->get_override("unwhiten")();
  }
  double distance (const Vector & v) const {
    return this->get_override("distance")();
  }
  void WhitenSystem (std::vector< Matrix > &A, Vector &b) const {
    this->get_override("WhitenSystem")();
  }
  void WhitenSystem (Matrix &A, Vector &b) const {
    this->get_override("WhitenSystem")();
  }
  void WhitenSystem (Matrix &A1, Matrix &A2, Vector &b) const {
    this->get_override("WhitenSystem")();
  }
  void WhitenSystem (Matrix &A1, Matrix &A2, Matrix &A3, Vector &b) const {
    this->get_override("WhitenSystem")();
  }

};

BOOST_PYTHON_MODULE(libnoiseModel_python)
{

// NOTE: Don't know if it's really necessary to register the matrices convertion here. 
import_array(); 
NumpyEigenConverter<Vector>::register_converter();
NumpyEigenConverter<Vector1>::register_converter();
NumpyEigenConverter<Vector2>::register_converter();
NumpyEigenConverter<Vector3>::register_converter();
NumpyEigenConverter<Vector4>::register_converter();
NumpyEigenConverter<Vector5>::register_converter();
NumpyEigenConverter<Vector6>::register_converter();
NumpyEigenConverter<Vector7>::register_converter();
NumpyEigenConverter<Vector8>::register_converter();
NumpyEigenConverter<Vector9>::register_converter();
NumpyEigenConverter<Vector10>::register_converter();

NumpyEigenConverter<Matrix>::register_converter();
NumpyEigenConverter<Matrix2>::register_converter();
NumpyEigenConverter<Matrix3>::register_converter();
NumpyEigenConverter<Matrix4>::register_converter();
NumpyEigenConverter<Matrix5>::register_converter();
NumpyEigenConverter<Matrix6>::register_converter();
NumpyEigenConverter<Matrix7>::register_converter();
NumpyEigenConverter<Matrix8>::register_converter();
NumpyEigenConverter<Matrix9>::register_converter();

class_<BaseWrap,boost::noncopyable>("Base")
  .def("print", pure_virtual(&Base::print))
;

class_<Gaussian, boost::shared_ptr<Gaussian>, bases<Base> >("Gaussian", no_init)
  .def("SqrtInformation",&Gaussian::SqrtInformation)
  .staticmethod("SqrtInformation")
  .def("Information",&Gaussian::Information)
  .staticmethod("Information")
  .def("Covariance",&Gaussian::Covariance)
  .staticmethod("Covariance")
;

class_<Diagonal, boost::shared_ptr<Diagonal> >("Diagonal", no_init)
  .def("Sigmas",&Diagonal::Sigmas)
  .staticmethod("Sigmas")
  .def("Variances",&Diagonal::Variances)
  .staticmethod("Variances")
  .def("Precisions",&Diagonal::Precisions)
  .staticmethod("Precisions")
;

class_<Isotropic, boost::shared_ptr<Isotropic> >("Isotropic", no_init)
  .def("Sigma",&Isotropic::Sigma)
  .staticmethod("Sigma")
  .def("Variance",&Isotropic::Variance)
  .staticmethod("Variance")
  .def("Precision",&Isotropic::Precision)
  .staticmethod("Precision")
;

class_<Unit, boost::shared_ptr<Unit> >("Unit", no_init)
  .def("Create",&Unit::Create)
  .staticmethod("Create")
;

}