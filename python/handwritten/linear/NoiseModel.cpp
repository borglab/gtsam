/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @brief wraps the noise model classes into the noiseModel module
 * @author Ellon Paiva Mendes (LAAS-CNRS)
 **/

 /** TODOs Summary:
  *
  * TODO(Ellon): Don't know yet it it's worth/needed to add 'Wrap structs' for each of the noise models.
  *              I think it's only worthy if we want to access virtual the virtual functions from python.
  * TODO(Ellon): Wrap non-pure virtual methods of Base on BaseWrap
  */

#include <boost/python.hpp>

#define NO_IMPORT_ARRAY
#include <numpy_eigen/NumpyEigenConverter.hpp>

#include "gtsam/linear/NoiseModel.h"

#include "python/handwritten/common.h"

using namespace boost::python;
using namespace gtsam;
using namespace gtsam::noiseModel;

// Wrap around pure virtual class Base.
// All pure virtual methods should be wrapped. Non-pure may be wrapped if we want to mimic the 
// overloading through inheritance in Python.
// See: http://www.boost.org/doc/libs/1_59_0/libs/python/doc/tutorial/doc/html/python/exposing.html#python.class_virtual_functions
struct BaseCallback : Base, wrapper<Base>
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

  // TODO(Ellon): Wrap non-pure virtual methods should go here.
  //              See: http://www.boost.org/doc/libs/1_59_0/libs/python/doc/tutorial/doc/html/python/exposing.html#python.virtual_functions_with_default_implementations

};

// Overloads for named constructors. Named constructors are static, so we declare them 
// using BOOST_PYTHON_FUNCTION_OVERLOADS instead of BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS
// See: http://www.boost.org/doc/libs/1_59_0/libs/python/doc/tutorial/doc/html/python/functions.html#python.default_arguments
BOOST_PYTHON_FUNCTION_OVERLOADS(Gaussian_SqrtInformation_overloads, Gaussian::SqrtInformation, 1, 2)
BOOST_PYTHON_FUNCTION_OVERLOADS(Gaussian_Information_overloads, Gaussian::Information, 1, 2)
BOOST_PYTHON_FUNCTION_OVERLOADS(Gaussian_Covariance_overloads, Gaussian::Covariance, 1, 2)
BOOST_PYTHON_FUNCTION_OVERLOADS(Diagonal_Sigmas_overloads, Diagonal::Sigmas, 1, 2)
BOOST_PYTHON_FUNCTION_OVERLOADS(Diagonal_Variances_overloads, Diagonal::Variances, 1, 2)
BOOST_PYTHON_FUNCTION_OVERLOADS(Diagonal_Precisions_overloads, Diagonal::Precisions, 1, 2)
BOOST_PYTHON_FUNCTION_OVERLOADS(Isotropic_Sigma_overloads, Isotropic::Sigma, 2, 3)
BOOST_PYTHON_FUNCTION_OVERLOADS(Isotropic_Variance_overloads, Isotropic::Variance, 2, 3)
BOOST_PYTHON_FUNCTION_OVERLOADS(Isotropic_Precision_overloads, Isotropic::Precision, 2, 3)


void exportNoiseModels(){

  // Create a scope "noiseModel". See: http://isolation-nation.blogspot.fr/2008/09/packages-in-python-extension-modules.html
  std::string noiseModel_name = extract<std::string>(scope().attr("__name__") + ".noiseModel");
  object noiseModel_module(handle<>(borrowed(PyImport_AddModule(noiseModel_name.c_str()))));
  scope().attr("noiseModel") = noiseModel_module;
  scope noiseModel_scope = noiseModel_module;
  
  // Then export our classes in the noiseModel scope
  class_<BaseCallback,boost::noncopyable>("Base")
    .def("print", pure_virtual(&Base::print))
  ;
  register_ptr_to_python< boost::shared_ptr<Base> >();
  
  // NOTE: We should use "Base" in "bases<...>", and not "BaseCallback" (it was not clear at the begining)
  class_<Gaussian, boost::shared_ptr<Gaussian>, bases<Base> >("Gaussian", no_init)
    .def("SqrtInformation",&Gaussian::SqrtInformation, Gaussian_SqrtInformation_overloads())
    .staticmethod("SqrtInformation")
    .def("Information",&Gaussian::Information, Gaussian_Information_overloads())
    .staticmethod("Information")
    .def("Covariance",&Gaussian::Covariance, Gaussian_Covariance_overloads())
    .staticmethod("Covariance")
  ;
  REGISTER_SHARED_PTR_TO_PYTHON(Gaussian);
  
  class_<Diagonal, boost::shared_ptr<Diagonal>, bases<Gaussian> >("Diagonal", no_init)
    .def("Sigmas",&Diagonal::Sigmas, Diagonal_Sigmas_overloads())
    .staticmethod("Sigmas")
    .def("Variances",&Diagonal::Variances, Diagonal_Variances_overloads())
    .staticmethod("Variances")
    .def("Precisions",&Diagonal::Precisions, Diagonal_Precisions_overloads())
    .staticmethod("Precisions")
  ;
  REGISTER_SHARED_PTR_TO_PYTHON(Diagonal);
  
  class_<Isotropic, boost::shared_ptr<Isotropic>, bases<Diagonal> >("Isotropic", no_init)
    .def("Sigma",&Isotropic::Sigma, Isotropic_Sigma_overloads())
    .staticmethod("Sigma")
    .def("Variance",&Isotropic::Variance, Isotropic_Variance_overloads())
    .staticmethod("Variance")
    .def("Precision",&Isotropic::Precision, Isotropic_Precision_overloads())
    .staticmethod("Precision")
  ;
  REGISTER_SHARED_PTR_TO_PYTHON(Isotropic);
  
  class_<Unit, boost::shared_ptr<Unit>, bases<Isotropic> >("Unit", no_init)
    .def("Create",&Unit::Create)
    .staticmethod("Create")
  ;

  REGISTER_SHARED_PTR_TO_PYTHON(Unit);
}
