/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file noiseModel_python.cpp
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

  // TODO(Ellon) Wrap non-pure virtual methods here. See: http://www.boost.org/doc/libs/1_59_0/libs/python/doc/tutorial/doc/html/python/exposing.html#python.virtual_functions_with_default_implementations

};

BOOST_PYTHON_MODULE(libnoiseModel_python)
{

class_<BaseWrap,boost::noncopyable>("Base")
  .def("print", pure_virtual(&Base::print))
;

class_<Gaussian, boost::shared_ptr<Gaussian>, bases<BaseWrap> >("Gaussian", no_init)
  .def("SqrtInformation",&Gaussian::SqrtInformation)
  .staticmethod("SqrtInformation")
  .def("Information",&Gaussian::Information)
  .staticmethod("Information")
  .def("Covariance",&Gaussian::Covariance)
  .staticmethod("Covariance")
;

class_<Diagonal, boost::shared_ptr<Diagonal>, bases<Gaussian> >("Diagonal", no_init)
  .def("Sigmas",&Diagonal::Sigmas)
  .staticmethod("Sigmas")
  .def("Variances",&Diagonal::Variances)
  .staticmethod("Variances")
  .def("Precisions",&Diagonal::Precisions)
  .staticmethod("Precisions")
;

class_<Isotropic, boost::shared_ptr<Isotropic>, bases<Diagonal> >("Isotropic", no_init)
  .def("Sigma",&Isotropic::Sigma)
  .staticmethod("Sigma")
  .def("Variance",&Isotropic::Variance)
  .staticmethod("Variance")
  .def("Precision",&Isotropic::Precision)
  .staticmethod("Precision")
;

class_<Unit, boost::shared_ptr<Unit>, bases<Isotropic> >("Unit", no_init)
  .def("Create",&Unit::Create)
  .staticmethod("Create")
;

}