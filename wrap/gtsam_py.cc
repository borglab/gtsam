


#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>

#include "gtsam/inference/Key.h"
#include "gtsam/base/Matrix.h"
#include "gtsam/base/Value.h"
#include "gtsam/base/GenericValue.h"
#include "gtsam/base/deprecated/LieScalar.h"
#include "gtsam/base/deprecated/LieVector.h"
#include "gtsam/base/deprecated/LieMatrix.h"
#include "gtsam/geometry/Point2.h"
#include "gtsam/geometry/Point2.h"
#include "gtsam/geometry/StereoPoint2.h"
#include "gtsam/geometry/Point3.h"
#include "gtsam/geometry/Rot2.h"
#include "gtsam/geometry/Rot3.h"
#include "gtsam/geometry/Pose2.h"
#include "gtsam/geometry/Pose3.h"
#include "gtsam/geometry/Pose3.h"
#include "gtsam/geometry/Unit3.h"
#include "gtsam/geometry/EssentialMatrix.h"
#include "gtsam/geometry/Cal3_S2.h"
#include "gtsam/geometry/Cal3DS2_Base.h"
#include "gtsam/geometry/Cal3DS2.h"
#include "gtsam/geometry/Cal3Unified.h"
#include "gtsam/geometry/Cal3_S2Stereo.h"
#include "gtsam/geometry/Cal3Bundler.h"
#include "gtsam/geometry/CalibratedCamera.h"
#include "gtsam/geometry/PinholeCamera.h"
#include "gtsam/geometry/SimpleCamera.h"
#include "gtsam/geometry/StereoCamera.h"
#include "gtsam/geometry/triangulation.h"
#include "gtsam/symbolic/SymbolicFactor.h"
#include "gtsam/symbolic/SymbolicFactorGraph.h"
#include "gtsam/symbolic/SymbolicConditional.h"
#include "gtsam/symbolic/SymbolicBayesNet.h"
#include "gtsam/symbolic/SymbolicBayesTree.h"
#include "gtsam/inference/VariableIndex.h"
#include "gtsam/linear/NoiseModel.h"
#include "gtsam/linear/Sampler.h"
#include "gtsam/linear/VectorValues.h"
#include "gtsam/linear/GaussianFactor.h"
#include "gtsam/linear/JacobianFactor.h"
#include "gtsam/linear/HessianFactor.h"
#include "gtsam/linear/GaussianFactorGraph.h"
#include "gtsam/linear/GaussianConditional.h"
#include "gtsam/linear/GaussianDensity.h"
#include "gtsam/linear/GaussianBayesNet.h"
#include "gtsam/linear/GaussianBayesTree.h"
#include "gtsam/linear/Errors.h"
#include "gtsam/linear/GaussianISAM.h"
#include "gtsam/linear/IterativeSolver.h"
#include "gtsam/linear/ConjugateGradientSolver.h"
#include "gtsam/linear/SubgraphSolver.h"
#include "gtsam/linear/KalmanFilter.h"
#include "gtsam/inference/Symbol.h"
#include "gtsam/inference/LabeledSymbol.h"
#include "gtsam/inference/Ordering.h"
#include "gtsam/nonlinear/NonlinearFactorGraph.h"
#include "gtsam/nonlinear/NonlinearFactor.h"
#include "gtsam/nonlinear/NonlinearFactor.h"
#include "gtsam/nonlinear/Values.h"
#include "gtsam/nonlinear/Marginals.h"
#include "gtsam/nonlinear/LinearContainerFactor.h"
#include "gtsam/nonlinear/NonlinearOptimizerParams.h"
#include "gtsam/nonlinear/GaussNewtonOptimizer.h"
#include "gtsam/nonlinear/LevenbergMarquardtOptimizer.h"
#include "gtsam/nonlinear/DoglegOptimizer.h"
#include "gtsam/nonlinear/NonlinearOptimizer.h"
#include "gtsam/nonlinear/GaussNewtonOptimizer.h"
#include "gtsam/nonlinear/DoglegOptimizer.h"
#include "gtsam/nonlinear/LevenbergMarquardtOptimizer.h"
#include "gtsam/nonlinear/ISAM2.h"
#include "gtsam/nonlinear/NonlinearISAM.h"
#include "gtsam/geometry/SimpleCamera.h"
#include "gtsam/geometry/CalibratedCamera.h"
#include "gtsam/geometry/StereoPoint2.h"
#include "gtsam/slam/PriorFactor.h"
#include "gtsam/slam/BetweenFactor.h"
#include "gtsam/nonlinear/NonlinearEquality.h"
#include "gtsam/sam/RangeFactor.h"
#include "gtsam/sam/RangeFactor.h"
#include "gtsam/sam/BearingFactor.h"
#include "gtsam/geometry/BearingRange.h"
#include "gtsam/sam/BearingRangeFactor.h"
#include "gtsam/slam/ProjectionFactor.h"
#include "gtsam/slam/GeneralSFMFactor.h"
#include "gtsam/slam/SmartProjectionFactor.h"
#include "gtsam/slam/SmartProjectionPoseFactor.h"
#include "gtsam/slam/StereoFactor.h"
#include "gtsam/slam/PoseTranslationPrior.h"
#include "gtsam/slam/PoseRotationPrior.h"
#include "gtsam/slam/EssentialMatrixFactor.h"
#include "gtsam/slam/dataset.h"
#include "gtsam/navigation/ImuBias.h"
#include "gtsam/navigation/NavState.h"
#include "gtsam/navigation/PreintegratedRotation.h"
#include "gtsam/navigation/PreintegrationParams.h"
#include "gtsam/navigation/ImuFactor.h"
#include "gtsam/navigation/CombinedImuFactor.h"
#include "gtsam/navigation/AHRSFactor.h"
#include "gtsam/navigation/AttitudeFactor.h"
#include "gtsam/navigation/Scenario.h"
#include "gtsam/nonlinear/utilities.h"
#include "gtsam/nonlinear/utilities.h"




using namespace std;

namespace py = pybind11;

PYBIND11_PLUGIN(gtsam_py) {
    pybind11::module m_("gtsam_py", "pybind11 wrapper of gtsam_py");

    pybind11::module m_gtsam = m_.def_submodule("gtsam", "gtsam submodule");

    py::class_<gtsam::KeyList, std::shared_ptr<gtsam::KeyList>>(m_gtsam, "KeyList")
        .def(py::init<>())
        .def(py::init<const gtsam::KeyList&>(), py::arg("other"))
        .def("size",[](gtsam::KeyList* self){return self->size();})
        .def("empty",[](gtsam::KeyList* self){return self->empty();})
        .def("clear",[](gtsam::KeyList* self){ self->clear();})
        .def("front",[](gtsam::KeyList* self){return self->front();})
        .def("back",[](gtsam::KeyList* self){return self->back();})
        .def("push_back",[](gtsam::KeyList* self, size_t key){ self->push_back(key);}, py::arg("key"))
        .def("push_front",[](gtsam::KeyList* self, size_t key){ self->push_front(key);}, py::arg("key"))
        .def("pop_back",[](gtsam::KeyList* self){ self->pop_back();})
        .def("pop_front",[](gtsam::KeyList* self){ self->pop_front();})
        .def("sort",[](gtsam::KeyList* self){ self->sort();})
        .def("remove",[](gtsam::KeyList* self, size_t key){ self->remove(key);}, py::arg("key"));

    py::class_<gtsam::KeySet, std::shared_ptr<gtsam::KeySet>>(m_gtsam, "KeySet")
        .def(py::init<>())
        .def(py::init<const gtsam::KeySet&>(), py::arg("set"))
        .def(py::init<const gtsam::KeyVector&>(), py::arg("vector"))
        .def(py::init<const gtsam::KeyList&>(), py::arg("list"))
        .def("print_",[](gtsam::KeySet* self, string s){ self->print(s);}, py::arg("s"))
        .def("equals",[](gtsam::KeySet* self,const gtsam::KeySet& other){return self->equals(other);}, py::arg("other"))
        .def("size",[](gtsam::KeySet* self){return self->size();})
        .def("empty",[](gtsam::KeySet* self){return self->empty();})
        .def("clear",[](gtsam::KeySet* self){ self->clear();})
        .def("insert",[](gtsam::KeySet* self, size_t key){ self->insert(key);}, py::arg("key"))
        .def("merge",[](gtsam::KeySet* self,const gtsam::KeySet& other){ self->merge(other);}, py::arg("other"))
        .def("erase",[](gtsam::KeySet* self, size_t key){return self->erase(key);}, py::arg("key"))
        .def("count",[](gtsam::KeySet* self, size_t key){return self->count(key);}, py::arg("key"));

    py::class_<gtsam::KeyVector, std::shared_ptr<gtsam::KeyVector>>(m_gtsam, "KeyVector")
        .def(py::init<>())
        .def(py::init<const gtsam::KeyVector&>(), py::arg("other"))
        .def("size",[](gtsam::KeyVector* self){return self->size();})
        .def("empty",[](gtsam::KeyVector* self){return self->empty();})
        .def("clear",[](gtsam::KeyVector* self){ self->clear();})
        .def("at",[](gtsam::KeyVector* self, size_t i){return self->at(i);}, py::arg("i"))
        .def("front",[](gtsam::KeyVector* self){return self->front();})
        .def("back",[](gtsam::KeyVector* self){return self->back();})
        .def("push_back",[](gtsam::KeyVector* self, size_t key){ self->push_back(key);}, py::arg("key"));

    py::class_<gtsam::KeyGroupMap, std::shared_ptr<gtsam::KeyGroupMap>>(m_gtsam, "KeyGroupMap")
        .def(py::init<>())
        .def("size",[](gtsam::KeyGroupMap* self){return self->size();})
        .def("empty",[](gtsam::KeyGroupMap* self){return self->empty();})
        .def("clear",[](gtsam::KeyGroupMap* self){ self->clear();})
        .def("at",[](gtsam::KeyGroupMap* self, size_t key){return self->at(key);}, py::arg("key"))
        .def("erase",[](gtsam::KeyGroupMap* self, size_t key){return self->erase(key);}, py::arg("key"))
        .def("insert2",[](gtsam::KeyGroupMap* self, size_t key, int val){return self->insert2(key, val);}, py::arg("key"), py::arg("val"));

    py::class_<gtsam::Value, std::shared_ptr<gtsam::Value>>(m_gtsam, "Value")
        .def("print_",[](gtsam::Value* self, string s){ self->print(s);}, py::arg("s"))
        .def("dim",[](gtsam::Value* self){return self->dim();});

    py::class_<gtsam::GenericValue<gtsam::Vector>, gtsam::Value, std::shared_ptr<gtsam::GenericValue<gtsam::Vector>>>(m_gtsam, "GenericValueVector");

    py::class_<gtsam::GenericValue<gtsam::Point2>, gtsam::Value, std::shared_ptr<gtsam::GenericValue<gtsam::Point2>>>(m_gtsam, "GenericValuePoint2");

    py::class_<gtsam::GenericValue<gtsam::Point3>, gtsam::Value, std::shared_ptr<gtsam::GenericValue<gtsam::Point3>>>(m_gtsam, "GenericValuePoint3");

    py::class_<gtsam::GenericValue<gtsam::Rot2>, gtsam::Value, std::shared_ptr<gtsam::GenericValue<gtsam::Rot2>>>(m_gtsam, "GenericValueRot2");

    py::class_<gtsam::GenericValue<gtsam::Rot3>, gtsam::Value, std::shared_ptr<gtsam::GenericValue<gtsam::Rot3>>>(m_gtsam, "GenericValueRot3");

    py::class_<gtsam::GenericValue<gtsam::Pose2>, gtsam::Value, std::shared_ptr<gtsam::GenericValue<gtsam::Pose2>>>(m_gtsam, "GenericValuePose2");

    py::class_<gtsam::GenericValue<gtsam::Pose3>, gtsam::Value, std::shared_ptr<gtsam::GenericValue<gtsam::Pose3>>>(m_gtsam, "GenericValuePose3");

    py::class_<gtsam::GenericValue<gtsam::StereoPoint2>, gtsam::Value, std::shared_ptr<gtsam::GenericValue<gtsam::StereoPoint2>>>(m_gtsam, "GenericValueStereoPoint2");

    py::class_<gtsam::GenericValue<gtsam::Cal3_S2>, gtsam::Value, std::shared_ptr<gtsam::GenericValue<gtsam::Cal3_S2>>>(m_gtsam, "GenericValueCal3_S2");

    py::class_<gtsam::GenericValue<gtsam::CalibratedCamera>, gtsam::Value, std::shared_ptr<gtsam::GenericValue<gtsam::CalibratedCamera>>>(m_gtsam, "GenericValueCalibratedCamera");

    py::class_<gtsam::GenericValue<gtsam::SimpleCamera>, gtsam::Value, std::shared_ptr<gtsam::GenericValue<gtsam::SimpleCamera>>>(m_gtsam, "GenericValueSimpleCamera");

    py::class_<gtsam::GenericValue<gtsam::imuBias::ConstantBias>, gtsam::Value, std::shared_ptr<gtsam::GenericValue<gtsam::imuBias::ConstantBias>>>(m_gtsam, "GenericValueConstantBias");

    py::class_<gtsam::LieScalar, std::shared_ptr<gtsam::LieScalar>>(m_gtsam, "LieScalar")
        .def(py::init<>())
        .def(py::init< double>(), py::arg("d"))
        .def("value",[](gtsam::LieScalar* self){return self->value();})
        .def("print_",[](gtsam::LieScalar* self, string s){ self->print(s);}, py::arg("s"))
        .def("equals",[](gtsam::LieScalar* self,const gtsam::LieScalar& expected, double tol){return self->equals(expected, tol);}, py::arg("expected"), py::arg("tol"))
        .def("inverse",[](gtsam::LieScalar* self){return self->inverse();})
        .def("compose",[](gtsam::LieScalar* self,const gtsam::LieScalar& p){return self->compose(p);}, py::arg("p"))
        .def("between",[](gtsam::LieScalar* self,const gtsam::LieScalar& l2){return self->between(l2);}, py::arg("l2"))
        .def("dim",[](gtsam::LieScalar* self){return self->dim();})
        .def("retract",[](gtsam::LieScalar* self,const gtsam::Vector& v){return self->retract(v);}, py::arg("v"))
        .def("localCoordinates",[](gtsam::LieScalar* self,const gtsam::LieScalar& t2){return self->localCoordinates(t2);}, py::arg("t2"))
        .def_static("identity",[](){return gtsam::LieScalar::identity();})
        .def_static("Expmap",[](const gtsam::Vector& v){return gtsam::LieScalar::Expmap(v);}, py::arg("v"))
        .def_static("Logmap",[](const gtsam::LieScalar& p){return gtsam::LieScalar::Logmap(p);}, py::arg("p"));

    py::class_<gtsam::LieVector, std::shared_ptr<gtsam::LieVector>>(m_gtsam, "LieVector")
        .def(py::init<>())
        .def(py::init<const gtsam::Vector&>(), py::arg("v"))
        .def("vector",[](gtsam::LieVector* self){return self->vector();})
        .def("print_",[](gtsam::LieVector* self, string s){ self->print(s);}, py::arg("s"))
        .def("equals",[](gtsam::LieVector* self,const gtsam::LieVector& expected, double tol){return self->equals(expected, tol);}, py::arg("expected"), py::arg("tol"))
        .def("inverse",[](gtsam::LieVector* self){return self->inverse();})
        .def("compose",[](gtsam::LieVector* self,const gtsam::LieVector& p){return self->compose(p);}, py::arg("p"))
        .def("between",[](gtsam::LieVector* self,const gtsam::LieVector& l2){return self->between(l2);}, py::arg("l2"))
        .def("dim",[](gtsam::LieVector* self){return self->dim();})
        .def("retract",[](gtsam::LieVector* self,const gtsam::Vector& v){return self->retract(v);}, py::arg("v"))
        .def("localCoordinates",[](gtsam::LieVector* self,const gtsam::LieVector& t2){return self->localCoordinates(t2);}, py::arg("t2"))
        .def_static("identity",[](){return gtsam::LieVector::identity();})
        .def_static("Expmap",[](const gtsam::Vector& v){return gtsam::LieVector::Expmap(v);}, py::arg("v"))
        .def_static("Logmap",[](const gtsam::LieVector& p){return gtsam::LieVector::Logmap(p);}, py::arg("p"));

    py::class_<gtsam::LieMatrix, std::shared_ptr<gtsam::LieMatrix>>(m_gtsam, "LieMatrix")
        .def(py::init<>())
        .def(py::init<const gtsam::Matrix&>(), py::arg("v"))
        .def("matrix",[](gtsam::LieMatrix* self){return self->matrix();})
        .def("print_",[](gtsam::LieMatrix* self, string s){ self->print(s);}, py::arg("s"))
        .def("equals",[](gtsam::LieMatrix* self,const gtsam::LieMatrix& expected, double tol){return self->equals(expected, tol);}, py::arg("expected"), py::arg("tol"))
        .def("inverse",[](gtsam::LieMatrix* self){return self->inverse();})
        .def("compose",[](gtsam::LieMatrix* self,const gtsam::LieMatrix& p){return self->compose(p);}, py::arg("p"))
        .def("between",[](gtsam::LieMatrix* self,const gtsam::LieMatrix& l2){return self->between(l2);}, py::arg("l2"))
        .def("dim",[](gtsam::LieMatrix* self){return self->dim();})
        .def("retract",[](gtsam::LieMatrix* self,const gtsam::Vector& v){return self->retract(v);}, py::arg("v"))
        .def("localCoordinates",[](gtsam::LieMatrix* self,const gtsam::LieMatrix& t2){return self->localCoordinates(t2);}, py::arg("t2"))
        .def_static("identity",[](){return gtsam::LieMatrix::identity();})
        .def_static("Expmap",[](const gtsam::Vector& v){return gtsam::LieMatrix::Expmap(v);}, py::arg("v"))
        .def_static("Logmap",[](const gtsam::LieMatrix& p){return gtsam::LieMatrix::Logmap(p);}, py::arg("p"));

    py::class_<gtsam::Point2, std::shared_ptr<gtsam::Point2>>(m_gtsam, "Point2")
        .def(py::init<>())
        .def(py::init< double,  double>(), py::arg("x"), py::arg("y"))
        .def(py::init<const gtsam::Vector&>(), py::arg("v"))
        .def("print_",[](gtsam::Point2* self, string s){ self->print(s);}, py::arg("s"))
        .def("equals",[](gtsam::Point2* self,const gtsam::Point2& point, double tol){return self->equals(point, tol);}, py::arg("point"), py::arg("tol"))
        .def("x",[](gtsam::Point2* self){return self->x();})
        .def("y",[](gtsam::Point2* self){return self->y();})
        .def("vector",[](gtsam::Point2* self){return self->vector();})
        .def("distance",[](gtsam::Point2* self,const gtsam::Point2& p2){return self->distance(p2);}, py::arg("p2"))
        .def("norm",[](gtsam::Point2* self){return self->norm();})
        .def_static("identity",[](){return gtsam::Point2::identity();});

    py::class_<gtsam::Point2Vector, std::shared_ptr<gtsam::Point2Vector>>(m_gtsam, "Point2Vector")
        .def(py::init<>())
        .def(py::init<const gtsam::Point2Vector&>(), py::arg("v"))
        .def("size",[](gtsam::Point2Vector* self){return self->size();})
        .def("max_size",[](gtsam::Point2Vector* self){return self->max_size();})
        .def("resize",[](gtsam::Point2Vector* self, size_t sz){ self->resize(sz);}, py::arg("sz"))
        .def("capacity",[](gtsam::Point2Vector* self){return self->capacity();})
        .def("empty",[](gtsam::Point2Vector* self){return self->empty();})
        .def("reserve",[](gtsam::Point2Vector* self, size_t n){ self->reserve(n);}, py::arg("n"))
        .def("at",[](gtsam::Point2Vector* self, size_t n){return self->at(n);}, py::arg("n"))
        .def("front",[](gtsam::Point2Vector* self){return self->front();})
        .def("back",[](gtsam::Point2Vector* self){return self->back();})
        .def("assign",[](gtsam::Point2Vector* self, size_t n,const gtsam::Point2& u){ self->assign(n, u);}, py::arg("n"), py::arg("u"))
        .def("push_back",[](gtsam::Point2Vector* self,const gtsam::Point2& x){ self->push_back(x);}, py::arg("x"))
        .def("pop_back",[](gtsam::Point2Vector* self){ self->pop_back();});

    py::class_<gtsam::StereoPoint2, std::shared_ptr<gtsam::StereoPoint2>>(m_gtsam, "StereoPoint2")
        .def(py::init<>())
        .def(py::init< double,  double,  double>(), py::arg("uL"), py::arg("uR"), py::arg("v"))
        .def("print_",[](gtsam::StereoPoint2* self, string s){ self->print(s);}, py::arg("s"))
        .def("equals",[](gtsam::StereoPoint2* self,const gtsam::StereoPoint2& point, double tol){return self->equals(point, tol);}, py::arg("point"), py::arg("tol"))
        .def("inverse",[](gtsam::StereoPoint2* self){return self->inverse();})
        .def("compose",[](gtsam::StereoPoint2* self,const gtsam::StereoPoint2& p2){return self->compose(p2);}, py::arg("p2"))
        .def("between",[](gtsam::StereoPoint2* self,const gtsam::StereoPoint2& p2){return self->between(p2);}, py::arg("p2"))
        .def("retract",[](gtsam::StereoPoint2* self,const gtsam::Vector& v){return self->retract(v);}, py::arg("v"))
        .def("localCoordinates",[](gtsam::StereoPoint2* self,const gtsam::StereoPoint2& p){return self->localCoordinates(p);}, py::arg("p"))
        .def("vector",[](gtsam::StereoPoint2* self){return self->vector();})
        .def("uL",[](gtsam::StereoPoint2* self){return self->uL();})
        .def("uR",[](gtsam::StereoPoint2* self){return self->uR();})
        .def("v",[](gtsam::StereoPoint2* self){return self->v();})
        .def_static("identity",[](){return gtsam::StereoPoint2::identity();})
        .def_static("Expmap",[](const gtsam::Vector& v){return gtsam::StereoPoint2::Expmap(v);}, py::arg("v"))
        .def_static("Logmap",[](const gtsam::StereoPoint2& p){return gtsam::StereoPoint2::Logmap(p);}, py::arg("p"));

    py::class_<gtsam::Point3, std::shared_ptr<gtsam::Point3>>(m_gtsam, "Point3")
        .def(py::init<>())
        .def(py::init< double,  double,  double>(), py::arg("x"), py::arg("y"), py::arg("z"))
        .def(py::init<const gtsam::Vector&>(), py::arg("v"))
        .def("print_",[](gtsam::Point3* self, string s){ self->print(s);}, py::arg("s"))
        .def("equals",[](gtsam::Point3* self,const gtsam::Point3& p, double tol){return self->equals(p, tol);}, py::arg("p"), py::arg("tol"))
        .def("vector",[](gtsam::Point3* self){return self->vector();})
        .def("x",[](gtsam::Point3* self){return self->x();})
        .def("y",[](gtsam::Point3* self){return self->y();})
        .def("z",[](gtsam::Point3* self){return self->z();})
        .def_static("identity",[](){return gtsam::Point3::identity();});

    py::class_<gtsam::Rot2, std::shared_ptr<gtsam::Rot2>>(m_gtsam, "Rot2")
        .def(py::init<>())
        .def(py::init< double>(), py::arg("theta"))
        .def("print_",[](gtsam::Rot2* self, string s){ self->print(s);}, py::arg("s"))
        .def("equals",[](gtsam::Rot2* self,const gtsam::Rot2& rot, double tol){return self->equals(rot, tol);}, py::arg("rot"), py::arg("tol"))
        .def("inverse",[](gtsam::Rot2* self){return self->inverse();})
        .def("compose",[](gtsam::Rot2* self,const gtsam::Rot2& p2){return self->compose(p2);}, py::arg("p2"))
        .def("between",[](gtsam::Rot2* self,const gtsam::Rot2& p2){return self->between(p2);}, py::arg("p2"))
        .def("retract",[](gtsam::Rot2* self,const gtsam::Vector& v){return self->retract(v);}, py::arg("v"))
        .def("localCoordinates",[](gtsam::Rot2* self,const gtsam::Rot2& p){return self->localCoordinates(p);}, py::arg("p"))
        .def("rotate",[](gtsam::Rot2* self,const gtsam::Point2& point){return self->rotate(point);}, py::arg("point"))
        .def("unrotate",[](gtsam::Rot2* self,const gtsam::Point2& point){return self->unrotate(point);}, py::arg("point"))
        .def("theta",[](gtsam::Rot2* self){return self->theta();})
        .def("degrees",[](gtsam::Rot2* self){return self->degrees();})
        .def("c",[](gtsam::Rot2* self){return self->c();})
        .def("s",[](gtsam::Rot2* self){return self->s();})
        .def("matrix",[](gtsam::Rot2* self){return self->matrix();})
        .def_static("fromAngle",[]( double theta){return gtsam::Rot2::fromAngle(theta);}, py::arg("theta"))
        .def_static("fromDegrees",[]( double theta){return gtsam::Rot2::fromDegrees(theta);}, py::arg("theta"))
        .def_static("fromCosSin",[]( double c, double s){return gtsam::Rot2::fromCosSin(c, s);}, py::arg("c"), py::arg("s"))
        .def_static("identity",[](){return gtsam::Rot2::identity();})
        .def_static("Expmap",[](const gtsam::Vector& v){return gtsam::Rot2::Expmap(v);}, py::arg("v"))
        .def_static("Logmap",[](const gtsam::Rot2& p){return gtsam::Rot2::Logmap(p);}, py::arg("p"))
        .def_static("relativeBearing",[](const gtsam::Point2& d){return gtsam::Rot2::relativeBearing(d);}, py::arg("d"))
        .def_static("atan2",[]( double y, double x){return gtsam::Rot2::atan2(y, x);}, py::arg("y"), py::arg("x"));

    py::class_<gtsam::Rot3, std::shared_ptr<gtsam::Rot3>>(m_gtsam, "Rot3")
        .def(py::init<>())
        .def(py::init<const gtsam::Matrix&>(), py::arg("R"))
        .def(py::init<const gtsam::Point3&, const gtsam::Point3&, const gtsam::Point3&>(), py::arg("col1"), py::arg("col2"), py::arg("col3"))
        .def(py::init< double,  double,  double,  double,  double,  double,  double,  double,  double>(), py::arg("R11"), py::arg("R12"), py::arg("R13"), py::arg("R21"), py::arg("R22"), py::arg("R23"), py::arg("R31"), py::arg("R32"), py::arg("R33"))
        .def("print_",[](gtsam::Rot3* self, string s){ self->print(s);}, py::arg("s"))
        .def("equals",[](gtsam::Rot3* self,const gtsam::Rot3& rot, double tol){return self->equals(rot, tol);}, py::arg("rot"), py::arg("tol"))
        .def("inverse",[](gtsam::Rot3* self){return self->inverse();})
        .def("compose",[](gtsam::Rot3* self,const gtsam::Rot3& p2){return self->compose(p2);}, py::arg("p2"))
        .def("between",[](gtsam::Rot3* self,const gtsam::Rot3& p2){return self->between(p2);}, py::arg("p2"))
        .def("retract",[](gtsam::Rot3* self,const gtsam::Vector& v){return self->retract(v);}, py::arg("v"))
        .def("localCoordinates",[](gtsam::Rot3* self,const gtsam::Rot3& p){return self->localCoordinates(p);}, py::arg("p"))
        .def("rotate",[](gtsam::Rot3* self,const gtsam::Point3& p){return self->rotate(p);}, py::arg("p"))
        .def("unrotate",[](gtsam::Rot3* self,const gtsam::Point3& p){return self->unrotate(p);}, py::arg("p"))
        .def("matrix",[](gtsam::Rot3* self){return self->matrix();})
        .def("transpose",[](gtsam::Rot3* self){return self->transpose();})
        .def("column",[](gtsam::Rot3* self, size_t index){return self->column(index);}, py::arg("index"))
        .def("xyz",[](gtsam::Rot3* self){return self->xyz();})
        .def("ypr",[](gtsam::Rot3* self){return self->ypr();})
        .def("rpy",[](gtsam::Rot3* self){return self->rpy();})
        .def("roll",[](gtsam::Rot3* self){return self->roll();})
        .def("pitch",[](gtsam::Rot3* self){return self->pitch();})
        .def("yaw",[](gtsam::Rot3* self){return self->yaw();})
        .def("quaternion",[](gtsam::Rot3* self){return self->quaternion();})
        .def_static("Rx",[]( double t){return gtsam::Rot3::Rx(t);}, py::arg("t"))
        .def_static("Ry",[]( double t){return gtsam::Rot3::Ry(t);}, py::arg("t"))
        .def_static("Rz",[]( double t){return gtsam::Rot3::Rz(t);}, py::arg("t"))
        .def_static("RzRyRx",[]( double x, double y, double z){return gtsam::Rot3::RzRyRx(x, y, z);}, py::arg("x"), py::arg("y"), py::arg("z"))
        .def_static("RzRyRx",[](const gtsam::Vector& xyz){return gtsam::Rot3::RzRyRx(xyz);}, py::arg("xyz"))
        .def_static("Yaw",[]( double t){return gtsam::Rot3::Yaw(t);}, py::arg("t"))
        .def_static("Pitch",[]( double t){return gtsam::Rot3::Pitch(t);}, py::arg("t"))
        .def_static("Roll",[]( double t){return gtsam::Rot3::Roll(t);}, py::arg("t"))
        .def_static("Ypr",[]( double y, double p, double r){return gtsam::Rot3::Ypr(y, p, r);}, py::arg("y"), py::arg("p"), py::arg("r"))
        .def_static("Quaternion",[]( double w, double x, double y, double z){return gtsam::Rot3::Quaternion(w, x, y, z);}, py::arg("w"), py::arg("x"), py::arg("y"), py::arg("z"))
        .def_static("Rodrigues",[](const gtsam::Vector& v){return gtsam::Rot3::Rodrigues(v);}, py::arg("v"))
        .def_static("Rodrigues",[]( double wx, double wy, double wz){return gtsam::Rot3::Rodrigues(wx, wy, wz);}, py::arg("wx"), py::arg("wy"), py::arg("wz"))
        .def_static("identity",[](){return gtsam::Rot3::identity();})
        .def_static("Expmap",[](const gtsam::Vector& v){return gtsam::Rot3::Expmap(v);}, py::arg("v"))
        .def_static("Logmap",[](const gtsam::Rot3& p){return gtsam::Rot3::Logmap(p);}, py::arg("p"));

    py::class_<gtsam::Pose2, std::shared_ptr<gtsam::Pose2>>(m_gtsam, "Pose2")
        .def(py::init<>())
        .def(py::init<const gtsam::Pose2&>(), py::arg("other"))
        .def(py::init< double,  double,  double>(), py::arg("x"), py::arg("y"), py::arg("theta"))
        .def(py::init< double, const gtsam::Point2&>(), py::arg("theta"), py::arg("t"))
        .def(py::init<const gtsam::Rot2&, const gtsam::Point2&>(), py::arg("r"), py::arg("t"))
        .def(py::init<const gtsam::Vector&>(), py::arg("v"))
        .def("print_",[](gtsam::Pose2* self, string s){ self->print(s);}, py::arg("s"))
        .def("equals",[](gtsam::Pose2* self,const gtsam::Pose2& pose, double tol){return self->equals(pose, tol);}, py::arg("pose"), py::arg("tol"))
        .def("inverse",[](gtsam::Pose2* self){return self->inverse();})
        .def("compose",[](gtsam::Pose2* self,const gtsam::Pose2& p2){return self->compose(p2);}, py::arg("p2"))
        .def("between",[](gtsam::Pose2* self,const gtsam::Pose2& p2){return self->between(p2);}, py::arg("p2"))
        .def("retract",[](gtsam::Pose2* self,const gtsam::Vector& v){return self->retract(v);}, py::arg("v"))
        .def("localCoordinates",[](gtsam::Pose2* self,const gtsam::Pose2& p){return self->localCoordinates(p);}, py::arg("p"))
        .def("AdjointMap",[](gtsam::Pose2* self){return self->AdjointMap();})
        .def("Adjoint",[](gtsam::Pose2* self,const gtsam::Vector& xi){return self->Adjoint(xi);}, py::arg("xi"))
        .def("adjoint",[](gtsam::Pose2* self,const gtsam::Vector& xi,const gtsam::Vector& y){return self->adjoint(xi, y);}, py::arg("xi"), py::arg("y"))
        .def("adjointTranspose",[](gtsam::Pose2* self,const gtsam::Vector& xi,const gtsam::Vector& y){return self->adjointTranspose(xi, y);}, py::arg("xi"), py::arg("y"))
        .def("transform_from",[](gtsam::Pose2* self,const gtsam::Point2& p){return self->transform_from(p);}, py::arg("p"))
        .def("transform_to",[](gtsam::Pose2* self,const gtsam::Point2& p){return self->transform_to(p);}, py::arg("p"))
        .def("x",[](gtsam::Pose2* self){return self->x();})
        .def("y",[](gtsam::Pose2* self){return self->y();})
        .def("theta",[](gtsam::Pose2* self){return self->theta();})
        .def("bearing",[](gtsam::Pose2* self,const gtsam::Point2& point){return self->bearing(point);}, py::arg("point"))
        .def("range",[](gtsam::Pose2* self,const gtsam::Point2& point){return self->range(point);}, py::arg("point"))
        .def("translation",[](gtsam::Pose2* self){return self->translation();})
        .def("rotation",[](gtsam::Pose2* self){return self->rotation();})
        .def("matrix",[](gtsam::Pose2* self){return self->matrix();})
        .def_static("identity",[](){return gtsam::Pose2::identity();})
        .def_static("Expmap",[](const gtsam::Vector& v){return gtsam::Pose2::Expmap(v);}, py::arg("v"))
        .def_static("Logmap",[](const gtsam::Pose2& p){return gtsam::Pose2::Logmap(p);}, py::arg("p"))
        .def_static("ExpmapDerivative",[](const gtsam::Vector& v){return gtsam::Pose2::ExpmapDerivative(v);}, py::arg("v"))
        .def_static("LogmapDerivative",[](const gtsam::Pose2& v){return gtsam::Pose2::LogmapDerivative(v);}, py::arg("v"))
        .def_static("adjointMap",[](const gtsam::Vector& v){return gtsam::Pose2::adjointMap(v);}, py::arg("v"))
        .def_static("wedge",[]( double vx, double vy, double w){return gtsam::Pose2::wedge(vx, vy, w);}, py::arg("vx"), py::arg("vy"), py::arg("w"));

    py::class_<gtsam::Pose3, std::shared_ptr<gtsam::Pose3>>(m_gtsam, "Pose3")
        .def(py::init<>())
        .def(py::init<const gtsam::Pose3&>(), py::arg("other"))
        .def(py::init<const gtsam::Rot3&, const gtsam::Point3&>(), py::arg("r"), py::arg("t"))
        .def(py::init<const gtsam::Pose2&>(), py::arg("pose2"))
        .def(py::init<const gtsam::Matrix&>(), py::arg("mat"))
        .def("print_",[](gtsam::Pose3* self, string s){ self->print(s);}, py::arg("s"))
        .def("equals",[](gtsam::Pose3* self,const gtsam::Pose3& pose, double tol){return self->equals(pose, tol);}, py::arg("pose"), py::arg("tol"))
        .def("inverse",[](gtsam::Pose3* self){return self->inverse();})
        .def("compose",[](gtsam::Pose3* self,const gtsam::Pose3& pose){return self->compose(pose);}, py::arg("pose"))
        .def("between",[](gtsam::Pose3* self,const gtsam::Pose3& pose){return self->between(pose);}, py::arg("pose"))
        .def("retract",[](gtsam::Pose3* self,const gtsam::Vector& v){return self->retract(v);}, py::arg("v"))
        .def("localCoordinates",[](gtsam::Pose3* self,const gtsam::Pose3& pose){return self->localCoordinates(pose);}, py::arg("pose"))
        .def("AdjointMap",[](gtsam::Pose3* self){return self->AdjointMap();})
        .def("Adjoint",[](gtsam::Pose3* self,const gtsam::Vector& xi){return self->Adjoint(xi);}, py::arg("xi"))
        .def("transform_from",[](gtsam::Pose3* self,const gtsam::Point3& point){return self->transform_from(point);}, py::arg("point"))
        .def("transform_to",[](gtsam::Pose3* self,const gtsam::Point3& point){return self->transform_to(point);}, py::arg("point"))
        .def("rotation",[](gtsam::Pose3* self){return self->rotation();})
        .def("translation",[](gtsam::Pose3* self){return self->translation();})
        .def("x",[](gtsam::Pose3* self){return self->x();})
        .def("y",[](gtsam::Pose3* self){return self->y();})
        .def("z",[](gtsam::Pose3* self){return self->z();})
        .def("matrix",[](gtsam::Pose3* self){return self->matrix();})
        .def("transform_to",[](gtsam::Pose3* self,const gtsam::Pose3& pose){return self->transform_to(pose);}, py::arg("pose"))
        .def("range",[](gtsam::Pose3* self,const gtsam::Point3& point){return self->range(point);}, py::arg("point"))
        .def("range",[](gtsam::Pose3* self,const gtsam::Pose3& pose){return self->range(pose);}, py::arg("pose"))
        .def_static("identity",[](){return gtsam::Pose3::identity();})
        .def_static("Expmap",[](const gtsam::Vector& v){return gtsam::Pose3::Expmap(v);}, py::arg("v"))
        .def_static("Logmap",[](const gtsam::Pose3& pose){return gtsam::Pose3::Logmap(pose);}, py::arg("pose"))
        .def_static("adjointMap",[](const gtsam::Vector& xi){return gtsam::Pose3::adjointMap(xi);}, py::arg("xi"))
        .def_static("adjoint",[](const gtsam::Vector& xi,const gtsam::Vector& y){return gtsam::Pose3::adjoint(xi, y);}, py::arg("xi"), py::arg("y"))
        .def_static("adjointTranspose",[](const gtsam::Vector& xi,const gtsam::Vector& y){return gtsam::Pose3::adjointTranspose(xi, y);}, py::arg("xi"), py::arg("y"))
        .def_static("ExpmapDerivative",[](const gtsam::Vector& xi){return gtsam::Pose3::ExpmapDerivative(xi);}, py::arg("xi"))
        .def_static("LogmapDerivative",[](const gtsam::Pose3& xi){return gtsam::Pose3::LogmapDerivative(xi);}, py::arg("xi"))
        .def_static("wedge",[]( double wx, double wy, double wz, double vx, double vy, double vz){return gtsam::Pose3::wedge(wx, wy, wz, vx, vy, vz);}, py::arg("wx"), py::arg("wy"), py::arg("wz"), py::arg("vx"), py::arg("vy"), py::arg("vz"));

    py::class_<gtsam::Pose3Vector, std::shared_ptr<gtsam::Pose3Vector>>(m_gtsam, "Pose3Vector")
        .def(py::init<>())
        .def("size",[](gtsam::Pose3Vector* self){return self->size();})
        .def("empty",[](gtsam::Pose3Vector* self){return self->empty();})
        .def("at",[](gtsam::Pose3Vector* self, size_t n){return self->at(n);}, py::arg("n"))
        .def("push_back",[](gtsam::Pose3Vector* self,const gtsam::Pose3& pose){ self->push_back(pose);}, py::arg("pose"));

    py::class_<gtsam::Unit3, std::shared_ptr<gtsam::Unit3>>(m_gtsam, "Unit3")
        .def(py::init<>())
        .def(py::init<const gtsam::Point3&>(), py::arg("pose"))
        .def("print_",[](gtsam::Unit3* self, string s){ self->print(s);}, py::arg("s"))
        .def("equals",[](gtsam::Unit3* self,const gtsam::Unit3& pose, double tol){return self->equals(pose, tol);}, py::arg("pose"), py::arg("tol"))
        .def("basis",[](gtsam::Unit3* self){return self->basis();})
        .def("skew",[](gtsam::Unit3* self){return self->skew();})
        .def("dim",[](gtsam::Unit3* self){return self->dim();})
        .def("retract",[](gtsam::Unit3* self,const gtsam::Vector& v){return self->retract(v);}, py::arg("v"))
        .def("localCoordinates",[](gtsam::Unit3* self,const gtsam::Unit3& s){return self->localCoordinates(s);}, py::arg("s"))
        .def_static("Dim",[](){return gtsam::Unit3::Dim();});

    py::class_<gtsam::EssentialMatrix, std::shared_ptr<gtsam::EssentialMatrix>>(m_gtsam, "EssentialMatrix")
        .def(py::init<const gtsam::Rot3&, const gtsam::Unit3&>(), py::arg("aRb"), py::arg("aTb"))
        .def("print_",[](gtsam::EssentialMatrix* self, string s){ self->print(s);}, py::arg("s"))
        .def("equals",[](gtsam::EssentialMatrix* self,const gtsam::EssentialMatrix& pose, double tol){return self->equals(pose, tol);}, py::arg("pose"), py::arg("tol"))
        .def("dim",[](gtsam::EssentialMatrix* self){return self->dim();})
        .def("retract",[](gtsam::EssentialMatrix* self,const gtsam::Vector& v){return self->retract(v);}, py::arg("v"))
        .def("localCoordinates",[](gtsam::EssentialMatrix* self,const gtsam::EssentialMatrix& s){return self->localCoordinates(s);}, py::arg("s"))
        .def("rotation",[](gtsam::EssentialMatrix* self){return self->rotation();})
        .def("direction",[](gtsam::EssentialMatrix* self){return self->direction();})
        .def("matrix",[](gtsam::EssentialMatrix* self){return self->matrix();})
        .def("error",[](gtsam::EssentialMatrix* self,const gtsam::Vector& vA,const gtsam::Vector& vB){return self->error(vA, vB);}, py::arg("vA"), py::arg("vB"))
        .def_static("Dim",[](){return gtsam::EssentialMatrix::Dim();});

    py::class_<gtsam::Cal3_S2, std::shared_ptr<gtsam::Cal3_S2>>(m_gtsam, "Cal3_S2")
        .def(py::init<>())
        .def(py::init< double,  double,  double,  double,  double>(), py::arg("fx"), py::arg("fy"), py::arg("s"), py::arg("u0"), py::arg("v0"))
        .def(py::init<const gtsam::Vector&>(), py::arg("v"))
        .def(py::init< double,  int,  int>(), py::arg("fov"), py::arg("w"), py::arg("h"))
        .def("print_",[](gtsam::Cal3_S2* self, string s){ self->print(s);}, py::arg("s"))
        .def("equals",[](gtsam::Cal3_S2* self,const gtsam::Cal3_S2& rhs, double tol){return self->equals(rhs, tol);}, py::arg("rhs"), py::arg("tol"))
        .def("dim",[](gtsam::Cal3_S2* self){return self->dim();})
        .def("retract",[](gtsam::Cal3_S2* self,const gtsam::Vector& v){return self->retract(v);}, py::arg("v"))
        .def("localCoordinates",[](gtsam::Cal3_S2* self,const gtsam::Cal3_S2& c){return self->localCoordinates(c);}, py::arg("c"))
        .def("calibrate",[](gtsam::Cal3_S2* self,const gtsam::Point2& p){return self->calibrate(p);}, py::arg("p"))
        .def("uncalibrate",[](gtsam::Cal3_S2* self,const gtsam::Point2& p){return self->uncalibrate(p);}, py::arg("p"))
        .def("fx",[](gtsam::Cal3_S2* self){return self->fx();})
        .def("fy",[](gtsam::Cal3_S2* self){return self->fy();})
        .def("skew",[](gtsam::Cal3_S2* self){return self->skew();})
        .def("px",[](gtsam::Cal3_S2* self){return self->px();})
        .def("py",[](gtsam::Cal3_S2* self){return self->py();})
        .def("principalPoint",[](gtsam::Cal3_S2* self){return self->principalPoint();})
        .def("vector",[](gtsam::Cal3_S2* self){return self->vector();})
        .def("matrix",[](gtsam::Cal3_S2* self){return self->matrix();})
        .def("matrix_inverse",[](gtsam::Cal3_S2* self){return self->matrix_inverse();})
        .def_static("Dim",[](){return gtsam::Cal3_S2::Dim();});

    py::class_<gtsam::Cal3DS2_Base, std::shared_ptr<gtsam::Cal3DS2_Base>>(m_gtsam, "Cal3DS2_Base")
        .def(py::init<>())
        .def("print_",[](gtsam::Cal3DS2_Base* self, string s){ self->print(s);}, py::arg("s"))
        .def("fx",[](gtsam::Cal3DS2_Base* self){return self->fx();})
        .def("fy",[](gtsam::Cal3DS2_Base* self){return self->fy();})
        .def("skew",[](gtsam::Cal3DS2_Base* self){return self->skew();})
        .def("px",[](gtsam::Cal3DS2_Base* self){return self->px();})
        .def("py",[](gtsam::Cal3DS2_Base* self){return self->py();})
        .def("k1",[](gtsam::Cal3DS2_Base* self){return self->k1();})
        .def("k2",[](gtsam::Cal3DS2_Base* self){return self->k2();})
        .def("K",[](gtsam::Cal3DS2_Base* self){return self->K();})
        .def("k",[](gtsam::Cal3DS2_Base* self){return self->k();})
        .def("vector",[](gtsam::Cal3DS2_Base* self){return self->vector();})
        .def("uncalibrate",[](gtsam::Cal3DS2_Base* self,const gtsam::Point2& p){return self->uncalibrate(p);}, py::arg("p"))
        .def("calibrate",[](gtsam::Cal3DS2_Base* self,const gtsam::Point2& p, double tol){return self->calibrate(p, tol);}, py::arg("p"), py::arg("tol"));

    py::class_<gtsam::Cal3DS2, gtsam::Cal3DS2_Base, std::shared_ptr<gtsam::Cal3DS2>>(m_gtsam, "Cal3DS2")
        .def(py::init<>())
        .def(py::init< double,  double,  double,  double,  double,  double,  double>(), py::arg("fx"), py::arg("fy"), py::arg("s"), py::arg("u0"), py::arg("v0"), py::arg("k1"), py::arg("k2"))
        .def(py::init< double,  double,  double,  double,  double,  double,  double,  double,  double>(), py::arg("fx"), py::arg("fy"), py::arg("s"), py::arg("u0"), py::arg("v0"), py::arg("k1"), py::arg("k2"), py::arg("p1"), py::arg("p2"))
        .def(py::init<const gtsam::Vector&>(), py::arg("v"))
        .def("equals",[](gtsam::Cal3DS2* self,const gtsam::Cal3DS2& rhs, double tol){return self->equals(rhs, tol);}, py::arg("rhs"), py::arg("tol"))
        .def("dim",[](gtsam::Cal3DS2* self){return self->dim();})
        .def("retract",[](gtsam::Cal3DS2* self,const gtsam::Vector& v){return self->retract(v);}, py::arg("v"))
        .def("localCoordinates",[](gtsam::Cal3DS2* self,const gtsam::Cal3DS2& c){return self->localCoordinates(c);}, py::arg("c"))
        .def_static("Dim",[](){return gtsam::Cal3DS2::Dim();});

    py::class_<gtsam::Cal3Unified, gtsam::Cal3DS2_Base, std::shared_ptr<gtsam::Cal3Unified>>(m_gtsam, "Cal3Unified")
        .def(py::init<>())
        .def(py::init< double,  double,  double,  double,  double,  double,  double>(), py::arg("fx"), py::arg("fy"), py::arg("s"), py::arg("u0"), py::arg("v0"), py::arg("k1"), py::arg("k2"))
        .def(py::init< double,  double,  double,  double,  double,  double,  double,  double,  double,  double>(), py::arg("fx"), py::arg("fy"), py::arg("s"), py::arg("u0"), py::arg("v0"), py::arg("k1"), py::arg("k2"), py::arg("p1"), py::arg("p2"), py::arg("xi"))
        .def(py::init<const gtsam::Vector&>(), py::arg("v"))
        .def("equals",[](gtsam::Cal3Unified* self,const gtsam::Cal3Unified& rhs, double tol){return self->equals(rhs, tol);}, py::arg("rhs"), py::arg("tol"))
        .def("xi",[](gtsam::Cal3Unified* self){return self->xi();})
        .def("spaceToNPlane",[](gtsam::Cal3Unified* self,const gtsam::Point2& p){return self->spaceToNPlane(p);}, py::arg("p"))
        .def("nPlaneToSpace",[](gtsam::Cal3Unified* self,const gtsam::Point2& p){return self->nPlaneToSpace(p);}, py::arg("p"))
        .def("dim",[](gtsam::Cal3Unified* self){return self->dim();})
        .def("retract",[](gtsam::Cal3Unified* self,const gtsam::Vector& v){return self->retract(v);}, py::arg("v"))
        .def("localCoordinates",[](gtsam::Cal3Unified* self,const gtsam::Cal3Unified& c){return self->localCoordinates(c);}, py::arg("c"))
        .def_static("Dim",[](){return gtsam::Cal3Unified::Dim();});

    py::class_<gtsam::Cal3_S2Stereo, std::shared_ptr<gtsam::Cal3_S2Stereo>>(m_gtsam, "Cal3_S2Stereo")
        .def(py::init<>())
        .def(py::init< double,  double,  double,  double,  double,  double>(), py::arg("fx"), py::arg("fy"), py::arg("s"), py::arg("u0"), py::arg("v0"), py::arg("b"))
        .def(py::init<const gtsam::Vector&>(), py::arg("v"))
        .def("print_",[](gtsam::Cal3_S2Stereo* self, string s){ self->print(s);}, py::arg("s"))
        .def("equals",[](gtsam::Cal3_S2Stereo* self,const gtsam::Cal3_S2Stereo& K, double tol){return self->equals(K, tol);}, py::arg("K"), py::arg("tol"))
        .def("fx",[](gtsam::Cal3_S2Stereo* self){return self->fx();})
        .def("fy",[](gtsam::Cal3_S2Stereo* self){return self->fy();})
        .def("skew",[](gtsam::Cal3_S2Stereo* self){return self->skew();})
        .def("px",[](gtsam::Cal3_S2Stereo* self){return self->px();})
        .def("py",[](gtsam::Cal3_S2Stereo* self){return self->py();})
        .def("principalPoint",[](gtsam::Cal3_S2Stereo* self){return self->principalPoint();})
        .def("baseline",[](gtsam::Cal3_S2Stereo* self){return self->baseline();});

    py::class_<gtsam::Cal3Bundler, std::shared_ptr<gtsam::Cal3Bundler>>(m_gtsam, "Cal3Bundler")
        .def(py::init<>())
        .def(py::init< double,  double,  double,  double,  double>(), py::arg("fx"), py::arg("k1"), py::arg("k2"), py::arg("u0"), py::arg("v0"))
        .def("print_",[](gtsam::Cal3Bundler* self, string s){ self->print(s);}, py::arg("s"))
        .def("equals",[](gtsam::Cal3Bundler* self,const gtsam::Cal3Bundler& rhs, double tol){return self->equals(rhs, tol);}, py::arg("rhs"), py::arg("tol"))
        .def("dim",[](gtsam::Cal3Bundler* self){return self->dim();})
        .def("retract",[](gtsam::Cal3Bundler* self,const gtsam::Vector& v){return self->retract(v);}, py::arg("v"))
        .def("localCoordinates",[](gtsam::Cal3Bundler* self,const gtsam::Cal3Bundler& c){return self->localCoordinates(c);}, py::arg("c"))
        .def("calibrate",[](gtsam::Cal3Bundler* self,const gtsam::Point2& p, double tol){return self->calibrate(p, tol);}, py::arg("p"), py::arg("tol"))
        .def("uncalibrate",[](gtsam::Cal3Bundler* self,const gtsam::Point2& p){return self->uncalibrate(p);}, py::arg("p"))
        .def("fx",[](gtsam::Cal3Bundler* self){return self->fx();})
        .def("fy",[](gtsam::Cal3Bundler* self){return self->fy();})
        .def("k1",[](gtsam::Cal3Bundler* self){return self->k1();})
        .def("k2",[](gtsam::Cal3Bundler* self){return self->k2();})
        .def("u0",[](gtsam::Cal3Bundler* self){return self->u0();})
        .def("v0",[](gtsam::Cal3Bundler* self){return self->v0();})
        .def("vector",[](gtsam::Cal3Bundler* self){return self->vector();})
        .def("k",[](gtsam::Cal3Bundler* self){return self->k();})
        .def_static("Dim",[](){return gtsam::Cal3Bundler::Dim();});

    py::class_<gtsam::CalibratedCamera, std::shared_ptr<gtsam::CalibratedCamera>>(m_gtsam, "CalibratedCamera")
        .def(py::init<>())
        .def(py::init<const gtsam::Pose3&>(), py::arg("pose"))
        .def(py::init<const gtsam::Vector&>(), py::arg("v"))
        .def("print_",[](gtsam::CalibratedCamera* self, string s){ self->print(s);}, py::arg("s"))
        .def("equals",[](gtsam::CalibratedCamera* self,const gtsam::CalibratedCamera& camera, double tol){return self->equals(camera, tol);}, py::arg("camera"), py::arg("tol"))
        .def("dim",[](gtsam::CalibratedCamera* self){return self->dim();})
        .def("retract",[](gtsam::CalibratedCamera* self,const gtsam::Vector& d){return self->retract(d);}, py::arg("d"))
        .def("localCoordinates",[](gtsam::CalibratedCamera* self,const gtsam::CalibratedCamera& T2){return self->localCoordinates(T2);}, py::arg("T2"))
        .def("project",[](gtsam::CalibratedCamera* self,const gtsam::Point3& point){return self->project(point);}, py::arg("point"))
        .def("pose",[](gtsam::CalibratedCamera* self){return self->pose();})
        .def("range",[](gtsam::CalibratedCamera* self,const gtsam::Point3& p){return self->range(p);}, py::arg("p"))
        .def_static("Level",[](const gtsam::Pose2& pose2, double height){return gtsam::CalibratedCamera::Level(pose2, height);}, py::arg("pose2"), py::arg("height"))
        .def_static("Dim",[](){return gtsam::CalibratedCamera::Dim();})
        .def_static("Project",[](const gtsam::Point3& cameraPoint){return gtsam::CalibratedCamera::Project(cameraPoint);}, py::arg("cameraPoint"));

    py::class_<gtsam::SimpleCamera, std::shared_ptr<gtsam::SimpleCamera>>(m_gtsam, "SimpleCamera")
        .def(py::init<>())
        .def(py::init<const gtsam::Pose3&>(), py::arg("pose"))
        .def(py::init<const gtsam::Pose3&, const gtsam::Cal3_S2&>(), py::arg("pose"), py::arg("K"))
        .def("print_",[](gtsam::SimpleCamera* self, string s){ self->print(s);}, py::arg("s"))
        .def("equals",[](gtsam::SimpleCamera* self,const gtsam::SimpleCamera& camera, double tol){return self->equals(camera, tol);}, py::arg("camera"), py::arg("tol"))
        .def("pose",[](gtsam::SimpleCamera* self){return self->pose();})
        .def("calibration",[](gtsam::SimpleCamera* self){return self->calibration();})
        .def("retract",[](gtsam::SimpleCamera* self,const gtsam::Vector& d){return self->retract(d);}, py::arg("d"))
        .def("localCoordinates",[](gtsam::SimpleCamera* self,const gtsam::SimpleCamera& T2){return self->localCoordinates(T2);}, py::arg("T2"))
        .def("dim",[](gtsam::SimpleCamera* self){return self->dim();})
        .def("projectSafe",[](gtsam::SimpleCamera* self,const gtsam::Point3& pw){return self->projectSafe(pw);}, py::arg("pw"))
        .def("project",[](gtsam::SimpleCamera* self,const gtsam::Point3& point){return self->project(point);}, py::arg("point"))
        .def("backproject",[](gtsam::SimpleCamera* self,const gtsam::Point2& p, double depth){return self->backproject(p, depth);}, py::arg("p"), py::arg("depth"))
        .def("range",[](gtsam::SimpleCamera* self,const gtsam::Point3& point){return self->range(point);}, py::arg("point"))
        .def("range",[](gtsam::SimpleCamera* self,const gtsam::Pose3& pose){return self->range(pose);}, py::arg("pose"))
        .def_static("Level",[](const gtsam::Cal3_S2& K,const gtsam::Pose2& pose, double height){return gtsam::SimpleCamera::Level(K, pose, height);}, py::arg("K"), py::arg("pose"), py::arg("height"))
        .def_static("Level",[](const gtsam::Pose2& pose, double height){return gtsam::SimpleCamera::Level(pose, height);}, py::arg("pose"), py::arg("height"))
        .def_static("Lookat",[](const gtsam::Point3& eye,const gtsam::Point3& target,const gtsam::Point3& upVector,const gtsam::Cal3_S2& K){return gtsam::SimpleCamera::Lookat(eye, target, upVector, K);}, py::arg("eye"), py::arg("target"), py::arg("upVector"), py::arg("K"))
        .def_static("Lookat",[](const gtsam::Point3& eye,const gtsam::Point3& target,const gtsam::Point3& upVector){return gtsam::SimpleCamera::Lookat(eye, target, upVector);}, py::arg("eye"), py::arg("target"), py::arg("upVector"))
        .def_static("Dim",[](){return gtsam::SimpleCamera::Dim();})
        .def_static("Project",[](const gtsam::Point3& cameraPoint){return gtsam::SimpleCamera::Project(cameraPoint);}, py::arg("cameraPoint"));

    py::class_<gtsam::PinholeCamera<gtsam::Cal3_S2>, std::shared_ptr<gtsam::PinholeCamera<gtsam::Cal3_S2>>>(m_gtsam, "PinholeCameraCal3_S2")
        .def(py::init<>())
        .def(py::init<const gtsam::Pose3&>(), py::arg("pose"))
        .def(py::init<const gtsam::Pose3&, const gtsam::Cal3_S2&>(), py::arg("pose"), py::arg("K"))
        .def("print_",[](gtsam::PinholeCamera<gtsam::Cal3_S2>* self, string s){ self->print(s);}, py::arg("s"))
        .def("equals",[](gtsam::PinholeCamera<gtsam::Cal3_S2>* self,const gtsam::PinholeCamera<gtsam::Cal3_S2>& camera, double tol){return self->equals(camera, tol);}, py::arg("camera"), py::arg("tol"))
        .def("pose",[](gtsam::PinholeCamera<gtsam::Cal3_S2>* self){return self->pose();})
        .def("calibration",[](gtsam::PinholeCamera<gtsam::Cal3_S2>* self){return self->calibration();})
        .def("retract",[](gtsam::PinholeCamera<gtsam::Cal3_S2>* self,const gtsam::Vector& d){return self->retract(d);}, py::arg("d"))
        .def("localCoordinates",[](gtsam::PinholeCamera<gtsam::Cal3_S2>* self,const gtsam::PinholeCamera<gtsam::Cal3_S2>& T2){return self->localCoordinates(T2);}, py::arg("T2"))
        .def("dim",[](gtsam::PinholeCamera<gtsam::Cal3_S2>* self){return self->dim();})
        .def("projectSafe",[](gtsam::PinholeCamera<gtsam::Cal3_S2>* self,const gtsam::Point3& pw){return self->projectSafe(pw);}, py::arg("pw"))
        .def("project",[](gtsam::PinholeCamera<gtsam::Cal3_S2>* self,const gtsam::Point3& point){return self->project(point);}, py::arg("point"))
        .def("backproject",[](gtsam::PinholeCamera<gtsam::Cal3_S2>* self,const gtsam::Point2& p, double depth){return self->backproject(p, depth);}, py::arg("p"), py::arg("depth"))
        .def("range",[](gtsam::PinholeCamera<gtsam::Cal3_S2>* self,const gtsam::Point3& point){return self->range(point);}, py::arg("point"))
        .def("range",[](gtsam::PinholeCamera<gtsam::Cal3_S2>* self,const gtsam::Pose3& pose){return self->range(pose);}, py::arg("pose"))
        .def_static("Level",[](const gtsam::Cal3_S2& K,const gtsam::Pose2& pose, double height){return gtsam::PinholeCamera<gtsam::Cal3_S2>::Level(K, pose, height);}, py::arg("K"), py::arg("pose"), py::arg("height"))
        .def_static("Level",[](const gtsam::Pose2& pose, double height){return gtsam::PinholeCamera<gtsam::Cal3_S2>::Level(pose, height);}, py::arg("pose"), py::arg("height"))
        .def_static("Lookat",[](const gtsam::Point3& eye,const gtsam::Point3& target,const gtsam::Point3& upVector,const gtsam::Cal3_S2& K){return gtsam::PinholeCamera<gtsam::Cal3_S2>::Lookat(eye, target, upVector, K);}, py::arg("eye"), py::arg("target"), py::arg("upVector"), py::arg("K"))
        .def_static("Dim",[](){return gtsam::PinholeCamera<gtsam::Cal3_S2>::Dim();})
        .def_static("Project",[](const gtsam::Point3& cameraPoint){return gtsam::PinholeCamera<gtsam::Cal3_S2>::Project(cameraPoint);}, py::arg("cameraPoint"));

    py::class_<gtsam::StereoCamera, std::shared_ptr<gtsam::StereoCamera>>(m_gtsam, "StereoCamera")
        .def(py::init<>())
        .def(py::init<const gtsam::Pose3&, const std::shared_ptr<gtsam::Cal3_S2Stereo>&>(), py::arg("pose"), py::arg("K"))
        .def("print_",[](gtsam::StereoCamera* self, string s){ self->print(s);}, py::arg("s"))
        .def("equals",[](gtsam::StereoCamera* self,const gtsam::StereoCamera& camera, double tol){return self->equals(camera, tol);}, py::arg("camera"), py::arg("tol"))
        .def("pose",[](gtsam::StereoCamera* self){return self->pose();})
        .def("baseline",[](gtsam::StereoCamera* self){return self->baseline();})
        .def("calibration",[](gtsam::StereoCamera* self){return self->calibration();})
        .def("retract",[](gtsam::StereoCamera* self,const gtsam::Vector& d){return self->retract(d);}, py::arg("d"))
        .def("localCoordinates",[](gtsam::StereoCamera* self,const gtsam::StereoCamera& T2){return self->localCoordinates(T2);}, py::arg("T2"))
        .def("dim",[](gtsam::StereoCamera* self){return self->dim();})
        .def("project",[](gtsam::StereoCamera* self,const gtsam::Point3& point){return self->project(point);}, py::arg("point"))
        .def("backproject",[](gtsam::StereoCamera* self,const gtsam::StereoPoint2& p){return self->backproject(p);}, py::arg("p"))
        .def_static("Dim",[](){return gtsam::StereoCamera::Dim();});

    py::class_<gtsam::SymbolicFactor, std::shared_ptr<gtsam::SymbolicFactor>>(m_gtsam, "SymbolicFactor")
        .def(py::init<const gtsam::SymbolicFactor&>(), py::arg("f"))
        .def(py::init<>())
        .def(py::init< size_t>(), py::arg("j"))
        .def(py::init< size_t,  size_t>(), py::arg("j1"), py::arg("j2"))
        .def(py::init< size_t,  size_t,  size_t>(), py::arg("j1"), py::arg("j2"), py::arg("j3"))
        .def(py::init< size_t,  size_t,  size_t,  size_t>(), py::arg("j1"), py::arg("j2"), py::arg("j3"), py::arg("j4"))
        .def(py::init< size_t,  size_t,  size_t,  size_t,  size_t>(), py::arg("j1"), py::arg("j2"), py::arg("j3"), py::arg("j4"), py::arg("j5"))
        .def(py::init< size_t,  size_t,  size_t,  size_t,  size_t,  size_t>(), py::arg("j1"), py::arg("j2"), py::arg("j3"), py::arg("j4"), py::arg("j5"), py::arg("j6"))
        .def("size",[](gtsam::SymbolicFactor* self){return self->size();})
        .def("print_",[](gtsam::SymbolicFactor* self, string s){ self->print(s);}, py::arg("s"))
        .def("equals",[](gtsam::SymbolicFactor* self,const gtsam::SymbolicFactor& other, double tol){return self->equals(other, tol);}, py::arg("other"), py::arg("tol"))
        .def("keys",[](gtsam::SymbolicFactor* self){return self->keys();})
        .def_static("FromKeys",[](const gtsam::KeyVector& js){return gtsam::SymbolicFactor::FromKeys(js);}, py::arg("js"));

    py::class_<gtsam::SymbolicFactorGraph, std::shared_ptr<gtsam::SymbolicFactorGraph>>(m_gtsam, "SymbolicFactorGraph")
        .def(py::init<>())
        .def(py::init<const gtsam::SymbolicBayesNet&>(), py::arg("bayesNet"))
        .def(py::init<const gtsam::SymbolicBayesTree&>(), py::arg("bayesTree"))
        .def("push_back",[](gtsam::SymbolicFactorGraph* self,const std::shared_ptr<gtsam::SymbolicFactor>& factor){ self->push_back(factor);}, py::arg("factor"))
        .def("print_",[](gtsam::SymbolicFactorGraph* self, string s){ self->print(s);}, py::arg("s"))
        .def("equals",[](gtsam::SymbolicFactorGraph* self,const gtsam::SymbolicFactorGraph& rhs, double tol){return self->equals(rhs, tol);}, py::arg("rhs"), py::arg("tol"))
        .def("size",[](gtsam::SymbolicFactorGraph* self){return self->size();})
        .def("exists",[](gtsam::SymbolicFactorGraph* self, size_t idx){return self->exists(idx);}, py::arg("idx"))
        .def("keys",[](gtsam::SymbolicFactorGraph* self){return self->keys();})
        .def("push_back",[](gtsam::SymbolicFactorGraph* self,const gtsam::SymbolicFactorGraph& graph){ self->push_back(graph);}, py::arg("graph"))
        .def("push_back",[](gtsam::SymbolicFactorGraph* self,const gtsam::SymbolicBayesNet& bayesNet){ self->push_back(bayesNet);}, py::arg("bayesNet"))
        .def("push_back",[](gtsam::SymbolicFactorGraph* self,const gtsam::SymbolicBayesTree& bayesTree){ self->push_back(bayesTree);}, py::arg("bayesTree"))
        .def("push_factor",[](gtsam::SymbolicFactorGraph* self, size_t key){ self->push_factor(key);}, py::arg("key"))
        .def("push_factor",[](gtsam::SymbolicFactorGraph* self, size_t key1, size_t key2){ self->push_factor(key1, key2);}, py::arg("key1"), py::arg("key2"))
        .def("push_factor",[](gtsam::SymbolicFactorGraph* self, size_t key1, size_t key2, size_t key3){ self->push_factor(key1, key2, key3);}, py::arg("key1"), py::arg("key2"), py::arg("key3"))
        .def("push_factor",[](gtsam::SymbolicFactorGraph* self, size_t key1, size_t key2, size_t key3, size_t key4){ self->push_factor(key1, key2, key3, key4);}, py::arg("key1"), py::arg("key2"), py::arg("key3"), py::arg("key4"))
        .def("eliminateSequential",[](gtsam::SymbolicFactorGraph* self){return self->eliminateSequential();})
        .def("eliminateSequential",[](gtsam::SymbolicFactorGraph* self,const gtsam::Ordering& ordering){return self->eliminateSequential(ordering);}, py::arg("ordering"))
        .def("eliminateMultifrontal",[](gtsam::SymbolicFactorGraph* self){return self->eliminateMultifrontal();})
        .def("eliminateMultifrontal",[](gtsam::SymbolicFactorGraph* self,const gtsam::Ordering& ordering){return self->eliminateMultifrontal(ordering);}, py::arg("ordering"))
        .def("eliminatePartialSequential",[](gtsam::SymbolicFactorGraph* self,const gtsam::Ordering& ordering){return self->eliminatePartialSequential(ordering);}, py::arg("ordering"))
        .def("eliminatePartialSequential",[](gtsam::SymbolicFactorGraph* self,const gtsam::KeyVector& keys){return self->eliminatePartialSequential(keys);}, py::arg("keys"))
        .def("eliminatePartialMultifrontal",[](gtsam::SymbolicFactorGraph* self,const gtsam::Ordering& ordering){return self->eliminatePartialMultifrontal(ordering);}, py::arg("ordering"))
        .def("eliminatePartialMultifrontal",[](gtsam::SymbolicFactorGraph* self,const gtsam::KeyVector& keys){return self->eliminatePartialMultifrontal(keys);}, py::arg("keys"))
        .def("marginalMultifrontalBayesNet",[](gtsam::SymbolicFactorGraph* self,const gtsam::Ordering& ordering){return self->marginalMultifrontalBayesNet(ordering);}, py::arg("ordering"))
        .def("marginalMultifrontalBayesNet",[](gtsam::SymbolicFactorGraph* self,const gtsam::KeyVector& key_vector){return self->marginalMultifrontalBayesNet(key_vector);}, py::arg("key_vector"))
        .def("marginalMultifrontalBayesNet",[](gtsam::SymbolicFactorGraph* self,const gtsam::Ordering& ordering,const gtsam::Ordering& marginalizedVariableOrdering){return self->marginalMultifrontalBayesNet(ordering, marginalizedVariableOrdering);}, py::arg("ordering"), py::arg("marginalizedVariableOrdering"))
        .def("marginalMultifrontalBayesNet",[](gtsam::SymbolicFactorGraph* self,const gtsam::KeyVector& key_vector,const gtsam::Ordering& marginalizedVariableOrdering){return self->marginalMultifrontalBayesNet(key_vector, marginalizedVariableOrdering);}, py::arg("key_vector"), py::arg("marginalizedVariableOrdering"))
        .def("marginal",[](gtsam::SymbolicFactorGraph* self,const gtsam::KeyVector& key_vector){return self->marginal(key_vector);}, py::arg("key_vector"));

    py::class_<gtsam::SymbolicConditional, gtsam::SymbolicFactor, std::shared_ptr<gtsam::SymbolicConditional>>(m_gtsam, "SymbolicConditional")
        .def(py::init<>())
        .def(py::init<const gtsam::SymbolicConditional&>(), py::arg("other"))
        .def(py::init< size_t>(), py::arg("key"))
        .def(py::init< size_t,  size_t>(), py::arg("key"), py::arg("parent"))
        .def(py::init< size_t,  size_t,  size_t>(), py::arg("key"), py::arg("parent1"), py::arg("parent2"))
        .def(py::init< size_t,  size_t,  size_t,  size_t>(), py::arg("key"), py::arg("parent1"), py::arg("parent2"), py::arg("parent3"))
        .def("print_",[](gtsam::SymbolicConditional* self, string s){ self->print(s);}, py::arg("s"))
        .def("equals",[](gtsam::SymbolicConditional* self,const gtsam::SymbolicConditional& other, double tol){return self->equals(other, tol);}, py::arg("other"), py::arg("tol"))
        .def("nrFrontals",[](gtsam::SymbolicConditional* self){return self->nrFrontals();})
        .def("nrParents",[](gtsam::SymbolicConditional* self){return self->nrParents();})
        .def_static("FromKeys",[](const gtsam::KeyVector& keys, size_t nrFrontals){return gtsam::SymbolicConditional::FromKeys(keys, nrFrontals);}, py::arg("keys"), py::arg("nrFrontals"));

    py::class_<gtsam::SymbolicBayesNet, std::shared_ptr<gtsam::SymbolicBayesNet>>(m_gtsam, "SymbolicBayesNet")
        .def(py::init<>())
        .def(py::init<const gtsam::SymbolicBayesNet&>(), py::arg("other"))
        .def("print_",[](gtsam::SymbolicBayesNet* self, string s){ self->print(s);}, py::arg("s"))
        .def("equals",[](gtsam::SymbolicBayesNet* self,const gtsam::SymbolicBayesNet& other, double tol){return self->equals(other, tol);}, py::arg("other"), py::arg("tol"))
        .def("size",[](gtsam::SymbolicBayesNet* self){return self->size();})
        .def("saveGraph",[](gtsam::SymbolicBayesNet* self, string s){ self->saveGraph(s);}, py::arg("s"))
        .def("at",[](gtsam::SymbolicBayesNet* self, size_t idx){return self->at(idx);}, py::arg("idx"))
        .def("front",[](gtsam::SymbolicBayesNet* self){return self->front();})
        .def("back",[](gtsam::SymbolicBayesNet* self){return self->back();})
        .def("push_back",[](gtsam::SymbolicBayesNet* self,const std::shared_ptr<gtsam::SymbolicConditional>& conditional){ self->push_back(conditional);}, py::arg("conditional"))
        .def("push_back",[](gtsam::SymbolicBayesNet* self,const gtsam::SymbolicBayesNet& bayesNet){ self->push_back(bayesNet);}, py::arg("bayesNet"));

    py::class_<gtsam::SymbolicBayesTree, std::shared_ptr<gtsam::SymbolicBayesTree>>(m_gtsam, "SymbolicBayesTree")
        .def(py::init<>())
        .def(py::init<const gtsam::SymbolicBayesTree&>(), py::arg("other"))
        .def("print_",[](gtsam::SymbolicBayesTree* self, string s){ self->print(s);}, py::arg("s"))
        .def("equals",[](gtsam::SymbolicBayesTree* self,const gtsam::SymbolicBayesTree& other, double tol){return self->equals(other, tol);}, py::arg("other"), py::arg("tol"))
        .def("size",[](gtsam::SymbolicBayesTree* self){return self->size();})
        .def("saveGraph",[](gtsam::SymbolicBayesTree* self, string s){ self->saveGraph(s);}, py::arg("s"))
        .def("clear",[](gtsam::SymbolicBayesTree* self){ self->clear();})
        .def("deleteCachedShortcuts",[](gtsam::SymbolicBayesTree* self){ self->deleteCachedShortcuts();})
        .def("numCachedSeparatorMarginals",[](gtsam::SymbolicBayesTree* self){return self->numCachedSeparatorMarginals();})
        .def("marginalFactor",[](gtsam::SymbolicBayesTree* self, size_t key){return self->marginalFactor(key);}, py::arg("key"))
        .def("joint",[](gtsam::SymbolicBayesTree* self, size_t key1, size_t key2){return self->joint(key1, key2);}, py::arg("key1"), py::arg("key2"))
        .def("jointBayesNet",[](gtsam::SymbolicBayesTree* self, size_t key1, size_t key2){return self->jointBayesNet(key1, key2);}, py::arg("key1"), py::arg("key2"));

    py::class_<gtsam::VariableIndex, std::shared_ptr<gtsam::VariableIndex>>(m_gtsam, "VariableIndex")
        .def(py::init<>())
        .def(py::init<const gtsam::SymbolicFactorGraph&>(), py::arg("sfg"))
        .def(py::init<const gtsam::GaussianFactorGraph&>(), py::arg("gfg"))
        .def(py::init<const gtsam::NonlinearFactorGraph&>(), py::arg("fg"))
        .def(py::init<const gtsam::VariableIndex&>(), py::arg("other"))
        .def("equals",[](gtsam::VariableIndex* self,const gtsam::VariableIndex& other, double tol){return self->equals(other, tol);}, py::arg("other"), py::arg("tol"))
        .def("print_",[](gtsam::VariableIndex* self, string s){ self->print(s);}, py::arg("s"))
        .def("size",[](gtsam::VariableIndex* self){return self->size();})
        .def("nFactors",[](gtsam::VariableIndex* self){return self->nFactors();})
        .def("nEntries",[](gtsam::VariableIndex* self){return self->nEntries();});
    pybind11::module m_gtsam_noiseModel = m_gtsam.def_submodule("noiseModel", "noiseModel submodule");

    py::class_<gtsam::noiseModel::Base, std::shared_ptr<gtsam::noiseModel::Base>>(m_gtsam_noiseModel, "Base");

    py::class_<gtsam::noiseModel::Gaussian, gtsam::noiseModel::Base, std::shared_ptr<gtsam::noiseModel::Gaussian>>(m_gtsam_noiseModel, "Gaussian")
        .def("R",[](gtsam::noiseModel::Gaussian* self){return self->R();})
        .def("equals",[](gtsam::noiseModel::Gaussian* self, gtsam::noiseModel::Base& expected, double tol){return self->equals(expected, tol);}, py::arg("expected"), py::arg("tol"))
        .def("print_",[](gtsam::noiseModel::Gaussian* self, string s){ self->print(s);}, py::arg("s"))
        .def_static("SqrtInformation",[](const gtsam::Matrix& R){return gtsam::noiseModel::Gaussian::SqrtInformation(R);}, py::arg("R"))
        .def_static("Covariance",[](const gtsam::Matrix& R){return gtsam::noiseModel::Gaussian::Covariance(R);}, py::arg("R"));

    py::class_<gtsam::noiseModel::Diagonal, gtsam::noiseModel::Gaussian, std::shared_ptr<gtsam::noiseModel::Diagonal>>(m_gtsam_noiseModel, "Diagonal")
        .def("R",[](gtsam::noiseModel::Diagonal* self){return self->R();})
        .def("print_",[](gtsam::noiseModel::Diagonal* self, string s){ self->print(s);}, py::arg("s"))
        .def_static("Sigmas",[](const gtsam::Vector& sigmas){return gtsam::noiseModel::Diagonal::Sigmas(sigmas);}, py::arg("sigmas"))
        .def_static("Variances",[](const gtsam::Vector& variances){return gtsam::noiseModel::Diagonal::Variances(variances);}, py::arg("variances"))
        .def_static("Precisions",[](const gtsam::Vector& precisions){return gtsam::noiseModel::Diagonal::Precisions(precisions);}, py::arg("precisions"));

    py::class_<gtsam::noiseModel::Constrained, gtsam::noiseModel::Diagonal, std::shared_ptr<gtsam::noiseModel::Constrained>>(m_gtsam_noiseModel, "Constrained")
        .def("unit",[](gtsam::noiseModel::Constrained* self){return self->unit();})
        .def_static("MixedSigmas",[](const gtsam::Vector& mu,const gtsam::Vector& sigmas){return gtsam::noiseModel::Constrained::MixedSigmas(mu, sigmas);}, py::arg("mu"), py::arg("sigmas"))
        .def_static("MixedSigmas",[]( double m,const gtsam::Vector& sigmas){return gtsam::noiseModel::Constrained::MixedSigmas(m, sigmas);}, py::arg("m"), py::arg("sigmas"))
        .def_static("MixedVariances",[](const gtsam::Vector& mu,const gtsam::Vector& variances){return gtsam::noiseModel::Constrained::MixedVariances(mu, variances);}, py::arg("mu"), py::arg("variances"))
        .def_static("MixedVariances",[](const gtsam::Vector& variances){return gtsam::noiseModel::Constrained::MixedVariances(variances);}, py::arg("variances"))
        .def_static("MixedPrecisions",[](const gtsam::Vector& mu,const gtsam::Vector& precisions){return gtsam::noiseModel::Constrained::MixedPrecisions(mu, precisions);}, py::arg("mu"), py::arg("precisions"))
        .def_static("MixedPrecisions",[](const gtsam::Vector& precisions){return gtsam::noiseModel::Constrained::MixedPrecisions(precisions);}, py::arg("precisions"))
        .def_static("All",[]( size_t dim){return gtsam::noiseModel::Constrained::All(dim);}, py::arg("dim"))
        .def_static("All",[]( size_t dim, double mu){return gtsam::noiseModel::Constrained::All(dim, mu);}, py::arg("dim"), py::arg("mu"));

    py::class_<gtsam::noiseModel::Isotropic, gtsam::noiseModel::Diagonal, std::shared_ptr<gtsam::noiseModel::Isotropic>>(m_gtsam_noiseModel, "Isotropic")
        .def("print_",[](gtsam::noiseModel::Isotropic* self, string s){ self->print(s);}, py::arg("s"))
        .def_static("Sigma",[]( size_t dim, double sigma){return gtsam::noiseModel::Isotropic::Sigma(dim, sigma);}, py::arg("dim"), py::arg("sigma"))
        .def_static("Variance",[]( size_t dim, double varianace){return gtsam::noiseModel::Isotropic::Variance(dim, varianace);}, py::arg("dim"), py::arg("varianace"))
        .def_static("Precision",[]( size_t dim, double precision){return gtsam::noiseModel::Isotropic::Precision(dim, precision);}, py::arg("dim"), py::arg("precision"));

    py::class_<gtsam::noiseModel::Unit, gtsam::noiseModel::Isotropic, std::shared_ptr<gtsam::noiseModel::Unit>>(m_gtsam_noiseModel, "Unit")
        .def("print_",[](gtsam::noiseModel::Unit* self, string s){ self->print(s);}, py::arg("s"))
        .def_static("Create",[]( size_t dim){return gtsam::noiseModel::Unit::Create(dim);}, py::arg("dim"));
    pybind11::module m_gtsam_noiseModel_mEstimator = m_gtsam_noiseModel.def_submodule("mEstimator", "mEstimator submodule");

    py::class_<gtsam::noiseModel::mEstimator::Base, std::shared_ptr<gtsam::noiseModel::mEstimator::Base>>(m_gtsam_noiseModel_mEstimator, "Base");

    py::class_<gtsam::noiseModel::mEstimator::Null, gtsam::noiseModel::mEstimator::Base, std::shared_ptr<gtsam::noiseModel::mEstimator::Null>>(m_gtsam_noiseModel_mEstimator, "Null")
        .def(py::init<>())
        .def("print_",[](gtsam::noiseModel::mEstimator::Null* self, string s){ self->print(s);}, py::arg("s"))
        .def_static("Create",[](){return gtsam::noiseModel::mEstimator::Null::Create();});

    py::class_<gtsam::noiseModel::mEstimator::Fair, gtsam::noiseModel::mEstimator::Base, std::shared_ptr<gtsam::noiseModel::mEstimator::Fair>>(m_gtsam_noiseModel_mEstimator, "Fair")
        .def(py::init< double>(), py::arg("c"))
        .def("print_",[](gtsam::noiseModel::mEstimator::Fair* self, string s){ self->print(s);}, py::arg("s"))
        .def_static("Create",[]( double c){return gtsam::noiseModel::mEstimator::Fair::Create(c);}, py::arg("c"));

    py::class_<gtsam::noiseModel::mEstimator::Huber, gtsam::noiseModel::mEstimator::Base, std::shared_ptr<gtsam::noiseModel::mEstimator::Huber>>(m_gtsam_noiseModel_mEstimator, "Huber")
        .def(py::init< double>(), py::arg("k"))
        .def("print_",[](gtsam::noiseModel::mEstimator::Huber* self, string s){ self->print(s);}, py::arg("s"))
        .def_static("Create",[]( double k){return gtsam::noiseModel::mEstimator::Huber::Create(k);}, py::arg("k"));

    py::class_<gtsam::noiseModel::mEstimator::Tukey, gtsam::noiseModel::mEstimator::Base, std::shared_ptr<gtsam::noiseModel::mEstimator::Tukey>>(m_gtsam_noiseModel_mEstimator, "Tukey")
        .def(py::init< double>(), py::arg("k"))
        .def("print_",[](gtsam::noiseModel::mEstimator::Tukey* self, string s){ self->print(s);}, py::arg("s"))
        .def_static("Create",[]( double k){return gtsam::noiseModel::mEstimator::Tukey::Create(k);}, py::arg("k"));

    py::class_<gtsam::noiseModel::Robust, gtsam::noiseModel::Base, std::shared_ptr<gtsam::noiseModel::Robust>>(m_gtsam_noiseModel, "Robust")
        .def(py::init<const std::shared_ptr<gtsam::noiseModel::mEstimator::Base>&, const std::shared_ptr<gtsam::noiseModel::Base>&>(), py::arg("robust"), py::arg("noise"))
        .def("print_",[](gtsam::noiseModel::Robust* self, string s){ self->print(s);}, py::arg("s"))
        .def_static("Create",[](const std::shared_ptr<gtsam::noiseModel::mEstimator::Base>& robust,const std::shared_ptr<gtsam::noiseModel::Base>& noise){return gtsam::noiseModel::Robust::Create(robust, noise);}, py::arg("robust"), py::arg("noise"));

    py::class_<gtsam::Sampler, std::shared_ptr<gtsam::Sampler>>(m_gtsam, "Sampler")
        .def(py::init<const std::shared_ptr<gtsam::noiseModel::Diagonal>&,  int>(), py::arg("model"), py::arg("seed"))
        .def(py::init<const gtsam::Vector&,  int>(), py::arg("sigmas"), py::arg("seed"))
        .def(py::init< int>(), py::arg("seed"))
        .def("dim",[](gtsam::Sampler* self){return self->dim();})
        .def("sigmas",[](gtsam::Sampler* self){return self->sigmas();})
        .def("model",[](gtsam::Sampler* self){return self->model();})
        .def("sample",[](gtsam::Sampler* self){return self->sample();})
        .def("sampleNewModel",[](gtsam::Sampler* self,const std::shared_ptr<gtsam::noiseModel::Diagonal>& model){return self->sampleNewModel(model);}, py::arg("model"));

    py::class_<gtsam::VectorValues, std::shared_ptr<gtsam::VectorValues>>(m_gtsam, "VectorValues")
        .def(py::init<>())
        .def(py::init<const gtsam::VectorValues&>(), py::arg("other"))
        .def("size",[](gtsam::VectorValues* self){return self->size();})
        .def("dim",[](gtsam::VectorValues* self, size_t j){return self->dim(j);}, py::arg("j"))
        .def("exists",[](gtsam::VectorValues* self, size_t j){return self->exists(j);}, py::arg("j"))
        .def("print_",[](gtsam::VectorValues* self, string s){ self->print(s);}, py::arg("s"))
        .def("equals",[](gtsam::VectorValues* self,const gtsam::VectorValues& expected, double tol){return self->equals(expected, tol);}, py::arg("expected"), py::arg("tol"))
        .def("insert",[](gtsam::VectorValues* self, size_t j,const gtsam::Vector& value){ self->insert(j, value);}, py::arg("j"), py::arg("value"))
        .def("vector",[](gtsam::VectorValues* self){return self->vector();})
        .def("at",[](gtsam::VectorValues* self, size_t j){return self->at(j);}, py::arg("j"))
        .def("update",[](gtsam::VectorValues* self,const gtsam::VectorValues& values){ self->update(values);}, py::arg("values"))
        .def("setZero",[](gtsam::VectorValues* self){ self->setZero();})
        .def("add",[](gtsam::VectorValues* self,const gtsam::VectorValues& c){return self->add(c);}, py::arg("c"))
        .def("addInPlace",[](gtsam::VectorValues* self,const gtsam::VectorValues& c){ self->addInPlace(c);}, py::arg("c"))
        .def("subtract",[](gtsam::VectorValues* self,const gtsam::VectorValues& c){return self->subtract(c);}, py::arg("c"))
        .def("scale",[](gtsam::VectorValues* self, double a){return self->scale(a);}, py::arg("a"))
        .def("scaleInPlace",[](gtsam::VectorValues* self, double a){ self->scaleInPlace(a);}, py::arg("a"))
        .def("hasSameStructure",[](gtsam::VectorValues* self,const gtsam::VectorValues& other){return self->hasSameStructure(other);}, py::arg("other"))
        .def("dot",[](gtsam::VectorValues* self,const gtsam::VectorValues& V){return self->dot(V);}, py::arg("V"))
        .def("norm",[](gtsam::VectorValues* self){return self->norm();})
        .def("squaredNorm",[](gtsam::VectorValues* self){return self->squaredNorm();})
        .def_static("Zero",[](const gtsam::VectorValues& model){return gtsam::VectorValues::Zero(model);}, py::arg("model"));

    py::class_<gtsam::GaussianFactor, std::shared_ptr<gtsam::GaussianFactor>>(m_gtsam, "GaussianFactor")
        .def("keys",[](gtsam::GaussianFactor* self){return self->keys();})
        .def("print_",[](gtsam::GaussianFactor* self, string s){ self->print(s);}, py::arg("s"))
        .def("equals",[](gtsam::GaussianFactor* self,const gtsam::GaussianFactor& lf, double tol){return self->equals(lf, tol);}, py::arg("lf"), py::arg("tol"))
        .def("error",[](gtsam::GaussianFactor* self,const gtsam::VectorValues& c){return self->error(c);}, py::arg("c"))
        .def("clone",[](gtsam::GaussianFactor* self){return self->clone();})
        .def("negate",[](gtsam::GaussianFactor* self){return self->negate();})
        .def("augmentedInformation",[](gtsam::GaussianFactor* self){return self->augmentedInformation();})
        .def("information",[](gtsam::GaussianFactor* self){return self->information();})
        .def("augmentedJacobian",[](gtsam::GaussianFactor* self){return self->augmentedJacobian();})
        .def("jacobian",[](gtsam::GaussianFactor* self){return self->jacobian();})
        .def("size",[](gtsam::GaussianFactor* self){return self->size();})
        .def("empty",[](gtsam::GaussianFactor* self){return self->empty();});

    py::class_<gtsam::JacobianFactor, gtsam::GaussianFactor, std::shared_ptr<gtsam::JacobianFactor>>(m_gtsam, "JacobianFactor")
        .def(py::init<>())
        .def(py::init<const gtsam::GaussianFactor&>(), py::arg("factor"))
        .def(py::init<const gtsam::Vector&>(), py::arg("b_in"))
        .def(py::init< size_t, const gtsam::Matrix&, const gtsam::Vector&, const std::shared_ptr<gtsam::noiseModel::Diagonal>&>(), py::arg("i1"), py::arg("A1"), py::arg("b"), py::arg("model"))
        .def(py::init< size_t, const gtsam::Matrix&,  size_t, const gtsam::Matrix&, const gtsam::Vector&, const std::shared_ptr<gtsam::noiseModel::Diagonal>&>(), py::arg("i1"), py::arg("A1"), py::arg("i2"), py::arg("A2"), py::arg("b"), py::arg("model"))
        .def(py::init< size_t, const gtsam::Matrix&,  size_t, const gtsam::Matrix&,  size_t, const gtsam::Matrix&, const gtsam::Vector&, const std::shared_ptr<gtsam::noiseModel::Diagonal>&>(), py::arg("i1"), py::arg("A1"), py::arg("i2"), py::arg("A2"), py::arg("i3"), py::arg("A3"), py::arg("b"), py::arg("model"))
        .def(py::init<const gtsam::GaussianFactorGraph&>(), py::arg("graph"))
        .def("print_",[](gtsam::JacobianFactor* self, string s){ self->print(s);}, py::arg("s"))
        .def("printKeys",[](gtsam::JacobianFactor* self, string s){ self->printKeys(s);}, py::arg("s"))
        .def("equals",[](gtsam::JacobianFactor* self,const gtsam::GaussianFactor& lf, double tol){return self->equals(lf, tol);}, py::arg("lf"), py::arg("tol"))
        .def("size",[](gtsam::JacobianFactor* self){return self->size();})
        .def("unweighted_error",[](gtsam::JacobianFactor* self,const gtsam::VectorValues& c){return self->unweighted_error(c);}, py::arg("c"))
        .def("error_vector",[](gtsam::JacobianFactor* self,const gtsam::VectorValues& c){return self->error_vector(c);}, py::arg("c"))
        .def("error",[](gtsam::JacobianFactor* self,const gtsam::VectorValues& c){return self->error(c);}, py::arg("c"))
        .def("getA",[](gtsam::JacobianFactor* self){return self->getA();})
        .def("getb",[](gtsam::JacobianFactor* self){return self->getb();})
        .def("rows",[](gtsam::JacobianFactor* self){return self->rows();})
        .def("cols",[](gtsam::JacobianFactor* self){return self->cols();})
        .def("isConstrained",[](gtsam::JacobianFactor* self){return self->isConstrained();})
        .def("jacobianUnweighted",[](gtsam::JacobianFactor* self){return self->jacobianUnweighted();})
        .def("augmentedJacobianUnweighted",[](gtsam::JacobianFactor* self){return self->augmentedJacobianUnweighted();})
        .def("transposeMultiplyAdd",[](gtsam::JacobianFactor* self, double alpha,const gtsam::Vector& e, gtsam::VectorValues& x){ self->transposeMultiplyAdd(alpha, e, x);}, py::arg("alpha"), py::arg("e"), py::arg("x"))
        .def("whiten",[](gtsam::JacobianFactor* self){return self->whiten();})
        .def("eliminate",[](gtsam::JacobianFactor* self,const gtsam::Ordering& keys){return self->eliminate(keys);}, py::arg("keys"))
        .def("setModel",[](gtsam::JacobianFactor* self, bool anyConstrained,const gtsam::Vector& sigmas){ self->setModel(anyConstrained, sigmas);}, py::arg("anyConstrained"), py::arg("sigmas"))
        .def("get_model",[](gtsam::JacobianFactor* self){return self->get_model();});

    py::class_<gtsam::HessianFactor, gtsam::GaussianFactor, std::shared_ptr<gtsam::HessianFactor>>(m_gtsam, "HessianFactor")
        .def(py::init<>())
        .def(py::init<const gtsam::GaussianFactor&>(), py::arg("factor"))
        .def(py::init< size_t, const gtsam::Matrix&, const gtsam::Vector&,  double>(), py::arg("j"), py::arg("G"), py::arg("g"), py::arg("f"))
        .def(py::init< size_t, const gtsam::Vector&, const gtsam::Matrix&>(), py::arg("j"), py::arg("mu"), py::arg("Sigma"))
        .def(py::init< size_t,  size_t, const gtsam::Matrix&, const gtsam::Matrix&, const gtsam::Vector&, const gtsam::Matrix&, const gtsam::Vector&,  double>(), py::arg("j1"), py::arg("j2"), py::arg("G11"), py::arg("G12"), py::arg("g1"), py::arg("G22"), py::arg("g2"), py::arg("f"))
        .def(py::init< size_t,  size_t,  size_t, const gtsam::Matrix&, const gtsam::Matrix&, const gtsam::Matrix&, const gtsam::Vector&, const gtsam::Matrix&, const gtsam::Matrix&, const gtsam::Vector&, const gtsam::Matrix&, const gtsam::Vector&,  double>(), py::arg("j1"), py::arg("j2"), py::arg("j3"), py::arg("G11"), py::arg("G12"), py::arg("G13"), py::arg("g1"), py::arg("G22"), py::arg("G23"), py::arg("g2"), py::arg("G33"), py::arg("g3"), py::arg("f"))
        .def(py::init<const gtsam::GaussianFactorGraph&>(), py::arg("factors"))
        .def("size",[](gtsam::HessianFactor* self){return self->size();})
        .def("print_",[](gtsam::HessianFactor* self, string s){ self->print(s);}, py::arg("s"))
        .def("printKeys",[](gtsam::HessianFactor* self, string s){ self->printKeys(s);}, py::arg("s"))
        .def("equals",[](gtsam::HessianFactor* self,const gtsam::GaussianFactor& lf, double tol){return self->equals(lf, tol);}, py::arg("lf"), py::arg("tol"))
        .def("error",[](gtsam::HessianFactor* self,const gtsam::VectorValues& c){return self->error(c);}, py::arg("c"))
        .def("rows",[](gtsam::HessianFactor* self){return self->rows();})
        .def("information",[](gtsam::HessianFactor* self){return self->information();})
        .def("constantTerm",[](gtsam::HessianFactor* self){return self->constantTerm();})
        .def("linearTerm",[](gtsam::HessianFactor* self){return self->linearTerm();});

    py::class_<gtsam::GaussianFactorGraph, std::shared_ptr<gtsam::GaussianFactorGraph>>(m_gtsam, "GaussianFactorGraph")
        .def(py::init<>())
        .def(py::init<const gtsam::GaussianBayesNet&>(), py::arg("bayesNet"))
        .def(py::init<const gtsam::GaussianBayesTree&>(), py::arg("bayesTree"))
        .def("print_",[](gtsam::GaussianFactorGraph* self, string s){ self->print(s);}, py::arg("s"))
        .def("equals",[](gtsam::GaussianFactorGraph* self,const gtsam::GaussianFactorGraph& lfgraph, double tol){return self->equals(lfgraph, tol);}, py::arg("lfgraph"), py::arg("tol"))
        .def("size",[](gtsam::GaussianFactorGraph* self){return self->size();})
        .def("at",[](gtsam::GaussianFactorGraph* self, size_t idx){return self->at(idx);}, py::arg("idx"))
        .def("keys",[](gtsam::GaussianFactorGraph* self){return self->keys();})
        .def("keyVector",[](gtsam::GaussianFactorGraph* self){return self->keyVector();})
        .def("exists",[](gtsam::GaussianFactorGraph* self, size_t idx){return self->exists(idx);}, py::arg("idx"))
        .def("push_back",[](gtsam::GaussianFactorGraph* self,const std::shared_ptr<gtsam::GaussianFactor>& factor){ self->push_back(factor);}, py::arg("factor"))
        .def("push_back",[](gtsam::GaussianFactorGraph* self,const std::shared_ptr<gtsam::GaussianConditional>& conditional){ self->push_back(conditional);}, py::arg("conditional"))
        .def("push_back",[](gtsam::GaussianFactorGraph* self,const gtsam::GaussianFactorGraph& graph){ self->push_back(graph);}, py::arg("graph"))
        .def("push_back",[](gtsam::GaussianFactorGraph* self,const gtsam::GaussianBayesNet& bayesNet){ self->push_back(bayesNet);}, py::arg("bayesNet"))
        .def("push_back",[](gtsam::GaussianFactorGraph* self,const gtsam::GaussianBayesTree& bayesTree){ self->push_back(bayesTree);}, py::arg("bayesTree"))
        .def("add",[](gtsam::GaussianFactorGraph* self,const gtsam::GaussianFactor& factor){ self->add(factor);}, py::arg("factor"))
        .def("add",[](gtsam::GaussianFactorGraph* self,const gtsam::Vector& b){ self->add(b);}, py::arg("b"))
        .def("add",[](gtsam::GaussianFactorGraph* self, size_t key1,const gtsam::Matrix& A1,const gtsam::Vector& b,const std::shared_ptr<gtsam::noiseModel::Diagonal>& model){ self->add(key1, A1, b, model);}, py::arg("key1"), py::arg("A1"), py::arg("b"), py::arg("model"))
        .def("add",[](gtsam::GaussianFactorGraph* self, size_t key1,const gtsam::Matrix& A1, size_t key2,const gtsam::Matrix& A2,const gtsam::Vector& b,const std::shared_ptr<gtsam::noiseModel::Diagonal>& model){ self->add(key1, A1, key2, A2, b, model);}, py::arg("key1"), py::arg("A1"), py::arg("key2"), py::arg("A2"), py::arg("b"), py::arg("model"))
        .def("add",[](gtsam::GaussianFactorGraph* self, size_t key1,const gtsam::Matrix& A1, size_t key2,const gtsam::Matrix& A2, size_t key3,const gtsam::Matrix& A3,const gtsam::Vector& b,const std::shared_ptr<gtsam::noiseModel::Diagonal>& model){ self->add(key1, A1, key2, A2, key3, A3, b, model);}, py::arg("key1"), py::arg("A1"), py::arg("key2"), py::arg("A2"), py::arg("key3"), py::arg("A3"), py::arg("b"), py::arg("model"))
        .def("error",[](gtsam::GaussianFactorGraph* self,const gtsam::VectorValues& c){return self->error(c);}, py::arg("c"))
        .def("probPrime",[](gtsam::GaussianFactorGraph* self,const gtsam::VectorValues& c){return self->probPrime(c);}, py::arg("c"))
        .def("clone",[](gtsam::GaussianFactorGraph* self){return self->clone();})
        .def("negate",[](gtsam::GaussianFactorGraph* self){return self->negate();})
        .def("optimize",[](gtsam::GaussianFactorGraph* self){return self->optimize();})
        .def("optimize",[](gtsam::GaussianFactorGraph* self,const gtsam::Ordering& ordering){return self->optimize(ordering);}, py::arg("ordering"))
        .def("optimizeGradientSearch",[](gtsam::GaussianFactorGraph* self){return self->optimizeGradientSearch();})
        .def("gradient",[](gtsam::GaussianFactorGraph* self,const gtsam::VectorValues& x0){return self->gradient(x0);}, py::arg("x0"))
        .def("gradientAtZero",[](gtsam::GaussianFactorGraph* self){return self->gradientAtZero();})
        .def("eliminateSequential",[](gtsam::GaussianFactorGraph* self){return self->eliminateSequential();})
        .def("eliminateSequential",[](gtsam::GaussianFactorGraph* self,const gtsam::Ordering& ordering){return self->eliminateSequential(ordering);}, py::arg("ordering"))
        .def("eliminateMultifrontal",[](gtsam::GaussianFactorGraph* self){return self->eliminateMultifrontal();})
        .def("eliminateMultifrontal",[](gtsam::GaussianFactorGraph* self,const gtsam::Ordering& ordering){return self->eliminateMultifrontal(ordering);}, py::arg("ordering"))
        .def("eliminatePartialSequential",[](gtsam::GaussianFactorGraph* self,const gtsam::Ordering& ordering){return self->eliminatePartialSequential(ordering);}, py::arg("ordering"))
        .def("eliminatePartialSequential",[](gtsam::GaussianFactorGraph* self,const gtsam::KeyVector& keys){return self->eliminatePartialSequential(keys);}, py::arg("keys"))
        .def("eliminatePartialMultifrontal",[](gtsam::GaussianFactorGraph* self,const gtsam::Ordering& ordering){return self->eliminatePartialMultifrontal(ordering);}, py::arg("ordering"))
        .def("eliminatePartialMultifrontal",[](gtsam::GaussianFactorGraph* self,const gtsam::KeyVector& keys){return self->eliminatePartialMultifrontal(keys);}, py::arg("keys"))
        .def("marginalMultifrontalBayesNet",[](gtsam::GaussianFactorGraph* self,const gtsam::Ordering& ordering){return self->marginalMultifrontalBayesNet(ordering);}, py::arg("ordering"))
        .def("marginalMultifrontalBayesNet",[](gtsam::GaussianFactorGraph* self,const gtsam::KeyVector& key_vector){return self->marginalMultifrontalBayesNet(key_vector);}, py::arg("key_vector"))
        .def("marginalMultifrontalBayesNet",[](gtsam::GaussianFactorGraph* self,const gtsam::Ordering& ordering,const gtsam::Ordering& marginalizedVariableOrdering){return self->marginalMultifrontalBayesNet(ordering, marginalizedVariableOrdering);}, py::arg("ordering"), py::arg("marginalizedVariableOrdering"))
        .def("marginalMultifrontalBayesNet",[](gtsam::GaussianFactorGraph* self,const gtsam::KeyVector& key_vector,const gtsam::Ordering& marginalizedVariableOrdering){return self->marginalMultifrontalBayesNet(key_vector, marginalizedVariableOrdering);}, py::arg("key_vector"), py::arg("marginalizedVariableOrdering"))
        .def("marginal",[](gtsam::GaussianFactorGraph* self,const gtsam::KeyVector& key_vector){return self->marginal(key_vector);}, py::arg("key_vector"))
        .def("sparseJacobian_",[](gtsam::GaussianFactorGraph* self){return self->sparseJacobian_();})
        .def("augmentedJacobian",[](gtsam::GaussianFactorGraph* self){return self->augmentedJacobian();})
        .def("augmentedJacobian",[](gtsam::GaussianFactorGraph* self,const gtsam::Ordering& ordering){return self->augmentedJacobian(ordering);}, py::arg("ordering"))
        .def("jacobian",[](gtsam::GaussianFactorGraph* self){return self->jacobian();})
        .def("jacobian",[](gtsam::GaussianFactorGraph* self,const gtsam::Ordering& ordering){return self->jacobian(ordering);}, py::arg("ordering"))
        .def("augmentedHessian",[](gtsam::GaussianFactorGraph* self){return self->augmentedHessian();})
        .def("augmentedHessian",[](gtsam::GaussianFactorGraph* self,const gtsam::Ordering& ordering){return self->augmentedHessian(ordering);}, py::arg("ordering"))
        .def("hessian",[](gtsam::GaussianFactorGraph* self){return self->hessian();})
        .def("hessian",[](gtsam::GaussianFactorGraph* self,const gtsam::Ordering& ordering){return self->hessian(ordering);}, py::arg("ordering"));

    py::class_<gtsam::GaussianConditional, gtsam::GaussianFactor, std::shared_ptr<gtsam::GaussianConditional>>(m_gtsam, "GaussianConditional")
        .def(py::init< size_t, const gtsam::Vector&, const gtsam::Matrix&, const std::shared_ptr<gtsam::noiseModel::Diagonal>&>(), py::arg("key"), py::arg("d"), py::arg("R"), py::arg("sigmas"))
        .def(py::init< size_t, const gtsam::Vector&, const gtsam::Matrix&,  size_t, const gtsam::Matrix&, const std::shared_ptr<gtsam::noiseModel::Diagonal>&>(), py::arg("key"), py::arg("d"), py::arg("R"), py::arg("name1"), py::arg("S"), py::arg("sigmas"))
        .def(py::init< size_t, const gtsam::Vector&, const gtsam::Matrix&,  size_t, const gtsam::Matrix&,  size_t, const gtsam::Matrix&, const std::shared_ptr<gtsam::noiseModel::Diagonal>&>(), py::arg("key"), py::arg("d"), py::arg("R"), py::arg("name1"), py::arg("S"), py::arg("name2"), py::arg("T"), py::arg("sigmas"))
        .def(py::init< size_t, const gtsam::Vector&, const gtsam::Matrix&>(), py::arg("key"), py::arg("d"), py::arg("R"))
        .def(py::init< size_t, const gtsam::Vector&, const gtsam::Matrix&,  size_t, const gtsam::Matrix&>(), py::arg("key"), py::arg("d"), py::arg("R"), py::arg("name1"), py::arg("S"))
        .def(py::init< size_t, const gtsam::Vector&, const gtsam::Matrix&,  size_t, const gtsam::Matrix&,  size_t, const gtsam::Matrix&>(), py::arg("key"), py::arg("d"), py::arg("R"), py::arg("name1"), py::arg("S"), py::arg("name2"), py::arg("T"))
        .def("print_",[](gtsam::GaussianConditional* self, string s){ self->print(s);}, py::arg("s"))
        .def("equals",[](gtsam::GaussianConditional* self,const gtsam::GaussianConditional& cg, double tol){return self->equals(cg, tol);}, py::arg("cg"), py::arg("tol"))
        .def("solve",[](gtsam::GaussianConditional* self,const gtsam::VectorValues& parents){return self->solve(parents);}, py::arg("parents"))
        .def("solveOtherRHS",[](gtsam::GaussianConditional* self,const gtsam::VectorValues& parents,const gtsam::VectorValues& rhs){return self->solveOtherRHS(parents, rhs);}, py::arg("parents"), py::arg("rhs"))
        .def("solveTransposeInPlace",[](gtsam::GaussianConditional* self, gtsam::VectorValues& gy){ self->solveTransposeInPlace(gy);}, py::arg("gy"))
        .def("scaleFrontalsBySigma",[](gtsam::GaussianConditional* self, gtsam::VectorValues& gy){ self->scaleFrontalsBySigma(gy);}, py::arg("gy"));

    py::class_<gtsam::GaussianDensity, gtsam::GaussianConditional, std::shared_ptr<gtsam::GaussianDensity>>(m_gtsam, "GaussianDensity")
        .def(py::init< size_t, const gtsam::Vector&, const gtsam::Matrix&, const std::shared_ptr<gtsam::noiseModel::Diagonal>&>(), py::arg("key"), py::arg("d"), py::arg("R"), py::arg("sigmas"))
        .def("print_",[](gtsam::GaussianDensity* self, string s){ self->print(s);}, py::arg("s"))
        .def("equals",[](gtsam::GaussianDensity* self,const gtsam::GaussianDensity& cg, double tol){return self->equals(cg, tol);}, py::arg("cg"), py::arg("tol"))
        .def("mean",[](gtsam::GaussianDensity* self){return self->mean();})
        .def("covariance",[](gtsam::GaussianDensity* self){return self->covariance();});

    py::class_<gtsam::GaussianBayesNet, std::shared_ptr<gtsam::GaussianBayesNet>>(m_gtsam, "GaussianBayesNet")
        .def(py::init<>())
        .def(py::init<const std::shared_ptr<gtsam::GaussianConditional>&>(), py::arg("conditional"))
        .def("print_",[](gtsam::GaussianBayesNet* self, string s){ self->print(s);}, py::arg("s"))
        .def("equals",[](gtsam::GaussianBayesNet* self,const gtsam::GaussianBayesNet& other, double tol){return self->equals(other, tol);}, py::arg("other"), py::arg("tol"))
        .def("size",[](gtsam::GaussianBayesNet* self){return self->size();})
        .def("at",[](gtsam::GaussianBayesNet* self, size_t idx){return self->at(idx);}, py::arg("idx"))
        .def("keys",[](gtsam::GaussianBayesNet* self){return self->keys();})
        .def("exists",[](gtsam::GaussianBayesNet* self, size_t idx){return self->exists(idx);}, py::arg("idx"))
        .def("front",[](gtsam::GaussianBayesNet* self){return self->front();})
        .def("back",[](gtsam::GaussianBayesNet* self){return self->back();})
        .def("push_back",[](gtsam::GaussianBayesNet* self,const std::shared_ptr<gtsam::GaussianConditional>& conditional){ self->push_back(conditional);}, py::arg("conditional"))
        .def("push_back",[](gtsam::GaussianBayesNet* self,const gtsam::GaussianBayesNet& bayesNet){ self->push_back(bayesNet);}, py::arg("bayesNet"))
        .def("optimize",[](gtsam::GaussianBayesNet* self){return self->optimize();})
        .def("optimize",[](gtsam::GaussianBayesNet* self, gtsam::VectorValues& solutionForMissing){return self->optimize(solutionForMissing);}, py::arg("solutionForMissing"))
        .def("optimizeGradientSearch",[](gtsam::GaussianBayesNet* self){return self->optimizeGradientSearch();})
        .def("gradient",[](gtsam::GaussianBayesNet* self,const gtsam::VectorValues& x0){return self->gradient(x0);}, py::arg("x0"))
        .def("gradientAtZero",[](gtsam::GaussianBayesNet* self){return self->gradientAtZero();})
        .def("error",[](gtsam::GaussianBayesNet* self,const gtsam::VectorValues& x){return self->error(x);}, py::arg("x"))
        .def("determinant",[](gtsam::GaussianBayesNet* self){return self->determinant();})
        .def("logDeterminant",[](gtsam::GaussianBayesNet* self){return self->logDeterminant();})
        .def("backSubstitute",[](gtsam::GaussianBayesNet* self,const gtsam::VectorValues& gx){return self->backSubstitute(gx);}, py::arg("gx"))
        .def("backSubstituteTranspose",[](gtsam::GaussianBayesNet* self,const gtsam::VectorValues& gx){return self->backSubstituteTranspose(gx);}, py::arg("gx"));

    py::class_<gtsam::GaussianBayesTree, std::shared_ptr<gtsam::GaussianBayesTree>>(m_gtsam, "GaussianBayesTree")
        .def(py::init<>())
        .def(py::init<const gtsam::GaussianBayesTree&>(), py::arg("other"))
        .def("equals",[](gtsam::GaussianBayesTree* self,const gtsam::GaussianBayesTree& other, double tol){return self->equals(other, tol);}, py::arg("other"), py::arg("tol"))
        .def("print_",[](gtsam::GaussianBayesTree* self, string s){ self->print(s);}, py::arg("s"))
        .def("size",[](gtsam::GaussianBayesTree* self){return self->size();})
        .def("empty",[](gtsam::GaussianBayesTree* self){return self->empty();})
        .def("numCachedSeparatorMarginals",[](gtsam::GaussianBayesTree* self){return self->numCachedSeparatorMarginals();})
        .def("saveGraph",[](gtsam::GaussianBayesTree* self, string s){ self->saveGraph(s);}, py::arg("s"))
        .def("optimize",[](gtsam::GaussianBayesTree* self){return self->optimize();})
        .def("optimizeGradientSearch",[](gtsam::GaussianBayesTree* self){return self->optimizeGradientSearch();})
        .def("gradient",[](gtsam::GaussianBayesTree* self,const gtsam::VectorValues& x0){return self->gradient(x0);}, py::arg("x0"))
        .def("gradientAtZero",[](gtsam::GaussianBayesTree* self){return self->gradientAtZero();})
        .def("error",[](gtsam::GaussianBayesTree* self,const gtsam::VectorValues& x){return self->error(x);}, py::arg("x"))
        .def("determinant",[](gtsam::GaussianBayesTree* self){return self->determinant();})
        .def("logDeterminant",[](gtsam::GaussianBayesTree* self){return self->logDeterminant();})
        .def("marginalCovariance",[](gtsam::GaussianBayesTree* self, size_t key){return self->marginalCovariance(key);}, py::arg("key"))
        .def("marginalFactor",[](gtsam::GaussianBayesTree* self, size_t key){return self->marginalFactor(key);}, py::arg("key"))
        .def("joint",[](gtsam::GaussianBayesTree* self, size_t key1, size_t key2){return self->joint(key1, key2);}, py::arg("key1"), py::arg("key2"))
        .def("jointBayesNet",[](gtsam::GaussianBayesTree* self, size_t key1, size_t key2){return self->jointBayesNet(key1, key2);}, py::arg("key1"), py::arg("key2"));

    py::class_<gtsam::Errors, std::shared_ptr<gtsam::Errors>>(m_gtsam, "Errors")
        .def(py::init<>())
        .def(py::init<const gtsam::VectorValues&>(), py::arg("V"))
        .def("print_",[](gtsam::Errors* self, string s){ self->print(s);}, py::arg("s"))
        .def("equals",[](gtsam::Errors* self,const gtsam::Errors& expected, double tol){return self->equals(expected, tol);}, py::arg("expected"), py::arg("tol"));

    py::class_<gtsam::GaussianISAM, std::shared_ptr<gtsam::GaussianISAM>>(m_gtsam, "GaussianISAM")
        .def(py::init<>())
        .def("update",[](gtsam::GaussianISAM* self,const gtsam::GaussianFactorGraph& newFactors){ self->update(newFactors);}, py::arg("newFactors"))
        .def("saveGraph",[](gtsam::GaussianISAM* self, string s){ self->saveGraph(s);}, py::arg("s"))
        .def("clear",[](gtsam::GaussianISAM* self){ self->clear();});

    py::class_<gtsam::IterativeOptimizationParameters, std::shared_ptr<gtsam::IterativeOptimizationParameters>>(m_gtsam, "IterativeOptimizationParameters")
        .def("getVerbosity",[](gtsam::IterativeOptimizationParameters* self){return self->getVerbosity();})
        .def("setVerbosity",[](gtsam::IterativeOptimizationParameters* self, string s){ self->setVerbosity(s);}, py::arg("s"))
        .def("print_",[](gtsam::IterativeOptimizationParameters* self){ self->print();});

    py::class_<gtsam::ConjugateGradientParameters, gtsam::IterativeOptimizationParameters, std::shared_ptr<gtsam::ConjugateGradientParameters>>(m_gtsam, "ConjugateGradientParameters")
        .def(py::init<>())
        .def("getMinIterations",[](gtsam::ConjugateGradientParameters* self){return self->getMinIterations();})
        .def("getMaxIterations",[](gtsam::ConjugateGradientParameters* self){return self->getMaxIterations();})
        .def("getReset",[](gtsam::ConjugateGradientParameters* self){return self->getReset();})
        .def("getEpsilon_rel",[](gtsam::ConjugateGradientParameters* self){return self->getEpsilon_rel();})
        .def("getEpsilon_abs",[](gtsam::ConjugateGradientParameters* self){return self->getEpsilon_abs();})
        .def("setMinIterations",[](gtsam::ConjugateGradientParameters* self, int value){ self->setMinIterations(value);}, py::arg("value"))
        .def("setMaxIterations",[](gtsam::ConjugateGradientParameters* self, int value){ self->setMaxIterations(value);}, py::arg("value"))
        .def("setReset",[](gtsam::ConjugateGradientParameters* self, int value){ self->setReset(value);}, py::arg("value"))
        .def("setEpsilon_rel",[](gtsam::ConjugateGradientParameters* self, double value){ self->setEpsilon_rel(value);}, py::arg("value"))
        .def("setEpsilon_abs",[](gtsam::ConjugateGradientParameters* self, double value){ self->setEpsilon_abs(value);}, py::arg("value"))
        .def("print_",[](gtsam::ConjugateGradientParameters* self){ self->print();});

    py::class_<gtsam::SubgraphSolverParameters, gtsam::ConjugateGradientParameters, std::shared_ptr<gtsam::SubgraphSolverParameters>>(m_gtsam, "SubgraphSolverParameters")
        .def(py::init<>())
        .def("print_",[](gtsam::SubgraphSolverParameters* self){ self->print();});

    py::class_<gtsam::SubgraphSolver, std::shared_ptr<gtsam::SubgraphSolver>>(m_gtsam, "SubgraphSolver")
        .def(py::init<const gtsam::GaussianFactorGraph&, const gtsam::SubgraphSolverParameters&, const gtsam::Ordering&>(), py::arg("A"), py::arg("parameters"), py::arg("ordering"))
        .def(py::init<const gtsam::GaussianFactorGraph&, const gtsam::GaussianFactorGraph&, const gtsam::SubgraphSolverParameters&, const gtsam::Ordering&>(), py::arg("Ab1"), py::arg("Ab2"), py::arg("parameters"), py::arg("ordering"))
        .def("optimize",[](gtsam::SubgraphSolver* self){return self->optimize();});

    py::class_<gtsam::KalmanFilter, std::shared_ptr<gtsam::KalmanFilter>>(m_gtsam, "KalmanFilter")
        .def(py::init< size_t>(), py::arg("n"))
        .def("init",[](gtsam::KalmanFilter* self,const gtsam::Vector& x0,const gtsam::Matrix& P0){return self->init(x0, P0);}, py::arg("x0"), py::arg("P0"))
        .def("print_",[](gtsam::KalmanFilter* self, string s){ self->print(s);}, py::arg("s"))
        .def("predict",[](gtsam::KalmanFilter* self,const std::shared_ptr<gtsam::GaussianDensity>& p,const gtsam::Matrix& F,const gtsam::Matrix& B,const gtsam::Vector& u,const std::shared_ptr<gtsam::noiseModel::Diagonal>& modelQ){return self->predict(p, F, B, u, modelQ);}, py::arg("p"), py::arg("F"), py::arg("B"), py::arg("u"), py::arg("modelQ"))
        .def("predictQ",[](gtsam::KalmanFilter* self,const std::shared_ptr<gtsam::GaussianDensity>& p,const gtsam::Matrix& F,const gtsam::Matrix& B,const gtsam::Vector& u,const gtsam::Matrix& Q){return self->predictQ(p, F, B, u, Q);}, py::arg("p"), py::arg("F"), py::arg("B"), py::arg("u"), py::arg("Q"))
        .def("predict2",[](gtsam::KalmanFilter* self,const std::shared_ptr<gtsam::GaussianDensity>& p,const gtsam::Matrix& A0,const gtsam::Matrix& A1,const gtsam::Vector& b,const std::shared_ptr<gtsam::noiseModel::Diagonal>& model){return self->predict2(p, A0, A1, b, model);}, py::arg("p"), py::arg("A0"), py::arg("A1"), py::arg("b"), py::arg("model"))
        .def("update",[](gtsam::KalmanFilter* self,const std::shared_ptr<gtsam::GaussianDensity>& p,const gtsam::Matrix& H,const gtsam::Vector& z,const std::shared_ptr<gtsam::noiseModel::Diagonal>& model){return self->update(p, H, z, model);}, py::arg("p"), py::arg("H"), py::arg("z"), py::arg("model"))
        .def("updateQ",[](gtsam::KalmanFilter* self,const std::shared_ptr<gtsam::GaussianDensity>& p,const gtsam::Matrix& H,const gtsam::Vector& z,const gtsam::Matrix& Q){return self->updateQ(p, H, z, Q);}, py::arg("p"), py::arg("H"), py::arg("z"), py::arg("Q"))
        .def_static("step",[](const std::shared_ptr<gtsam::GaussianDensity>& p){return gtsam::KalmanFilter::step(p);}, py::arg("p"));

    py::class_<gtsam::LabeledSymbol, std::shared_ptr<gtsam::LabeledSymbol>>(m_gtsam, "LabeledSymbol")
        .def(py::init< size_t>(), py::arg("full_key"))
        .def(py::init<const gtsam::LabeledSymbol&>(), py::arg("key"))
        .def(py::init< unsigned char,  unsigned char,  size_t>(), py::arg("valType"), py::arg("label"), py::arg("j"))
        .def("key",[](gtsam::LabeledSymbol* self){return self->key();})
        .def("label",[](gtsam::LabeledSymbol* self){return self->label();})
        .def("chr",[](gtsam::LabeledSymbol* self){return self->chr();})
        .def("index",[](gtsam::LabeledSymbol* self){return self->index();})
        .def("upper",[](gtsam::LabeledSymbol* self){return self->upper();})
        .def("lower",[](gtsam::LabeledSymbol* self){return self->lower();})
        .def("newChr",[](gtsam::LabeledSymbol* self, unsigned char c){return self->newChr(c);}, py::arg("c"))
        .def("newLabel",[](gtsam::LabeledSymbol* self, unsigned char label){return self->newLabel(label);}, py::arg("label"))
        .def("print_",[](gtsam::LabeledSymbol* self, string s){ self->print(s);}, py::arg("s"));

    py::class_<gtsam::Ordering, std::shared_ptr<gtsam::Ordering>>(m_gtsam, "Ordering")
        .def(py::init<>())
        .def(py::init<const gtsam::Ordering&>(), py::arg("other"))
        .def("print_",[](gtsam::Ordering* self, string s){ self->print(s);}, py::arg("s"))
        .def("equals",[](gtsam::Ordering* self,const gtsam::Ordering& ord, double tol){return self->equals(ord, tol);}, py::arg("ord"), py::arg("tol"))
        .def("size",[](gtsam::Ordering* self){return self->size();})
        .def("at",[](gtsam::Ordering* self, size_t key){return self->at(key);}, py::arg("key"))
        .def("push_back",[](gtsam::Ordering* self, size_t key){ self->push_back(key);}, py::arg("key"));

    py::class_<gtsam::NonlinearFactorGraph, std::shared_ptr<gtsam::NonlinearFactorGraph>>(m_gtsam, "NonlinearFactorGraph")
        .def(py::init<>())
        .def(py::init<const gtsam::NonlinearFactorGraph&>(), py::arg("graph"))
        .def("print_",[](gtsam::NonlinearFactorGraph* self, string s){ self->print(s);}, py::arg("s"))
        .def("equals",[](gtsam::NonlinearFactorGraph* self,const gtsam::NonlinearFactorGraph& fg, double tol){return self->equals(fg, tol);}, py::arg("fg"), py::arg("tol"))
        .def("size",[](gtsam::NonlinearFactorGraph* self){return self->size();})
        .def("empty",[](gtsam::NonlinearFactorGraph* self){return self->empty();})
        .def("remove",[](gtsam::NonlinearFactorGraph* self, size_t i){ self->remove(i);}, py::arg("i"))
        .def("replace",[](gtsam::NonlinearFactorGraph* self, size_t i,const std::shared_ptr<gtsam::NonlinearFactor>& factors){ self->replace(i, factors);}, py::arg("i"), py::arg("factors"))
        .def("resize",[](gtsam::NonlinearFactorGraph* self, size_t size){ self->resize(size);}, py::arg("size"))
        .def("nrFactors",[](gtsam::NonlinearFactorGraph* self){return self->nrFactors();})
        .def("at",[](gtsam::NonlinearFactorGraph* self, size_t idx){return self->at(idx);}, py::arg("idx"))
        .def("push_back",[](gtsam::NonlinearFactorGraph* self,const gtsam::NonlinearFactorGraph& factors){ self->push_back(factors);}, py::arg("factors"))
        .def("push_back",[](gtsam::NonlinearFactorGraph* self,const std::shared_ptr<gtsam::NonlinearFactor>& factor){ self->push_back(factor);}, py::arg("factor"))
        .def("add",[](gtsam::NonlinearFactorGraph* self,const std::shared_ptr<gtsam::NonlinearFactor>& factor){ self->add(factor);}, py::arg("factor"))
        .def("exists",[](gtsam::NonlinearFactorGraph* self, size_t idx){return self->exists(idx);}, py::arg("idx"))
        .def("keys",[](gtsam::NonlinearFactorGraph* self){return self->keys();})
        .def("keyVector",[](gtsam::NonlinearFactorGraph* self){return self->keyVector();})
        .def("error",[](gtsam::NonlinearFactorGraph* self,const gtsam::Values& values){return self->error(values);}, py::arg("values"))
        .def("probPrime",[](gtsam::NonlinearFactorGraph* self,const gtsam::Values& values){return self->probPrime(values);}, py::arg("values"))
        .def("orderingCOLAMD",[](gtsam::NonlinearFactorGraph* self){return self->orderingCOLAMD();})
        .def("linearize",[](gtsam::NonlinearFactorGraph* self,const gtsam::Values& values){return self->linearize(values);}, py::arg("values"))
        .def("clone",[](gtsam::NonlinearFactorGraph* self){return self->clone();});

    py::class_<gtsam::NonlinearFactor, std::shared_ptr<gtsam::NonlinearFactor>>(m_gtsam, "NonlinearFactor")
        .def("size",[](gtsam::NonlinearFactor* self){return self->size();})
        .def("keys",[](gtsam::NonlinearFactor* self){return self->keys();})
        .def("print_",[](gtsam::NonlinearFactor* self, string s){ self->print(s);}, py::arg("s"))
        .def("printKeys",[](gtsam::NonlinearFactor* self, string s){ self->printKeys(s);}, py::arg("s"))
        .def("equals",[](gtsam::NonlinearFactor* self,const gtsam::NonlinearFactor& other, double tol){return self->equals(other, tol);}, py::arg("other"), py::arg("tol"))
        .def("error",[](gtsam::NonlinearFactor* self,const gtsam::Values& c){return self->error(c);}, py::arg("c"))
        .def("dim",[](gtsam::NonlinearFactor* self){return self->dim();})
        .def("active",[](gtsam::NonlinearFactor* self,const gtsam::Values& c){return self->active(c);}, py::arg("c"))
        .def("linearize",[](gtsam::NonlinearFactor* self,const gtsam::Values& c){return self->linearize(c);}, py::arg("c"))
        .def("clone",[](gtsam::NonlinearFactor* self){return self->clone();});

    py::class_<gtsam::NoiseModelFactor, gtsam::NonlinearFactor, std::shared_ptr<gtsam::NoiseModelFactor>>(m_gtsam, "NoiseModelFactor")
        .def("equals",[](gtsam::NoiseModelFactor* self,const gtsam::NoiseModelFactor& other, double tol){return self->equals(other, tol);}, py::arg("other"), py::arg("tol"))
        .def("get_noiseModel",[](gtsam::NoiseModelFactor* self){return self->get_noiseModel();})
        .def("noiseModel",[](gtsam::NoiseModelFactor* self){return self->noiseModel();})
        .def("unwhitenedError",[](gtsam::NoiseModelFactor* self,const gtsam::Values& x){return self->unwhitenedError(x);}, py::arg("x"))
        .def("whitenedError",[](gtsam::NoiseModelFactor* self,const gtsam::Values& x){return self->whitenedError(x);}, py::arg("x"));

    py::class_<gtsam::Values, std::shared_ptr<gtsam::Values>>(m_gtsam, "Values")
        .def(py::init<>())
        .def(py::init<const gtsam::Values&>(), py::arg("other"))
        .def("size",[](gtsam::Values* self){return self->size();})
        .def("empty",[](gtsam::Values* self){return self->empty();})
        .def("clear",[](gtsam::Values* self){ self->clear();})
        .def("dim",[](gtsam::Values* self){return self->dim();})
        .def("print_",[](gtsam::Values* self, string s){ self->print(s);}, py::arg("s"))
        .def("equals",[](gtsam::Values* self,const gtsam::Values& other, double tol){return self->equals(other, tol);}, py::arg("other"), py::arg("tol"))
        .def("insert",[](gtsam::Values* self,const gtsam::Values& values){ self->insert(values);}, py::arg("values"))
        .def("update",[](gtsam::Values* self,const gtsam::Values& values){ self->update(values);}, py::arg("values"))
        .def("erase",[](gtsam::Values* self, size_t j){ self->erase(j);}, py::arg("j"))
        .def("swap",[](gtsam::Values* self, gtsam::Values& values){ self->swap(values);}, py::arg("values"))
        .def("exists",[](gtsam::Values* self, size_t j){return self->exists(j);}, py::arg("j"))
        .def("keys",[](gtsam::Values* self){return self->keys();})
        .def("zeroVectors",[](gtsam::Values* self){return self->zeroVectors();})
        .def("retract",[](gtsam::Values* self,const gtsam::VectorValues& delta){return self->retract(delta);}, py::arg("delta"))
        .def("localCoordinates",[](gtsam::Values* self,const gtsam::Values& cp){return self->localCoordinates(cp);}, py::arg("cp"))
        .def("insert",[](gtsam::Values* self, size_t j,const gtsam::Point2& point2){ self->insert(j, point2);}, py::arg("j"), py::arg("point2"))
        .def("insert",[](gtsam::Values* self, size_t j,const gtsam::Point3& point3){ self->insert(j, point3);}, py::arg("j"), py::arg("point3"))
        .def("insert",[](gtsam::Values* self, size_t j,const gtsam::Rot2& rot2){ self->insert(j, rot2);}, py::arg("j"), py::arg("rot2"))
        .def("insert",[](gtsam::Values* self, size_t j,const gtsam::Pose2& pose2){ self->insert(j, pose2);}, py::arg("j"), py::arg("pose2"))
        .def("insert",[](gtsam::Values* self, size_t j,const gtsam::Rot3& rot3){ self->insert(j, rot3);}, py::arg("j"), py::arg("rot3"))
        .def("insert",[](gtsam::Values* self, size_t j,const gtsam::Pose3& pose3){ self->insert(j, pose3);}, py::arg("j"), py::arg("pose3"))
        .def("insert",[](gtsam::Values* self, size_t j,const gtsam::Cal3_S2& cal3_s2){ self->insert(j, cal3_s2);}, py::arg("j"), py::arg("cal3_s2"))
        .def("insert",[](gtsam::Values* self, size_t j,const gtsam::Cal3DS2& cal3ds2){ self->insert(j, cal3ds2);}, py::arg("j"), py::arg("cal3ds2"))
        .def("insert",[](gtsam::Values* self, size_t j,const gtsam::Cal3Bundler& cal3bundler){ self->insert(j, cal3bundler);}, py::arg("j"), py::arg("cal3bundler"))
        .def("insert",[](gtsam::Values* self, size_t j,const gtsam::EssentialMatrix& essential_matrix){ self->insert(j, essential_matrix);}, py::arg("j"), py::arg("essential_matrix"))
        .def("insert",[](gtsam::Values* self, size_t j,const gtsam::SimpleCamera& simpel_camera){ self->insert(j, simpel_camera);}, py::arg("j"), py::arg("simpel_camera"))
        .def("insert",[](gtsam::Values* self, size_t j,const gtsam::imuBias::ConstantBias& constant_bias){ self->insert(j, constant_bias);}, py::arg("j"), py::arg("constant_bias"))
        .def("insert",[](gtsam::Values* self, size_t j,const gtsam::Vector& vector){ self->insert(j, vector);}, py::arg("j"), py::arg("vector"))
        .def("insert",[](gtsam::Values* self, size_t j,const gtsam::Matrix& matrix){ self->insert(j, matrix);}, py::arg("j"), py::arg("matrix"))
        .def("update",[](gtsam::Values* self, size_t j,const gtsam::Point2& point2){ self->update(j, point2);}, py::arg("j"), py::arg("point2"))
        .def("update",[](gtsam::Values* self, size_t j,const gtsam::Point3& point3){ self->update(j, point3);}, py::arg("j"), py::arg("point3"))
        .def("update",[](gtsam::Values* self, size_t j,const gtsam::Rot2& rot2){ self->update(j, rot2);}, py::arg("j"), py::arg("rot2"))
        .def("update",[](gtsam::Values* self, size_t j,const gtsam::Pose2& pose2){ self->update(j, pose2);}, py::arg("j"), py::arg("pose2"))
        .def("update",[](gtsam::Values* self, size_t j,const gtsam::Rot3& rot3){ self->update(j, rot3);}, py::arg("j"), py::arg("rot3"))
        .def("update",[](gtsam::Values* self, size_t j,const gtsam::Pose3& pose3){ self->update(j, pose3);}, py::arg("j"), py::arg("pose3"))
        .def("update",[](gtsam::Values* self, size_t j,const gtsam::Cal3_S2& cal3_s2){ self->update(j, cal3_s2);}, py::arg("j"), py::arg("cal3_s2"))
        .def("update",[](gtsam::Values* self, size_t j,const gtsam::Cal3DS2& cal3ds2){ self->update(j, cal3ds2);}, py::arg("j"), py::arg("cal3ds2"))
        .def("update",[](gtsam::Values* self, size_t j,const gtsam::Cal3Bundler& cal3bundler){ self->update(j, cal3bundler);}, py::arg("j"), py::arg("cal3bundler"))
        .def("update",[](gtsam::Values* self, size_t j,const gtsam::EssentialMatrix& essential_matrix){ self->update(j, essential_matrix);}, py::arg("j"), py::arg("essential_matrix"))
        .def("update",[](gtsam::Values* self, size_t j,const gtsam::imuBias::ConstantBias& constant_bias){ self->update(j, constant_bias);}, py::arg("j"), py::arg("constant_bias"))
        .def("update",[](gtsam::Values* self, size_t j,const gtsam::Vector& vector){ self->update(j, vector);}, py::arg("j"), py::arg("vector"))
        .def("update",[](gtsam::Values* self, size_t j,const gtsam::Matrix& matrix){ self->update(j, matrix);}, py::arg("j"), py::arg("matrix"))
        .def("atPoint2",[](gtsam::Values* self, size_t j){return self->at<gtsam::Point2>(j);}, py::arg("j"))
        .def("atPoint3",[](gtsam::Values* self, size_t j){return self->at<gtsam::Point3>(j);}, py::arg("j"))
        .def("atRot2",[](gtsam::Values* self, size_t j){return self->at<gtsam::Rot2>(j);}, py::arg("j"))
        .def("atPose2",[](gtsam::Values* self, size_t j){return self->at<gtsam::Pose2>(j);}, py::arg("j"))
        .def("atRot3",[](gtsam::Values* self, size_t j){return self->at<gtsam::Rot3>(j);}, py::arg("j"))
        .def("atPose3",[](gtsam::Values* self, size_t j){return self->at<gtsam::Pose3>(j);}, py::arg("j"))
        .def("atCal3_S2",[](gtsam::Values* self, size_t j){return self->at<gtsam::Cal3_S2>(j);}, py::arg("j"))
        .def("atCal3DS2",[](gtsam::Values* self, size_t j){return self->at<gtsam::Cal3DS2>(j);}, py::arg("j"))
        .def("atCal3Bundler",[](gtsam::Values* self, size_t j){return self->at<gtsam::Cal3Bundler>(j);}, py::arg("j"))
        .def("atEssentialMatrix",[](gtsam::Values* self, size_t j){return self->at<gtsam::EssentialMatrix>(j);}, py::arg("j"))
        .def("atConstantBias",[](gtsam::Values* self, size_t j){return self->at<gtsam::imuBias::ConstantBias>(j);}, py::arg("j"))
        .def("atVector",[](gtsam::Values* self, size_t j){return self->at<gtsam::Vector>(j);}, py::arg("j"))
        .def("atMatrix",[](gtsam::Values* self, size_t j){return self->at<gtsam::Matrix>(j);}, py::arg("j"))
        .def("insertDouble",[](gtsam::Values* self, size_t j, double c){ self->insertDouble(j, c);}, py::arg("j"), py::arg("c"))
        .def("atDouble",[](gtsam::Values* self, size_t j){return self->atDouble(j);}, py::arg("j"));

    py::class_<gtsam::Marginals, std::shared_ptr<gtsam::Marginals>>(m_gtsam, "Marginals")
        .def(py::init<const gtsam::NonlinearFactorGraph&, const gtsam::Values&>(), py::arg("graph"), py::arg("solution"))
        .def("print_",[](gtsam::Marginals* self, string s){ self->print(s);}, py::arg("s"))
        .def("marginalCovariance",[](gtsam::Marginals* self, size_t variable){return self->marginalCovariance(variable);}, py::arg("variable"))
        .def("marginalInformation",[](gtsam::Marginals* self, size_t variable){return self->marginalInformation(variable);}, py::arg("variable"))
        .def("jointMarginalCovariance",[](gtsam::Marginals* self,const gtsam::KeyVector& variables){return self->jointMarginalCovariance(variables);}, py::arg("variables"))
        .def("jointMarginalInformation",[](gtsam::Marginals* self,const gtsam::KeyVector& variables){return self->jointMarginalInformation(variables);}, py::arg("variables"));

    py::class_<gtsam::JointMarginal, std::shared_ptr<gtsam::JointMarginal>>(m_gtsam, "JointMarginal")
        .def("at",[](gtsam::JointMarginal* self, size_t iVariable, size_t jVariable){return self->at(iVariable, jVariable);}, py::arg("iVariable"), py::arg("jVariable"))
        .def("fullMatrix",[](gtsam::JointMarginal* self){return self->fullMatrix();})
        .def("print_",[](gtsam::JointMarginal* self, string s){ self->print(s);}, py::arg("s"))
        .def("print_",[](gtsam::JointMarginal* self){ self->print();});

    py::class_<gtsam::LinearContainerFactor, gtsam::NonlinearFactor, std::shared_ptr<gtsam::LinearContainerFactor>>(m_gtsam, "LinearContainerFactor")
        .def(py::init<const std::shared_ptr<gtsam::GaussianFactor>&, const gtsam::Values&>(), py::arg("factor"), py::arg("linearizationPoint"))
        .def(py::init<const std::shared_ptr<gtsam::GaussianFactor>&>(), py::arg("factor"))
        .def("factor",[](gtsam::LinearContainerFactor* self){return self->factor();})
        .def("isJacobian",[](gtsam::LinearContainerFactor* self){return self->isJacobian();})
        .def("toJacobian",[](gtsam::LinearContainerFactor* self){return self->toJacobian();})
        .def("toHessian",[](gtsam::LinearContainerFactor* self){return self->toHessian();})
        .def_static("ConvertLinearGraph",[](const gtsam::GaussianFactorGraph& linear_graph,const gtsam::Values& linearizationPoint){return gtsam::LinearContainerFactor::ConvertLinearGraph(linear_graph, linearizationPoint);}, py::arg("linear_graph"), py::arg("linearizationPoint"))
        .def_static("ConvertLinearGraph",[](const gtsam::GaussianFactorGraph& linear_graph){return gtsam::LinearContainerFactor::ConvertLinearGraph(linear_graph);}, py::arg("linear_graph"));

    py::class_<gtsam::NonlinearOptimizerParams, std::shared_ptr<gtsam::NonlinearOptimizerParams>>(m_gtsam, "NonlinearOptimizerParams")
        .def(py::init<>())
        .def("print_",[](gtsam::NonlinearOptimizerParams* self, string s){ self->print(s);}, py::arg("s"))
        .def("getMaxIterations",[](gtsam::NonlinearOptimizerParams* self){return self->getMaxIterations();})
        .def("getRelativeErrorTol",[](gtsam::NonlinearOptimizerParams* self){return self->getRelativeErrorTol();})
        .def("getAbsoluteErrorTol",[](gtsam::NonlinearOptimizerParams* self){return self->getAbsoluteErrorTol();})
        .def("getErrorTol",[](gtsam::NonlinearOptimizerParams* self){return self->getErrorTol();})
        .def("getVerbosity",[](gtsam::NonlinearOptimizerParams* self){return self->getVerbosity();})
        .def("setMaxIterations",[](gtsam::NonlinearOptimizerParams* self, int value){ self->setMaxIterations(value);}, py::arg("value"))
        .def("setRelativeErrorTol",[](gtsam::NonlinearOptimizerParams* self, double value){ self->setRelativeErrorTol(value);}, py::arg("value"))
        .def("setAbsoluteErrorTol",[](gtsam::NonlinearOptimizerParams* self, double value){ self->setAbsoluteErrorTol(value);}, py::arg("value"))
        .def("setErrorTol",[](gtsam::NonlinearOptimizerParams* self, double value){ self->setErrorTol(value);}, py::arg("value"))
        .def("setVerbosity",[](gtsam::NonlinearOptimizerParams* self, string s){ self->setVerbosity(s);}, py::arg("s"))
        .def("getLinearSolverType",[](gtsam::NonlinearOptimizerParams* self){return self->getLinearSolverType();})
        .def("setLinearSolverType",[](gtsam::NonlinearOptimizerParams* self, string solver){ self->setLinearSolverType(solver);}, py::arg("solver"))
        .def("setOrdering",[](gtsam::NonlinearOptimizerParams* self,const gtsam::Ordering& ordering){ self->setOrdering(ordering);}, py::arg("ordering"))
        .def("setIterativeParams",[](gtsam::NonlinearOptimizerParams* self,const std::shared_ptr<gtsam::IterativeOptimizationParameters>& params){ self->setIterativeParams(params);}, py::arg("params"))
        .def("isMultifrontal",[](gtsam::NonlinearOptimizerParams* self){return self->isMultifrontal();})
        .def("isSequential",[](gtsam::NonlinearOptimizerParams* self){return self->isSequential();})
        .def("isCholmod",[](gtsam::NonlinearOptimizerParams* self){return self->isCholmod();})
        .def("isIterative",[](gtsam::NonlinearOptimizerParams* self){return self->isIterative();});

    py::class_<gtsam::GaussNewtonParams, gtsam::NonlinearOptimizerParams, std::shared_ptr<gtsam::GaussNewtonParams>>(m_gtsam, "GaussNewtonParams")
        .def(py::init<>());

    py::class_<gtsam::LevenbergMarquardtParams, gtsam::NonlinearOptimizerParams, std::shared_ptr<gtsam::LevenbergMarquardtParams>>(m_gtsam, "LevenbergMarquardtParams")
        .def(py::init<>())
        .def("getlambdaInitial",[](gtsam::LevenbergMarquardtParams* self){return self->getlambdaInitial();})
        .def("getlambdaFactor",[](gtsam::LevenbergMarquardtParams* self){return self->getlambdaFactor();})
        .def("getlambdaUpperBound",[](gtsam::LevenbergMarquardtParams* self){return self->getlambdaUpperBound();})
        .def("getVerbosityLM",[](gtsam::LevenbergMarquardtParams* self){return self->getVerbosityLM();})
        .def("setlambdaInitial",[](gtsam::LevenbergMarquardtParams* self, double value){ self->setlambdaInitial(value);}, py::arg("value"))
        .def("setlambdaFactor",[](gtsam::LevenbergMarquardtParams* self, double value){ self->setlambdaFactor(value);}, py::arg("value"))
        .def("setlambdaUpperBound",[](gtsam::LevenbergMarquardtParams* self, double value){ self->setlambdaUpperBound(value);}, py::arg("value"))
        .def("setVerbosityLM",[](gtsam::LevenbergMarquardtParams* self, string s){ self->setVerbosityLM(s);}, py::arg("s"));

    py::class_<gtsam::DoglegParams, gtsam::NonlinearOptimizerParams, std::shared_ptr<gtsam::DoglegParams>>(m_gtsam, "DoglegParams")
        .def(py::init<>())
        .def("getDeltaInitial",[](gtsam::DoglegParams* self){return self->getDeltaInitial();})
        .def("getVerbosityDL",[](gtsam::DoglegParams* self){return self->getVerbosityDL();})
        .def("setDeltaInitial",[](gtsam::DoglegParams* self, double deltaInitial){ self->setDeltaInitial(deltaInitial);}, py::arg("deltaInitial"))
        .def("setVerbosityDL",[](gtsam::DoglegParams* self, string verbosityDL){ self->setVerbosityDL(verbosityDL);}, py::arg("verbosityDL"));

    py::class_<gtsam::NonlinearOptimizer, std::shared_ptr<gtsam::NonlinearOptimizer>>(m_gtsam, "NonlinearOptimizer")
        .def("optimize",[](gtsam::NonlinearOptimizer* self){return self->optimize();})
        .def("optimizeSafely",[](gtsam::NonlinearOptimizer* self){return self->optimizeSafely();})
        .def("error",[](gtsam::NonlinearOptimizer* self){return self->error();})
        .def("iterations",[](gtsam::NonlinearOptimizer* self){return self->iterations();})
        .def("values",[](gtsam::NonlinearOptimizer* self){return self->values();})
        .def("iterate",[](gtsam::NonlinearOptimizer* self){ self->iterate();});

    py::class_<gtsam::GaussNewtonOptimizer, gtsam::NonlinearOptimizer, std::shared_ptr<gtsam::GaussNewtonOptimizer>>(m_gtsam, "GaussNewtonOptimizer")
        .def(py::init<const gtsam::NonlinearFactorGraph&, const gtsam::Values&>(), py::arg("graph"), py::arg("initialValues"))
        .def(py::init<const gtsam::NonlinearFactorGraph&, const gtsam::Values&, const gtsam::GaussNewtonParams&>(), py::arg("graph"), py::arg("initialValues"), py::arg("params"));

    py::class_<gtsam::DoglegOptimizer, gtsam::NonlinearOptimizer, std::shared_ptr<gtsam::DoglegOptimizer>>(m_gtsam, "DoglegOptimizer")
        .def(py::init<const gtsam::NonlinearFactorGraph&, const gtsam::Values&>(), py::arg("graph"), py::arg("initialValues"))
        .def(py::init<const gtsam::NonlinearFactorGraph&, const gtsam::Values&, const gtsam::DoglegParams&>(), py::arg("graph"), py::arg("initialValues"), py::arg("params"))
        .def("getDelta",[](gtsam::DoglegOptimizer* self){return self->getDelta();});

    py::class_<gtsam::LevenbergMarquardtOptimizer, gtsam::NonlinearOptimizer, std::shared_ptr<gtsam::LevenbergMarquardtOptimizer>>(m_gtsam, "LevenbergMarquardtOptimizer")
        .def(py::init<const gtsam::NonlinearFactorGraph&, const gtsam::Values&>(), py::arg("graph"), py::arg("initialValues"))
        .def(py::init<const gtsam::NonlinearFactorGraph&, const gtsam::Values&, const gtsam::LevenbergMarquardtParams&>(), py::arg("graph"), py::arg("initialValues"), py::arg("params"))
        .def("lambda",[](gtsam::LevenbergMarquardtOptimizer* self){return self->lambda();})
        .def("print_",[](gtsam::LevenbergMarquardtOptimizer* self, string str){ self->print(str);}, py::arg("str"));

    py::class_<gtsam::ISAM2GaussNewtonParams, std::shared_ptr<gtsam::ISAM2GaussNewtonParams>>(m_gtsam, "ISAM2GaussNewtonParams")
        .def(py::init<>())
        .def("print_",[](gtsam::ISAM2GaussNewtonParams* self, string str){ self->print(str);}, py::arg("str"))
        .def("getWildfireThreshold",[](gtsam::ISAM2GaussNewtonParams* self){return self->getWildfireThreshold();})
        .def("setWildfireThreshold",[](gtsam::ISAM2GaussNewtonParams* self, double wildfireThreshold){ self->setWildfireThreshold(wildfireThreshold);}, py::arg("wildfireThreshold"));

    py::class_<gtsam::ISAM2DoglegParams, std::shared_ptr<gtsam::ISAM2DoglegParams>>(m_gtsam, "ISAM2DoglegParams")
        .def(py::init<>())
        .def("print_",[](gtsam::ISAM2DoglegParams* self, string str){ self->print(str);}, py::arg("str"))
        .def("getWildfireThreshold",[](gtsam::ISAM2DoglegParams* self){return self->getWildfireThreshold();})
        .def("setWildfireThreshold",[](gtsam::ISAM2DoglegParams* self, double wildfireThreshold){ self->setWildfireThreshold(wildfireThreshold);}, py::arg("wildfireThreshold"))
        .def("getInitialDelta",[](gtsam::ISAM2DoglegParams* self){return self->getInitialDelta();})
        .def("setInitialDelta",[](gtsam::ISAM2DoglegParams* self, double initialDelta){ self->setInitialDelta(initialDelta);}, py::arg("initialDelta"))
        .def("getAdaptationMode",[](gtsam::ISAM2DoglegParams* self){return self->getAdaptationMode();})
        .def("setAdaptationMode",[](gtsam::ISAM2DoglegParams* self, string adaptationMode){ self->setAdaptationMode(adaptationMode);}, py::arg("adaptationMode"))
        .def("isVerbose",[](gtsam::ISAM2DoglegParams* self){return self->isVerbose();})
        .def("setVerbose",[](gtsam::ISAM2DoglegParams* self, bool verbose){ self->setVerbose(verbose);}, py::arg("verbose"));

    py::class_<gtsam::ISAM2ThresholdMapValue, std::shared_ptr<gtsam::ISAM2ThresholdMapValue>>(m_gtsam, "ISAM2ThresholdMapValue")
        .def(py::init< char, const gtsam::Vector&>(), py::arg("c"), py::arg("thresholds"))
        .def(py::init<const gtsam::ISAM2ThresholdMapValue&>(), py::arg("other"));

    py::class_<gtsam::ISAM2ThresholdMap, std::shared_ptr<gtsam::ISAM2ThresholdMap>>(m_gtsam, "ISAM2ThresholdMap")
        .def(py::init<>())
        .def(py::init<const gtsam::ISAM2ThresholdMap&>(), py::arg("other"))
        .def("size",[](gtsam::ISAM2ThresholdMap* self){return self->size();})
        .def("empty",[](gtsam::ISAM2ThresholdMap* self){return self->empty();})
        .def("clear",[](gtsam::ISAM2ThresholdMap* self){ self->clear();})
        .def("insert",[](gtsam::ISAM2ThresholdMap* self,const gtsam::ISAM2ThresholdMapValue& value){ self->insert(value);}, py::arg("value"));

    py::class_<gtsam::ISAM2Params, std::shared_ptr<gtsam::ISAM2Params>>(m_gtsam, "ISAM2Params")
        .def(py::init<>())
        .def("print_",[](gtsam::ISAM2Params* self, string str){ self->print(str);}, py::arg("str"))
        .def("setOptimizationParams",[](gtsam::ISAM2Params* self,const gtsam::ISAM2GaussNewtonParams& gauss_newton__params){ self->setOptimizationParams(gauss_newton__params);}, py::arg("gauss_newton__params"))
        .def("setOptimizationParams",[](gtsam::ISAM2Params* self,const gtsam::ISAM2DoglegParams& dogleg_params){ self->setOptimizationParams(dogleg_params);}, py::arg("dogleg_params"))
        .def("setRelinearizeThreshold",[](gtsam::ISAM2Params* self, double threshold){ self->setRelinearizeThreshold(threshold);}, py::arg("threshold"))
        .def("setRelinearizeThreshold",[](gtsam::ISAM2Params* self,const gtsam::ISAM2ThresholdMap& threshold_map){ self->setRelinearizeThreshold(threshold_map);}, py::arg("threshold_map"))
        .def("getRelinearizeSkip",[](gtsam::ISAM2Params* self){return self->getRelinearizeSkip();})
        .def("setRelinearizeSkip",[](gtsam::ISAM2Params* self, int relinearizeSkip){ self->setRelinearizeSkip(relinearizeSkip);}, py::arg("relinearizeSkip"))
        .def("isEnableRelinearization",[](gtsam::ISAM2Params* self){return self->isEnableRelinearization();})
        .def("setEnableRelinearization",[](gtsam::ISAM2Params* self, bool enableRelinearization){ self->setEnableRelinearization(enableRelinearization);}, py::arg("enableRelinearization"))
        .def("isEvaluateNonlinearError",[](gtsam::ISAM2Params* self){return self->isEvaluateNonlinearError();})
        .def("setEvaluateNonlinearError",[](gtsam::ISAM2Params* self, bool evaluateNonlinearError){ self->setEvaluateNonlinearError(evaluateNonlinearError);}, py::arg("evaluateNonlinearError"))
        .def("getFactorization",[](gtsam::ISAM2Params* self){return self->getFactorization();})
        .def("setFactorization",[](gtsam::ISAM2Params* self, string factorization){ self->setFactorization(factorization);}, py::arg("factorization"))
        .def("isCacheLinearizedFactors",[](gtsam::ISAM2Params* self){return self->isCacheLinearizedFactors();})
        .def("setCacheLinearizedFactors",[](gtsam::ISAM2Params* self, bool cacheLinearizedFactors){ self->setCacheLinearizedFactors(cacheLinearizedFactors);}, py::arg("cacheLinearizedFactors"))
        .def("isEnableDetailedResults",[](gtsam::ISAM2Params* self){return self->isEnableDetailedResults();})
        .def("setEnableDetailedResults",[](gtsam::ISAM2Params* self, bool enableDetailedResults){ self->setEnableDetailedResults(enableDetailedResults);}, py::arg("enableDetailedResults"))
        .def("isEnablePartialRelinearizationCheck",[](gtsam::ISAM2Params* self){return self->isEnablePartialRelinearizationCheck();})
        .def("setEnablePartialRelinearizationCheck",[](gtsam::ISAM2Params* self, bool enablePartialRelinearizationCheck){ self->setEnablePartialRelinearizationCheck(enablePartialRelinearizationCheck);}, py::arg("enablePartialRelinearizationCheck"));

    py::class_<gtsam::ISAM2Clique, std::shared_ptr<gtsam::ISAM2Clique>>(m_gtsam, "ISAM2Clique")
        .def(py::init<>())
        .def("gradientContribution",[](gtsam::ISAM2Clique* self){return self->gradientContribution();})
        .def("print_",[](gtsam::ISAM2Clique* self, string s){ self->print(s);}, py::arg("s"));

    py::class_<gtsam::ISAM2Result, std::shared_ptr<gtsam::ISAM2Result>>(m_gtsam, "ISAM2Result")
        .def(py::init<>())
        .def("print_",[](gtsam::ISAM2Result* self, string str){ self->print(str);}, py::arg("str"))
        .def("getVariablesRelinearized",[](gtsam::ISAM2Result* self){return self->getVariablesRelinearized();})
        .def("getVariablesReeliminated",[](gtsam::ISAM2Result* self){return self->getVariablesReeliminated();})
        .def("getCliques",[](gtsam::ISAM2Result* self){return self->getCliques();});

    py::class_<gtsam::FactorIndices, std::shared_ptr<gtsam::FactorIndices>>(m_gtsam, "FactorIndices");

    py::class_<gtsam::ISAM2, std::shared_ptr<gtsam::ISAM2>>(m_gtsam, "ISAM2")
        .def(py::init<>())
        .def(py::init<const gtsam::ISAM2Params&>(), py::arg("params"))
        .def(py::init<const gtsam::ISAM2&>(), py::arg("other"))
        .def("equals",[](gtsam::ISAM2* self,const gtsam::ISAM2& other, double tol){return self->equals(other, tol);}, py::arg("other"), py::arg("tol"))
        .def("print_",[](gtsam::ISAM2* self, string s){ self->print(s);}, py::arg("s"))
        .def("printStats",[](gtsam::ISAM2* self){ self->printStats();})
        .def("saveGraph",[](gtsam::ISAM2* self, string s){ self->saveGraph(s);}, py::arg("s"))
        .def("update",[](gtsam::ISAM2* self){return self->update();})
        .def("update",[](gtsam::ISAM2* self,const gtsam::NonlinearFactorGraph& newFactors,const gtsam::Values& newTheta){return self->update(newFactors, newTheta);}, py::arg("newFactors"), py::arg("newTheta"))
        .def("update",[](gtsam::ISAM2* self,const gtsam::NonlinearFactorGraph& newFactors,const gtsam::Values& newTheta,const gtsam::FactorIndices& removeFactorIndices){return self->update(newFactors, newTheta, removeFactorIndices);}, py::arg("newFactors"), py::arg("newTheta"), py::arg("removeFactorIndices"))
        .def("update",[](gtsam::ISAM2* self,const gtsam::NonlinearFactorGraph& newFactors,const gtsam::Values& newTheta,const gtsam::FactorIndices& removeFactorIndices,const gtsam::KeyGroupMap& constrainedKeys){return self->update(newFactors, newTheta, removeFactorIndices, constrainedKeys);}, py::arg("newFactors"), py::arg("newTheta"), py::arg("removeFactorIndices"), py::arg("constrainedKeys"))
        .def("getLinearizationPoint",[](gtsam::ISAM2* self){return self->getLinearizationPoint();})
        .def("calculateEstimate",[](gtsam::ISAM2* self){return self->calculateEstimate();})
        .def("calculateEstimatePoint2",[](gtsam::ISAM2* self, size_t key){return self->calculateEstimate<gtsam::Point2>(key);}, py::arg("key"))
        .def("calculateEstimateRot2",[](gtsam::ISAM2* self, size_t key){return self->calculateEstimate<gtsam::Rot2>(key);}, py::arg("key"))
        .def("calculateEstimatePose2",[](gtsam::ISAM2* self, size_t key){return self->calculateEstimate<gtsam::Pose2>(key);}, py::arg("key"))
        .def("calculateEstimatePoint3",[](gtsam::ISAM2* self, size_t key){return self->calculateEstimate<gtsam::Point3>(key);}, py::arg("key"))
        .def("calculateEstimateRot3",[](gtsam::ISAM2* self, size_t key){return self->calculateEstimate<gtsam::Rot3>(key);}, py::arg("key"))
        .def("calculateEstimatePose3",[](gtsam::ISAM2* self, size_t key){return self->calculateEstimate<gtsam::Pose3>(key);}, py::arg("key"))
        .def("calculateEstimateCal3_S2",[](gtsam::ISAM2* self, size_t key){return self->calculateEstimate<gtsam::Cal3_S2>(key);}, py::arg("key"))
        .def("calculateEstimateCal3DS2",[](gtsam::ISAM2* self, size_t key){return self->calculateEstimate<gtsam::Cal3DS2>(key);}, py::arg("key"))
        .def("calculateEstimateCal3Bundler",[](gtsam::ISAM2* self, size_t key){return self->calculateEstimate<gtsam::Cal3Bundler>(key);}, py::arg("key"))
        .def("calculateEstimateEssentialMatrix",[](gtsam::ISAM2* self, size_t key){return self->calculateEstimate<gtsam::EssentialMatrix>(key);}, py::arg("key"))
        .def("calculateEstimateSimpleCamera",[](gtsam::ISAM2* self, size_t key){return self->calculateEstimate<gtsam::SimpleCamera>(key);}, py::arg("key"))
        .def("calculateEstimateVector",[](gtsam::ISAM2* self, size_t key){return self->calculateEstimate<gtsam::Vector>(key);}, py::arg("key"))
        .def("calculateEstimateMatrix",[](gtsam::ISAM2* self, size_t key){return self->calculateEstimate<gtsam::Matrix>(key);}, py::arg("key"))
        .def("calculateBestEstimate",[](gtsam::ISAM2* self){return self->calculateBestEstimate();})
        .def("marginalCovariance",[](gtsam::ISAM2* self, size_t key){return self->marginalCovariance(key);}, py::arg("key"))
        .def("getDelta",[](gtsam::ISAM2* self){return self->getDelta();})
        .def("getFactorsUnsafe",[](gtsam::ISAM2* self){return self->getFactorsUnsafe();})
        .def("getVariableIndex",[](gtsam::ISAM2* self){return self->getVariableIndex();})
        .def("params",[](gtsam::ISAM2* self){return self->params();});

    py::class_<gtsam::NonlinearISAM, std::shared_ptr<gtsam::NonlinearISAM>>(m_gtsam, "NonlinearISAM")
        .def(py::init<>())
        .def(py::init< int>(), py::arg("reorderInterval"))
        .def("print_",[](gtsam::NonlinearISAM* self, string s){ self->print(s);}, py::arg("s"))
        .def("printStats",[](gtsam::NonlinearISAM* self){ self->printStats();})
        .def("saveGraph",[](gtsam::NonlinearISAM* self, string s){ self->saveGraph(s);}, py::arg("s"))
        .def("estimate",[](gtsam::NonlinearISAM* self){return self->estimate();})
        .def("marginalCovariance",[](gtsam::NonlinearISAM* self, size_t key){return self->marginalCovariance(key);}, py::arg("key"))
        .def("reorderInterval",[](gtsam::NonlinearISAM* self){return self->reorderInterval();})
        .def("reorderCounter",[](gtsam::NonlinearISAM* self){return self->reorderCounter();})
        .def("update",[](gtsam::NonlinearISAM* self,const gtsam::NonlinearFactorGraph& newFactors,const gtsam::Values& initialValues){ self->update(newFactors, initialValues);}, py::arg("newFactors"), py::arg("initialValues"))
        .def("reorder_relinearize",[](gtsam::NonlinearISAM* self){ self->reorder_relinearize();})
        .def("bayesTree",[](gtsam::NonlinearISAM* self){return self->bayesTree();})
        .def("getLinearizationPoint",[](gtsam::NonlinearISAM* self){return self->getLinearizationPoint();})
        .def("getFactorsUnsafe",[](gtsam::NonlinearISAM* self){return self->getFactorsUnsafe();});

    py::class_<gtsam::PriorFactor<gtsam::Vector>, gtsam::NoiseModelFactor, std::shared_ptr<gtsam::PriorFactor<gtsam::Vector>>>(m_gtsam, "PriorFactorVector")
        .def(py::init< size_t, const gtsam::Vector&, const std::shared_ptr<gtsam::noiseModel::Base>&>(), py::arg("key"), py::arg("prior"), py::arg("noiseModel"))
        .def("prior",[](gtsam::PriorFactor<gtsam::Vector>* self){return self->prior();});

    py::class_<gtsam::PriorFactor<gtsam::Point2>, gtsam::NoiseModelFactor, std::shared_ptr<gtsam::PriorFactor<gtsam::Point2>>>(m_gtsam, "PriorFactorPoint2")
        .def(py::init< size_t, const gtsam::Point2&, const std::shared_ptr<gtsam::noiseModel::Base>&>(), py::arg("key"), py::arg("prior"), py::arg("noiseModel"))
        .def("prior",[](gtsam::PriorFactor<gtsam::Point2>* self){return self->prior();});

    py::class_<gtsam::PriorFactor<gtsam::StereoPoint2>, gtsam::NoiseModelFactor, std::shared_ptr<gtsam::PriorFactor<gtsam::StereoPoint2>>>(m_gtsam, "PriorFactorStereoPoint2")
        .def(py::init< size_t, const gtsam::StereoPoint2&, const std::shared_ptr<gtsam::noiseModel::Base>&>(), py::arg("key"), py::arg("prior"), py::arg("noiseModel"))
        .def("prior",[](gtsam::PriorFactor<gtsam::StereoPoint2>* self){return self->prior();});

    py::class_<gtsam::PriorFactor<gtsam::Point3>, gtsam::NoiseModelFactor, std::shared_ptr<gtsam::PriorFactor<gtsam::Point3>>>(m_gtsam, "PriorFactorPoint3")
        .def(py::init< size_t, const gtsam::Point3&, const std::shared_ptr<gtsam::noiseModel::Base>&>(), py::arg("key"), py::arg("prior"), py::arg("noiseModel"))
        .def("prior",[](gtsam::PriorFactor<gtsam::Point3>* self){return self->prior();});

    py::class_<gtsam::PriorFactor<gtsam::Rot2>, gtsam::NoiseModelFactor, std::shared_ptr<gtsam::PriorFactor<gtsam::Rot2>>>(m_gtsam, "PriorFactorRot2")
        .def(py::init< size_t, const gtsam::Rot2&, const std::shared_ptr<gtsam::noiseModel::Base>&>(), py::arg("key"), py::arg("prior"), py::arg("noiseModel"))
        .def("prior",[](gtsam::PriorFactor<gtsam::Rot2>* self){return self->prior();});

    py::class_<gtsam::PriorFactor<gtsam::Rot3>, gtsam::NoiseModelFactor, std::shared_ptr<gtsam::PriorFactor<gtsam::Rot3>>>(m_gtsam, "PriorFactorRot3")
        .def(py::init< size_t, const gtsam::Rot3&, const std::shared_ptr<gtsam::noiseModel::Base>&>(), py::arg("key"), py::arg("prior"), py::arg("noiseModel"))
        .def("prior",[](gtsam::PriorFactor<gtsam::Rot3>* self){return self->prior();});

    py::class_<gtsam::PriorFactor<gtsam::Pose2>, gtsam::NoiseModelFactor, std::shared_ptr<gtsam::PriorFactor<gtsam::Pose2>>>(m_gtsam, "PriorFactorPose2")
        .def(py::init< size_t, const gtsam::Pose2&, const std::shared_ptr<gtsam::noiseModel::Base>&>(), py::arg("key"), py::arg("prior"), py::arg("noiseModel"))
        .def("prior",[](gtsam::PriorFactor<gtsam::Pose2>* self){return self->prior();});

    py::class_<gtsam::PriorFactor<gtsam::Pose3>, gtsam::NoiseModelFactor, std::shared_ptr<gtsam::PriorFactor<gtsam::Pose3>>>(m_gtsam, "PriorFactorPose3")
        .def(py::init< size_t, const gtsam::Pose3&, const std::shared_ptr<gtsam::noiseModel::Base>&>(), py::arg("key"), py::arg("prior"), py::arg("noiseModel"))
        .def("prior",[](gtsam::PriorFactor<gtsam::Pose3>* self){return self->prior();});

    py::class_<gtsam::PriorFactor<gtsam::Cal3_S2>, gtsam::NoiseModelFactor, std::shared_ptr<gtsam::PriorFactor<gtsam::Cal3_S2>>>(m_gtsam, "PriorFactorCal3_S2")
        .def(py::init< size_t, const gtsam::Cal3_S2&, const std::shared_ptr<gtsam::noiseModel::Base>&>(), py::arg("key"), py::arg("prior"), py::arg("noiseModel"))
        .def("prior",[](gtsam::PriorFactor<gtsam::Cal3_S2>* self){return self->prior();});

    py::class_<gtsam::PriorFactor<gtsam::CalibratedCamera>, gtsam::NoiseModelFactor, std::shared_ptr<gtsam::PriorFactor<gtsam::CalibratedCamera>>>(m_gtsam, "PriorFactorCalibratedCamera")
        .def(py::init< size_t, const gtsam::CalibratedCamera&, const std::shared_ptr<gtsam::noiseModel::Base>&>(), py::arg("key"), py::arg("prior"), py::arg("noiseModel"))
        .def("prior",[](gtsam::PriorFactor<gtsam::CalibratedCamera>* self){return self->prior();});

    py::class_<gtsam::PriorFactor<gtsam::SimpleCamera>, gtsam::NoiseModelFactor, std::shared_ptr<gtsam::PriorFactor<gtsam::SimpleCamera>>>(m_gtsam, "PriorFactorSimpleCamera")
        .def(py::init< size_t, const gtsam::SimpleCamera&, const std::shared_ptr<gtsam::noiseModel::Base>&>(), py::arg("key"), py::arg("prior"), py::arg("noiseModel"))
        .def("prior",[](gtsam::PriorFactor<gtsam::SimpleCamera>* self){return self->prior();});

    py::class_<gtsam::PriorFactor<gtsam::imuBias::ConstantBias>, gtsam::NoiseModelFactor, std::shared_ptr<gtsam::PriorFactor<gtsam::imuBias::ConstantBias>>>(m_gtsam, "PriorFactorConstantBias")
        .def(py::init< size_t, const gtsam::imuBias::ConstantBias&, const std::shared_ptr<gtsam::noiseModel::Base>&>(), py::arg("key"), py::arg("prior"), py::arg("noiseModel"))
        .def("prior",[](gtsam::PriorFactor<gtsam::imuBias::ConstantBias>* self){return self->prior();});

    py::class_<gtsam::BetweenFactor<gtsam::Point2>, gtsam::NoiseModelFactor, std::shared_ptr<gtsam::BetweenFactor<gtsam::Point2>>>(m_gtsam, "BetweenFactorPoint2")
        .def(py::init< size_t,  size_t, const gtsam::Point2&, const std::shared_ptr<gtsam::noiseModel::Base>&>(), py::arg("key1"), py::arg("key2"), py::arg("relativePose"), py::arg("noiseModel"))
        .def("measured",[](gtsam::BetweenFactor<gtsam::Point2>* self){return self->measured();});

    py::class_<gtsam::BetweenFactor<gtsam::Point3>, gtsam::NoiseModelFactor, std::shared_ptr<gtsam::BetweenFactor<gtsam::Point3>>>(m_gtsam, "BetweenFactorPoint3")
        .def(py::init< size_t,  size_t, const gtsam::Point3&, const std::shared_ptr<gtsam::noiseModel::Base>&>(), py::arg("key1"), py::arg("key2"), py::arg("relativePose"), py::arg("noiseModel"))
        .def("measured",[](gtsam::BetweenFactor<gtsam::Point3>* self){return self->measured();});

    py::class_<gtsam::BetweenFactor<gtsam::Rot2>, gtsam::NoiseModelFactor, std::shared_ptr<gtsam::BetweenFactor<gtsam::Rot2>>>(m_gtsam, "BetweenFactorRot2")
        .def(py::init< size_t,  size_t, const gtsam::Rot2&, const std::shared_ptr<gtsam::noiseModel::Base>&>(), py::arg("key1"), py::arg("key2"), py::arg("relativePose"), py::arg("noiseModel"))
        .def("measured",[](gtsam::BetweenFactor<gtsam::Rot2>* self){return self->measured();});

    py::class_<gtsam::BetweenFactor<gtsam::Rot3>, gtsam::NoiseModelFactor, std::shared_ptr<gtsam::BetweenFactor<gtsam::Rot3>>>(m_gtsam, "BetweenFactorRot3")
        .def(py::init< size_t,  size_t, const gtsam::Rot3&, const std::shared_ptr<gtsam::noiseModel::Base>&>(), py::arg("key1"), py::arg("key2"), py::arg("relativePose"), py::arg("noiseModel"))
        .def("measured",[](gtsam::BetweenFactor<gtsam::Rot3>* self){return self->measured();});

    py::class_<gtsam::BetweenFactor<gtsam::Pose2>, gtsam::NoiseModelFactor, std::shared_ptr<gtsam::BetweenFactor<gtsam::Pose2>>>(m_gtsam, "BetweenFactorPose2")
        .def(py::init< size_t,  size_t, const gtsam::Pose2&, const std::shared_ptr<gtsam::noiseModel::Base>&>(), py::arg("key1"), py::arg("key2"), py::arg("relativePose"), py::arg("noiseModel"))
        .def("measured",[](gtsam::BetweenFactor<gtsam::Pose2>* self){return self->measured();});

    py::class_<gtsam::BetweenFactor<gtsam::Pose3>, gtsam::NoiseModelFactor, std::shared_ptr<gtsam::BetweenFactor<gtsam::Pose3>>>(m_gtsam, "BetweenFactorPose3")
        .def(py::init< size_t,  size_t, const gtsam::Pose3&, const std::shared_ptr<gtsam::noiseModel::Base>&>(), py::arg("key1"), py::arg("key2"), py::arg("relativePose"), py::arg("noiseModel"))
        .def("measured",[](gtsam::BetweenFactor<gtsam::Pose3>* self){return self->measured();});

    py::class_<gtsam::BetweenFactor<gtsam::imuBias::ConstantBias>, gtsam::NoiseModelFactor, std::shared_ptr<gtsam::BetweenFactor<gtsam::imuBias::ConstantBias>>>(m_gtsam, "BetweenFactorConstantBias")
        .def(py::init< size_t,  size_t, const gtsam::imuBias::ConstantBias&, const std::shared_ptr<gtsam::noiseModel::Base>&>(), py::arg("key1"), py::arg("key2"), py::arg("relativePose"), py::arg("noiseModel"))
        .def("measured",[](gtsam::BetweenFactor<gtsam::imuBias::ConstantBias>* self){return self->measured();});

    py::class_<gtsam::NonlinearEquality<gtsam::Point2>, gtsam::NoiseModelFactor, std::shared_ptr<gtsam::NonlinearEquality<gtsam::Point2>>>(m_gtsam, "NonlinearEqualityPoint2")
        .def(py::init< size_t, const gtsam::Point2&>(), py::arg("j"), py::arg("feasible"))
        .def(py::init< size_t, const gtsam::Point2&,  double>(), py::arg("j"), py::arg("feasible"), py::arg("error_gain"));

    py::class_<gtsam::NonlinearEquality<gtsam::StereoPoint2>, gtsam::NoiseModelFactor, std::shared_ptr<gtsam::NonlinearEquality<gtsam::StereoPoint2>>>(m_gtsam, "NonlinearEqualityStereoPoint2")
        .def(py::init< size_t, const gtsam::StereoPoint2&>(), py::arg("j"), py::arg("feasible"))
        .def(py::init< size_t, const gtsam::StereoPoint2&,  double>(), py::arg("j"), py::arg("feasible"), py::arg("error_gain"));

    py::class_<gtsam::NonlinearEquality<gtsam::Point3>, gtsam::NoiseModelFactor, std::shared_ptr<gtsam::NonlinearEquality<gtsam::Point3>>>(m_gtsam, "NonlinearEqualityPoint3")
        .def(py::init< size_t, const gtsam::Point3&>(), py::arg("j"), py::arg("feasible"))
        .def(py::init< size_t, const gtsam::Point3&,  double>(), py::arg("j"), py::arg("feasible"), py::arg("error_gain"));

    py::class_<gtsam::NonlinearEquality<gtsam::Rot2>, gtsam::NoiseModelFactor, std::shared_ptr<gtsam::NonlinearEquality<gtsam::Rot2>>>(m_gtsam, "NonlinearEqualityRot2")
        .def(py::init< size_t, const gtsam::Rot2&>(), py::arg("j"), py::arg("feasible"))
        .def(py::init< size_t, const gtsam::Rot2&,  double>(), py::arg("j"), py::arg("feasible"), py::arg("error_gain"));

    py::class_<gtsam::NonlinearEquality<gtsam::Rot3>, gtsam::NoiseModelFactor, std::shared_ptr<gtsam::NonlinearEquality<gtsam::Rot3>>>(m_gtsam, "NonlinearEqualityRot3")
        .def(py::init< size_t, const gtsam::Rot3&>(), py::arg("j"), py::arg("feasible"))
        .def(py::init< size_t, const gtsam::Rot3&,  double>(), py::arg("j"), py::arg("feasible"), py::arg("error_gain"));

    py::class_<gtsam::NonlinearEquality<gtsam::Pose2>, gtsam::NoiseModelFactor, std::shared_ptr<gtsam::NonlinearEquality<gtsam::Pose2>>>(m_gtsam, "NonlinearEqualityPose2")
        .def(py::init< size_t, const gtsam::Pose2&>(), py::arg("j"), py::arg("feasible"))
        .def(py::init< size_t, const gtsam::Pose2&,  double>(), py::arg("j"), py::arg("feasible"), py::arg("error_gain"));

    py::class_<gtsam::NonlinearEquality<gtsam::Pose3>, gtsam::NoiseModelFactor, std::shared_ptr<gtsam::NonlinearEquality<gtsam::Pose3>>>(m_gtsam, "NonlinearEqualityPose3")
        .def(py::init< size_t, const gtsam::Pose3&>(), py::arg("j"), py::arg("feasible"))
        .def(py::init< size_t, const gtsam::Pose3&,  double>(), py::arg("j"), py::arg("feasible"), py::arg("error_gain"));

    py::class_<gtsam::NonlinearEquality<gtsam::Cal3_S2>, gtsam::NoiseModelFactor, std::shared_ptr<gtsam::NonlinearEquality<gtsam::Cal3_S2>>>(m_gtsam, "NonlinearEqualityCal3_S2")
        .def(py::init< size_t, const gtsam::Cal3_S2&>(), py::arg("j"), py::arg("feasible"))
        .def(py::init< size_t, const gtsam::Cal3_S2&,  double>(), py::arg("j"), py::arg("feasible"), py::arg("error_gain"));

    py::class_<gtsam::NonlinearEquality<gtsam::CalibratedCamera>, gtsam::NoiseModelFactor, std::shared_ptr<gtsam::NonlinearEquality<gtsam::CalibratedCamera>>>(m_gtsam, "NonlinearEqualityCalibratedCamera")
        .def(py::init< size_t, const gtsam::CalibratedCamera&>(), py::arg("j"), py::arg("feasible"))
        .def(py::init< size_t, const gtsam::CalibratedCamera&,  double>(), py::arg("j"), py::arg("feasible"), py::arg("error_gain"));

    py::class_<gtsam::NonlinearEquality<gtsam::SimpleCamera>, gtsam::NoiseModelFactor, std::shared_ptr<gtsam::NonlinearEquality<gtsam::SimpleCamera>>>(m_gtsam, "NonlinearEqualitySimpleCamera")
        .def(py::init< size_t, const gtsam::SimpleCamera&>(), py::arg("j"), py::arg("feasible"))
        .def(py::init< size_t, const gtsam::SimpleCamera&,  double>(), py::arg("j"), py::arg("feasible"), py::arg("error_gain"));

    py::class_<gtsam::NonlinearEquality<gtsam::imuBias::ConstantBias>, gtsam::NoiseModelFactor, std::shared_ptr<gtsam::NonlinearEquality<gtsam::imuBias::ConstantBias>>>(m_gtsam, "NonlinearEqualityConstantBias")
        .def(py::init< size_t, const gtsam::imuBias::ConstantBias&>(), py::arg("j"), py::arg("feasible"))
        .def(py::init< size_t, const gtsam::imuBias::ConstantBias&,  double>(), py::arg("j"), py::arg("feasible"), py::arg("error_gain"));

    py::class_<gtsam::RangeFactor<gtsam::Pose2, gtsam::Point2>, gtsam::NoiseModelFactor, std::shared_ptr<gtsam::RangeFactor<gtsam::Pose2, gtsam::Point2>>>(m_gtsam, "RangeFactor2D")
        .def(py::init< size_t,  size_t,  double, const std::shared_ptr<gtsam::noiseModel::Base>&>(), py::arg("key1"), py::arg("key2"), py::arg("measured"), py::arg("noiseModel"));

    py::class_<gtsam::RangeFactor<gtsam::Pose3, gtsam::Point3>, gtsam::NoiseModelFactor, std::shared_ptr<gtsam::RangeFactor<gtsam::Pose3, gtsam::Point3>>>(m_gtsam, "RangeFactor3D")
        .def(py::init< size_t,  size_t,  double, const std::shared_ptr<gtsam::noiseModel::Base>&>(), py::arg("key1"), py::arg("key2"), py::arg("measured"), py::arg("noiseModel"));

    py::class_<gtsam::RangeFactor<gtsam::Pose2, gtsam::Pose2>, gtsam::NoiseModelFactor, std::shared_ptr<gtsam::RangeFactor<gtsam::Pose2, gtsam::Pose2>>>(m_gtsam, "RangeFactorPose2")
        .def(py::init< size_t,  size_t,  double, const std::shared_ptr<gtsam::noiseModel::Base>&>(), py::arg("key1"), py::arg("key2"), py::arg("measured"), py::arg("noiseModel"));

    py::class_<gtsam::RangeFactor<gtsam::Pose3, gtsam::Pose3>, gtsam::NoiseModelFactor, std::shared_ptr<gtsam::RangeFactor<gtsam::Pose3, gtsam::Pose3>>>(m_gtsam, "RangeFactorPose3")
        .def(py::init< size_t,  size_t,  double, const std::shared_ptr<gtsam::noiseModel::Base>&>(), py::arg("key1"), py::arg("key2"), py::arg("measured"), py::arg("noiseModel"));

    py::class_<gtsam::RangeFactor<gtsam::CalibratedCamera, gtsam::Point3>, gtsam::NoiseModelFactor, std::shared_ptr<gtsam::RangeFactor<gtsam::CalibratedCamera, gtsam::Point3>>>(m_gtsam, "RangeFactorCalibratedCameraPoint")
        .def(py::init< size_t,  size_t,  double, const std::shared_ptr<gtsam::noiseModel::Base>&>(), py::arg("key1"), py::arg("key2"), py::arg("measured"), py::arg("noiseModel"));

    py::class_<gtsam::RangeFactor<gtsam::SimpleCamera, gtsam::Point3>, gtsam::NoiseModelFactor, std::shared_ptr<gtsam::RangeFactor<gtsam::SimpleCamera, gtsam::Point3>>>(m_gtsam, "RangeFactorSimpleCameraPoint")
        .def(py::init< size_t,  size_t,  double, const std::shared_ptr<gtsam::noiseModel::Base>&>(), py::arg("key1"), py::arg("key2"), py::arg("measured"), py::arg("noiseModel"));

    py::class_<gtsam::RangeFactor<gtsam::CalibratedCamera, gtsam::CalibratedCamera>, gtsam::NoiseModelFactor, std::shared_ptr<gtsam::RangeFactor<gtsam::CalibratedCamera, gtsam::CalibratedCamera>>>(m_gtsam, "RangeFactorCalibratedCamera")
        .def(py::init< size_t,  size_t,  double, const std::shared_ptr<gtsam::noiseModel::Base>&>(), py::arg("key1"), py::arg("key2"), py::arg("measured"), py::arg("noiseModel"));

    py::class_<gtsam::RangeFactor<gtsam::SimpleCamera, gtsam::SimpleCamera>, gtsam::NoiseModelFactor, std::shared_ptr<gtsam::RangeFactor<gtsam::SimpleCamera, gtsam::SimpleCamera>>>(m_gtsam, "RangeFactorSimpleCamera")
        .def(py::init< size_t,  size_t,  double, const std::shared_ptr<gtsam::noiseModel::Base>&>(), py::arg("key1"), py::arg("key2"), py::arg("measured"), py::arg("noiseModel"));

    py::class_<gtsam::RangeFactorWithTransform<gtsam::Pose2, gtsam::Point2>, gtsam::NoiseModelFactor, std::shared_ptr<gtsam::RangeFactorWithTransform<gtsam::Pose2, gtsam::Point2>>>(m_gtsam, "RangeFactorWithTransform2D")
        .def(py::init< size_t,  size_t,  double, const std::shared_ptr<gtsam::noiseModel::Base>&, const gtsam::Pose2&>(), py::arg("key1"), py::arg("key2"), py::arg("measured"), py::arg("noiseModel"), py::arg("body_T_sensor"));

    py::class_<gtsam::RangeFactorWithTransform<gtsam::Pose3, gtsam::Point3>, gtsam::NoiseModelFactor, std::shared_ptr<gtsam::RangeFactorWithTransform<gtsam::Pose3, gtsam::Point3>>>(m_gtsam, "RangeFactorWithTransform3D")
        .def(py::init< size_t,  size_t,  double, const std::shared_ptr<gtsam::noiseModel::Base>&, const gtsam::Pose3&>(), py::arg("key1"), py::arg("key2"), py::arg("measured"), py::arg("noiseModel"), py::arg("body_T_sensor"));

    py::class_<gtsam::RangeFactorWithTransform<gtsam::Pose2, gtsam::Pose2>, gtsam::NoiseModelFactor, std::shared_ptr<gtsam::RangeFactorWithTransform<gtsam::Pose2, gtsam::Pose2>>>(m_gtsam, "RangeFactorWithTransformPose2")
        .def(py::init< size_t,  size_t,  double, const std::shared_ptr<gtsam::noiseModel::Base>&, const gtsam::Pose2&>(), py::arg("key1"), py::arg("key2"), py::arg("measured"), py::arg("noiseModel"), py::arg("body_T_sensor"));

    py::class_<gtsam::RangeFactorWithTransform<gtsam::Pose3, gtsam::Pose3>, gtsam::NoiseModelFactor, std::shared_ptr<gtsam::RangeFactorWithTransform<gtsam::Pose3, gtsam::Pose3>>>(m_gtsam, "RangeFactorWithTransformPose3")
        .def(py::init< size_t,  size_t,  double, const std::shared_ptr<gtsam::noiseModel::Base>&, const gtsam::Pose3&>(), py::arg("key1"), py::arg("key2"), py::arg("measured"), py::arg("noiseModel"), py::arg("body_T_sensor"));

    py::class_<gtsam::BearingFactor<gtsam::Pose2, gtsam::Point2, gtsam::Rot2>, gtsam::NoiseModelFactor, std::shared_ptr<gtsam::BearingFactor<gtsam::Pose2, gtsam::Point2, gtsam::Rot2>>>(m_gtsam, "BearingFactor2D")
        .def(py::init< size_t,  size_t, const gtsam::Rot2&, const std::shared_ptr<gtsam::noiseModel::Base>&>(), py::arg("key1"), py::arg("key2"), py::arg("measured"), py::arg("noiseModel"));

    py::class_<gtsam::BearingFactor<gtsam::Pose2, gtsam::Pose2, gtsam::Rot2>, gtsam::NoiseModelFactor, std::shared_ptr<gtsam::BearingFactor<gtsam::Pose2, gtsam::Pose2, gtsam::Rot2>>>(m_gtsam, "BearingFactorPose2")
        .def(py::init< size_t,  size_t, const gtsam::Rot2&, const std::shared_ptr<gtsam::noiseModel::Base>&>(), py::arg("key1"), py::arg("key2"), py::arg("measured"), py::arg("noiseModel"));

    py::class_<gtsam::BearingRange<gtsam::Pose2, gtsam::Point2, gtsam::Rot2, double>, std::shared_ptr<gtsam::BearingRange<gtsam::Pose2, gtsam::Point2, gtsam::Rot2, double>>>(m_gtsam, "BearingRange2D")
        .def(py::init<const gtsam::Rot2&, const double&>(), py::arg("b"), py::arg("r"))
        .def("bearing",[](gtsam::BearingRange<gtsam::Pose2, gtsam::Point2, gtsam::Rot2, double>* self){return self->bearing();})
        .def("range",[](gtsam::BearingRange<gtsam::Pose2, gtsam::Point2, gtsam::Rot2, double>* self){return self->range();})
        .def("print_",[](gtsam::BearingRange<gtsam::Pose2, gtsam::Point2, gtsam::Rot2, double>* self, string s){ self->print(s);}, py::arg("s"))
        .def_static("MeasureBearing",[](const gtsam::Pose2& pose,const gtsam::Point2& point){return gtsam::BearingRange<gtsam::Pose2, gtsam::Point2, gtsam::Rot2, double>::MeasureBearing(pose, point);}, py::arg("pose"), py::arg("point"))
        .def_static("MeasureRange",[](const gtsam::Pose2& pose,const gtsam::Point2& point){return gtsam::BearingRange<gtsam::Pose2, gtsam::Point2, gtsam::Rot2, double>::MeasureRange(pose, point);}, py::arg("pose"), py::arg("point"));

    py::class_<gtsam::BearingRangeFactor<gtsam::Pose2, gtsam::Point2, gtsam::Rot2, double>, gtsam::NoiseModelFactor, std::shared_ptr<gtsam::BearingRangeFactor<gtsam::Pose2, gtsam::Point2, gtsam::Rot2, double>>>(m_gtsam, "BearingRangeFactor2D")
        .def(py::init< size_t,  size_t, const gtsam::Rot2&, const double&, const std::shared_ptr<gtsam::noiseModel::Base>&>(), py::arg("poseKey"), py::arg("pointKey"), py::arg("measuredBearing"), py::arg("measuredRange"), py::arg("noiseModel"));

    py::class_<gtsam::BearingRangeFactor<gtsam::Pose2, gtsam::Pose2, gtsam::Rot2, double>, gtsam::NoiseModelFactor, std::shared_ptr<gtsam::BearingRangeFactor<gtsam::Pose2, gtsam::Pose2, gtsam::Rot2, double>>>(m_gtsam, "BearingRangeFactorPose2")
        .def(py::init< size_t,  size_t, const gtsam::Rot2&, const double&, const std::shared_ptr<gtsam::noiseModel::Base>&>(), py::arg("poseKey"), py::arg("pointKey"), py::arg("measuredBearing"), py::arg("measuredRange"), py::arg("noiseModel"));

    py::class_<gtsam::GenericProjectionFactor<gtsam::Pose3, gtsam::Point3, gtsam::Cal3_S2>, gtsam::NoiseModelFactor, std::shared_ptr<gtsam::GenericProjectionFactor<gtsam::Pose3, gtsam::Point3, gtsam::Cal3_S2>>>(m_gtsam, "GenericProjectionFactorCal3_S2")
        .def(py::init<const gtsam::Point2&, const std::shared_ptr<gtsam::noiseModel::Base>&,  size_t,  size_t, const std::shared_ptr<gtsam::Cal3_S2>&>(), py::arg("measured"), py::arg("noiseModel"), py::arg("poseKey"), py::arg("pointKey"), py::arg("k"))
        .def(py::init<const gtsam::Point2&, const std::shared_ptr<gtsam::noiseModel::Base>&,  size_t,  size_t, const std::shared_ptr<gtsam::Cal3_S2>&, const gtsam::Pose3&>(), py::arg("measured"), py::arg("noiseModel"), py::arg("poseKey"), py::arg("pointKey"), py::arg("k"), py::arg("body_P_sensor"))
        .def(py::init<const gtsam::Point2&, const std::shared_ptr<gtsam::noiseModel::Base>&,  size_t,  size_t, const std::shared_ptr<gtsam::Cal3_S2>&,  bool,  bool>(), py::arg("measured"), py::arg("noiseModel"), py::arg("poseKey"), py::arg("pointKey"), py::arg("k"), py::arg("throwCheirality"), py::arg("verboseCheirality"))
        .def(py::init<const gtsam::Point2&, const std::shared_ptr<gtsam::noiseModel::Base>&,  size_t,  size_t, const std::shared_ptr<gtsam::Cal3_S2>&,  bool,  bool, const gtsam::Pose3&>(), py::arg("measured"), py::arg("noiseModel"), py::arg("poseKey"), py::arg("pointKey"), py::arg("k"), py::arg("throwCheirality"), py::arg("verboseCheirality"), py::arg("body_P_sensor"))
        .def("measured",[](gtsam::GenericProjectionFactor<gtsam::Pose3, gtsam::Point3, gtsam::Cal3_S2>* self){return self->measured();})
        .def("calibration",[](gtsam::GenericProjectionFactor<gtsam::Pose3, gtsam::Point3, gtsam::Cal3_S2>* self){return self->calibration();})
        .def("verboseCheirality",[](gtsam::GenericProjectionFactor<gtsam::Pose3, gtsam::Point3, gtsam::Cal3_S2>* self){return self->verboseCheirality();})
        .def("throwCheirality",[](gtsam::GenericProjectionFactor<gtsam::Pose3, gtsam::Point3, gtsam::Cal3_S2>* self){return self->throwCheirality();});

    py::class_<gtsam::GenericProjectionFactor<gtsam::Pose3, gtsam::Point3, gtsam::Cal3DS2>, gtsam::NoiseModelFactor, std::shared_ptr<gtsam::GenericProjectionFactor<gtsam::Pose3, gtsam::Point3, gtsam::Cal3DS2>>>(m_gtsam, "GenericProjectionFactorCal3DS2")
        .def(py::init<const gtsam::Point2&, const std::shared_ptr<gtsam::noiseModel::Base>&,  size_t,  size_t, const std::shared_ptr<gtsam::Cal3DS2>&>(), py::arg("measured"), py::arg("noiseModel"), py::arg("poseKey"), py::arg("pointKey"), py::arg("k"))
        .def(py::init<const gtsam::Point2&, const std::shared_ptr<gtsam::noiseModel::Base>&,  size_t,  size_t, const std::shared_ptr<gtsam::Cal3DS2>&, const gtsam::Pose3&>(), py::arg("measured"), py::arg("noiseModel"), py::arg("poseKey"), py::arg("pointKey"), py::arg("k"), py::arg("body_P_sensor"))
        .def(py::init<const gtsam::Point2&, const std::shared_ptr<gtsam::noiseModel::Base>&,  size_t,  size_t, const std::shared_ptr<gtsam::Cal3DS2>&,  bool,  bool>(), py::arg("measured"), py::arg("noiseModel"), py::arg("poseKey"), py::arg("pointKey"), py::arg("k"), py::arg("throwCheirality"), py::arg("verboseCheirality"))
        .def(py::init<const gtsam::Point2&, const std::shared_ptr<gtsam::noiseModel::Base>&,  size_t,  size_t, const std::shared_ptr<gtsam::Cal3DS2>&,  bool,  bool, const gtsam::Pose3&>(), py::arg("measured"), py::arg("noiseModel"), py::arg("poseKey"), py::arg("pointKey"), py::arg("k"), py::arg("throwCheirality"), py::arg("verboseCheirality"), py::arg("body_P_sensor"))
        .def("measured",[](gtsam::GenericProjectionFactor<gtsam::Pose3, gtsam::Point3, gtsam::Cal3DS2>* self){return self->measured();})
        .def("calibration",[](gtsam::GenericProjectionFactor<gtsam::Pose3, gtsam::Point3, gtsam::Cal3DS2>* self){return self->calibration();})
        .def("verboseCheirality",[](gtsam::GenericProjectionFactor<gtsam::Pose3, gtsam::Point3, gtsam::Cal3DS2>* self){return self->verboseCheirality();})
        .def("throwCheirality",[](gtsam::GenericProjectionFactor<gtsam::Pose3, gtsam::Point3, gtsam::Cal3DS2>* self){return self->throwCheirality();});

    py::class_<gtsam::GeneralSFMFactor<gtsam::SimpleCamera, gtsam::Point3>, gtsam::NoiseModelFactor, std::shared_ptr<gtsam::GeneralSFMFactor<gtsam::SimpleCamera, gtsam::Point3>>>(m_gtsam, "GeneralSFMFactorCal3_S2")
        .def(py::init<const gtsam::Point2&, const std::shared_ptr<gtsam::noiseModel::Base>&,  size_t,  size_t>(), py::arg("measured"), py::arg("model"), py::arg("cameraKey"), py::arg("landmarkKey"))
        .def("measured",[](gtsam::GeneralSFMFactor<gtsam::SimpleCamera, gtsam::Point3>* self){return self->measured();});

    py::class_<gtsam::GeneralSFMFactor2<gtsam::Cal3_S2>, gtsam::NoiseModelFactor, std::shared_ptr<gtsam::GeneralSFMFactor2<gtsam::Cal3_S2>>>(m_gtsam, "GeneralSFMFactor2Cal3_S2")
        .def(py::init<const gtsam::Point2&, const std::shared_ptr<gtsam::noiseModel::Base>&,  size_t,  size_t,  size_t>(), py::arg("measured"), py::arg("model"), py::arg("poseKey"), py::arg("landmarkKey"), py::arg("calibKey"))
        .def("measured",[](gtsam::GeneralSFMFactor2<gtsam::Cal3_S2>* self){return self->measured();});

    py::class_<gtsam::SmartProjectionParams, std::shared_ptr<gtsam::SmartProjectionParams>>(m_gtsam, "SmartProjectionParams")
        .def(py::init<>())
        .def("setRankTolerance",[](gtsam::SmartProjectionParams* self, double rankTol){ self->setRankTolerance(rankTol);}, py::arg("rankTol"))
        .def("setEnableEPI",[](gtsam::SmartProjectionParams* self, bool enableEPI){ self->setEnableEPI(enableEPI);}, py::arg("enableEPI"))
        .def("setLandmarkDistanceThreshold",[](gtsam::SmartProjectionParams* self, bool landmarkDistanceThreshold){ self->setLandmarkDistanceThreshold(landmarkDistanceThreshold);}, py::arg("landmarkDistanceThreshold"))
        .def("setDynamicOutlierRejectionThreshold",[](gtsam::SmartProjectionParams* self, bool dynOutRejectionThreshold){ self->setDynamicOutlierRejectionThreshold(dynOutRejectionThreshold);}, py::arg("dynOutRejectionThreshold"));

    py::class_<gtsam::SmartProjectionPoseFactor<gtsam::Cal3_S2>, gtsam::NonlinearFactor, std::shared_ptr<gtsam::SmartProjectionPoseFactor<gtsam::Cal3_S2>>>(m_gtsam, "SmartProjectionPose3Factor")
        .def(py::init<const std::shared_ptr<gtsam::noiseModel::Base>&, const std::shared_ptr<gtsam::Cal3_S2>&>(), py::arg("noise"), py::arg("K"))
        .def(py::init<const std::shared_ptr<gtsam::noiseModel::Base>&, const std::shared_ptr<gtsam::Cal3_S2>&, const gtsam::Pose3&>(), py::arg("noise"), py::arg("K"), py::arg("body_P_sensor"))
        .def(py::init<const std::shared_ptr<gtsam::noiseModel::Base>&, const std::shared_ptr<gtsam::Cal3_S2>&, const gtsam::Pose3&, const gtsam::SmartProjectionParams&>(), py::arg("noise"), py::arg("K"), py::arg("body_P_sensor"), py::arg("params"))
        .def("add",[](gtsam::SmartProjectionPoseFactor<gtsam::Cal3_S2>* self,const gtsam::Point2& measured_i, size_t poseKey_i){ self->add(measured_i, poseKey_i);}, py::arg("measured_i"), py::arg("poseKey_i"));

    py::class_<gtsam::GenericStereoFactor<gtsam::Pose3, gtsam::Point3>, gtsam::NoiseModelFactor, std::shared_ptr<gtsam::GenericStereoFactor<gtsam::Pose3, gtsam::Point3>>>(m_gtsam, "GenericStereoFactor3D")
        .def(py::init<const gtsam::StereoPoint2&, const std::shared_ptr<gtsam::noiseModel::Base>&,  size_t,  size_t, const std::shared_ptr<gtsam::Cal3_S2Stereo>&>(), py::arg("measured"), py::arg("noiseModel"), py::arg("poseKey"), py::arg("landmarkKey"), py::arg("K"))
        .def("measured",[](gtsam::GenericStereoFactor<gtsam::Pose3, gtsam::Point3>* self){return self->measured();})
        .def("calibration",[](gtsam::GenericStereoFactor<gtsam::Pose3, gtsam::Point3>* self){return self->calibration();});

    py::class_<gtsam::PoseTranslationPrior<gtsam::Pose2>, gtsam::NoiseModelFactor, std::shared_ptr<gtsam::PoseTranslationPrior<gtsam::Pose2>>>(m_gtsam, "PoseTranslationPrior2D")
        .def(py::init< size_t, const gtsam::Pose2&, const std::shared_ptr<gtsam::noiseModel::Base>&>(), py::arg("key"), py::arg("pose_z"), py::arg("noiseModel"));

    py::class_<gtsam::PoseTranslationPrior<gtsam::Pose3>, gtsam::NoiseModelFactor, std::shared_ptr<gtsam::PoseTranslationPrior<gtsam::Pose3>>>(m_gtsam, "PoseTranslationPrior3D")
        .def(py::init< size_t, const gtsam::Pose3&, const std::shared_ptr<gtsam::noiseModel::Base>&>(), py::arg("key"), py::arg("pose_z"), py::arg("noiseModel"));

    py::class_<gtsam::PoseRotationPrior<gtsam::Pose2>, gtsam::NoiseModelFactor, std::shared_ptr<gtsam::PoseRotationPrior<gtsam::Pose2>>>(m_gtsam, "PoseRotationPrior2D")
        .def(py::init< size_t, const gtsam::Pose2&, const std::shared_ptr<gtsam::noiseModel::Base>&>(), py::arg("key"), py::arg("pose_z"), py::arg("noiseModel"));

    py::class_<gtsam::PoseRotationPrior<gtsam::Pose3>, gtsam::NoiseModelFactor, std::shared_ptr<gtsam::PoseRotationPrior<gtsam::Pose3>>>(m_gtsam, "PoseRotationPrior3D")
        .def(py::init< size_t, const gtsam::Pose3&, const std::shared_ptr<gtsam::noiseModel::Base>&>(), py::arg("key"), py::arg("pose_z"), py::arg("noiseModel"));

    py::class_<gtsam::EssentialMatrixFactor, gtsam::NoiseModelFactor, std::shared_ptr<gtsam::EssentialMatrixFactor>>(m_gtsam, "EssentialMatrixFactor")
        .def(py::init< size_t, const gtsam::Point2&, const gtsam::Point2&, const std::shared_ptr<gtsam::noiseModel::Base>&>(), py::arg("key"), py::arg("pA"), py::arg("pB"), py::arg("noiseModel"));
    pybind11::module m_gtsam_imuBias = m_gtsam.def_submodule("imuBias", "imuBias submodule");

    py::class_<gtsam::imuBias::ConstantBias, std::shared_ptr<gtsam::imuBias::ConstantBias>>(m_gtsam_imuBias, "ConstantBias")
        .def(py::init<>())
        .def(py::init<const gtsam::Vector&, const gtsam::Vector&>(), py::arg("biasAcc"), py::arg("biasGyro"))
        .def("print_",[](gtsam::imuBias::ConstantBias* self, string s){ self->print(s);}, py::arg("s"))
        .def("equals",[](gtsam::imuBias::ConstantBias* self,const gtsam::imuBias::ConstantBias& expected, double tol){return self->equals(expected, tol);}, py::arg("expected"), py::arg("tol"))
        .def("inverse",[](gtsam::imuBias::ConstantBias* self){return self->inverse();})
        .def("compose",[](gtsam::imuBias::ConstantBias* self,const gtsam::imuBias::ConstantBias& b){return self->compose(b);}, py::arg("b"))
        .def("between",[](gtsam::imuBias::ConstantBias* self,const gtsam::imuBias::ConstantBias& b){return self->between(b);}, py::arg("b"))
        .def("retract",[](gtsam::imuBias::ConstantBias* self,const gtsam::Vector& v){return self->retract(v);}, py::arg("v"))
        .def("localCoordinates",[](gtsam::imuBias::ConstantBias* self,const gtsam::imuBias::ConstantBias& b){return self->localCoordinates(b);}, py::arg("b"))
        .def("vector",[](gtsam::imuBias::ConstantBias* self){return self->vector();})
        .def("accelerometer",[](gtsam::imuBias::ConstantBias* self){return self->accelerometer();})
        .def("gyroscope",[](gtsam::imuBias::ConstantBias* self){return self->gyroscope();})
        .def("correctAccelerometer",[](gtsam::imuBias::ConstantBias* self,const gtsam::Vector& measurement){return self->correctAccelerometer(measurement);}, py::arg("measurement"))
        .def("correctGyroscope",[](gtsam::imuBias::ConstantBias* self,const gtsam::Vector& measurement){return self->correctGyroscope(measurement);}, py::arg("measurement"))
        .def_static("identity",[](){return gtsam::imuBias::ConstantBias::identity();})
        .def_static("Expmap",[](const gtsam::Vector& v){return gtsam::imuBias::ConstantBias::Expmap(v);}, py::arg("v"))
        .def_static("Logmap",[](const gtsam::imuBias::ConstantBias& b){return gtsam::imuBias::ConstantBias::Logmap(b);}, py::arg("b"));

    py::class_<gtsam::NavState, std::shared_ptr<gtsam::NavState>>(m_gtsam, "NavState")
        .def(py::init<>())
        .def(py::init<const gtsam::Rot3&, const gtsam::Point3&, const gtsam::Vector&>(), py::arg("R"), py::arg("t"), py::arg("v"))
        .def(py::init<const gtsam::Pose3&, const gtsam::Vector&>(), py::arg("pose"), py::arg("v"))
        .def("print_",[](gtsam::NavState* self, string s){ self->print(s);}, py::arg("s"))
        .def("equals",[](gtsam::NavState* self,const gtsam::NavState& expected, double tol){return self->equals(expected, tol);}, py::arg("expected"), py::arg("tol"))
        .def("attitude",[](gtsam::NavState* self){return self->attitude();})
        .def("position",[](gtsam::NavState* self){return self->position();})
        .def("velocity",[](gtsam::NavState* self){return self->velocity();})
        .def("pose",[](gtsam::NavState* self){return self->pose();});

    py::class_<gtsam::PreintegratedRotationParams, std::shared_ptr<gtsam::PreintegratedRotationParams>>(m_gtsam, "PreintegratedRotationParams")
        .def(py::init<>())
        .def("print_",[](gtsam::PreintegratedRotationParams* self, string s){ self->print(s);}, py::arg("s"))
        .def("equals",[](gtsam::PreintegratedRotationParams* self,const gtsam::PreintegratedRotationParams& expected, double tol){return self->equals(expected, tol);}, py::arg("expected"), py::arg("tol"))
        .def("setGyroscopeCovariance",[](gtsam::PreintegratedRotationParams* self,const gtsam::Matrix& cov){ self->setGyroscopeCovariance(cov);}, py::arg("cov"))
        .def("setOmegaCoriolis",[](gtsam::PreintegratedRotationParams* self,const gtsam::Vector& omega){ self->setOmegaCoriolis(omega);}, py::arg("omega"))
        .def("setBodyPSensor",[](gtsam::PreintegratedRotationParams* self,const gtsam::Pose3& pose){ self->setBodyPSensor(pose);}, py::arg("pose"))
        .def("getGyroscopeCovariance",[](gtsam::PreintegratedRotationParams* self){return self->getGyroscopeCovariance();});

    py::class_<gtsam::PreintegrationParams, gtsam::PreintegratedRotationParams, std::shared_ptr<gtsam::PreintegrationParams>>(m_gtsam, "PreintegrationParams")
        .def(py::init<const gtsam::Vector&>(), py::arg("n_gravity"))
        .def("print_",[](gtsam::PreintegrationParams* self, string s){ self->print(s);}, py::arg("s"))
        .def("equals",[](gtsam::PreintegrationParams* self,const gtsam::PreintegrationParams& expected, double tol){return self->equals(expected, tol);}, py::arg("expected"), py::arg("tol"))
        .def("setAccelerometerCovariance",[](gtsam::PreintegrationParams* self,const gtsam::Matrix& cov){ self->setAccelerometerCovariance(cov);}, py::arg("cov"))
        .def("setIntegrationCovariance",[](gtsam::PreintegrationParams* self,const gtsam::Matrix& cov){ self->setIntegrationCovariance(cov);}, py::arg("cov"))
        .def("setUse2ndOrderCoriolis",[](gtsam::PreintegrationParams* self, bool flag){ self->setUse2ndOrderCoriolis(flag);}, py::arg("flag"))
        .def("getAccelerometerCovariance",[](gtsam::PreintegrationParams* self){return self->getAccelerometerCovariance();})
        .def("getIntegrationCovariance",[](gtsam::PreintegrationParams* self){return self->getIntegrationCovariance();})
        .def("getUse2ndOrderCoriolis",[](gtsam::PreintegrationParams* self){return self->getUse2ndOrderCoriolis();})
        .def_static("MakeSharedD",[]( double g){return gtsam::PreintegrationParams::MakeSharedD(g);}, py::arg("g"))
        .def_static("MakeSharedU",[]( double g){return gtsam::PreintegrationParams::MakeSharedU(g);}, py::arg("g"))
        .def_static("MakeSharedD",[](){return gtsam::PreintegrationParams::MakeSharedD();})
        .def_static("MakeSharedU",[](){return gtsam::PreintegrationParams::MakeSharedU();});

    py::class_<gtsam::PreintegratedImuMeasurements, std::shared_ptr<gtsam::PreintegratedImuMeasurements>>(m_gtsam, "PreintegratedImuMeasurements")
        .def(py::init<const std::shared_ptr<gtsam::PreintegrationParams>&>(), py::arg("params"))
        .def(py::init<const std::shared_ptr<gtsam::PreintegrationParams>&, const gtsam::imuBias::ConstantBias&>(), py::arg("params"), py::arg("bias"))
        .def("print_",[](gtsam::PreintegratedImuMeasurements* self, string s){ self->print(s);}, py::arg("s"))
        .def("equals",[](gtsam::PreintegratedImuMeasurements* self,const gtsam::PreintegratedImuMeasurements& expected, double tol){return self->equals(expected, tol);}, py::arg("expected"), py::arg("tol"))
        .def("integrateMeasurement",[](gtsam::PreintegratedImuMeasurements* self,const gtsam::Vector& measuredAcc,const gtsam::Vector& measuredOmega, double deltaT){ self->integrateMeasurement(measuredAcc, measuredOmega, deltaT);}, py::arg("measuredAcc"), py::arg("measuredOmega"), py::arg("deltaT"))
        .def("resetIntegration",[](gtsam::PreintegratedImuMeasurements* self){ self->resetIntegration();})
        .def("preintMeasCov",[](gtsam::PreintegratedImuMeasurements* self){return self->preintMeasCov();})
        .def("deltaTij",[](gtsam::PreintegratedImuMeasurements* self){return self->deltaTij();})
        .def("deltaRij",[](gtsam::PreintegratedImuMeasurements* self){return self->deltaRij();})
        .def("deltaPij",[](gtsam::PreintegratedImuMeasurements* self){return self->deltaPij();})
        .def("deltaVij",[](gtsam::PreintegratedImuMeasurements* self){return self->deltaVij();})
        .def("biasHatVector",[](gtsam::PreintegratedImuMeasurements* self){return self->biasHatVector();})
        .def("predict",[](gtsam::PreintegratedImuMeasurements* self,const gtsam::NavState& state_i,const gtsam::imuBias::ConstantBias& bias){return self->predict(state_i, bias);}, py::arg("state_i"), py::arg("bias"));

    py::class_<gtsam::ImuFactor, gtsam::NonlinearFactor, std::shared_ptr<gtsam::ImuFactor>>(m_gtsam, "ImuFactor")
        .def(py::init< size_t,  size_t,  size_t,  size_t,  size_t, const gtsam::PreintegratedImuMeasurements&>(), py::arg("pose_i"), py::arg("vel_i"), py::arg("pose_j"), py::arg("vel_j"), py::arg("bias"), py::arg("preintegratedMeasurements"))
        .def("preintegratedMeasurements",[](gtsam::ImuFactor* self){return self->preintegratedMeasurements();})
        .def("evaluateError",[](gtsam::ImuFactor* self,const gtsam::Pose3& pose_i,const gtsam::Vector& vel_i,const gtsam::Pose3& pose_j,const gtsam::Vector& vel_j,const gtsam::imuBias::ConstantBias& bias){return self->evaluateError(pose_i, vel_i, pose_j, vel_j, bias);}, py::arg("pose_i"), py::arg("vel_i"), py::arg("pose_j"), py::arg("vel_j"), py::arg("bias"));

    py::class_<gtsam::PreintegratedCombinedMeasurements, std::shared_ptr<gtsam::PreintegratedCombinedMeasurements>>(m_gtsam, "PreintegratedCombinedMeasurements")
        .def("print_",[](gtsam::PreintegratedCombinedMeasurements* self, string s){ self->print(s);}, py::arg("s"))
        .def("equals",[](gtsam::PreintegratedCombinedMeasurements* self,const gtsam::PreintegratedCombinedMeasurements& expected, double tol){return self->equals(expected, tol);}, py::arg("expected"), py::arg("tol"))
        .def("integrateMeasurement",[](gtsam::PreintegratedCombinedMeasurements* self,const gtsam::Vector& measuredAcc,const gtsam::Vector& measuredOmega, double deltaT){ self->integrateMeasurement(measuredAcc, measuredOmega, deltaT);}, py::arg("measuredAcc"), py::arg("measuredOmega"), py::arg("deltaT"))
        .def("resetIntegration",[](gtsam::PreintegratedCombinedMeasurements* self){ self->resetIntegration();})
        .def("preintMeasCov",[](gtsam::PreintegratedCombinedMeasurements* self){return self->preintMeasCov();})
        .def("deltaTij",[](gtsam::PreintegratedCombinedMeasurements* self){return self->deltaTij();})
        .def("deltaRij",[](gtsam::PreintegratedCombinedMeasurements* self){return self->deltaRij();})
        .def("deltaPij",[](gtsam::PreintegratedCombinedMeasurements* self){return self->deltaPij();})
        .def("deltaVij",[](gtsam::PreintegratedCombinedMeasurements* self){return self->deltaVij();})
        .def("biasHatVector",[](gtsam::PreintegratedCombinedMeasurements* self){return self->biasHatVector();})
        .def("predict",[](gtsam::PreintegratedCombinedMeasurements* self,const gtsam::NavState& state_i,const gtsam::imuBias::ConstantBias& bias){return self->predict(state_i, bias);}, py::arg("state_i"), py::arg("bias"));

    py::class_<gtsam::CombinedImuFactor, gtsam::NonlinearFactor, std::shared_ptr<gtsam::CombinedImuFactor>>(m_gtsam, "CombinedImuFactor")
        .def(py::init< size_t,  size_t,  size_t,  size_t,  size_t,  size_t, const gtsam::PreintegratedCombinedMeasurements&>(), py::arg("pose_i"), py::arg("vel_i"), py::arg("pose_j"), py::arg("vel_j"), py::arg("bias_i"), py::arg("bias_j"), py::arg("CombinedPreintegratedMeasurements"))
        .def("preintegratedMeasurements",[](gtsam::CombinedImuFactor* self){return self->preintegratedMeasurements();})
        .def("evaluateError",[](gtsam::CombinedImuFactor* self,const gtsam::Pose3& pose_i,const gtsam::Vector& vel_i,const gtsam::Pose3& pose_j,const gtsam::Vector& vel_j,const gtsam::imuBias::ConstantBias& bias_i,const gtsam::imuBias::ConstantBias& bias_j){return self->evaluateError(pose_i, vel_i, pose_j, vel_j, bias_i, bias_j);}, py::arg("pose_i"), py::arg("vel_i"), py::arg("pose_j"), py::arg("vel_j"), py::arg("bias_i"), py::arg("bias_j"));

    py::class_<gtsam::PreintegratedAhrsMeasurements, std::shared_ptr<gtsam::PreintegratedAhrsMeasurements>>(m_gtsam, "PreintegratedAhrsMeasurements")
        .def(py::init<const gtsam::Vector&, const gtsam::Matrix&>(), py::arg("bias"), py::arg("measuredOmegaCovariance"))
        .def(py::init<const gtsam::PreintegratedAhrsMeasurements&>(), py::arg("rhs"))
        .def("print_",[](gtsam::PreintegratedAhrsMeasurements* self, string s){ self->print(s);}, py::arg("s"))
        .def("equals",[](gtsam::PreintegratedAhrsMeasurements* self,const gtsam::PreintegratedAhrsMeasurements& expected, double tol){return self->equals(expected, tol);}, py::arg("expected"), py::arg("tol"))
        .def("deltaRij",[](gtsam::PreintegratedAhrsMeasurements* self){return self->deltaRij();})
        .def("deltaTij",[](gtsam::PreintegratedAhrsMeasurements* self){return self->deltaTij();})
        .def("biasHat",[](gtsam::PreintegratedAhrsMeasurements* self){return self->biasHat();})
        .def("integrateMeasurement",[](gtsam::PreintegratedAhrsMeasurements* self,const gtsam::Vector& measuredOmega, double deltaT){ self->integrateMeasurement(measuredOmega, deltaT);}, py::arg("measuredOmega"), py::arg("deltaT"))
        .def("resetIntegration",[](gtsam::PreintegratedAhrsMeasurements* self){ self->resetIntegration();});

    py::class_<gtsam::AHRSFactor, gtsam::NonlinearFactor, std::shared_ptr<gtsam::AHRSFactor>>(m_gtsam, "AHRSFactor")
        .def(py::init< size_t,  size_t,  size_t, const gtsam::PreintegratedAhrsMeasurements&, const gtsam::Vector&>(), py::arg("rot_i"), py::arg("rot_j"), py::arg("bias"), py::arg("preintegratedMeasurements"), py::arg("omegaCoriolis"))
        .def(py::init< size_t,  size_t,  size_t, const gtsam::PreintegratedAhrsMeasurements&, const gtsam::Vector&, const gtsam::Pose3&>(), py::arg("rot_i"), py::arg("rot_j"), py::arg("bias"), py::arg("preintegratedMeasurements"), py::arg("omegaCoriolis"), py::arg("body_P_sensor"))
        .def("preintegratedMeasurements",[](gtsam::AHRSFactor* self){return self->preintegratedMeasurements();})
        .def("evaluateError",[](gtsam::AHRSFactor* self,const gtsam::Rot3& rot_i,const gtsam::Rot3& rot_j,const gtsam::Vector& bias){return self->evaluateError(rot_i, rot_j, bias);}, py::arg("rot_i"), py::arg("rot_j"), py::arg("bias"))
        .def("predict",[](gtsam::AHRSFactor* self,const gtsam::Rot3& rot_i,const gtsam::Vector& bias,const gtsam::PreintegratedAhrsMeasurements& preintegratedMeasurements,const gtsam::Vector& omegaCoriolis){return self->predict(rot_i, bias, preintegratedMeasurements, omegaCoriolis);}, py::arg("rot_i"), py::arg("bias"), py::arg("preintegratedMeasurements"), py::arg("omegaCoriolis"));

    py::class_<gtsam::Rot3AttitudeFactor, gtsam::NonlinearFactor, std::shared_ptr<gtsam::Rot3AttitudeFactor>>(m_gtsam, "Rot3AttitudeFactor")
        .def(py::init< size_t, const gtsam::Unit3&, const std::shared_ptr<gtsam::noiseModel::Diagonal>&, const gtsam::Unit3&>(), py::arg("key"), py::arg("nZ"), py::arg("model"), py::arg("bRef"))
        .def(py::init< size_t, const gtsam::Unit3&, const std::shared_ptr<gtsam::noiseModel::Diagonal>&>(), py::arg("key"), py::arg("nZ"), py::arg("model"))
        .def(py::init<>())
        .def("print_",[](gtsam::Rot3AttitudeFactor* self, string s){ self->print(s);}, py::arg("s"))
        .def("equals",[](gtsam::Rot3AttitudeFactor* self,const gtsam::NonlinearFactor& expected, double tol){return self->equals(expected, tol);}, py::arg("expected"), py::arg("tol"))
        .def("nZ",[](gtsam::Rot3AttitudeFactor* self){return self->nZ();})
        .def("bRef",[](gtsam::Rot3AttitudeFactor* self){return self->bRef();});

    py::class_<gtsam::Pose3AttitudeFactor, gtsam::NonlinearFactor, std::shared_ptr<gtsam::Pose3AttitudeFactor>>(m_gtsam, "Pose3AttitudeFactor")
        .def(py::init< size_t, const gtsam::Unit3&, const std::shared_ptr<gtsam::noiseModel::Diagonal>&, const gtsam::Unit3&>(), py::arg("key"), py::arg("nZ"), py::arg("model"), py::arg("bRef"))
        .def(py::init< size_t, const gtsam::Unit3&, const std::shared_ptr<gtsam::noiseModel::Diagonal>&>(), py::arg("key"), py::arg("nZ"), py::arg("model"))
        .def(py::init<>())
        .def("print_",[](gtsam::Pose3AttitudeFactor* self, string s){ self->print(s);}, py::arg("s"))
        .def("equals",[](gtsam::Pose3AttitudeFactor* self,const gtsam::NonlinearFactor& expected, double tol){return self->equals(expected, tol);}, py::arg("expected"), py::arg("tol"))
        .def("nZ",[](gtsam::Pose3AttitudeFactor* self){return self->nZ();})
        .def("bRef",[](gtsam::Pose3AttitudeFactor* self){return self->bRef();});

    py::class_<gtsam::Scenario, std::shared_ptr<gtsam::Scenario>>(m_gtsam, "Scenario")
        .def("pose",[](gtsam::Scenario* self, double t){return self->pose(t);}, py::arg("t"))
        .def("omega_b",[](gtsam::Scenario* self, double t){return self->omega_b(t);}, py::arg("t"))
        .def("velocity_n",[](gtsam::Scenario* self, double t){return self->velocity_n(t);}, py::arg("t"))
        .def("acceleration_n",[](gtsam::Scenario* self, double t){return self->acceleration_n(t);}, py::arg("t"))
        .def("rotation",[](gtsam::Scenario* self, double t){return self->rotation(t);}, py::arg("t"))
        .def("navState",[](gtsam::Scenario* self, double t){return self->navState(t);}, py::arg("t"))
        .def("velocity_b",[](gtsam::Scenario* self, double t){return self->velocity_b(t);}, py::arg("t"))
        .def("acceleration_b",[](gtsam::Scenario* self, double t){return self->acceleration_b(t);}, py::arg("t"));

    py::class_<gtsam::ConstantTwistScenario, gtsam::Scenario, std::shared_ptr<gtsam::ConstantTwistScenario>>(m_gtsam, "ConstantTwistScenario")
        .def(py::init<const gtsam::Vector&, const gtsam::Vector&>(), py::arg("w"), py::arg("v"))
        .def(py::init<const gtsam::Vector&, const gtsam::Vector&, const gtsam::Pose3&>(), py::arg("w"), py::arg("v"), py::arg("nTb0"));

    py::class_<gtsam::AcceleratingScenario, gtsam::Scenario, std::shared_ptr<gtsam::AcceleratingScenario>>(m_gtsam, "AcceleratingScenario")
        .def(py::init<const gtsam::Rot3&, const gtsam::Point3&, const gtsam::Vector&, const gtsam::Vector&, const gtsam::Vector&>(), py::arg("nRb"), py::arg("p0"), py::arg("v0"), py::arg("a_n"), py::arg("omega_b"));
    pybind11::module m_gtsam_utilities = m_gtsam.def_submodule("utilities", "utilities submodule");

    m_gtsam_utilities.def("createKeyList",[](const gtsam::Vector& I){return gtsam::utilities::createKeyList(I);}, py::arg("I"));
    m_gtsam_utilities.def("createKeyList",[]( string s,const gtsam::Vector& I){return gtsam::utilities::createKeyList(s, I);}, py::arg("s"), py::arg("I"));
    m_gtsam_utilities.def("createKeyVector",[](const gtsam::Vector& I){return gtsam::utilities::createKeyVector(I);}, py::arg("I"));
    m_gtsam_utilities.def("createKeyVector",[]( string s,const gtsam::Vector& I){return gtsam::utilities::createKeyVector(s, I);}, py::arg("s"), py::arg("I"));
    m_gtsam_utilities.def("createKeySet",[](const gtsam::Vector& I){return gtsam::utilities::createKeySet(I);}, py::arg("I"));
    m_gtsam_utilities.def("createKeySet",[]( string s,const gtsam::Vector& I){return gtsam::utilities::createKeySet(s, I);}, py::arg("s"), py::arg("I"));
    m_gtsam_utilities.def("extractPoint2",[](const gtsam::Values& values){return gtsam::utilities::extractPoint2(values);}, py::arg("values"));
    m_gtsam_utilities.def("extractPoint3",[](const gtsam::Values& values){return gtsam::utilities::extractPoint3(values);}, py::arg("values"));
    m_gtsam_utilities.def("extractPose2",[](const gtsam::Values& values){return gtsam::utilities::extractPose2(values);}, py::arg("values"));
    m_gtsam_utilities.def("allPose3s",[]( gtsam::Values& values){return gtsam::utilities::allPose3s(values);}, py::arg("values"));
    m_gtsam_utilities.def("extractPose3",[](const gtsam::Values& values){return gtsam::utilities::extractPose3(values);}, py::arg("values"));
    m_gtsam_utilities.def("perturbPoint2",[]( gtsam::Values& values, double sigma, int seed){ gtsam::utilities::perturbPoint2(values, sigma, seed);}, py::arg("values"), py::arg("sigma"), py::arg("seed"));
    m_gtsam_utilities.def("perturbPose2",[]( gtsam::Values& values, double sigmaT, double sigmaR, int seed){ gtsam::utilities::perturbPose2(values, sigmaT, sigmaR, seed);}, py::arg("values"), py::arg("sigmaT"), py::arg("sigmaR"), py::arg("seed"));
    m_gtsam_utilities.def("perturbPoint3",[]( gtsam::Values& values, double sigma, int seed){ gtsam::utilities::perturbPoint3(values, sigma, seed);}, py::arg("values"), py::arg("sigma"), py::arg("seed"));
    m_gtsam_utilities.def("insertBackprojections",[]( gtsam::Values& values,const gtsam::SimpleCamera& c,const gtsam::Vector& J,const gtsam::Matrix& Z, double depth){ gtsam::utilities::insertBackprojections(values, c, J, Z, depth);}, py::arg("values"), py::arg("c"), py::arg("J"), py::arg("Z"), py::arg("depth"));
    m_gtsam_utilities.def("insertProjectionFactors",[]( gtsam::NonlinearFactorGraph& graph, size_t i,const gtsam::Vector& J,const gtsam::Matrix& Z,const std::shared_ptr<gtsam::noiseModel::Base>& model,const std::shared_ptr<gtsam::Cal3_S2>& K){ gtsam::utilities::insertProjectionFactors(graph, i, J, Z, model, K);}, py::arg("graph"), py::arg("i"), py::arg("J"), py::arg("Z"), py::arg("model"), py::arg("K"));
    m_gtsam_utilities.def("insertProjectionFactors",[]( gtsam::NonlinearFactorGraph& graph, size_t i,const gtsam::Vector& J,const gtsam::Matrix& Z,const std::shared_ptr<gtsam::noiseModel::Base>& model,const std::shared_ptr<gtsam::Cal3_S2>& K,const gtsam::Pose3& body_P_sensor){ gtsam::utilities::insertProjectionFactors(graph, i, J, Z, model, K, body_P_sensor);}, py::arg("graph"), py::arg("i"), py::arg("J"), py::arg("Z"), py::arg("model"), py::arg("K"), py::arg("body_P_sensor"));
    m_gtsam_utilities.def("reprojectionErrors",[](const gtsam::NonlinearFactorGraph& graph,const gtsam::Values& values){return gtsam::utilities::reprojectionErrors(graph, values);}, py::arg("graph"), py::arg("values"));
    m_gtsam_utilities.def("localToWorld",[](const gtsam::Values& local,const gtsam::Pose2& base){return gtsam::utilities::localToWorld(local, base);}, py::arg("local"), py::arg("base"));
    m_gtsam_utilities.def("localToWorld",[](const gtsam::Values& local,const gtsam::Pose2& base,const gtsam::KeyVector& keys){return gtsam::utilities::localToWorld(local, base, keys);}, py::arg("local"), py::arg("base"), py::arg("keys"));
    py::class_<gtsam::RedirectCout, std::shared_ptr<gtsam::RedirectCout>>(m_gtsam, "RedirectCout")
        .def(py::init<>())
        .def("str",[](gtsam::RedirectCout* self){return self->str();});

    m_gtsam.def("linear_independent",[](const gtsam::Matrix& A,const gtsam::Matrix& B, double tol){return gtsam::linear_independent(A, B, tol);}, py::arg("A"), py::arg("B"), py::arg("tol"));
    m_gtsam.def("triangulatePoint3",[](const gtsam::Pose3Vector& poses,const std::shared_ptr<gtsam::Cal3_S2>& sharedCal,const gtsam::Point2Vector& measurements, double rank_tol, bool optimize){return gtsam::triangulatePoint3(poses, sharedCal, measurements, rank_tol, optimize);}, py::arg("poses"), py::arg("sharedCal"), py::arg("measurements"), py::arg("rank_tol"), py::arg("optimize"));
    m_gtsam.def("triangulatePoint3",[](const gtsam::Pose3Vector& poses,const std::shared_ptr<gtsam::Cal3Bundler>& sharedCal,const gtsam::Point2Vector& measurements, double rank_tol, bool optimize){return gtsam::triangulatePoint3(poses, sharedCal, measurements, rank_tol, optimize);}, py::arg("poses"), py::arg("sharedCal"), py::arg("measurements"), py::arg("rank_tol"), py::arg("optimize"));
    m_gtsam.def("symbol",[]( char chr, size_t index){return gtsam::symbol(chr, index);}, py::arg("chr"), py::arg("index"));
    m_gtsam.def("symbolChr",[]( size_t key){return gtsam::symbolChr(key);}, py::arg("key"));
    m_gtsam.def("symbolIndex",[]( size_t key){return gtsam::symbolIndex(key);}, py::arg("key"));
    m_gtsam.def("PrintKeyList",[](const gtsam::KeyList& keys){ gtsam::PrintKeyList(keys);}, py::arg("keys"));
    m_gtsam.def("PrintKeyList",[](const gtsam::KeyList& keys, string s){ gtsam::PrintKeyList(keys, s);}, py::arg("keys"), py::arg("s"));
    m_gtsam.def("PrintKeyVector",[](const gtsam::KeyVector& keys){ gtsam::PrintKeyVector(keys);}, py::arg("keys"));
    m_gtsam.def("PrintKeyVector",[](const gtsam::KeyVector& keys, string s){ gtsam::PrintKeyVector(keys, s);}, py::arg("keys"), py::arg("s"));
    m_gtsam.def("PrintKeySet",[](const gtsam::KeySet& keys){ gtsam::PrintKeySet(keys);}, py::arg("keys"));
    m_gtsam.def("PrintKeySet",[](const gtsam::KeySet& keys, string s){ gtsam::PrintKeySet(keys, s);}, py::arg("keys"), py::arg("s"));
    m_gtsam.def("mrsymbol",[]( unsigned char c, unsigned char label, size_t j){return gtsam::mrsymbol(c, label, j);}, py::arg("c"), py::arg("label"), py::arg("j"));
    m_gtsam.def("mrsymbolChr",[]( size_t key){return gtsam::mrsymbolChr(key);}, py::arg("key"));
    m_gtsam.def("mrsymbolLabel",[]( size_t key){return gtsam::mrsymbolLabel(key);}, py::arg("key"));
    m_gtsam.def("mrsymbolIndex",[]( size_t key){return gtsam::mrsymbolIndex(key);}, py::arg("key"));
    m_gtsam.def("checkConvergence",[]( double relativeErrorTreshold, double absoluteErrorTreshold, double errorThreshold, double currentError, double newError){return gtsam::checkConvergence(relativeErrorTreshold, absoluteErrorTreshold, errorThreshold, currentError, newError);}, py::arg("relativeErrorTreshold"), py::arg("absoluteErrorTreshold"), py::arg("errorThreshold"), py::arg("currentError"), py::arg("newError"));
    m_gtsam.def("findExampleDataFile",[]( string name){return gtsam::findExampleDataFile(name);}, py::arg("name"));
    m_gtsam.def("load2D",[]( string filename,const std::shared_ptr<gtsam::noiseModel::Diagonal>& model, int maxID, bool addNoise, bool smart){return gtsam::load2D(filename, model, maxID, addNoise, smart);}, py::arg("filename"), py::arg("model"), py::arg("maxID"), py::arg("addNoise"), py::arg("smart"));
    m_gtsam.def("load2D",[]( string filename,const std::shared_ptr<gtsam::noiseModel::Diagonal>& model, int maxID, bool addNoise){return gtsam::load2D(filename, model, maxID, addNoise);}, py::arg("filename"), py::arg("model"), py::arg("maxID"), py::arg("addNoise"));
    m_gtsam.def("load2D",[]( string filename,const std::shared_ptr<gtsam::noiseModel::Diagonal>& model, int maxID){return gtsam::load2D(filename, model, maxID);}, py::arg("filename"), py::arg("model"), py::arg("maxID"));
    m_gtsam.def("load2D",[]( string filename,const std::shared_ptr<gtsam::noiseModel::Diagonal>& model){return gtsam::load2D(filename, model);}, py::arg("filename"), py::arg("model"));
    m_gtsam.def("load2D",[]( string filename){return gtsam::load2D(filename);}, py::arg("filename"));
    m_gtsam.def("load2D_robust",[]( string filename,const std::shared_ptr<gtsam::noiseModel::Base>& model){return gtsam::load2D_robust(filename, model);}, py::arg("filename"), py::arg("model"));
    m_gtsam.def("save2D",[](const gtsam::NonlinearFactorGraph& graph,const gtsam::Values& config,const std::shared_ptr<gtsam::noiseModel::Diagonal>& model, string filename){ gtsam::save2D(graph, config, model, filename);}, py::arg("graph"), py::arg("config"), py::arg("model"), py::arg("filename"));
    m_gtsam.def("readG2o",[]( string filename){return gtsam::readG2o(filename);}, py::arg("filename"));
    m_gtsam.def("readG2o",[]( string filename, bool is3D){return gtsam::readG2o(filename, is3D);}, py::arg("filename"), py::arg("is3D"));
    m_gtsam.def("writeG2o",[](const gtsam::NonlinearFactorGraph& graph,const gtsam::Values& estimate, string filename){ gtsam::writeG2o(graph, estimate, filename);}, py::arg("graph"), py::arg("estimate"), py::arg("filename"));

    return m_.ptr();
}

