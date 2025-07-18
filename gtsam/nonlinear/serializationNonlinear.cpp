/**
 * @file serialization.cpp
 *
 * @date Jun 12, 2013
 * @author Alex Cunningham
 */

#if GTSAM_ENABLE_BOOST_SERIALIZATION
#include <boost/serialization/export.hpp>

#include <gtsam/base/serializationTestHelpers.h>
#include <gtsam/nonlinear/serializationNonlinear.h>

#include <gtsam/sam/BearingRangeFactor.h>
#include <gtsam/sam/RangeFactor.h>

#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/GeneralSFMFactor.h>
#include <gtsam/slam/ProjectionFactor.h>
#include <gtsam/slam/StereoFactor.h>

#include <gtsam/geometry/PinholeCamera.h>
#include <gtsam/geometry/Cal3_S2.h>
#include <gtsam/geometry/Cal3DS2.h>
#include <gtsam/geometry/Cal3Bundler.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Pose3.h>

#include <gtsam/inference/Symbol.h>

#include <gtsam/linear/VectorValues.h>
#include <gtsam/linear/JacobianFactor.h>
#include <gtsam/linear/HessianFactor.h>
#include <gtsam/linear/GaussianISAM.h>
#include <gtsam/linear/NoiseModel.h>

#include <gtsam/navigation/AttitudeFactor.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/ImuFactor.h>

#include <gtsam/nonlinear/NonlinearEquality.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/PriorFactor.h>
#include <gtsam/nonlinear/LinearContainerFactor.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/IncrementalFixedLagSmoother.h>

#include <CppUnitLite/TestHarness.h>

using namespace std;
using namespace gtsam;
using namespace gtsam::serializationTestHelpers;

// Creating as many permutations of factors as possible
typedef PriorFactor<Point2> PriorFactorPoint2;
typedef PriorFactor<StereoPoint2> PriorFactorStereoPoint2;
typedef PriorFactor<Point3> PriorFactorPoint3;
typedef PriorFactor<Rot2> PriorFactorRot2;
typedef PriorFactor<Rot3> PriorFactorRot3;
typedef PriorFactor<Pose2> PriorFactorPose2;
typedef PriorFactor<Pose3> PriorFactorPose3;
typedef PriorFactor<Cal3_S2> PriorFactorCal3_S2;
typedef PriorFactor<Cal3DS2> PriorFactorCal3DS2;
typedef PriorFactor<CalibratedCamera> PriorFactorCalibratedCamera;
typedef PriorFactor<StereoCamera> PriorFactorStereoCamera;

typedef BetweenFactor<Point2> BetweenFactorPoint2;
typedef BetweenFactor<Point3> BetweenFactorPoint3;
typedef BetweenFactor<Rot2> BetweenFactorRot2;
typedef BetweenFactor<Rot3> BetweenFactorRot3;
typedef BetweenFactor<Pose2> BetweenFactorPose2;
typedef BetweenFactor<Pose3> BetweenFactorPose3;

typedef NonlinearEquality<Point2> NonlinearEqualityPoint2;
typedef NonlinearEquality<StereoPoint2> NonlinearEqualityStereoPoint2;
typedef NonlinearEquality<Point3> NonlinearEqualityPoint3;
typedef NonlinearEquality<Rot2> NonlinearEqualityRot2;
typedef NonlinearEquality<Rot3> NonlinearEqualityRot3;
typedef NonlinearEquality<Pose2> NonlinearEqualityPose2;
typedef NonlinearEquality<Pose3> NonlinearEqualityPose3;
typedef NonlinearEquality<Cal3_S2> NonlinearEqualityCal3_S2;
typedef NonlinearEquality<Cal3DS2> NonlinearEqualityCal3DS2;
typedef NonlinearEquality<CalibratedCamera> NonlinearEqualityCalibratedCamera;
typedef NonlinearEquality<StereoCamera> NonlinearEqualityStereoCamera;

typedef RangeFactor<Pose2, Point2> RangeFactor2D;
typedef RangeFactor<Pose3, Point3> RangeFactor3D;
typedef RangeFactor<Pose2, Pose2> RangeFactorPose2;
typedef RangeFactor<Pose3, Pose3> RangeFactorPose3;
typedef RangeFactor<CalibratedCamera, Point3> RangeFactorCalibratedCameraPoint;
typedef RangeFactor<CalibratedCamera, CalibratedCamera>
    RangeFactorCalibratedCamera;

typedef BearingRangeFactor<Pose2, Point2> BearingRangeFactor2D;
typedef BearingRangeFactor<Pose3, Point3> BearingRangeFactor3D;

typedef GenericProjectionFactor<Pose3, Point3, Cal3_S2>
    GenericProjectionFactorCal3_S2;
typedef GenericProjectionFactor<Pose3, Point3, Cal3DS2>
    GenericProjectionFactorCal3DS2;

typedef gtsam::GeneralSFMFactor2<gtsam::Cal3_S2> GeneralSFMFactor2Cal3_S2;

typedef gtsam::GenericStereoFactor<gtsam::Pose3, gtsam::Point3>
    GenericStereoFactor3D;

/* Create GUIDs for Noisemodels */
/* ************************************************************************* */
BOOST_SERIALIZATION_ASSUME_ABSTRACT(gtsam::noiseModel::Base);
// BOOST_CLASS_EXPORT_GUID(gtsam::noiseModel::Base, "gtsam_noiseModel_Base");
BOOST_CLASS_EXPORT_GUID(gtsam::noiseModel::Diagonal, "gtsam_noiseModel_Diagonal");

BOOST_CLASS_EXPORT_GUID(gtsam::noiseModel::mEstimator::Base,
                        "gtsam_noiseModel_mEstimator_Base");
BOOST_CLASS_EXPORT_GUID(gtsam::noiseModel::mEstimator::Null,
                        "gtsam_noiseModel_mEstimator_Null");
BOOST_CLASS_EXPORT_GUID(gtsam::noiseModel::mEstimator::Fair,
                        "gtsam_noiseModel_mEstimator_Fair");
BOOST_CLASS_EXPORT_GUID(gtsam::noiseModel::mEstimator::Huber,
                        "gtsam_noiseModel_mEstimator_Huber");
BOOST_CLASS_EXPORT_GUID(gtsam::noiseModel::mEstimator::Tukey,
                        "gtsam_noiseModel_mEstimator_Tukey");

BOOST_CLASS_EXPORT_GUID(gtsam::noiseModel::Gaussian, "gtsam_noiseModel_Gaussian");
BOOST_CLASS_EXPORT_GUID(gtsam::noiseModel::Constrained, "gtsam_noiseModel_Constrained");
BOOST_CLASS_EXPORT_GUID(gtsam::noiseModel::Unit, "gtsam_noiseModel_Unit");
BOOST_CLASS_EXPORT_GUID(gtsam::noiseModel::Isotropic, "gtsam_noiseModel_Isotropic");

BOOST_CLASS_EXPORT_GUID(gtsam::SharedNoiseModel, "gtsam_SharedNoiseModel");
BOOST_CLASS_EXPORT_GUID(gtsam::SharedDiagonal, "gtsam_SharedDiagonal");


/* Create GUIDs for pim */
/* ************************************************************************* */
BOOST_SERIALIZATION_ASSUME_ABSTRACT(gtsam::PreintegrationBase);
BOOST_CLASS_EXPORT_GUID(PreintegratedImuMeasurements, "gtsam_PreintegratedImuMeasurements");
BOOST_CLASS_EXPORT_GUID(PreintegrationCombinedParams, "gtsam_PreintegrationCombinedParams");
BOOST_CLASS_EXPORT_GUID(PreintegratedCombinedMeasurements, "gtsam_PreintegratedCombinedMeasurements");

/* Create GUIDs for geometry */
/* ************************************************************************* */
GTSAM_VALUE_EXPORT(gtsam::Cal3_S2);
GTSAM_VALUE_EXPORT(gtsam::Cal3Bundler);
GTSAM_VALUE_EXPORT(gtsam::Point3);
GTSAM_VALUE_EXPORT(gtsam::Pose3);
GTSAM_VALUE_EXPORT(gtsam::Rot3);
GTSAM_VALUE_EXPORT(gtsam::PinholeCamera<Cal3_S2>);
GTSAM_VALUE_EXPORT(gtsam::PinholeCamera<Cal3DS2>);
GTSAM_VALUE_EXPORT(gtsam::PinholeCamera<Cal3Bundler>);

/* Create GUIDs for factors */
/* ************************************************************************* */
BOOST_SERIALIZATION_ASSUME_ABSTRACT(gtsam::NonlinearFactor);
BOOST_CLASS_EXPORT_GUID(gtsam::JacobianFactor, "gtsam::JacobianFactor");
BOOST_CLASS_EXPORT_GUID(gtsam::HessianFactor, "gtsam::HessianFactor");

BOOST_CLASS_EXPORT_GUID(PriorFactorPoint2, "gtsam::PriorFactorPoint2");
BOOST_CLASS_EXPORT_GUID(PriorFactorStereoPoint2,
                        "gtsam::PriorFactorStereoPoint2");
BOOST_CLASS_EXPORT_GUID(PriorFactorPoint3, "gtsam::PriorFactorPoint3");
BOOST_CLASS_EXPORT_GUID(PriorFactorRot2, "gtsam::PriorFactorRot2");
BOOST_CLASS_EXPORT_GUID(PriorFactorRot3, "gtsam::PriorFactorRot3");
BOOST_CLASS_EXPORT_GUID(PriorFactorPose2, "gtsam::PriorFactorPose2");
BOOST_CLASS_EXPORT_GUID(PriorFactorPose3, "gtsam::PriorFactorPose3");
BOOST_CLASS_EXPORT_GUID(PriorFactorCal3_S2, "gtsam::PriorFactorCal3_S2");
BOOST_CLASS_EXPORT_GUID(PriorFactorCal3DS2, "gtsam::PriorFactorCal3DS2");
BOOST_CLASS_EXPORT_GUID(PriorFactorCalibratedCamera,
                        "gtsam::PriorFactorCalibratedCamera");
BOOST_CLASS_EXPORT_GUID(PriorFactorStereoCamera,
                        "gtsam::PriorFactorStereoCamera");

BOOST_CLASS_EXPORT_GUID(ImuFactor, "gtsam::ImuFactor");
BOOST_CLASS_EXPORT_GUID(CombinedImuFactor, "gtsam::CombinedImuFactor");
BOOST_CLASS_EXPORT_GUID(LinearContainerFactor, "gtsam::LinearContainerFactor");

BOOST_CLASS_EXPORT_GUID(BetweenFactorPoint2, "gtsam::BetweenFactorPoint2");
BOOST_CLASS_EXPORT_GUID(BetweenFactorPoint3, "gtsam::BetweenFactorPoint3");
BOOST_CLASS_EXPORT_GUID(BetweenFactorRot2, "gtsam::BetweenFactorRot2");
BOOST_CLASS_EXPORT_GUID(BetweenFactorRot3, "gtsam::BetweenFactorRot3");
BOOST_CLASS_EXPORT_GUID(BetweenFactorPose2, "gtsam::BetweenFactorPose2");
BOOST_CLASS_EXPORT_GUID(BetweenFactorPose3, "gtsam::BetweenFactorPose3");

BOOST_CLASS_EXPORT_GUID(NonlinearEqualityPoint2,
                        "gtsam::NonlinearEqualityPoint2");
BOOST_CLASS_EXPORT_GUID(NonlinearEqualityStereoPoint2,
                        "gtsam::NonlinearEqualityStereoPoint2");
BOOST_CLASS_EXPORT_GUID(NonlinearEqualityPoint3,
                        "gtsam::NonlinearEqualityPoint3");
BOOST_CLASS_EXPORT_GUID(NonlinearEqualityRot2, "gtsam::NonlinearEqualityRot2");
BOOST_CLASS_EXPORT_GUID(NonlinearEqualityRot3, "gtsam::NonlinearEqualityRot3");
BOOST_CLASS_EXPORT_GUID(NonlinearEqualityPose2,
                        "gtsam::NonlinearEqualityPose2");
BOOST_CLASS_EXPORT_GUID(NonlinearEqualityPose3,
                        "gtsam::NonlinearEqualityPose3");
BOOST_CLASS_EXPORT_GUID(NonlinearEqualityCal3_S2,
                        "gtsam::NonlinearEqualityCal3_S2");
BOOST_CLASS_EXPORT_GUID(NonlinearEqualityCal3DS2,
                        "gtsam::NonlinearEqualityCal3DS2");
BOOST_CLASS_EXPORT_GUID(NonlinearEqualityCalibratedCamera,
                        "gtsam::NonlinearEqualityCalibratedCamera");
BOOST_CLASS_EXPORT_GUID(NonlinearEqualityStereoCamera,
                        "gtsam::NonlinearEqualityStereoCamera");

BOOST_CLASS_EXPORT_GUID(RangeFactor2D, "gtsam::RangeFactor2D");
BOOST_CLASS_EXPORT_GUID(RangeFactor3D, "gtsam::RangeFactor3D");
BOOST_CLASS_EXPORT_GUID(RangeFactorPose2, "gtsam::RangeFactorPose2");
BOOST_CLASS_EXPORT_GUID(RangeFactorPose3, "gtsam::RangeFactorPose3");
BOOST_CLASS_EXPORT_GUID(RangeFactorCalibratedCameraPoint,
                        "gtsam::RangeFactorCalibratedCameraPoint");
BOOST_CLASS_EXPORT_GUID(RangeFactorCalibratedCamera,
                        "gtsam::RangeFactorCalibratedCamera");

BOOST_CLASS_EXPORT_GUID(BearingRangeFactor2D, "gtsam::BearingRangeFactor2D");

BOOST_CLASS_EXPORT_GUID(GenericProjectionFactorCal3_S2,
                        "gtsam::GenericProjectionFactorCal3_S2");
BOOST_CLASS_EXPORT_GUID(GenericProjectionFactorCal3DS2,
                        "gtsam::GenericProjectionFactorCal3DS2");

BOOST_CLASS_EXPORT_GUID(GeneralSFMFactor2Cal3_S2,
                        "gtsam::GeneralSFMFactor2Cal3_S2");

BOOST_CLASS_EXPORT_GUID(GenericStereoFactor3D, "gtsam::GenericStereoFactor3D");

/* Create GUIDs for parameters */
/* ************************************************************************* */

BOOST_CLASS_EXPORT_GUID(gtsam::imuBias::ConstantBias,
                        "gtsam::imuBias::ConstantBias")
// BOOST_CLASS_EXPORT_GUID(gtsam::ISAM2Params, "gtsam::ISAM2Params");
/* Create GUIDs for smoother */
/* ************************************************************************* */

BOOST_SERIALIZATION_ASSUME_ABSTRACT(gtsam::FixedLagSmoother);
BOOST_CLASS_EXPORT_GUID(gtsam::ISAM2, "gtsam_ISAM2");
BOOST_CLASS_EXPORT_GUID(gtsam::IncrementalFixedLagSmoother, "gtsam_IncrementalFixedLagSmoother");

/* ************************************************************************* */
// Actual implementations of functions
/* ************************************************************************* */
std::string gtsam::serializeGraph(const NonlinearFactorGraph& graph) {
  return serialize<NonlinearFactorGraph>(graph);
}

/* ************************************************************************* */
NonlinearFactorGraph::shared_ptr gtsam::deserializeGraph(
    const std::string& serialized_graph) {
  NonlinearFactorGraph::shared_ptr result(new NonlinearFactorGraph());
  deserialize<NonlinearFactorGraph>(serialized_graph, *result);
  return result;
}

/* ************************************************************************* */
std::string gtsam::serializeGraphXML(const NonlinearFactorGraph& graph,
                                     const std::string& name) {
  return serializeXML<NonlinearFactorGraph>(graph, name);
}

/* ************************************************************************* */
NonlinearFactorGraph::shared_ptr gtsam::deserializeGraphXML(
    const std::string& serialized_graph, const std::string& name) {
  NonlinearFactorGraph::shared_ptr result(new NonlinearFactorGraph());
  deserializeXML<NonlinearFactorGraph>(serialized_graph, *result, name);
  return result;
}

/* ************************************************************************* */
std::string gtsam::serializeValues(const Values& values) {
  return serialize<Values>(values);
}

/* ************************************************************************* */
Values::shared_ptr gtsam::deserializeValues(
    const std::string& serialized_values) {
  Values::shared_ptr result(new Values());
  deserialize<Values>(serialized_values, *result);
  return result;
}

/* ************************************************************************* */
std::string gtsam::serializeValuesXML(const Values& values,
                                      const std::string& name) {
  return serializeXML<Values>(values, name);
}

/* ************************************************************************* */
Values::shared_ptr gtsam::deserializeValuesXML(
    const std::string& serialized_values, const std::string& name) {
  Values::shared_ptr result(new Values());
  deserializeXML<Values>(serialized_values, *result, name);
  return result;
}

/* ************************************************************************* */
bool gtsam::serializeGraphToFile(const NonlinearFactorGraph& graph,
                                 const std::string& fname) {
  return serializeToFile<NonlinearFactorGraph>(graph, fname);
}

/* ************************************************************************* */
bool gtsam::serializeGraphToXMLFile(const NonlinearFactorGraph& graph,
                                    const std::string& fname,
                                    const std::string& name) {
  return serializeToXMLFile<NonlinearFactorGraph>(graph, fname, name);
}

/* ************************************************************************* */
bool gtsam::serializeValuesToFile(const Values& values,
                                  const std::string& fname) {
  return serializeToFile<Values>(values, fname);
}

/* ************************************************************************* */
bool gtsam::serializeValuesToXMLFile(const Values& values,
                                     const std::string& fname,
                                     const std::string& name) {
  return serializeToXMLFile<Values>(values, fname, name);
}

/* ************************************************************************* */
NonlinearFactorGraph::shared_ptr gtsam::deserializeGraphFromFile(
    const std::string& fname) {
  NonlinearFactorGraph::shared_ptr result(new NonlinearFactorGraph());
  if (!deserializeFromFile<NonlinearFactorGraph>(fname, *result))
    throw std::invalid_argument("Failed to open file" + fname);
  return result;
}

/* ************************************************************************* */
NonlinearFactorGraph::shared_ptr gtsam::deserializeGraphFromXMLFile(
    const std::string& fname, const std::string& name) {
  NonlinearFactorGraph::shared_ptr result(new NonlinearFactorGraph());
  if (!deserializeFromXMLFile<NonlinearFactorGraph>(fname, *result, name))
    throw std::invalid_argument("Failed to open file" + fname);
  return result;
}

/* ************************************************************************* */
Values::shared_ptr gtsam::deserializeValuesFromFile(const std::string& fname) {
  Values::shared_ptr result(new Values());
  if (!deserializeFromFile<Values>(fname, *result))
    throw std::invalid_argument("Failed to open file" + fname);
  return result;
}

/* ************************************************************************* */
Values::shared_ptr gtsam::deserializeValuesFromXMLFile(
    const std::string& fname, const std::string& name) {
  Values::shared_ptr result(new Values());
  if (!deserializeFromXMLFile<Values>(fname, *result, name))
    throw std::invalid_argument("Failed to open file" + fname);
  return result;
}

/* ************************************************************************* */
ISAM2 gtsam::serializationDeepClone(ISAM2 originIsam, std::string fileName) {
  gtsam::ISAM2 copiedIsam;
  std::string smootherName = "isam2";
  std::string smootherString;
  {
    smootherString = serializeBinary<ISAM2>(originIsam, smootherName);
    // serializeToXMLFile<ISAM2>(originIsam, fileName, smootherName);

    /*std::ofstream outputStream(fileName, std::ios::out | std::ios::binary);
    boost::archive::binary_oarchive outputArchive(outputStream);
    outputArchive << originIsam;*/
  }
  {
    deserializeBinary<ISAM2>(smootherString, copiedIsam, smootherName);
    // deserializeFromXMLFile<ISAM2>(fileName, copiedIsam, smootherName);

    /*std::ifstream ifs(fileName, std::ios::in | std::ios::binary);
    boost::archive::binary_iarchive inputArchive(ifs);
    inputArchive >> copiedIsam;*/
  }
  return copiedIsam;
}
#endif

/* ************************************************************************* */