/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file dataset.h
 * @date Jan 22, 2010
 * @author nikai, Luca Carlone
 * @brief utility functions for loading datasets
 */

#pragma once

#include <gtsam/geometry/Cal3Bundler.h>
#include <gtsam/geometry/PinholeCamera.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/base/types.h>

#include <boost/smart_ptr/shared_ptr.hpp>
#include <string>
#include <utility> // for pair
#include <vector>

namespace gtsam {

#ifndef MATLAB_MEX_FILE
/**
 * Find the full path to an example dataset distributed with gtsam.  The name
 * may be specified with or without a file extension - if no extension is
 * given, this function first looks for the .graph extension, then .txt.  We
 * first check the gtsam source tree for the file, followed by the installed
 * example dataset location.  Both the source tree and installed locations
 * are obtained from CMake during compilation.
 * @return The full path and filename to the requested dataset.
 * @throw std::invalid_argument if no matching file could be found using the
 * search process described above.
 */
GTSAM_EXPORT std::string findExampleDataFile(const std::string& name);

/**
 * Creates a temporary file name that needs to be ignored in .gitingnore
 * for checking read-write oprations
 */
GTSAM_EXPORT std::string createRewrittenFileName(const std::string& name);
#endif

/// Indicates how noise parameters are stored in file
enum NoiseFormat {
  NoiseFormatG2O, ///< Information matrix I11, I12, I13, I22, I23, I33
  NoiseFormatTORO, ///< Information matrix, but inf_ff inf_fs inf_ss inf_rr inf_fr inf_sr
  NoiseFormatGRAPH, ///< default: toro-style order, but covariance matrix !
  NoiseFormatCOV, ///< Covariance matrix C11, C12, C13, C22, C23, C33
  NoiseFormatAUTO  ///< Try to guess covariance matrix layout
};

/// Robust kernel type to wrap around quadratic noise model
enum KernelFunctionType {
  KernelFunctionTypeNONE, KernelFunctionTypeHUBER, KernelFunctionTypeTUKEY
};

/// Return type for load functions
typedef std::pair<NonlinearFactorGraph::shared_ptr, Values::shared_ptr> GraphAndValues;

/**
 * Load TORO 2D Graph
 * @param dataset/model pair as constructed by [dataset]
 * @param maxID if non-zero cut out vertices >= maxID
 * @param addNoise add noise to the edges
 * @param smart try to reduce complexity of covariance to cheapest model
 */
GTSAM_EXPORT GraphAndValues load2D(
    std::pair<std::string, SharedNoiseModel> dataset, int maxID = 0,
    bool addNoise = false,
    bool smart = true, //
    NoiseFormat noiseFormat = NoiseFormatAUTO,
    KernelFunctionType kernelFunctionType = KernelFunctionTypeNONE);

/**
 * Load TORO/G2O style graph files
 * @param filename
 * @param model optional noise model to use instead of one specified by file
 * @param maxID if non-zero cut out vertices >= maxID
 * @param addNoise add noise to the edges
 * @param smart try to reduce complexity of covariance to cheapest model
 * @param noiseFormat how noise parameters are stored
 * @param kernelFunctionType whether to wrap the noise model in a robust kernel
 * @return graph and initial values
 */
GTSAM_EXPORT GraphAndValues load2D(const std::string& filename,
    SharedNoiseModel model = SharedNoiseModel(), Key maxID = 0, bool addNoise =
        false, bool smart = true, NoiseFormat noiseFormat = NoiseFormatAUTO, //
    KernelFunctionType kernelFunctionType = KernelFunctionTypeNONE);

/// @deprecated load2D now allows for arbitrary models and wrapping a robust kernel
GTSAM_EXPORT GraphAndValues load2D_robust(const std::string& filename,
    noiseModel::Base::shared_ptr& model, int maxID = 0);

/** save 2d graph */
GTSAM_EXPORT void save2D(const NonlinearFactorGraph& graph,
    const Values& config, const noiseModel::Diagonal::shared_ptr model,
    const std::string& filename);

/**
 * @brief This function parses a g2o file and stores the measurements into a
 * NonlinearFactorGraph and the initial guess in a Values structure
 * @param filename The name of the g2o file\
 * @param is3D indicates if the file describes a 2D or 3D problem
 * @param kernelFunctionType whether to wrap the noise model in a robust kernel
 * @return graph and initial values
 */
GTSAM_EXPORT GraphAndValues readG2o(const std::string& g2oFile, const bool is3D = false,
    KernelFunctionType kernelFunctionType = KernelFunctionTypeNONE);

/**
 * @brief This function writes a g2o file from
 * NonlinearFactorGraph and a Values structure
 * @param filename The name of the g2o file to write
 * @param graph NonlinearFactor graph storing the measurements
 * @param estimate Values
 */
GTSAM_EXPORT void writeG2o(const NonlinearFactorGraph& graph,
    const Values& estimate, const std::string& filename);

/**
 * Load TORO 3D Graph
 */
GTSAM_EXPORT GraphAndValues load3D(const std::string& filename);

/// A measurement with its camera index
typedef std::pair<size_t, Point2> SfM_Measurement;

/// SfM_Track
typedef std::pair<size_t, size_t> SIFT_Index;

/// Define the structure for the 3D points
struct SfM_Track {
  SfM_Track():p(0,0,0) {}
  Point3 p; ///< 3D position of the point
  float r, g, b; ///< RGB color of the 3D point
  std::vector<SfM_Measurement> measurements; ///< The 2D image projections (id,(u,v))
  std::vector<SIFT_Index> siftIndices;
  size_t number_measurements() const {
    return measurements.size();
  }
};

/// Define the structure for the camera poses
typedef PinholeCamera<Cal3Bundler> SfM_Camera;

/// Define the structure for SfM data
struct SfM_data {
  std::vector<SfM_Camera> cameras; ///< Set of cameras
  std::vector<SfM_Track> tracks; ///< Sparse set of points
  size_t number_cameras() const {
    return cameras.size();
  } ///< The number of camera poses
  size_t number_tracks() const {
    return tracks.size();
  } ///< The number of reconstructed 3D points
};

/**
 * @brief This function parses a bundler output file and stores the data into a
 * SfM_data structure
 * @param filename The name of the bundler file
 * @param data SfM structure where the data is stored
 * @return true if the parsing was successful, false otherwise
 */
GTSAM_EXPORT bool readBundler(const std::string& filename, SfM_data &data);

/**
 * @brief This function parses a "Bundle Adjustment in the Large" (BAL) file and stores the data into a
 * SfM_data structure
 * @param filename The name of the BAL file
 * @param data SfM structure where the data is stored
 * @return true if the parsing was successful, false otherwise
 */
GTSAM_EXPORT bool readBAL(const std::string& filename, SfM_data &data);

/**
 * @brief This function writes a "Bundle Adjustment in the Large" (BAL) file from a
 * SfM_data structure
 * @param filename The name of the BAL file to write
 * @param data SfM structure where the data is stored
 * @return true if the parsing was successful, false otherwise
 */
GTSAM_EXPORT bool writeBAL(const std::string& filename, SfM_data &data);

/**
 * @brief This function writes a "Bundle Adjustment in the Large" (BAL) file from a
 * SfM_data structure and a value structure (measurements are the same as the SfM input data,
 * while camera poses and values are read from Values)
 * @param filename The name of the BAL file to write
 * @param data SfM structure where the data is stored
 * @param values structure where the graph values are stored (values can be either Pose3 or PinholeCamera<Cal3Bundler> for the
 * cameras, and should be Point3 for the 3D points). Note that the current version
 * assumes that the keys are "x1" for pose 1 (or "c1" for camera 1) and "l1" for landmark 1
 * @return true if the parsing was successful, false otherwise
 */
GTSAM_EXPORT bool writeBALfromValues(const std::string& filename,
    const SfM_data &data, Values& values);

/**
 * @brief This function converts an openGL camera pose to an GTSAM camera pose
 * @param R rotation in openGL
 * @param tx x component of the translation in openGL
 * @param ty y component of the translation in openGL
 * @param tz z component of the translation in openGL
 * @return Pose3 in GTSAM format
 */
GTSAM_EXPORT Pose3 openGL2gtsam(const Rot3& R, double tx, double ty, double tz);

/**
 * @brief This function converts a GTSAM camera pose to an openGL camera pose
 * @param R rotation in GTSAM
 * @param tx x component of the translation in GTSAM
 * @param ty y component of the translation in GTSAM
 * @param tz z component of the translation in GTSAM
 * @return Pose3 in openGL format
 */
GTSAM_EXPORT Pose3 gtsam2openGL(const Rot3& R, double tx, double ty, double tz);

/**
 * @brief This function converts a GTSAM camera pose to an openGL camera pose
 * @param PoseGTSAM pose in GTSAM format
 * @return Pose3 in openGL format
 */
GTSAM_EXPORT Pose3 gtsam2openGL(const Pose3& PoseGTSAM);

/**
 * @brief This function creates initial values for cameras from db
 * @param SfM_data
 * @return Values
 */
GTSAM_EXPORT Values initialCamerasEstimate(const SfM_data& db);

/**
 * @brief This function creates initial values for cameras and points from db
 * @param SfM_data
 * @return Values
 */
GTSAM_EXPORT Values initialCamerasAndPointsEstimate(const SfM_data& db);

} // namespace gtsam
