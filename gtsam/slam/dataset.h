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
 * @author Ni Kai
 * @author Luca Carlone
 * @author Varun Agrawal
 * @brief utility functions for loading datasets
 */

#pragma once

#include <gtsam/sfm/BinaryMeasurement.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/sfm/SfmData.h>
#include <gtsam/geometry/Cal3Bundler.h>
#include <gtsam/geometry/PinholeCamera.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/base/serialization.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/types.h>

#include <boost/smart_ptr/shared_ptr.hpp>
#include <string>
#include <utility> // for pair
#include <vector>
#include <iosfwd>
#include <map>

namespace gtsam {

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

/**
 * Parse variables in a line-based text format (like g2o) into a map.
 * Instantiated in .cpp Pose2, Point2, Pose3, and Point3.
 * Note the map keys are integer indices, *not* gtsam::Keys. This is is
 * different below where landmarks will use L(index) symbols.
 */
template <typename T>
GTSAM_EXPORT std::map<size_t, T> parseVariables(const std::string &filename,
                                                size_t maxIndex = 0);

/**
 * Parse binary measurements in a line-based text format (like g2o) into a
 * vector. Instantiated in .cpp for Pose2, Rot2, Pose3, and Rot3. The rotation
 * versions parse poses and extract only the rotation part, using the marginal
 * covariance as noise model.
 */
template <typename T>
GTSAM_EXPORT std::vector<BinaryMeasurement<T>>
parseMeasurements(const std::string &filename,
                  const noiseModel::Diagonal::shared_ptr &model = nullptr,
                  size_t maxIndex = 0);

/**
 * Parse BetweenFactors in a line-based text format (like g2o) into a vector of
 * shared pointers. Instantiated in .cpp T equal to Pose2 and Pose3.
 */
template <typename T>
GTSAM_EXPORT std::vector<typename BetweenFactor<T>::shared_ptr>
parseFactors(const std::string &filename,
             const noiseModel::Diagonal::shared_ptr &model = nullptr,
             size_t maxIndex = 0);

/// Return type for auxiliary functions
typedef std::pair<size_t, Pose2> IndexedPose;
typedef std::pair<size_t, Point2> IndexedLandmark;
typedef std::pair<std::pair<size_t, size_t>, Pose2> IndexedEdge;

/**
 * Parse TORO/G2O vertex "id x y yaw"
 * @param is input stream
 * @param tag string parsed from input stream, will only parse if vertex type
 */
GTSAM_EXPORT boost::optional<IndexedPose> parseVertexPose(std::istream& is,
    const std::string& tag);

/**
 * Parse G2O landmark vertex "id x y"
 * @param is input stream
 * @param tag string parsed from input stream, will only parse if vertex type
 */
GTSAM_EXPORT boost::optional<IndexedLandmark> parseVertexLandmark(std::istream& is,
    const std::string& tag);

/**
 * Parse TORO/G2O edge "id1 id2 x y yaw"
 * @param is input stream
 * @param tag string parsed from input stream, will only parse if edge type
 */
GTSAM_EXPORT boost::optional<IndexedEdge> parseEdge(std::istream& is,
    const std::string& tag);

/// Return type for load functions, which return a graph and initial values. For
/// landmarks, the gtsam::Symbol L(index) is used to insert into the Values.
/// Bearing-range measurements also refer to landmarks with L(index).
using GraphAndValues =
    std::pair<NonlinearFactorGraph::shared_ptr, Values::shared_ptr>;

/**
 * Load TORO 2D Graph
 * @param dataset/model pair as constructed by [dataset]
 * @param maxIndex if non-zero cut out vertices >= maxIndex
 * @param addNoise add noise to the edges
 * @param smart try to reduce complexity of covariance to cheapest model
 */
GTSAM_EXPORT GraphAndValues load2D(
    std::pair<std::string, SharedNoiseModel> dataset, size_t maxIndex = 0,
    bool addNoise = false,
    bool smart = true, //
    NoiseFormat noiseFormat = NoiseFormatAUTO,
    KernelFunctionType kernelFunctionType = KernelFunctionTypeNONE);

/**
 * Load TORO/G2O style graph files
 * @param filename
 * @param model optional noise model to use instead of one specified by file
 * @param maxIndex if non-zero cut out vertices >= maxIndex
 * @param addNoise add noise to the edges
 * @param smart try to reduce complexity of covariance to cheapest model
 * @param noiseFormat how noise parameters are stored
 * @param kernelFunctionType whether to wrap the noise model in a robust kernel
 * @return graph and initial values
 */
GTSAM_EXPORT GraphAndValues
load2D(const std::string& filename, SharedNoiseModel model = SharedNoiseModel(),
       size_t maxIndex = 0, bool addNoise = false, bool smart = true,
       NoiseFormat noiseFormat = NoiseFormatAUTO,  //
       KernelFunctionType kernelFunctionType = KernelFunctionTypeNONE);

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
GTSAM_EXPORT GraphAndValues
readG2o(const std::string& g2oFile, const bool is3D = false,
        KernelFunctionType kernelFunctionType = KernelFunctionTypeNONE);

/**
 * @brief This function writes a g2o file from
 * NonlinearFactorGraph and a Values structure
 * @param filename The name of the g2o file to write
 * @param graph NonlinearFactor graph storing the measurements
 * @param estimate Values
 *
 * Note:behavior change in PR #471: to be consistent with load2D and load3D, we
 * write the *indices* to file and not the full Keys. This change really only
 * affects landmarks, which get read as indices but stored in values with the
 * symbol L(index).
 */
GTSAM_EXPORT void writeG2o(const NonlinearFactorGraph& graph,
    const Values& estimate, const std::string& filename);

/// Load TORO 3D Graph
GTSAM_EXPORT GraphAndValues load3D(const std::string& filename);

// Wrapper-friendly versions of parseFactors<Pose2> and parseFactors<Pose2>
using BetweenFactorPose2s = std::vector<BetweenFactor<Pose2>::shared_ptr>;
GTSAM_EXPORT BetweenFactorPose2s
parse2DFactors(const std::string &filename,
               const noiseModel::Diagonal::shared_ptr &model = nullptr,
               size_t maxIndex = 0);

using BetweenFactorPose3s = std::vector<BetweenFactor<Pose3>::shared_ptr>;
GTSAM_EXPORT BetweenFactorPose3s
parse3DFactors(const std::string &filename,
               const noiseModel::Diagonal::shared_ptr &model = nullptr,
               size_t maxIndex = 0);

using BinaryMeasurementsUnit3 = std::vector<BinaryMeasurement<Unit3>>;
using BinaryMeasurementsPoint3 = std::vector<BinaryMeasurement<Point3>>;
using BinaryMeasurementsRot3 = std::vector<BinaryMeasurement<Rot3>>;
}  // namespace gtsam
