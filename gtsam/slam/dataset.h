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
GTSAM_EXPORT GraphAndValues load2D(const std::string& filename,
    SharedNoiseModel model = SharedNoiseModel(), size_t maxIndex = 0, bool addNoise =
        false, bool smart = true, NoiseFormat noiseFormat = NoiseFormatAUTO, //
    KernelFunctionType kernelFunctionType = KernelFunctionTypeNONE);

/// @deprecated load2D now allows for arbitrary models and wrapping a robust kernel
GTSAM_EXPORT GraphAndValues load2D_robust(const std::string& filename,
    const noiseModel::Base::shared_ptr& model, size_t maxIndex = 0);

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

/// A measurement with its camera index
typedef std::pair<size_t, Point2> SfmMeasurement;

/// Sift index for SfmTrack
typedef std::pair<size_t, size_t> SiftIndex;

/// Define the structure for the 3D points
struct SfmTrack {
  SfmTrack(float r = 0, float g = 0, float b = 0): p(0,0,0), r(r), g(g), b(b) {}
  SfmTrack(const gtsam::Point3& pt, float r = 0, float g = 0, float b = 0) : p(pt), r(r), g(g), b(b) {}
 
  Point3 p; ///< 3D position of the point
  float r, g, b; ///< RGB color of the 3D point
  std::vector<SfmMeasurement> measurements; ///< The 2D image projections (id,(u,v))
  std::vector<SiftIndex> siftIndices;
  
  /// Get RGB values describing 3d point
  const Point3 rgb() const { return Point3(r, g, b); }

  /// Total number of measurements in this track
  size_t number_measurements() const {
    return measurements.size();
  }
  /// Get the measurement (camera index, Point2) at pose index `idx`
  SfmMeasurement measurement(size_t idx) const {
    return measurements[idx];
  }
  /// Get the SIFT feature index corresponding to the measurement at `idx`
  SiftIndex siftIndex(size_t idx) const {
    return siftIndices[idx];
  }
  /// Get 3D point
  const Point3& point3() const {
    return p;
  }
  /// Add measurement (camera_idx, Point2) to track
  void add_measurement(size_t idx, const gtsam::Point2& m) {
    measurements.emplace_back(idx, m);
  }

  /** Serialization function */
  friend class boost::serialization::access;
  template<class ARCHIVE>
  void serialize(ARCHIVE & ar, const unsigned int /*version*/) {
    ar & BOOST_SERIALIZATION_NVP(p);
    ar & BOOST_SERIALIZATION_NVP(r);
    ar & BOOST_SERIALIZATION_NVP(g);
    ar & BOOST_SERIALIZATION_NVP(b);
    ar & BOOST_SERIALIZATION_NVP(measurements);
    ar & BOOST_SERIALIZATION_NVP(siftIndices);
  }

  /// assert equality up to a tolerance
  bool equals(const SfmTrack &sfmTrack, double tol = 1e-9) const {
    // check the 3D point
    if (!p.isApprox(sfmTrack.p)) {
      return false;
    }

    // check the RGB values
    if (r!=sfmTrack.r || g!=sfmTrack.g || b!=sfmTrack.b) {
      return false;
    }

    // compare size of vectors for measurements and siftIndices
    if (number_measurements() != sfmTrack.number_measurements() ||
        siftIndices.size() != sfmTrack.siftIndices.size()) {
      return false;
    }

    // compare measurements (order sensitive)
    for (size_t idx = 0; idx < number_measurements(); ++idx) {
      SfmMeasurement measurement = measurements[idx];
      SfmMeasurement otherMeasurement = sfmTrack.measurements[idx];

      if (measurement.first != otherMeasurement.first ||
          !measurement.second.isApprox(otherMeasurement.second)) {
        return false;
      }
    }

    // compare sift indices (order sensitive)
    for (size_t idx = 0; idx < siftIndices.size(); ++idx) {
      SiftIndex index = siftIndices[idx];
      SiftIndex otherIndex = sfmTrack.siftIndices[idx];

      if (index.first != otherIndex.first ||
          index.second != otherIndex.second) {
        return false;
      }
    }

    return true;
  }

  /// print
  void print(const std::string& s = "") const {
    std::cout << "Track with " << measurements.size();
    std::cout << " measurements of point " << p << std::endl;
  }
};

/* ************************************************************************* */
/// traits
template<>
struct traits<SfmTrack> : public Testable<SfmTrack> {
};


/// Define the structure for the camera poses
typedef PinholeCamera<Cal3Bundler> SfmCamera;

/// Define the structure for SfM data
struct SfmData {
  std::vector<SfmCamera> cameras; ///< Set of cameras
  std::vector<SfmTrack> tracks; ///< Sparse set of points
  size_t number_cameras() const {
    return cameras.size();
  }
  /// The number of reconstructed 3D points
  size_t number_tracks() const {
    return tracks.size();
  }
  /// The camera pose at frame index `idx`
  SfmCamera camera(size_t idx) const {
    return cameras[idx];
  }
  /// The track formed by series of landmark measurements
  SfmTrack track(size_t idx) const {
    return tracks[idx];
  }
  /// Add a track to SfmData
  void add_track(const SfmTrack& t) {
    tracks.push_back(t);
  }
  /// Add a camera to SfmData
  void add_camera(const SfmCamera& cam) {
    cameras.push_back(cam);
  }

  /** Serialization function */
  friend class boost::serialization::access;
  template<class Archive>
  void serialize(Archive & ar, const unsigned int /*version*/) {
    ar & BOOST_SERIALIZATION_NVP(cameras);
    ar & BOOST_SERIALIZATION_NVP(tracks);
  }

  /// @}
  /// @name Testable
  /// @{

  /// assert equality up to a tolerance
  bool equals(const SfmData &sfmData, double tol = 1e-9) const {
    // check number of cameras and tracks
    if (number_cameras() != sfmData.number_cameras() ||
        number_tracks() != sfmData.number_tracks()) {
      return false;
    }

    // check each camera
    for (size_t i = 0; i < number_cameras(); ++i) {
      if (!camera(i).equals(sfmData.camera(i), tol)) {
        return false;
      }
    }

    // check each track
    for (size_t j = 0; j < number_tracks(); ++j) {
      if (!track(j).equals(sfmData.track(j), tol)) {
        return false;
      }
    }

    return true;
  }

  /// print
  void print(const std::string& s = "") const {
    std::cout << "Number of cameras = " << number_cameras() << std::endl;
    std::cout << "Number of tracks = " << number_tracks() << std::endl;
  }
};

/* ************************************************************************* */
/// traits
template<>
struct traits<SfmData> : public Testable<SfmData> {
};

/**
 * @brief This function parses a bundler output file and stores the data into a
 * SfmData structure
 * @param filename The name of the bundler file
 * @param data SfM structure where the data is stored
 * @return true if the parsing was successful, false otherwise
 */
GTSAM_EXPORT bool readBundler(const std::string& filename, SfmData &data);

/**
 * @brief This function parses a "Bundle Adjustment in the Large" (BAL) file and stores the data into a
 * SfmData structure
 * @param filename The name of the BAL file
 * @param data SfM structure where the data is stored
 * @return true if the parsing was successful, false otherwise
 */
GTSAM_EXPORT bool readBAL(const std::string& filename, SfmData &data);

/**
 * @brief This function parses a "Bundle Adjustment in the Large" (BAL) file and returns the data
 * as a SfmData structure. Mainly used by wrapped code.
 * @param filename The name of the BAL file.
 * @return SfM structure where the data is stored.
 */
GTSAM_EXPORT SfmData readBal(const std::string& filename);

/**
 * @brief This function writes a "Bundle Adjustment in the Large" (BAL) file from a
 * SfmData structure
 * @param filename The name of the BAL file to write
 * @param data SfM structure where the data is stored
 * @return true if the parsing was successful, false otherwise
 */
GTSAM_EXPORT bool writeBAL(const std::string& filename, SfmData &data);

/**
 * @brief This function writes a "Bundle Adjustment in the Large" (BAL) file from a
 * SfmData structure and a value structure (measurements are the same as the SfM input data,
 * while camera poses and values are read from Values)
 * @param filename The name of the BAL file to write
 * @param data SfM structure where the data is stored
 * @param values structure where the graph values are stored (values can be either Pose3 or PinholeCamera<Cal3Bundler> for the
 * cameras, and should be Point3 for the 3D points). Note that the current version
 * assumes that the keys are "x1" for pose 1 (or "c1" for camera 1) and "l1" for landmark 1
 * @return true if the parsing was successful, false otherwise
 */
GTSAM_EXPORT bool writeBALfromValues(const std::string& filename,
    const SfmData &data, Values& values);

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
 * @param SfmData
 * @return Values
 */
GTSAM_EXPORT Values initialCamerasEstimate(const SfmData& db);

/**
 * @brief This function creates initial values for cameras and points from db
 * @param SfmData
 * @return Values
 */
GTSAM_EXPORT Values initialCamerasAndPointsEstimate(const SfmData& db);

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
#ifdef GTSAM_ALLOW_DEPRECATED_SINCE_V41
inline boost::optional<IndexedPose> parseVertex(std::istream &is,
                                                const std::string &tag) {
  return parseVertexPose(is, tag);
}

GTSAM_EXPORT std::map<size_t, Pose3> parse3DPoses(const std::string &filename,
                                                  size_t maxIndex = 0);

GTSAM_EXPORT std::map<size_t, Point3>
parse3DLandmarks(const std::string &filename, size_t maxIndex = 0);

#endif
}  // namespace gtsam
