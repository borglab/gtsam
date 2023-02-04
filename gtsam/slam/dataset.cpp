/* ----------------------------------------------------------------------------
 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file dataset.cpp
 * @date Jan 22, 2010
 * @author Kai Ni
 * @author Luca Carlone
 * @author Frank Dellaert
 * @author Varun Agrawal
 * @brief utility functions for loading datasets
 */

#include <gtsam/sam/BearingRangeFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/dataset.h>

#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Rot3.h>

#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/nonlinear/Values-inl.h>

#include <gtsam/linear/Sampler.h>

#include <gtsam/inference/FactorGraph.h>
#include <gtsam/inference/Symbol.h>

#include <gtsam/base/GenericValue.h>
#include <gtsam/base/Lie.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Value.h>
#include <gtsam/base/Vector.h>
#include <gtsam/base/types.h>

#include <optional>

#include <cmath>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>

#if defined(__GNUC__) && (__GNUC__ == 7)
#include <experimental/filesystem>
#else
#include <filesystem>
#endif

namespace fs = std::filesystem;
using gtsam::symbol_shorthand::L;

using std::cout;
using std::endl;

#define LINESIZE 81920

namespace gtsam {

/* ************************************************************************* */
std::string findExampleDataFile(const std::string &name) {
  // Search source tree and installed location
  std::vector<std::string> rootsToSearch;

  // Constants below are defined by CMake, see gtsam/gtsam/CMakeLists.txt
  rootsToSearch.push_back(GTSAM_SOURCE_TREE_DATASET_DIR);
  rootsToSearch.push_back(GTSAM_INSTALLED_DATASET_DIR);

  // Search for filename as given, and with .graph and .txt extensions
  std::vector<std::string> namesToSearch;
  namesToSearch.push_back(name);
  namesToSearch.push_back(name + ".graph");
  namesToSearch.push_back(name + ".txt");
  namesToSearch.push_back(name + ".out");
  namesToSearch.push_back(name + ".xml");
  namesToSearch.push_back(name + ".g2o");

  // Find first name that exists
  for (const fs::path root : rootsToSearch) {
    for (const fs::path name : namesToSearch) {
      if (fs::is_regular_file(root / name))
        return (root / name).string();
    }
  }

  // If we did not return already, then we did not find the file
  throw std::invalid_argument(
      "gtsam::findExampleDataFile could not find a matching "
      "file in\n" GTSAM_SOURCE_TREE_DATASET_DIR
      " or\n" GTSAM_INSTALLED_DATASET_DIR " named\n" +
      name + ", " + name + ".g2o, " + ", " + name + ".graph, or " + name +
      ".txt");
}

/* ************************************************************************* */
std::string createRewrittenFileName(const std::string &name) {
  // Search source tree and installed location
  if (!exists(fs::path(name))) {
    throw std::invalid_argument(
        "gtsam::createRewrittenFileName could not find a matching file in\n" +
        name);
  }

  fs::path p(name);
  fs::path newpath = fs::path(p.parent_path().string()) /
                     fs::path(p.stem().string() + "-rewritten.txt");

  return newpath.string();
}

/* ************************************************************************* */
// Type for parser functions used in parseLines below.
template <typename T>
using Parser =
    std::function<std::optional<T>(std::istream &is, const std::string &tag)>;

// Parse a file by calling the parse(is, tag) function for every line.
// The result of calling the function is ignored, so typically parse function
// works via a side effect, like adding a factor into a graph etc.
template <typename T>
static void parseLines(const std::string &filename, Parser<T> parse) {
  std::ifstream is(filename.c_str());
  if (!is)
    throw std::invalid_argument("parse: can not find file " + filename);
  std::string tag;
  while (is >> tag) {
    parse(is, tag); // ignore return value
    is.ignore(LINESIZE, '\n');
  }
}

/* ************************************************************************* */
// Parse types T into a size_t-indexed map
template <typename T>
std::map<size_t, T> parseToMap(const std::string &filename, Parser<std::pair<size_t, T>> parse,
                          size_t maxIndex) {
  std::map<size_t, T> result;
  Parser<std::pair<size_t, T>> emplace = [&](std::istream &is, const std::string &tag) {
    if (auto t = parse(is, tag)) {
      if (!maxIndex || t->first <= maxIndex)
        result.emplace(*t);
    }
    return std::nullopt;
  };
  parseLines(filename, emplace);
  return result;
}

/* ************************************************************************* */
// Parse a file and push results on a vector
template <typename T>
static std::vector<T> parseToVector(const std::string &filename, Parser<T> parse) {
  std::vector<T> result;
  Parser<T> add = [&result, parse](std::istream &is, const std::string &tag) {
    if (auto t = parse(is, tag))
      result.push_back(*t);
    return std::nullopt;
  };
  parseLines(filename, add);
  return result;
}

/* ************************************************************************* */
std::optional<IndexedPose> parseVertexPose(std::istream &is, const std::string &tag) {
  if ((tag == "VERTEX2") || (tag == "VERTEX_SE2") || (tag == "VERTEX")) {
    size_t id;
    double x, y, yaw;
    if (!(is >> id >> x >> y >> yaw)) {
      throw std::runtime_error("parseVertexPose encountered malformed line");
    }
    return IndexedPose(id, Pose2(x, y, yaw));
  } else {
    return std::nullopt;
  }
}

template <>
GTSAM_EXPORT std::map<size_t, Pose2> parseVariables<Pose2>(
    const std::string &filename, size_t maxIndex) {
  return parseToMap<Pose2>(filename, parseVertexPose, maxIndex);
}

/* ************************************************************************* */
std::optional<IndexedLandmark> parseVertexLandmark(std::istream &is,
                                                     const std::string &tag) {
  if (tag == "VERTEX_XY") {
    size_t id;
    double x, y;
    if (!(is >> id >> x >> y)) {
      throw std::runtime_error(
          "parseVertexLandmark encountered malformed line");
    }
    return IndexedLandmark(id, Point2(x, y));
  } else {
    return std::nullopt;
  }
}

template <>
GTSAM_EXPORT std::map<size_t, Point2> parseVariables<Point2>(
    const std::string &filename, size_t maxIndex) {
  return parseToMap<Point2>(filename, parseVertexLandmark, maxIndex);
}

/* ************************************************************************* */
// Interpret noise parameters according to flags
static SharedNoiseModel createNoiseModel(
    const Vector6 &v, bool smart, NoiseFormat noiseFormat,
    KernelFunctionType kernelFunctionType) {
  if (noiseFormat == NoiseFormatAUTO) {
    // Try to guess covariance matrix layout
    if (v(0) != 0.0 && v(1) == 0.0 && v(2) != 0.0 &&  //
        v(3) != 0.0 && v(4) == 0.0 && v(5) == 0.0) {
      noiseFormat = NoiseFormatGRAPH;
    } else if (v(0) != 0.0 && v(1) == 0.0 && v(2) == 0.0 &&  //
               v(3) != 0.0 && v(4) == 0.0 && v(5) != 0.0) {
      noiseFormat = NoiseFormatCOV;
    } else {
      throw std::invalid_argument(
          "load2D: unrecognized covariance matrix format in dataset file. "
          "Please specify the noise format.");
    }
  }

  // Read matrix and check that diagonal entries are non-zero
  Matrix3 M;
  switch (noiseFormat) {
  case NoiseFormatG2O:
  case NoiseFormatCOV:
    // i.e., [v(0)  v(1)  v(2);
    //        v(1)' v(3)  v(4);
    //        v(2)' v(4)' v(5) ]
    if (v(0) == 0.0 || v(3) == 0.0 || v(5) == 0.0)
      throw std::runtime_error(
          "load2D::readNoiseModel looks like this is not G2O matrix order");
    M << v(0), v(1), v(2), v(1), v(3), v(4), v(2), v(4), v(5);
    break;
  case NoiseFormatTORO:
  case NoiseFormatGRAPH:
    // http://www.openslam.org/toro.html
    // inf_ff inf_fs inf_ss inf_rr inf_fr inf_sr
    // i.e., [v(0)  v(1)  v(4);
    //        v(1)' v(2)  v(5);
    //        v(4)' v(5)' v(3) ]
    if (v(0) == 0.0 || v(2) == 0.0 || v(3) == 0.0)
      throw std::invalid_argument(
          "load2D::readNoiseModel looks like this is not TORO matrix order");
    M << v(0), v(1), v(4), v(1), v(2), v(5), v(4), v(5), v(3);
    break;
  default:
    throw std::runtime_error("load2D: invalid noise format");
  }

  // Now, create a Gaussian noise model
  // The smart flag will try to detect a simpler model, e.g., unit
  SharedNoiseModel model;
  switch (noiseFormat) {
  case NoiseFormatG2O:
  case NoiseFormatTORO:
    // In both cases, what is stored in file is the information matrix
    model = noiseModel::Gaussian::Information(M, smart);
    break;
  case NoiseFormatGRAPH:
  case NoiseFormatCOV:
    // These cases expect covariance matrix
    model = noiseModel::Gaussian::Covariance(M, smart);
    break;
  default:
    throw std::invalid_argument("load2D: invalid noise format");
  }

  switch (kernelFunctionType) {
  case KernelFunctionTypeNONE:
    return model;
    break;
  case KernelFunctionTypeHUBER:
    return noiseModel::Robust::Create(
        noiseModel::mEstimator::Huber::Create(1.345), model);
    break;
  case KernelFunctionTypeTUKEY:
    return noiseModel::Robust::Create(
        noiseModel::mEstimator::Tukey::Create(4.6851), model);
    break;
  default:
    throw std::invalid_argument("load2D: invalid kernel function type");
  }
}

/* ************************************************************************* */
std::optional<IndexedEdge> parseEdge(std::istream &is, const std::string &tag) {
  if ((tag == "EDGE2") || (tag == "EDGE") || (tag == "EDGE_SE2") ||
      (tag == "ODOMETRY")) {

    size_t id1, id2;
    double x, y, yaw;
    if (!(is >> id1 >> id2 >> x >> y >> yaw)) {
      throw std::runtime_error("parseEdge encountered malformed line");
    }
    return IndexedEdge({id1, id2}, Pose2(x, y, yaw));
  } else {
    return std::nullopt;
  }
}

/* ************************************************************************* */
// Measurement parsers are implemented as a functor, as they have several
// options to filter and create the measurement model.
template <typename T> struct ParseMeasurement;

/* ************************************************************************* */
// Converting from Measurement to BetweenFactor is generic
template <typename T> struct ParseFactor : ParseMeasurement<T> {
  ParseFactor(const ParseMeasurement<T> &parent)
      : ParseMeasurement<T>(parent) {}

  // We parse a measurement then convert
  typename std::optional<typename BetweenFactor<T>::shared_ptr>
  operator()(std::istream &is, const std::string &tag) {
    if (auto m = ParseMeasurement<T>::operator()(is, tag))
      return std::make_shared<BetweenFactor<T>>(
          m->key1(), m->key2(), m->measured(), m->noiseModel());
    else
      return std::nullopt;
  }
};

/* ************************************************************************* */
// Pose2 measurement parser
template <> struct ParseMeasurement<Pose2> {
  // The arguments
  std::shared_ptr<Sampler> sampler;
  size_t maxIndex;

  // For noise model creation
  bool smart;
  NoiseFormat noiseFormat;
  KernelFunctionType kernelFunctionType;

  // If this is not null, will use instead of parsed model:
  SharedNoiseModel model;

  // The actual parser
  std::optional<BinaryMeasurement<Pose2>> operator()(std::istream &is,
                                                       const std::string &tag) {
    auto edge = parseEdge(is, tag);
    if (!edge)
      return std::nullopt;

    // parse noise model
    Vector6 v;
    is >> v(0) >> v(1) >> v(2) >> v(3) >> v(4) >> v(5);

    // optional filter
    size_t id1, id2;
    std::tie(id1, id2) = edge->first;
    if (maxIndex && (id1 > maxIndex || id2 > maxIndex))
      return std::nullopt;

    // Get pose and optionally add noise
    Pose2 &pose = edge->second;
    if (sampler)
      pose = pose.retract(sampler->sample());

    // emplace measurement
    auto modelFromFile =
        createNoiseModel(v, smart, noiseFormat, kernelFunctionType);
    return BinaryMeasurement<Pose2>(id1, id2, pose,
                                    model ? model : modelFromFile);
  }
};

/* ************************************************************************* */
// Create a sampler to corrupt a measurement
std::shared_ptr<Sampler> createSampler(const SharedNoiseModel &model) {
  auto noise = std::dynamic_pointer_cast<noiseModel::Diagonal>(model);
  if (!noise)
    throw std::invalid_argument("gtsam::load: invalid noise model for adding noise"
                           "(current version assumes diagonal noise model)!");
  return std::shared_ptr<Sampler>(new Sampler(noise));
}

/* ************************************************************************* */
// Implementation of parseMeasurements for Pose2
template <>
GTSAM_EXPORT
std::vector<BinaryMeasurement<Pose2>>
parseMeasurements(const std::string &filename,
                  const noiseModel::Diagonal::shared_ptr &model,
                  size_t maxIndex) {
  ParseMeasurement<Pose2> parse{model ? createSampler(model) : nullptr,
                                maxIndex, true, NoiseFormatAUTO,
                                KernelFunctionTypeNONE, nullptr};
  return parseToVector<BinaryMeasurement<Pose2>>(filename, parse);
}

/* ************************************************************************* */
// Implementation of parseMeasurements for Rot2, which converts from Pose2

// Extract Rot2 measurement from Pose2 measurement
static BinaryMeasurement<Rot2> convert(const BinaryMeasurement<Pose2> &p) {
  auto gaussian =
      std::dynamic_pointer_cast<noiseModel::Gaussian>(p.noiseModel());
  if (!gaussian)
    throw std::invalid_argument(
        "parseMeasurements<Rot2> can only convert Pose2 measurements "
        "with Gaussian noise models.");
  const Matrix3 M = gaussian->covariance();
  return BinaryMeasurement<Rot2>(p.key1(), p.key2(), p.measured().rotation(),
                                 noiseModel::Isotropic::Variance(1, M(2, 2)));
}

template <>
GTSAM_EXPORT
std::vector<BinaryMeasurement<Rot2>>
parseMeasurements(const std::string &filename,
                  const noiseModel::Diagonal::shared_ptr &model,
                  size_t maxIndex) {
  auto poses = parseMeasurements<Pose2>(filename, model, maxIndex);
  std::vector<BinaryMeasurement<Rot2>> result;
  result.reserve(poses.size());
  for (const auto &p : poses)
    result.push_back(convert(p));
  return result;
}

/* ************************************************************************* */
// Implementation of parseFactors for Pose2
template <>
GTSAM_EXPORT
std::vector<BetweenFactor<Pose2>::shared_ptr>
parseFactors<Pose2>(const std::string &filename,
                    const noiseModel::Diagonal::shared_ptr &model,
                    size_t maxIndex) {
  ParseFactor<Pose2> parse({model ? createSampler(model) : nullptr, maxIndex,
                            true, NoiseFormatAUTO, KernelFunctionTypeNONE,
                            nullptr});
  return parseToVector<BetweenFactor<Pose2>::shared_ptr>(filename, parse);
}

/* ************************************************************************* */
using BearingRange2D = BearingRange<Pose2, Point2>;

template <> struct ParseMeasurement<BearingRange2D> {
  // arguments
  size_t maxIndex;

  // The actual parser
  std::optional<BinaryMeasurement<BearingRange2D>>
  operator()(std::istream &is, const std::string &tag) {
    if (tag != "BR" && tag != "LANDMARK")
      return std::nullopt;

    size_t id1, id2;
    is >> id1 >> id2;
    double bearing, range, bearing_std, range_std;

    if (tag == "BR") {
      is >> bearing >> range >> bearing_std >> range_std;
    }

    // A landmark measurement, converted to bearing-range
    if (tag == "LANDMARK") {
      double lmx, lmy;
      double v1, v2, v3;

      is >> lmx >> lmy >> v1 >> v2 >> v3;

      // Convert x,y to bearing,range
      bearing = atan2(lmy, lmx);
      range = sqrt(lmx * lmx + lmy * lmy);

      // In our experience, the x-y covariance on landmark sightings is not
      // very good, so assume it describes the uncertainty at a range of 10m,
      // and convert that to bearing/range uncertainty.
      if (std::abs(v1 - v3) < 1e-4) {
        bearing_std = sqrt(v1 / 10.0);
        range_std = sqrt(v1);
      } else {
        // TODO(frank): we are ignoring the non-uniform covariance
        bearing_std = 1;
        range_std = 1;
      }
    }

    // optional filter
    if (maxIndex && id1 > maxIndex)
      return std::nullopt;

    // Create noise model
    auto measurementNoise = noiseModel::Diagonal::Sigmas(
        (Vector(2) << bearing_std, range_std).finished());

    return BinaryMeasurement<BearingRange2D>(
        id1, L(id2), BearingRange2D(bearing, range), measurementNoise);
  }
};

/* ************************************************************************* */
GraphAndValues load2D(const std::string &filename, SharedNoiseModel model,
                      size_t maxIndex, bool addNoise, bool smart,
                      NoiseFormat noiseFormat,
                      KernelFunctionType kernelFunctionType) {

  // Single pass for poses and landmarks.
  auto initial = std::make_shared<Values>();
  Parser<int> insert = [maxIndex, &initial](std::istream &is, const std::string &tag) {
    if (auto indexedPose = parseVertexPose(is, tag)) {
      if (!maxIndex || indexedPose->first <= maxIndex)
        initial->insert(indexedPose->first, indexedPose->second);
    } else if (auto indexedLandmark = parseVertexLandmark(is, tag)) {
      if (!maxIndex || indexedLandmark->first <= maxIndex)
        initial->insert(L(indexedLandmark->first), indexedLandmark->second);
    }
    return 0;
  };
  parseLines(filename, insert);

  // Single pass for Pose2 and bearing-range factors.
  auto graph = std::make_shared<NonlinearFactorGraph>();

  // Instantiate factor parser
  ParseFactor<Pose2> parseBetweenFactor(
      {addNoise ? createSampler(model) : nullptr, maxIndex, smart, noiseFormat,
       kernelFunctionType, model});

  // Instantiate bearing-range parser
  ParseMeasurement<BearingRange2D> parseBearingRange{maxIndex};

  // Combine in a single parser that adds factors to `graph`, but also inserts
  // new variables into `initial` when needed.
  Parser<int> parse = [&](std::istream &is, const std::string &tag) {
    if (auto f = parseBetweenFactor(is, tag)) {
      graph->push_back(*f);

      // Insert vertices if pure odometry file
      Key key1 = (*f)->key<1>(), key2 = (*f)->key<2>();
      if (!initial->exists(key1))
        initial->insert(key1, Pose2());
      if (!initial->exists(key2))
        initial->insert(key2, initial->at<Pose2>(key1) * (*f)->measured());
    } else if (auto m = parseBearingRange(is, tag)) {
      Key key1 = m->key1(), key2 = m->key2();
      BearingRange2D br = m->measured();
      graph->emplace_shared<BearingRangeFactor<Pose2, Point2>>(key1, key2, br,
                                                               m->noiseModel());

      // Insert poses or points if they do not exist yet
      if (!initial->exists(key1))
        initial->insert(key1, Pose2());
      if (!initial->exists(key2)) {
        Pose2 pose = initial->at<Pose2>(key1);
        Point2 local = br.bearing() * Point2(br.range(), 0);
        Point2 global = pose.transformFrom(local);
        initial->insert(key2, global);
      }
    }
    return 0;
  };

  parseLines(filename, parse);

  return {graph, initial};
}

/* ************************************************************************* */
GraphAndValues load2D(std::pair<std::string, SharedNoiseModel> dataset,
                      size_t maxIndex, bool addNoise, bool smart,
                      NoiseFormat noiseFormat,
                      KernelFunctionType kernelFunctionType) {
  return load2D(dataset.first, dataset.second, maxIndex, addNoise, smart,
                noiseFormat, kernelFunctionType);
}

/* ************************************************************************* */
GraphAndValues load2D_robust(const std::string &filename,
                             const noiseModel::Base::shared_ptr &model,
                             size_t maxIndex) {
  return load2D(filename, model, maxIndex);
}

/* ************************************************************************* */
void save2D(const NonlinearFactorGraph &graph, const Values &config,
            const noiseModel::Diagonal::shared_ptr model,
            const std::string &filename) {

  std::fstream stream(filename.c_str(), std::fstream::out);

  // save poses
  for (const auto &key_pose : config.extract<Pose2>()) {
    const Pose2 &pose = key_pose.second;
    stream << "VERTEX2 " << key_pose.first << " " << pose.x() << " " << pose.y()
           << " " << pose.theta() << endl;
  }

  // save edges
  // TODO(frank): why don't we use model in factor?
  Matrix3 R = model->R();
  Matrix3 RR = R.transpose() * R;
  for (auto f : graph) {
    auto factor = std::dynamic_pointer_cast<BetweenFactor<Pose2>>(f);
    if (!factor)
      continue;

    const Pose2 pose = factor->measured().inverse();
    stream << "EDGE2 " << factor->key<2>() << " " << factor->key<1>() << " "
           << pose.x() << " " << pose.y() << " " << pose.theta() << " "
           << RR(0, 0) << " " << RR(0, 1) << " " << RR(1, 1) << " " << RR(2, 2)
           << " " << RR(0, 2) << " " << RR(1, 2) << endl;
  }

  stream.close();
}

/* ************************************************************************* */
GraphAndValues readG2o(const std::string &g2oFile, const bool is3D,
                       KernelFunctionType kernelFunctionType) {
  if (is3D) {
    return load3D(g2oFile);
  } else {
    // just call load2D
    size_t maxIndex = 0;
    bool addNoise = false;
    bool smart = true;
    return load2D(g2oFile, SharedNoiseModel(), maxIndex, addNoise, smart,
                  NoiseFormatG2O, kernelFunctionType);
  }
}

/* ************************************************************************* */
void writeG2o(const NonlinearFactorGraph &graph, const Values &estimate,
              const std::string &filename) {
  std::fstream stream(filename.c_str(), std::fstream::out);

  // Use a lambda here to more easily modify behavior in future.
  auto index = [](gtsam::Key key) { return Symbol(key).index(); };

  // save 2D poses
  for (const auto &pair : estimate.extract<Pose2>()) {
    const Pose2 &pose = pair.second;
    stream << "VERTEX_SE2 " << index(pair.first) << " " << pose.x() << " "
           << pose.y() << " " << pose.theta() << endl;
  }

  // save 3D poses
  for (const auto &pair : estimate.extract<Pose3>()) {
    const Pose3 &pose = pair.second;
    const Point3 t = pose.translation();
    const auto q = pose.rotation().toQuaternion();
    stream << "VERTEX_SE3:QUAT " << index(pair.first) << " " << t.x() << " "
           << t.y() << " " << t.z() << " " << q.x() << " " << q.y() << " "
           << q.z() << " " << q.w() << endl;
  }

  // save 2D landmarks
  for (const auto &pair : estimate.extract<Point2>()) {
    const Point2 &point = pair.second;
    stream << "VERTEX_XY " << index(pair.first) << " " << point.x() << " "
           << point.y() << endl;
  }

  // save 3D landmarks
  for (const auto &pair : estimate.extract<Point3>()) {
    const Point3 &point = pair.second;
    stream << "VERTEX_TRACKXYZ " << index(pair.first) << " " << point.x()
           << " " << point.y() << " " << point.z() << endl;
  }

  // save edges (2D or 3D)
  for (const auto &factor_ : graph) {
    auto factor = std::dynamic_pointer_cast<BetweenFactor<Pose2>>(factor_);
    if (factor) {
      SharedNoiseModel model = factor->noiseModel();
      auto gaussianModel =
          std::dynamic_pointer_cast<noiseModel::Gaussian>(model);
      if (!gaussianModel) {
        model->print("model\n");
        throw std::invalid_argument("writeG2o: invalid noise model!");
      }
      Matrix3 Info = gaussianModel->R().transpose() * gaussianModel->R();
      Pose2 pose = factor->measured(); //.inverse();
      stream << "EDGE_SE2 " << index(factor->key<1>()) << " "
             << index(factor->key<2>()) << " " << pose.x() << " " << pose.y()
             << " " << pose.theta();
      for (size_t i = 0; i < 3; i++) {
        for (size_t j = i; j < 3; j++) {
          stream << " " << Info(i, j);
        }
      }
      stream << endl;
    }

    auto factor3D = std::dynamic_pointer_cast<BetweenFactor<Pose3>>(factor_);

    if (factor3D) {
      SharedNoiseModel model = factor3D->noiseModel();

      std::shared_ptr<noiseModel::Gaussian> gaussianModel =
          std::dynamic_pointer_cast<noiseModel::Gaussian>(model);
      if (!gaussianModel) {
        model->print("model\n");
        throw std::invalid_argument("writeG2o: invalid noise model!");
      }
      Matrix6 Info = gaussianModel->R().transpose() * gaussianModel->R();
      const Pose3 pose3D = factor3D->measured();
      const Point3 p = pose3D.translation();
      const auto q = pose3D.rotation().toQuaternion();
      stream << "EDGE_SE3:QUAT " << index(factor3D->key<1>()) << " "
             << index(factor3D->key<2>()) << " " << p.x() << " " << p.y() << " "
             << p.z() << " " << q.x() << " " << q.y() << " " << q.z() << " "
             << q.w();

      // g2o's EDGE_SE3:QUAT stores information/precision of Pose3 in t,R order, unlike GTSAM:
      Matrix6 InfoG2o = I_6x6;
      InfoG2o.block<3, 3>(0, 0) = Info.block<3, 3>(3, 3); // cov translation
      InfoG2o.block<3, 3>(3, 3) = Info.block<3, 3>(0, 0); // cov rotation
      InfoG2o.block<3, 3>(0, 3) = Info.block<3, 3>(3, 0); // off diagonal R,t -> t,R
      InfoG2o.block<3, 3>(3, 0) = Info.block<3, 3>(0, 3); // off diagonal t,R -> R,t

      for (size_t i = 0; i < 6; i++) {
        for (size_t j = i; j < 6; j++) {
          stream << " " << InfoG2o(i, j);
        }
      }
      stream << endl;
    }
  }
  stream.close();
}

/* ************************************************************************* */
// parse quaternion in x,y,z,w order, and normalize to unit length
std::istream &operator>>(std::istream &is, Quaternion &q) {
  double x, y, z, w;
  is >> x >> y >> z >> w;
  const double norm = sqrt(w * w + x * x + y * y + z * z), f = 1.0 / norm;
  q = Quaternion(f * w, f * x, f * y, f * z);
  return is;
}

/* ************************************************************************* */
// parse Rot3 from roll, pitch, yaw
std::istream &operator>>(std::istream &is, Rot3 &R) {
  double yaw, pitch, roll;
  is >> roll >> pitch >> yaw; // notice order !
  R = Rot3::Ypr(yaw, pitch, roll);
  return is;
}

/* ************************************************************************* */
std::optional<std::pair<size_t, Pose3>> parseVertexPose3(std::istream &is,
                                                      const std::string &tag) {
  if (tag == "VERTEX3") {
    size_t id;
    double x, y, z;
    Rot3 R;
    is >> id >> x >> y >> z >> R;
    return std::make_pair(id, Pose3(R, {x, y, z}));
  } else if (tag == "VERTEX_SE3:QUAT") {
    size_t id;
    double x, y, z;
    Quaternion q;
    is >> id >> x >> y >> z >> q;
    return std::make_pair(id, Pose3(q, {x, y, z}));
  } else
    return std::nullopt;
}

template <>
GTSAM_EXPORT std::map<size_t, Pose3> parseVariables<Pose3>(
    const std::string &filename, size_t maxIndex) {
  return parseToMap<Pose3>(filename, parseVertexPose3, maxIndex);
}

/* ************************************************************************* */
std::optional<std::pair<size_t, Point3>> parseVertexPoint3(std::istream &is,
                                                        const std::string &tag) {
  if (tag == "VERTEX_TRACKXYZ") {
    size_t id;
    double x, y, z;
    is >> id >> x >> y >> z;
    return std::make_pair(id, Point3(x, y, z));
  } else
    return std::nullopt;
}

template <>
GTSAM_EXPORT std::map<size_t, Point3> parseVariables<Point3>(
    const std::string &filename, size_t maxIndex) {
  return parseToMap<Point3>(filename, parseVertexPoint3, maxIndex);
}

/* ************************************************************************* */
// Parse a symmetric covariance matrix (onlyupper-triangular part is stored)
std::istream &operator>>(std::istream &is, Matrix6 &m) {
  for (size_t i = 0; i < 6; i++)
    for (size_t j = i; j < 6; j++) {
      is >> m(i, j);
      m(j, i) = m(i, j);
    }
  return is;
}

/* ************************************************************************* */
// Pose3 measurement parser
template <> struct ParseMeasurement<Pose3> {
  // The arguments
  std::shared_ptr<Sampler> sampler;
  size_t maxIndex;

  // The actual parser
  std::optional<BinaryMeasurement<Pose3>> operator()(std::istream &is,
                                                       const std::string &tag) {
    if (tag != "EDGE3" && tag != "EDGE_SE3:QUAT")
      return std::nullopt;

    // parse ids and optionally filter
    size_t id1, id2;
    is >> id1 >> id2;
    if (maxIndex && (id1 > maxIndex || id2 > maxIndex))
      return std::nullopt;

    Matrix6 m;
    if (tag == "EDGE3") {
      double x, y, z;
      Rot3 R;
      is >> x >> y >> z >> R >> m;
      Pose3 T12(R, {x, y, z});
      //  optionally add noise
      if (sampler)
        T12 = T12.retract(sampler->sample());

      return BinaryMeasurement<Pose3>(id1, id2, T12,
                                      noiseModel::Gaussian::Information(m));
    } else if (tag == "EDGE_SE3:QUAT") {
      double x, y, z;
      Quaternion q;
      is >> x >> y >> z >> q >> m;

      Pose3 T12(q, {x, y, z});
      //  optionally add noise
      if (sampler)
        T12 = T12.retract(sampler->sample());

      // g2o's EDGE_SE3:QUAT stores information/precision of Pose3 in t,R order, unlike GTSAM:
      Matrix6 mgtsam;
      mgtsam.block<3, 3>(0, 0) = m.block<3, 3>(3, 3); // info rotation
      mgtsam.block<3, 3>(3, 3) = m.block<3, 3>(0, 0); // info translation
      mgtsam.block<3, 3>(3, 0) = m.block<3, 3>(0, 3); // off diagonal g2o t,R -> GTSAM R,t
      mgtsam.block<3, 3>(0, 3) = m.block<3, 3>(3, 0); // off diagonal g2o R,t -> GTSAM t,R
      SharedNoiseModel model = noiseModel::Gaussian::Information(mgtsam);

      return BinaryMeasurement<Pose3>(
          id1, id2, T12, noiseModel::Gaussian::Information(mgtsam));
    } else
      return std::nullopt;
  }
};

/* ************************************************************************* */
// Implementation of parseMeasurements for Pose3
template <>
GTSAM_EXPORT
std::vector<BinaryMeasurement<Pose3>>
parseMeasurements(const std::string &filename,
                  const noiseModel::Diagonal::shared_ptr &model,
                  size_t maxIndex) {
  ParseMeasurement<Pose3> parse{model ? createSampler(model) : nullptr,
                                maxIndex};
  return parseToVector<BinaryMeasurement<Pose3>>(filename, parse);
}

/* ************************************************************************* */
// Implementation of parseMeasurements for Rot3, which converts from Pose3

// Extract Rot3 measurement from Pose3 measurement
static BinaryMeasurement<Rot3> convert(const BinaryMeasurement<Pose3> &p) {
  auto gaussian =
      std::dynamic_pointer_cast<noiseModel::Gaussian>(p.noiseModel());
  if (!gaussian)
    throw std::invalid_argument(
        "parseMeasurements<Rot3> can only convert Pose3 measurements "
        "with Gaussian noise models.");
  const Matrix6 M = gaussian->covariance();
  return BinaryMeasurement<Rot3>(
      p.key1(), p.key2(), p.measured().rotation(),
      noiseModel::Gaussian::Covariance(M.block<3, 3>(0, 0), true));
}

template <>
GTSAM_EXPORT
std::vector<BinaryMeasurement<Rot3>>
parseMeasurements(const std::string &filename,
                  const noiseModel::Diagonal::shared_ptr &model,
                  size_t maxIndex) {
  auto poses = parseMeasurements<Pose3>(filename, model, maxIndex);
  std::vector<BinaryMeasurement<Rot3>> result;
  result.reserve(poses.size());
  for (const auto &p : poses)
    result.push_back(convert(p));
  return result;
}

/* ************************************************************************* */
// Implementation of parseFactors for Pose3
template <>
GTSAM_EXPORT
std::vector<BetweenFactor<Pose3>::shared_ptr>
parseFactors<Pose3>(const std::string &filename,
                    const noiseModel::Diagonal::shared_ptr &model,
                    size_t maxIndex) {
  ParseFactor<Pose3> parse({model ? createSampler(model) : nullptr, maxIndex});
  return parseToVector<BetweenFactor<Pose3>::shared_ptr>(filename, parse);
}

/* ************************************************************************* */
GraphAndValues load3D(const std::string &filename) {
  auto graph = std::make_shared<NonlinearFactorGraph>();
  auto initial = std::make_shared<Values>();

  // Instantiate factor parser. maxIndex is always zero for load3D.
  ParseFactor<Pose3> parseFactor({nullptr, 0});

  // Single pass for variables and factors. Unlike 2D version, does *not* insert
  // variables into `initial` if referenced but not present.
  Parser<int> parse = [&](std::istream &is, const std::string &tag) {
    if (auto indexedPose = parseVertexPose3(is, tag)) {
      initial->insert(indexedPose->first, indexedPose->second);
    } else if (auto indexedLandmark = parseVertexPoint3(is, tag)) {
      initial->insert(L(indexedLandmark->first), indexedLandmark->second);
    } else if (auto factor = parseFactor(is, tag)) {
      graph->push_back(*factor);
    }
    return 0;
  };
  parseLines(filename, parse);

  return {graph, initial};
}

// Wrapper-friendly versions of parseFactors<Pose2> and parseFactors<Pose2>
BetweenFactorPose2s
parse2DFactors(const std::string &filename,
               const noiseModel::Diagonal::shared_ptr &model, size_t maxIndex) {
  return parseFactors<Pose2>(filename, model, maxIndex);
}

BetweenFactorPose3s
parse3DFactors(const std::string &filename,
               const noiseModel::Diagonal::shared_ptr &model, size_t maxIndex) {
  return parseFactors<Pose3>(filename, model, maxIndex);
}

} // namespace gtsam
