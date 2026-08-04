/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file    exportBayesTreeCovarianceVisuals.cpp
 * @brief   Export representative pose-graph query and covariance visuals.
 * @date    March 2026
 * @author  Frank Dellaert
 */

#include <gtsam/geometry/Pose2.h>
#include <gtsam/inference/Ordering.h>
#include <gtsam/linear/GaussianBayesTree.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/dataset.h>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <optional>
#include <set>
#include <string>
#include <unordered_map>
#include <vector>

using namespace gtsam;
using namespace std;

namespace {

/// Bundle the optimized result together with its COLAMD Bayes tree.
struct OptimizedBayesTreeResult {
  NonlinearFactorGraph graph;
  Values values;
  GaussianBayesTree bayesTree;
};

/// Return all Pose2 keys in sorted order.
KeyVector poseKeys(const Values& values) {
  KeyVector keys;
  for (const auto& keyValue : values.extract<Pose2>()) {
    keys.push_back(keyValue.first);
  }
  sort(keys.begin(), keys.end());
  return keys;
}

/// Compute a final pose-graph estimate with sequential iSAM2 updates.
Values solveSequentiallyWithISAM2(const NonlinearFactorGraph& graph,
                                  const Values& initial) {
  const KeyVector allPoseKeys = poseKeys(initial);
  if (allPoseKeys.empty()) {
    return Values();
  }

  unordered_map<Key, size_t> stepByKey;
  for (size_t index = 0; index < allPoseKeys.size(); ++index) {
    stepByKey[allPoseKeys[index]] = index;
  }

  ISAM2Params parameters;
  parameters.optimizationParams = ISAM2GaussNewtonParams();
  parameters.relinearizeSkip = 1;
  parameters.relinearizeThreshold = 0.01;
  ISAM2 isam2(parameters);

  const auto priorNoise = noiseModel::Diagonal::Sigmas(
      (Vector(3) << 1e-6, 1e-6, 1e-6).finished());
  size_t nextFactor = 0;

  for (size_t stepIndex = 0; stepIndex < allPoseKeys.size(); ++stepIndex) {
    const Key currentKey = allPoseKeys[stepIndex];
    NonlinearFactorGraph newFactors;
    Values newTheta;

    if (stepIndex == 0) {
      newTheta.insert(currentKey, initial.at<Pose2>(currentKey));
      newFactors.addPrior(currentKey, initial.at<Pose2>(currentKey), priorNoise);
    }

    optional<Pose2> predictedPose;
    while (nextFactor < graph.size()) {
      const auto& factor = graph[nextFactor];
      size_t newestStep = 0;
      bool allKeysKnown = true;
      for (Key key : factor->keys()) {
        const auto it = stepByKey.find(key);
        if (it == stepByKey.end()) {
          allKeysKnown = false;
          break;
        }
        newestStep = max(newestStep, it->second);
      }
      if (!allKeysKnown) {
        ++nextFactor;
        continue;
      }
      if (newestStep > stepIndex) {
        break;
      }

      newFactors.push_back(factor);
      if (stepIndex > 0) {
        const Key previousKey = allPoseKeys[stepIndex - 1];
        const auto between = dynamic_pointer_cast<const BetweenFactor<Pose2>>(factor);
        if (between && isam2.valueExists(previousKey)) {
          if (between->key1() == previousKey && between->key2() == currentKey) {
            predictedPose = isam2.calculateEstimate<Pose2>(previousKey) *
                            between->measured();
          } else if (between->key1() == currentKey &&
                     between->key2() == previousKey) {
            predictedPose = isam2.calculateEstimate<Pose2>(previousKey) *
                            between->measured().inverse();
          }
        }
      }
      ++nextFactor;
    }

    if (stepIndex > 0 && !isam2.valueExists(currentKey)) {
      if (predictedPose) {
        newTheta.insert(currentKey, *predictedPose);
      } else {
        newTheta.insert(currentKey, initial.at<Pose2>(currentKey));
      }
    }

    isam2.update(newFactors, newTheta);
  }

  return isam2.calculateEstimate();
}

/// Select a contiguous window near the center of the trajectory.
KeyVector centeredWindow(const KeyVector& keys, size_t querySize) {
  if (keys.size() <= querySize) {
    return keys;
  }
  const size_t start = (keys.size() - querySize) / 2;
  return KeyVector(keys.begin() + start, keys.begin() + start + querySize);
}

/// Select approximately evenly spaced keys across the full trajectory.
KeyVector evenlySpaced(const KeyVector& keys, size_t querySize) {
  if (keys.size() <= querySize) {
    return keys;
  }

  KeyVector query;
  query.reserve(querySize);
  for (size_t i = 0; i < querySize; ++i) {
    const double alpha =
        querySize == 1 ? 0.0 : double(i) / double(querySize - 1);
    const size_t index =
        static_cast<size_t>(llround(alpha * double(keys.size() - 1)));
    query.push_back(keys.at(index));
  }
  query.erase(unique(query.begin(), query.end()), query.end());
  return query;
}

/// Write a one-column key list CSV.
void writeKeyListCsv(const filesystem::path& path, const KeyVector& keys) {
  ofstream stream(path);
  stream << "key\n";
  for (Key key : keys) {
    stream << key << '\n';
  }
}

/// Write a dense matrix to CSV with fixed precision.
void writeMatrixCsv(const filesystem::path& path, const Matrix& matrix) {
  ofstream stream(path);
  stream << fixed << setprecision(12);
  for (Eigen::Index row = 0; row < matrix.rows(); ++row) {
    for (Eigen::Index column = 0; column < matrix.cols(); ++column) {
      if (column) {
        stream << ',';
      }
      stream << matrix(row, column);
    }
    stream << '\n';
  }
}

/// Export optimized Pose2 values together with query membership flags.
void writePoseCsv(const filesystem::path& path, const Values& values,
                  const KeyVector& localQuery, const KeyVector& wideQuery) {
  const set<Key> localKeys(localQuery.begin(), localQuery.end());
  const set<Key> wideKeys(wideQuery.begin(), wideQuery.end());

  ofstream stream(path);
  stream << "key,x,y,theta,in_local,in_wide\n";
  stream << fixed << setprecision(9);
  for (Key key : poseKeys(values)) {
    const Pose2 pose = values.at<Pose2>(key);
    stream << key << ',' << pose.x() << ',' << pose.y() << ',' << pose.theta()
           << ',' << (localKeys.count(key) ? 1 : 0) << ','
           << (wideKeys.count(key) ? 1 : 0) << '\n';
  }
}

/// Export the unique Pose2 measurement edges in the factor graph.
void writeEdgeCsv(const filesystem::path& path,
                  const NonlinearFactorGraph& graph) {
  set<pair<Key, Key>> edges;
  for (const auto& factor : graph) {
    auto between = dynamic_pointer_cast<const BetweenFactor<Pose2>>(factor);
    if (!between) {
      continue;
    }
    const Key key1 = between->key1();
    const Key key2 = between->key2();
    const auto edge = minmax(key1, key2);
    edges.insert(edge);
  }

  ofstream stream(path);
  stream << "key1,key2\n";
  for (const auto& [key1, key2] : edges) {
    stream << key1 << ',' << key2 << '\n';
  }
}

/// Remove common dataset filename suffixes for generated CSV names.
string datasetStem(const string& datasetName) {
  filesystem::path path(datasetName);
  string stem = path.filename().string();
  for (const string& suffix : {".graph", ".txt"}) {
    if (stem.size() >= suffix.size() &&
        stem.compare(stem.size() - suffix.size(), suffix.size(), suffix) == 0) {
      stem.resize(stem.size() - suffix.size());
      break;
    }
  }
  return stem;
}

/// Optimize one dataset and build its COLAMD Bayes tree.
OptimizedBayesTreeResult optimizeWithBayesTree(const string& datasetName) {
  const auto [graphPtr, initialPtr] = load2D(findExampleDataFile(datasetName));
  Values result = solveSequentiallyWithISAM2(*graphPtr, *initialPtr);
  const KeyVector poseKeyList = poseKeys(result);
  if (!poseKeyList.empty()) {
    graphPtr->addPrior(poseKeyList.front(), result.at<Pose2>(poseKeyList.front()),
                       noiseModel::Diagonal::Sigmas(
                           (Vector(3) << 1e-6, 1e-6, 1e-6).finished()));
  }
  GaussianFactorGraph linearGraph = *graphPtr->linearize(result);
  const Ordering ordering = Ordering::Create(Ordering::COLAMD, linearGraph);
  GaussianBayesTree bayesTree =
      *linearGraph.eliminateMultifrontal(ordering, EliminatePreferCholesky);
  return {*graphPtr, result, bayesTree};
}

/// Export optimized Pose2 values together with their COLAMD clique sizes.
void writeCliquePoseCsv(const filesystem::path& path, const Values& values,
                        const GaussianBayesTree& bayesTree) {
  ofstream stream(path);
  stream << "key,x,y,theta,clique_size\n";
  stream << fixed << setprecision(9);
  for (Key key : poseKeys(values)) {
    const Pose2 pose = values.at<Pose2>(key);
    const auto clique = bayesTree.clique(key);
    const auto conditional = clique->conditional();
    const size_t cliqueSize =
        conditional->nrFrontals() + conditional->nrParents();
    stream << key << ',' << pose.x() << ',' << pose.y() << ','
           << pose.theta() << ',' << cliqueSize << '\n';
  }
}

/// Read a string argument from argv or return a default value.
string argumentOrDefault(char** begin, char** end, const string& flag,
                         const string& defaultValue) {
  for (auto it = begin; it != end; ++it) {
    if (string(*it) == flag && it + 1 != end) {
      return *(it + 1);
    }
  }
  return defaultValue;
}

}  // namespace

int main(int argc, char** argv) {
  const string datasetName =
      argumentOrDefault(argv, argv + argc, "--dataset", "w100.graph");
  const filesystem::path outputDir =
      argumentOrDefault(argv, argv + argc, "--output-dir",
                        (filesystem::path("timing") / "results" /
                         "bayes_tree_covariance" / "visuals")
                            .string());
  filesystem::create_directories(outputDir);

  const OptimizedBayesTreeResult w100 = optimizeWithBayesTree(datasetName);
  const Values& result = w100.values;
  const KeyVector queryCandidates = poseKeys(result);
  const KeyVector localQuery = centeredWindow(queryCandidates, 10);
  const KeyVector wideQuery = evenlySpaced(queryCandidates, 10);

  const Marginals marginals(w100.graph, result, Marginals::CHOLESKY);
  const Matrix localCovariance =
      marginals.jointMarginalCovariance(localQuery).fullMatrix();
  const Matrix wideCovariance =
      marginals.jointMarginalCovariance(wideQuery).fullMatrix();

  writePoseCsv(outputDir / "w100_poses.csv", result, localQuery, wideQuery);
  writeEdgeCsv(outputDir / "w100_edges.csv", w100.graph);
  writeKeyListCsv(outputDir / "w100_local_keys.csv", localQuery);
  writeKeyListCsv(outputDir / "w100_wide_keys.csv", wideQuery);
  writeMatrixCsv(outputDir / "w100_local_covariance.csv", localCovariance);
  writeMatrixCsv(outputDir / "w100_wide_covariance.csv", wideCovariance);

  for (const string& cliqueDatasetName : {"w10000.graph", "w20000.txt"}) {
    const OptimizedBayesTreeResult optimized =
        optimizeWithBayesTree(cliqueDatasetName);
    writeCliquePoseCsv(outputDir / (datasetStem(cliqueDatasetName) + "_cliques.csv"),
                       optimized.values, optimized.bayesTree);
  }

  return 0;
}
