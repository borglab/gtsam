/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file    timeISAM2Covariance.cpp
 * @brief   Incremental timing study for ISAM2 joint covariance queries.
 * @date    April 2026
 * @author  Codex 5.4, prompted by Frank Dellaert
 */

#include <gtsam/geometry/Pose2.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/dataset.h>

#include <algorithm>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <numeric>
#include <optional>
#include <set>
#include <sstream>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

using namespace gtsam;
using namespace std;

namespace {

/// Structural statistics for one localized query support.
struct SupportStats {
  size_t supportCliques = 0;
  size_t compressedCliques = 0;
  size_t maxFrontalDim = 0;
  size_t maxSeparatorDim = 0;
};

/// Structural data for one localized query support.
template <class CLIQUE>
struct SupportAnalysis {
  shared_ptr<CLIQUE> root;
  unordered_set<shared_ptr<CLIQUE>> support;
  unordered_set<shared_ptr<CLIQUE>> queryCliques;
  unordered_map<shared_ptr<CLIQUE>, size_t> supportChildren;
  size_t compressedCliques = 0;
  size_t maxFrontalDim = 0;
  size_t maxSeparatorDim = 0;
};

/// One incremental timing sample.
struct StepResult {
  size_t stepIndex = 0;
  Key currentKey = 0;
  size_t poseCount = 0;
  size_t factorCount = 0;
  double updateMs = 0.0;
  double queryMeanMs = 0.0;
  double queryMedianMs = 0.0;
  double queryStddevMs = 0.0;
  size_t supportCliques = 0;
  size_t compressedCliques = 0;
  size_t reducedStateDim = 0;
  size_t maxFrontalDim = 0;
  size_t maxSeparatorDim = 0;
  size_t numCachedSeparatorMarginals = 0;
};

/// Metadata for one exported support snapshot.
struct SnapshotRecord {
  size_t stepIndex = 0;
  Key currentKey = 0;
  size_t poseCount = 0;
  size_t supportCliques = 0;
  size_t compressedCliques = 0;
  size_t maxSeparatorDim = 0;
  string dotFile;
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

/// Sum the dimensions of an ordered key range.
template <class RANGE>
size_t totalDimension(const RANGE& keys, const Values& values) {
  size_t dim = 0;
  for (Key key : keys) {
    dim += values.at(key).dim();
  }
  return dim;
}

/// Compute the lowest common ancestor of two Bayes-tree cliques.
template <class CLIQUE>
shared_ptr<CLIQUE> findLowestCommonAncestor(const shared_ptr<CLIQUE>& lhs,
                                            const shared_ptr<CLIQUE>& rhs) {
  unordered_set<shared_ptr<CLIQUE>> ancestors;
  for (auto current = lhs; current; current = current->parent()) {
    ancestors.insert(current);
  }
  for (auto current = rhs; current; current = current->parent()) {
    if (ancestors.count(current)) {
      return current;
    }
  }
  return nullptr;
}

/// Collect the exact support subtree and its basic statistics for a query.
template <class BAYESTREE>
SupportAnalysis<typename BAYESTREE::Clique> collectSupportAnalysis(
    const BAYESTREE& bayesTree, const KeyVector& queryKeys, const Values& values) {
  using Clique = typename BAYESTREE::Clique;
  vector<shared_ptr<Clique>> queryCliques;
  unordered_set<shared_ptr<Clique>> seen;
  for (Key key : queryKeys) {
    auto clique = bayesTree.clique(key);
    if (seen.insert(clique).second) {
      queryCliques.push_back(clique);
    }
  }
  if (queryCliques.empty()) {
    return {};
  }

  auto root = queryCliques.front();
  for (size_t i = 1; i < queryCliques.size(); ++i) {
    root = findLowestCommonAncestor(root, queryCliques[i]);
  }

  unordered_set<shared_ptr<Clique>> support;
  support.insert(root);
  for (const auto& clique : queryCliques) {
    for (auto current = clique; current && current != root;
         current = current->parent()) {
      support.insert(current);
    }
  }

  unordered_map<shared_ptr<Clique>, size_t> supportChildren;
  for (const auto& clique : support) {
    supportChildren[clique] = 0;
  }
  for (const auto& clique : support) {
    if (clique == root) {
      continue;
    }
    auto parent = clique->parent();
    if (parent && support.count(parent)) {
      ++supportChildren[parent];
    }
  }

  unordered_set<shared_ptr<Clique>> querySet(queryCliques.begin(),
                                             queryCliques.end());
  size_t compressed = 1;
  for (const auto& clique : support) {
    if (clique == root) {
      continue;
    }
    if (querySet.count(clique) || supportChildren[clique] > 1) {
      ++compressed;
    }
  }

  size_t maxFrontalDim = 0;
  size_t maxSeparatorDim = 0;
  for (const auto& clique : support) {
    const auto conditional = clique->conditional();
    maxFrontalDim =
        max(maxFrontalDim, totalDimension(conditional->frontals(), values));
    maxSeparatorDim =
        max(maxSeparatorDim, totalDimension(conditional->parents(), values));
  }

  SupportAnalysis<Clique> analysis;
  analysis.root = root;
  analysis.support = std::move(support);
  analysis.queryCliques =
      unordered_set<shared_ptr<Clique>>(queryCliques.begin(), queryCliques.end());
  analysis.supportChildren = std::move(supportChildren);
  analysis.compressedCliques = compressed;
  analysis.maxFrontalDim = maxFrontalDim;
  analysis.maxSeparatorDim = maxSeparatorDim;
  return analysis;
}

/// Estimate support size and clique-width statistics for a query.
template <class BAYESTREE>
SupportStats analyzeSupport(const BAYESTREE& bayesTree,
                            const KeyVector& queryKeys,
                            const Values& values) {
  const auto analysis = collectSupportAnalysis(bayesTree, queryKeys, values);
  return {analysis.support.size(), analysis.compressedCliques,
          analysis.maxFrontalDim, analysis.maxSeparatorDim};
}

/// Choose five representative incremental steps for support snapshots.
vector<size_t> chooseSnapshotSteps(size_t stepLimit) {
  if (stepLimit == 0) {
    return {};
  }
  if (stepLimit <= 5) {
    vector<size_t> steps(stepLimit);
    iota(steps.begin(), steps.end(), 0);
    return steps;
  }

  set<size_t> chosen;
  chosen.insert(min<size_t>(100, stepLimit - 1));
  chosen.insert(stepLimit / 4);
  chosen.insert(stepLimit / 2);
  chosen.insert((3 * stepLimit) / 4);
  chosen.insert(stepLimit - 1);
  return vector<size_t>(chosen.begin(), chosen.end());
}

/// Return a stable node identifier for one clique.
template <class CLIQUE>
string cliqueNodeId(const shared_ptr<CLIQUE>& clique) {
  return "n" + to_string(clique->conditional()->firstFrontalKey());
}

/// Return true if the clique survives support compression.
template <class CLIQUE>
bool isCompressedNode(const shared_ptr<CLIQUE>& clique,
                      const SupportAnalysis<CLIQUE>& analysis) {
  return clique == analysis.root || analysis.queryCliques.count(clique) > 0 ||
         analysis.supportChildren.at(clique) > 1;
}

/// Write one compressed-support snapshot in DOT format.
template <class CLIQUE>
void writeSupportSnapshotDot(const filesystem::path& path,
                             const SupportAnalysis<CLIQUE>& analysis,
                             const SnapshotRecord& record) {
  vector<shared_ptr<CLIQUE>> cliques;
  for (const auto& clique : analysis.support) {
    if (isCompressedNode(clique, analysis)) {
      cliques.push_back(clique);
    }
  }
  sort(cliques.begin(), cliques.end(),
       [](const shared_ptr<CLIQUE>& lhs, const shared_ptr<CLIQUE>& rhs) {
         return lhs->conditional()->firstFrontalKey() <
                rhs->conditional()->firstFrontalKey();
       });

  ofstream os(path);
  os << "digraph G {\n";
  os << "  graph [rankdir=LR, splines=false, nodesep=0.5, ranksep=0.35, "
        "margin=0.02, dpi=180];\n";
  os << "  node [shape=circle, width=0.32, height=0.32, fixedsize=true, "
        "label=\"\", style=filled, fillcolor=\"#8fb3d1\", color=\"#4c6a88\", "
        "penwidth=1.2];\n";
  os << "  edge [color=\"#667788\", arrowsize=0.55, penwidth=1.4, "
        "fontname=\"Helvetica\", fontsize=11];\n";

  for (const auto& clique : cliques) {
    const string nodeId = cliqueNodeId(clique);
    string fillColor = "#8fb3d1";
    string borderColor = "#4c6a88";
    if (analysis.root && clique == analysis.root) {
      fillColor = "#f0c36a";
      borderColor = "#a06b00";
    }
    if (analysis.queryCliques.count(clique)) {
      fillColor = "#c94c4c";
      borderColor = "#7c1f1f";
    }
    os << "  " << nodeId << " [fillcolor=\"" << fillColor << "\", color=\""
       << borderColor << "\"];\n";
  }

  for (const auto& clique : cliques) {
    if (clique == analysis.root) {
      continue;
    }
    size_t rawCliques = 1;
    auto current = clique;
    auto parent = current->parent();
    while (parent && analysis.support.count(parent) &&
           !isCompressedNode(parent, analysis)) {
      current = parent;
      parent = current->parent();
      ++rawCliques;
    }
    if (parent && analysis.support.count(parent)) {
      os << "  " << cliqueNodeId(parent) << " -> " << cliqueNodeId(clique)
         << " [label=\"" << rawCliques << "\"];\n";
    }
  }
  os << "}\n";
}

/// Write support-snapshot metadata to CSV.
void writeSnapshotCsv(const filesystem::path& path,
                      const vector<SnapshotRecord>& records) {
  ofstream os(path);
  os << "step_index,current_key,pose_count,support_cliques,compressed_cliques,"
        "max_separator_dim,dot_file\n";
  for (const auto& record : records) {
    os << record.stepIndex << ',' << record.currentKey << ','
       << record.poseCount << ',' << record.supportCliques << ','
       << record.compressedCliques << ',' << record.maxSeparatorDim << ','
       << record.dotFile << '\n';
  }
}

/// Compute the median of a small sample vector.
double median(vector<double> values) {
  if (values.empty()) {
    return 0.0;
  }
  sort(values.begin(), values.end());
  const size_t mid = values.size() / 2;
  if (values.size() % 2 == 0) {
    return 0.5 * (values[mid - 1] + values[mid]);
  }
  return values[mid];
}

/// Compute the arithmetic mean of a sample vector.
double mean(const vector<double>& values) {
  if (values.empty()) {
    return 0.0;
  }
  const double sum = accumulate(values.begin(), values.end(), 0.0);
  return sum / static_cast<double>(values.size());
}

/// Compute the population standard deviation of a sample vector.
double stddev(const vector<double>& values) {
  if (values.size() < 2) {
    return 0.0;
  }
  const double sampleMean = mean(values);
  double sumSquares = 0.0;
  for (double value : values) {
    const double delta = value - sampleMean;
    sumSquares += delta * delta;
  }
  return sqrt(sumSquares / static_cast<double>(values.size()));
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

/// Read an unsigned integer argument from argv or return a default value.
size_t sizeTArgumentOrDefault(char** begin, char** end, const string& flag,
                              size_t defaultValue) {
  return static_cast<size_t>(
      stoull(argumentOrDefault(begin, end, flag, to_string(defaultValue))));
}

/// Write incremental timing results to CSV.
void writeCsv(const filesystem::path& path, const vector<StepResult>& results) {
  ofstream os(path);
  os << "step_index,current_key,pose_count,factor_count,update_ms,query_mean_ms,"
        "query_median_ms,query_stddev_ms,support_cliques,compressed_cliques,"
        "reduced_state_dim,max_frontal_dim,max_separator_dim,"
        "num_cached_separator_marginals\n";
  os << fixed << setprecision(6);
  for (const auto& result : results) {
    os << result.stepIndex << ',' << result.currentKey << ','
       << result.poseCount << ',' << result.factorCount << ','
       << result.updateMs << ',' << result.queryMeanMs << ','
       << result.queryMedianMs << ',' << result.queryStddevMs << ','
       << result.supportCliques << ',' << result.compressedCliques << ','
       << result.reducedStateDim << ',' << result.maxFrontalDim << ','
       << result.maxSeparatorDim << ','
       << result.numCachedSeparatorMarginals << '\n';
  }
}

}  // namespace

int main(int argc, char** argv) {
  const string datasetName =
      argumentOrDefault(argv, argv + argc, "--dataset", "w20000.txt");
  const filesystem::path outputPath = argumentOrDefault(
      argv, argv + argc, "--output",
      (filesystem::path("timing") / "results" / "bayes_tree_covariance" /
       "isam2_w20000.csv")
          .string());
  const filesystem::path snapshotDir = argumentOrDefault(
      argv, argv + argc, "--snapshot-dir",
      (outputPath.parent_path() / "isam2_support_snapshots").string());
  const size_t queryRepeats =
      sizeTArgumentOrDefault(argv, argv + argc, "--query-repeats", 5);
  const size_t maxSteps =
      sizeTArgumentOrDefault(argv, argv + argc, "--max-steps", 0);

  const auto [graphPtr, initialPtr] =
      load2D(findExampleDataFile(datasetName));
  const KeyVector allPoseKeys = poseKeys(*initialPtr);
  if (allPoseKeys.empty()) {
    throw runtime_error("Dataset contains no Pose2 variables.");
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

  const Key firstKey = allPoseKeys.front();
  const auto priorNoise = noiseModel::Diagonal::Sigmas(
      (Vector(3) << 1e-6, 1e-6, 1e-6).finished());

  vector<StepResult> results;
  results.reserve(allPoseKeys.size());
  vector<SnapshotRecord> snapshots;
  size_t factorCount = 0;
  size_t nextMeasurement = 0;

  const size_t stepLimit =
      maxSteps == 0 ? allPoseKeys.size() : min(maxSteps, allPoseKeys.size());
  const vector<size_t> snapshotStepVector = chooseSnapshotSteps(stepLimit);
  const set<size_t> snapshotSteps(snapshotStepVector.begin(),
                                  snapshotStepVector.end());
  for (size_t stepIndex = 0; stepIndex < stepLimit; ++stepIndex) {
    const Key currentKey = allPoseKeys[stepIndex];
    NonlinearFactorGraph newFactors;
    Values newTheta;

    if (stepIndex == 0) {
      newTheta.insert(currentKey, initialPtr->at<Pose2>(currentKey));
      newFactors.addPrior(currentKey, initialPtr->at<Pose2>(currentKey),
                          priorNoise);
    }

    optional<Pose2> predictedPose;
    while (nextMeasurement < graphPtr->size()) {
      auto between =
          dynamic_pointer_cast<BetweenFactor<Pose2>>((*graphPtr)[nextMeasurement]);
      if (!between) {
        ++nextMeasurement;
        continue;
      }

      size_t newestStep = 0;
      for (Key key : between->keys()) {
        newestStep = max(newestStep, stepByKey.at(key));
      }
      if (newestStep > stepIndex) {
        break;
      }

      newFactors.push_back(between);
      if (stepIndex > 0) {
        const Key previousKey = allPoseKeys[stepIndex - 1];
        if (between->key1() == previousKey && between->key2() == currentKey) {
          predictedPose =
              isam2.calculateEstimate<Pose2>(previousKey) * between->measured();
        } else if (between->key1() == currentKey &&
                   between->key2() == previousKey) {
          predictedPose = isam2.calculateEstimate<Pose2>(previousKey) *
                          between->measured().inverse();
        }
      }
      ++nextMeasurement;
    }

    if (stepIndex > 0 && !isam2.valueExists(currentKey)) {
      if (predictedPose) {
        newTheta.insert(currentKey, *predictedPose);
      } else {
        newTheta.insert(currentKey, initialPtr->at<Pose2>(currentKey));
      }
    }

    const auto updateStart = chrono::steady_clock::now();
    try {
      isam2.update(newFactors, newTheta);
    } catch (const std::exception& error) {
      cerr << "ISAM2 update failed at step " << stepIndex << " key "
           << currentKey << ": " << error.what() << endl;
      throw;
    }
    const auto updateEnd = chrono::steady_clock::now();
    factorCount += newFactors.size();

    const KeyVector queryKeys{firstKey, currentKey};
    vector<double> queryTimes;
    queryTimes.reserve(queryRepeats);
    if (queryRepeats > 0) {
      try {
        (void)isam2.jointMarginalCovariance(queryKeys);
      } catch (const std::exception& error) {
        cerr << "Warmup covariance query failed at step " << stepIndex << " key "
             << currentKey << ": " << error.what() << endl;
        throw;
      }

      for (size_t repeatIndex = 0; repeatIndex < queryRepeats; ++repeatIndex) {
        const auto queryStart = chrono::steady_clock::now();
        JointMarginal covariance;
        try {
          covariance = isam2.jointMarginalCovariance(queryKeys);
        } catch (const std::exception& error) {
          cerr << "Covariance query failed at step " << stepIndex << " key "
               << currentKey << " repeat " << repeatIndex << ": "
               << error.what() << endl;
          throw;
        }
        const auto queryEnd = chrono::steady_clock::now();
        volatile double checksum = covariance.fullMatrix().sum();
        (void)checksum;
        queryTimes.push_back(
            chrono::duration<double, milli>(queryEnd - queryStart).count());
      }
    }

    const Values& linearizationPoint = isam2.getLinearizationPoint();
    const auto supportAnalysis =
        collectSupportAnalysis(isam2, queryKeys, linearizationPoint);
    const SupportStats supportStats = {supportAnalysis.support.size(),
                                       supportAnalysis.compressedCliques,
                                       supportAnalysis.maxFrontalDim,
                                       supportAnalysis.maxSeparatorDim};
    const GaussianFactorGraph reducedGraph =
        *isam2.joint(queryKeys, parameters.getEliminationFunction());

    StepResult result;
    result.stepIndex = stepIndex;
    result.currentKey = currentKey;
    result.poseCount = stepIndex + 1;
    result.factorCount = factorCount;
    result.updateMs =
        chrono::duration<double, milli>(updateEnd - updateStart).count();
    result.queryMeanMs = mean(queryTimes);
    result.queryMedianMs = median(queryTimes);
    result.queryStddevMs = stddev(queryTimes);
    result.supportCliques = supportStats.supportCliques;
    result.compressedCliques = supportStats.compressedCliques;
    result.reducedStateDim = totalDimension(reducedGraph.keys(), linearizationPoint);
    result.maxFrontalDim = supportStats.maxFrontalDim;
    result.maxSeparatorDim = supportStats.maxSeparatorDim;
    result.numCachedSeparatorMarginals = isam2.numCachedSeparatorMarginals();
    results.push_back(result);

    if (snapshotSteps.count(stepIndex)) {
      filesystem::create_directories(snapshotDir);
      const string dotFile =
          "support_step_" + to_string(stepIndex + 1) + ".dot";
      SnapshotRecord record;
      record.stepIndex = stepIndex;
      record.currentKey = currentKey;
      record.poseCount = stepIndex + 1;
      record.supportCliques = supportStats.supportCliques;
      record.compressedCliques = supportStats.compressedCliques;
      record.maxSeparatorDim = supportStats.maxSeparatorDim;
      record.dotFile = dotFile;
      writeSupportSnapshotDot(snapshotDir / dotFile, supportAnalysis, record);
      snapshots.push_back(record);
    }
  }

  filesystem::create_directories(outputPath.parent_path());
  writeCsv(outputPath, results);
  if (!snapshots.empty()) {
    filesystem::create_directories(snapshotDir);
    writeSnapshotCsv(snapshotDir / "snapshots.csv", snapshots);
  }
  cout << "Wrote incremental timing results to " << outputPath << endl;
  return 0;
}
