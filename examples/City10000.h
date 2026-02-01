/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010-2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file   City10000.h
 * @brief  Class for City10000 dataset
 * @author Varun Agrawal
 * @date   February 3, 2025
 */

#include <gtsam/geometry/Pose2.h>

#include <fstream>

using namespace gtsam;

using symbol_shorthand::X;

auto kOpenLoopModel = noiseModel::Diagonal::Sigmas(Vector3::Ones() * 10);
const double kOpenLoopConstant = kOpenLoopModel->negLogConstant();

auto kPriorNoiseModel = noiseModel::Diagonal::Sigmas(
    (Vector(3) << 0.0001, 0.0001, 0.0001).finished());

auto kPoseNoiseModel = noiseModel::Diagonal::Sigmas(
    (Vector(3) << 1.0 / 30.0, 1.0 / 30.0, 1.0 / 100.0).finished());
const double kPoseNoiseConstant = kPoseNoiseModel->negLogConstant();

class City10000Dataset {
  std::ifstream in_;

  /// Read a `line` from the dataset, separated by the `delimiter`.
  std::vector<std::string> readLine(const std::string& line,
                                    const std::string& delimiter = " ") const {
    std::vector<std::string> parts;
    auto start = 0U;
    auto end = line.find(delimiter);
    while (end != std::string::npos) {
      parts.push_back(line.substr(start, end - start));
      start = end + delimiter.length();
      end = line.find(delimiter, start);
    }
    return parts;
  }

 public:
  City10000Dataset(const std::string& filename) : in_(filename) {
    if (!in_.is_open()) {
      std::cerr << "Failed to open file: " << filename << std::endl;
    }
  }

  /// Parse line from file
  std::pair<std::vector<Pose2>, std::pair<size_t, size_t>> parseLine(
      const std::string& line) const {
    std::vector<std::string> parts = readLine(line);

    size_t keyS = stoi(parts[1]);
    size_t keyT = stoi(parts[3]);

    int numMeasurements = stoi(parts[5]);
    std::vector<Pose2> poseArray(numMeasurements);
    for (int i = 0; i < numMeasurements; ++i) {
      double x = stod(parts[6 + 3 * i]);
      double y = stod(parts[7 + 3 * i]);
      double rad = stod(parts[8 + 3 * i]);
      poseArray[i] = Pose2(x, y, rad);
    }
    return {poseArray, {keyS, keyT}};
  }

  /// Read and parse the next line.
  bool next(std::vector<Pose2>* poseArray, std::pair<size_t, size_t>* keys) {
    std::string line;
    if (getline(in_, line)) {
      std::tie(*poseArray, *keys) = parseLine(line);
      return true;
    } else
      return false;
  }
};

/**
 * @brief Write the result of optimization to file.
 *
 * @param result The Values object with the final result.
 * @param num_poses The number of poses to write to the file.
 * @param filename The file name to save the result to.
 */
void writeResult(const Values& result, size_t numPoses,
                 const std::string& filename = "Hybrid_city10000.txt") {
  std::ofstream outfile;
  outfile.open(filename);

  for (size_t i = 0; i < numPoses; ++i) {
    Pose2 outPose = result.at<Pose2>(X(i));
    outfile << outPose.x() << " " << outPose.y() << " " << outPose.theta()
            << std::endl;
  }
  outfile.close();
  std::cout << "Output written to " << filename << std::endl;
}
