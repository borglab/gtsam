/* ----------------------------------------------------------------------------
 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)
 *
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file    simple_gtsam_deserialize.cpp
 * @brief   C++ version of simple_gtsam_deserialize.py
 * @brief   Deserialize and optimize a GTSAM graph from text files
 */

 #include <gtsam/nonlinear/NonlinearFactorGraph.h>
 #include <gtsam/nonlinear/Values.h>
 #include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
 #include <gtsam/slam/PriorFactor.h>
 #include <gtsam/sam/RangeFactor.h>
 #include <gtsam_unstable/slam/ProjectionFactorRollingShutter.h>
 #include <gtsam/geometry/Pose3.h>
 #include <gtsam/geometry/Point3.h>
 #include <gtsam/geometry/Point2.h>
 #include <gtsam/geometry/Cal3_S2.h>
 #include <gtsam/linear/NoiseModel.h>
 
 #include <iostream>
 #include <fstream>
 #include <sstream>
 #include <vector>
 #include <string>
 #include <set>
 #include <chrono>
 #include <exception>
 #include <iomanip>
 #include <cassert>
 #include <optional>
 #include <filesystem>
 
 using namespace std;
 using namespace gtsam;
 
 // Helper function to trim whitespace
 inline std::string trim(const std::string& str) {
     size_t first = str.find_first_not_of(" \t\n\r");
     if (first == std::string::npos) return "";
     size_t last = str.find_last_not_of(" \t\n\r");
     return str.substr(first, (last - first + 1));
 }
 
 // Parse [int1, int2, ...] string to vector of integers
 std::vector<Key> parse_int_list(const std::string& s) {
     std::vector<Key> result;
     std::string trimmed = trim(s);
     // Strip brackets
     if (!trimmed.empty() && trimmed.front() == '[') {
         trimmed = trimmed.substr(1);
     }
     if (!trimmed.empty() && trimmed.back() == ']') {
         trimmed = trimmed.substr(0, trimmed.length() - 1);
     }
     trimmed = trim(trimmed);
     if (trimmed.empty()) return result;
     
     std::stringstream ss(trimmed);
     std::string item;
     while (std::getline(ss, item, ',')) {
         std::string trimmed_item = trim(item);
         if (!trimmed_item.empty()) {
             result.push_back(static_cast<Key>(std::stoull(trimmed_item)));
         }
     }
     return result;
 }
 
 // Parse [float1, float2, ...] string to vector of doubles
 std::vector<double> parse_float_array(const std::string& s) {
     std::vector<double> result;
     std::string trimmed = trim(s);
     // Strip brackets
     if (!trimmed.empty() && trimmed.front() == '[') {
         trimmed = trimmed.substr(1);
     }
     if (!trimmed.empty() && trimmed.back() == ']') {
         trimmed = trimmed.substr(0, trimmed.length() - 1);
     }
     trimmed = trim(trimmed);
     if (trimmed.empty()) return result;
     
     std::stringstream ss(trimmed);
     std::string item;
     while (std::getline(ss, item, ',')) {
         std::string trimmed_item = trim(item);
         if (!trimmed_item.empty()) {
             result.push_back(std::stod(trimmed_item));
         }
     }
     return result;
 }
 
 int main(int argc, char* argv[]) {
     std::cout << "simple_gtsam_deserialize2.cpp" << std::endl;
     
     try {
         // File paths - match Python defaults
         std::filesystem::path source_file(__FILE__);
         std::filesystem::path source_dir = source_file.parent_path();
         std::filesystem::path data_dir = source_dir / "data";
         // File paths - match Python defaults, relative to source file location
         string graph_fl = (argc > 1) ? argv[1] : (data_dir / "simple_graph_enormous.txt").string();
         string vals_fl = (argc > 2) ? argv[2] : (data_dir / "simple_values_enormous1.txt").string();
 
         
         // Create noise models - match Python
         const double REPROJ_SIGMA = std::sqrt(3.0);
         Vector2 sigmas(REPROJ_SIGMA, REPROJ_SIGMA);
         auto point_noise = noiseModel::Diagonal::Sigmas(sigmas);
         auto cauchy = noiseModel::mEstimator::Cauchy::Create(REPROJ_SIGMA);
         auto point_noise_cauchy = noiseModel::Robust::Create(cauchy, point_noise);
         
         NonlinearFactorGraph graph;
         Values values;
         
         // Read values file
         auto t0 = std::chrono::high_resolution_clock::now();
         std::vector<std::string> values_lines;
         std::ifstream vfile(vals_fl);
         if (!vfile.is_open()) {
             throw std::runtime_error("Cannot open values file: " + vals_fl);
         }
         std::string line;
         while (std::getline(vfile, line)) {
             values_lines.push_back(line);
         }
         vfile.close();
         auto t1 = std::chrono::high_resolution_clock::now();
         double elapsed = std::chrono::duration<double>(t1 - t0).count();
         std::cout << std::fixed << std::setprecision(4) 
                   << "Reading values file took " << elapsed << " seconds" << std::endl;
         
         // Read graph file
         t0 = std::chrono::high_resolution_clock::now();
         std::vector<std::string> factor_lines;
         std::ifstream gfile(graph_fl);
         if (!gfile.is_open()) {
             throw std::runtime_error("Cannot open graph file: " + graph_fl);
         }
         while (std::getline(gfile, line)) {
             factor_lines.push_back(line);
         }
         gfile.close();
         t1 = std::chrono::high_resolution_clock::now();
         elapsed = std::chrono::duration<double>(t1 - t0).count();
         std::cout << std::fixed << std::setprecision(4) 
                   << "Reading graph file took " << elapsed << " seconds" << std::endl;
         
         // Extract c2m from values (needed for RSC2CProjectionFactor conversion)
         // Also collect keys referenced by PriorFactors (these 'c' values need to be inserted)
         std::optional<Pose3> c2m;
         std::set<Key> prior_factor_keys; // Keys referenced by PriorFactors
         
         // First pass: collect PriorFactor keys
         for (const auto& line : factor_lines) {
             if (line.empty()) continue;
             
             std::vector<std::string> data;
             std::stringstream ss(line);
             std::string item;
             while (std::getline(ss, item, ';')) {
                 data.push_back(item);
             }
             
             if (data.empty()) continue;
             
             if (data[0] == "PriorFactor") {
                 auto keys = parse_int_list(data[1]);
                 if (!keys.empty()) {
                     prior_factor_keys.insert(keys[0]);
                 }
             }
         }
         
         // Extract c2m from values (needed for RSC2CProjectionFactor conversion)
         for (const auto& line : values_lines) {
             if (line.empty()) continue;
             
             std::vector<std::string> data;
             std::stringstream ss(line);
             std::string item;
             while (std::getline(ss, item, ';')) {
                 data.push_back(item);
             }
             
             if (data.size() < 3) continue;
             
             std::string value_type = trim(data[0]);
             if (value_type == "c") {
                 [[maybe_unused]] Key key = static_cast<Key>(std::stoull(trim(data[1])));
                 auto matrix_vec = parse_float_array(data[2]);
                 if (matrix_vec.size() == 16) {
                     Matrix4 matrix;
                     for (int i = 0; i < 4; i++) {
                         for (int j = 0; j < 4; j++) {
                             matrix(i, j) = matrix_vec[i * 4 + j];
                         }
                     }
                     Pose3 pose(matrix);
                     // Store c2m from first 'c' value (for RSC2CProjectionFactor conversion)
                     if (!c2m.has_value()) {
                         c2m = pose;
                     }
                     // If this 'c' key is referenced by a PriorFactor, we'll insert it later
                 }
             }
         }
         
         // Process factor lines
         t0 = std::chrono::high_resolution_clock::now();
         for (const auto& line : factor_lines) {
             if (line.empty()) continue;
             
             std::vector<std::string> data;
             std::stringstream ss(line);
             std::string item;
             while (std::getline(ss, item, ';')) {
                 data.push_back(item);
             }
             
             if (data.empty()) continue;
             
             if (data[0] == "PriorFactor") {
                 auto keys = parse_int_list(data[1]);
                 if (keys.empty()) continue;
                 Key key = keys[0];
                 
                 auto matrix_vec = parse_float_array(data[2]);
                 if (matrix_vec.size() != 16) continue;
                 Matrix4 matrix;
                 for (int i = 0; i < 4; i++) {
                     for (int j = 0; j < 4; j++) {
                         matrix(i, j) = matrix_vec[i * 4 + j];
                     }
                 }
                 Pose3 prior(matrix);
                 
                 auto sigmas_vec = parse_float_array(data[3]);
                 Vector6 sigmas;
                 for (size_t i = 0; i < 6 && i < sigmas_vec.size(); i++) {
                     sigmas(i) = sigmas_vec[i];
                 }
                 auto noise = noiseModel::Diagonal::Sigmas(sigmas);
                 
                 auto factor = std::make_shared<PriorFactor<Pose3>>(key, prior, noise);
                 graph.push_back(factor);
                 
             } else if (data[0] == "RangeFactor") {
                 auto keys = parse_int_list(data[1]);
                 if (keys.size() < 2) continue;
                 
                 double measured = std::stod(trim(data[2]));
                 
                 auto sigmas_vec = parse_float_array(data[3]);
                 double sigma = sigmas_vec.empty() ? 1.0 : sigmas_vec[0];
                 auto noise = noiseModel::Isotropic::Sigma(1, sigma);
                 
                 auto factor = std::make_shared<RangeFactor<Pose3, Pose3>>(
                     keys[0], keys[1], measured, noise);
                 graph.push_back(factor);
                 
             } else if (data[0] == "ProjectionFactorRollingShutter") {
                 auto keys = parse_int_list(data[1]);
                 if (keys.size() < 3) continue;
                 
                 auto measured_vec = parse_float_array(data[2]);
                 if (measured_vec.size() < 2) continue;
                 Point2 measured(measured_vec[0], measured_vec[1]);
                 
                 double alpha = std::stod(trim(data[3]));
                 
                 auto K_vec = parse_float_array(data[4]);
                 if (K_vec.size() < 9) continue;
                 // K matrix is stored row-major: [K00, K01, K02, K10, K11, K12, K20, K21, K22]
                 // K[0][0] = K_vec[0] (fx), K[1][1] = K_vec[4] (fy)
                 double fx = K_vec[0];
                 double fy = K_vec[4];
                 auto cal = std::make_shared<Cal3_S2>(fx, fy, 0.0, 0.0, 0.0);
                 
                 auto factor = std::make_shared<ProjectionFactorRollingShutter>(
                     measured, alpha, point_noise_cauchy, keys[0], keys[1], keys[2], cal, false, false);
                 graph.push_back(factor);
                 
             } else if (data[0] == "RSC2CProjectionFactor") {
                 auto keys = parse_int_list(data[1]);
                 if (keys.size() < 3) continue;
                 
                 auto measured_vec = parse_float_array(data[2]);
                 if (measured_vec.size() < 2) continue;
                 Point2 measured(measured_vec[0], measured_vec[1]);
                 
                 double alpha = std::stod(trim(data[3]));
                 
                 auto K_vec = parse_float_array(data[4]);
                 if (K_vec.size() < 9) continue;
                 // K matrix is stored row-major: [K00, K01, K02, K10, K11, K12, K20, K21, K22]
                 // K[0][0] = K_vec[0] (fx), K[1][1] = K_vec[4] (fy)
                 double fx = K_vec[0];
                 double fy = K_vec[4];
                 auto cal = std::make_shared<Cal3_S2>(fx, fy, 0.0, 0.0, 0.0);
                 
                 auto factor = std::make_shared<ProjectionFactorRollingShutter>(
                     measured, alpha, point_noise_cauchy, keys[0], keys[1], keys[2], cal, false, false, c2m);
                 graph.push_back(factor);
             }
         }
         t1 = std::chrono::high_resolution_clock::now();
         elapsed = std::chrono::duration<double>(t1 - t0).count();
         std::cout << std::fixed << std::setprecision(4) 
                   << "Processing factor lines took " << elapsed << " seconds" << std::endl;
         
         // Process values lines
         t0 = std::chrono::high_resolution_clock::now();
         for (const auto& line : values_lines) {
             if (line.empty()) continue;
             
             std::vector<std::string> data;
             std::stringstream ss(line);
             std::string item;
             while (std::getline(ss, item, ';')) {
                 data.push_back(item);
             }
             
             if (data.size() < 3) continue;
             
             Key key = static_cast<Key>(std::stoull(trim(data[1])));
             
             std::string value_type = trim(data[0]);
             assert(value_type == "c" || value_type == "x" || value_type == "l");
             
             // Match Python behavior: only insert 'x' and 'l' values, not 'c' values
             // Exception: insert 'c' values if they're referenced by PriorFactors
             if (value_type == "x") {
                 auto matrix_vec = parse_float_array(data[2]);
                 if (matrix_vec.size() != 16) continue;
                 Matrix4 matrix;
                 for (int i = 0; i < 4; i++) {
                     for (int j = 0; j < 4; j++) {
                         matrix(i, j) = matrix_vec[i * 4 + j];
                     }
                 }
                 Pose3 pose(matrix);
                 values.insert(key, pose);
             } else if (value_type == "c") {
                 // Insert 'c' values only if referenced by PriorFactors
                 if (prior_factor_keys.find(key) != prior_factor_keys.end()) {
                     auto matrix_vec = parse_float_array(data[2]);
                     if (matrix_vec.size() != 16) continue;
                     Matrix4 matrix;
                     for (int i = 0; i < 4; i++) {
                         for (int j = 0; j < 4; j++) {
                             matrix(i, j) = matrix_vec[i * 4 + j];
                         }
                     }
                     Pose3 pose(matrix);
                     values.insert(key, pose);
                 }
                 // Otherwise skip (matching Python behavior)
                 
             } else if (value_type == "l") {
                 auto point_vec = parse_float_array(data[2]);
                 if (point_vec.size() >= 3) {
                     Point3 point(point_vec[0], point_vec[1], point_vec[2]);
                     values.insert(key, point);
                 }
             }
         }
         t1 = std::chrono::high_resolution_clock::now();
         elapsed = std::chrono::duration<double>(t1 - t0).count();
         std::cout << std::fixed << std::setprecision(4) 
                   << "Processing values lines took " << elapsed << " seconds" << std::endl;
         
        // Set up optimizer
        t0 = std::chrono::high_resolution_clock::now();
        LevenbergMarquardtParams params;
        params.setVerbosityLM("SUMMARY");
        params.setMaxIterations(20);
        params.linearSolverType = LevenbergMarquardtParams::MULTIFRONTAL_CHOLESKY;
        // Set output stream for statistics reporting
        params.multifrontalParams.reportStream = &std::cout;
        LevenbergMarquardtOptimizer optimizer(graph, values, params);
         t1 = std::chrono::high_resolution_clock::now();
         elapsed = std::chrono::duration<double>(t1 - t0).count();
         std::cout << std::fixed << std::setprecision(4) 
                   << "Setting up optimizer took " << elapsed << " seconds" << std::endl;
         
         // Optimize
         t0 = std::chrono::high_resolution_clock::now();
         Values result = optimizer.optimize();
         t1 = std::chrono::high_resolution_clock::now();
         elapsed = std::chrono::duration<double>(t1 - t0).count();
         std::cout << std::fixed << std::setprecision(4) 
                   << "Running gtsam optimizer took " << elapsed << " seconds" << std::endl;
         
         return 0;
         
     } catch (const exception& e) {
         std::cerr << "Error: " << e.what() << std::endl;
         return 1;
     }
 }
 