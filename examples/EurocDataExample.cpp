/**
 * @file EurocDataExample.cpp
 * @brief Example evaluating IMU Preintegration consistency (15-DOF) on EuRoC dataset.
 * Mimics the evaluation methodology from Section V-B of
 * Fornasier et al., "Equivariant IMU Preintegration with Biases:
 * a Galilean Group Approach" (arXiv:2411.05548v4), calculating
 * 15-DOF NEES for NavState + Bias.
 * @author Matt Kielo, Porter Zach
 * @modifier AI Assistant (Refactoring and Enhancements)
 */

 #include <gtsam/navigation/NavState.h>
 #include <gtsam/navigation/ImuBias.h>
 #include <gtsam/navigation/CombinedImuFactor.h>
 #include <gtsam/navigation/PreintegrationCombinedParams.h>
 
 #include <gtsam/geometry/Pose3.h>
 #include <gtsam/geometry/Rot3.h>
 #include <gtsam/base/Matrix.h> // For I_3x3, I_6x6
 
 #include <iostream>
 #include <fstream>
 #include <vector>
 #include <string>
 #include <sstream>
 #include <iomanip>
 #include <cmath>
 #include <algorithm>
 #include <limits>
 #include <stdexcept>
 #include <map> // For results table
 
 // Define custom types needed for this example
 namespace gtsam {
     typedef Eigen::Matrix<double, 15, 1> Vector15;
     // Define Matrix15x15 type for convenience, avoiding conflict with GTSAM's Matrix15 (1x5)
     typedef Eigen::Matrix<double, 15, 15> Matrix15x15;
 }
 
 using namespace gtsam;
 using namespace std;
 
 // --- Configuration ---
 // Base path for EuRoC datasets. Individual dataset folders (e.g., MH_01) should be inside this.
 const string base_euroc_data_path = "C:/Users/porte/Desktop/projects/works25/euroc/data";
 
 // List of datasets to process
 const vector<string> dataset_sequences = {
     "MH_01",
     // "MH_02",
     // "MH_03",
     // "MH_04",
     // "MH_05",
     // "V1_01",
     // "V1_02",
     // "V1_03",
     // "V2_01",
     "V2_02"
     // "V2_03" // This one has different GT format (no bias)
 };
 
 // List of preintegration interval durations (seconds)
 const vector<double> deltaTij_values = {0.2, 0.5, 1.0};
 
 // Integration noise sigma (usually very small or zero)
 const double integration_noise_sigma = 1e-8;
 
 // Gravity vector in navigation frame (ENU convention: Z-up, gravity is negative Z)
 const Vector3 gravity_n(0, 0, -9.81);
 
 
 // --- Data Structures ---
 
 struct ImuData {
     double timestamp; // seconds
     Vector3 omega;    // rad/s (gyro)
     Vector3 acc;      // m/s^2 (accel)
 };
 
 struct GroundTruthData {
     double timestamp;  // seconds
     NavState navState;
     imuBias::ConstantBias bias;
 };
 
 struct SensorParams {
     double gyro_noise_density = 0.0;
     double gyro_bias_rw_density = 0.0;
     double accel_noise_density = 0.0;
     double accel_bias_rw_density = 0.0;
     Pose3 body_P_sensor; // T_BS: IMU frame in body frame
     double rate_hz = 200.0; // Default, will be overridden by YAML if present
 };
 
 struct IntervalEvaluationResult {
     double nees = numeric_limits<double>::quiet_NaN();
     bool success = false;
     double actual_dt = 0.0;
 };
 
 struct AggregatedRunResults {
     string dataset_name;
     double deltaTij_target = 0.0;
 
     double median_nees = numeric_limits<double>::quiet_NaN();
     double mean_nees = numeric_limits<double>::quiet_NaN();
     double std_dev_nees = numeric_limits<double>::quiet_NaN();
     
     double median_actual_dt = numeric_limits<double>::quiet_NaN();
     double mean_actual_dt = numeric_limits<double>::quiet_NaN();
 
     int processed_intervals = 0;
     int skipped_intervals_gt = 0;
     int skipped_intervals_imu = 0;
     int numerical_issue_count = 0;
 
     void print_summary() const {
         cout << "  Summary for " << dataset_name << " (deltaTij: " << fixed << setprecision(1) << deltaTij_target << "s):" << endl;
         cout << "    Processed Intervals: " << processed_intervals << endl;
         if (skipped_intervals_gt > 0) cout << "    Skipped (GT Gaps): " << skipped_intervals_gt << endl;
         if (skipped_intervals_imu > 0) cout << "    Skipped (IMU Gaps): " << skipped_intervals_imu << endl;
         if (numerical_issue_count > 0) cout << "    Skipped (Numerical Issues): " << numerical_issue_count << endl;
         
         if (processed_intervals > 0) {
             cout << "    Median Actual dt: " << fixed << setprecision(5) << median_actual_dt << " s" << endl;
             cout << "    Mean Actual dt:   " << fixed << setprecision(5) << mean_actual_dt << " s" << endl;
             cout << "    Median 15-DOF NEES: " << fixed << setprecision(3) << median_nees << endl;
             cout << "    Mean   15-DOF NEES: " << fixed << setprecision(3) << mean_nees << endl;
             cout << "    StdDev 15-DOF NEES: " << fixed << setprecision(3) << std_dev_nees << endl;
         } else {
             cout << "    No NEES results to report." << endl;
         }
         cout << "  -----------------------------------" << endl;
     }
 };
 
 
 // --- String Helper ---
 string trim(const string& str, const string& whitespace = " \t\n\r\f\v") {
     const auto strBegin = str.find_first_not_of(whitespace);
     if (strBegin == string::npos) return ""; // no content
     const auto strEnd = str.find_last_not_of(whitespace);
     const auto strRange = strEnd - strBegin + 1;
     return str.substr(strBegin, strRange);
 }
 
 // --- YAML Parsing Helper ---
 bool parseSensorYaml(const string& yaml_path, SensorParams& params) {
     ifstream file(yaml_path);
     if (!file.is_open()) {
         cerr << "Error opening sensor YAML file: " << yaml_path << endl;
         return false;
     }
 
     string line;
     Matrix4 T_BS_mat = Matrix4::Identity();
     bool t_bs_data_mode = false;
     vector<double> t_bs_coeffs;
 
     auto extract_value = [&](const string& l, const string& key) {
         size_t key_pos = l.find(key);
         if (key_pos != string::npos) {
             string val_str = l.substr(key_pos + key.length());
             return trim(val_str);
         }
         return string("");
     };
     
     while (getline(file, line)) {
         line = trim(line);
         if (line.empty() || line[0] == '#') continue;
 
         if (t_bs_data_mode) {
             stringstream ss_line(line);
             string val_token;
             while(getline(ss_line, val_token, ',')){
                 val_token.erase(remove(val_token.begin(), val_token.end(), '['), val_token.end());
                 val_token.erase(remove(val_token.begin(), val_token.end(), ']'), val_token.end());
                 val_token = trim(val_token);
                 if(!val_token.empty()){
                     try {
                         t_bs_coeffs.push_back(stod(val_token));
                     } catch (const std::exception& e) {
                          cerr << "Warning: Could not parse T_BS data value: " << val_token << endl;
                     }
                 }
             }
             if (t_bs_coeffs.size() >= 16) {
                 t_bs_data_mode = false; // Done collecting
             }
         } else {
             string val_str;
             val_str = extract_value(line, "gyroscope_noise_density:");
             if (!val_str.empty()) params.gyro_noise_density = stod(val_str);
 
             val_str = extract_value(line, "gyroscope_random_walk:");
             if (!val_str.empty()) params.gyro_bias_rw_density = stod(val_str);
 
             val_str = extract_value(line, "accelerometer_noise_density:");
             if (!val_str.empty()) params.accel_noise_density = stod(val_str);
 
             val_str = extract_value(line, "accelerometer_random_walk:");
             if (!val_str.empty()) params.accel_bias_rw_density = stod(val_str);
             
             val_str = extract_value(line, "rate_hz:");
             if (!val_str.empty()) params.rate_hz = stod(val_str);
 
             if (line.find("T_BS:") != string::npos) {
                 // Next lines might be rows, cols, then data
             } else if (line.find("data:") != string::npos && t_bs_coeffs.empty()) {
                 t_bs_data_mode = true;
                 // Remove "data:" part and process rest of the line
                 line = line.substr(line.find("data:") + 5);
                  stringstream ss_line(line);
                 string val_token;
                  while(getline(ss_line, val_token, ',')){
                     val_token.erase(remove(val_token.begin(), val_token.end(), '['), val_token.end());
                     val_token.erase(remove(val_token.begin(), val_token.end(), ']'), val_token.end());
                     val_token = trim(val_token);
                     if(!val_token.empty()){
                         try {
                             t_bs_coeffs.push_back(stod(val_token));
                         } catch (const std::exception& e) {
                             cerr << "Warning: Could not parse T_BS data value: " << val_token << endl;
                         }
                     }
                 }
             }
         }
     }
 
     if (t_bs_coeffs.size() >= 16) {
         for (int i = 0; i < 4; ++i) {
             for (int j = 0; j < 4; ++j) {
                 T_BS_mat(i, j) = t_bs_coeffs[i * 4 + j];
             }
         }
         params.body_P_sensor = Pose3(T_BS_mat);
     } else {
         cout << "Warning: T_BS data not fully parsed or missing from YAML. Using Identity." << endl;
         params.body_P_sensor = Pose3(); // Identity
     }
     
     // Basic check if critical params were loaded
     if (params.gyro_noise_density == 0.0 || params.accel_noise_density == 0.0) {
         cerr << "Warning: Some noise densities are zero after parsing YAML. Check file content." << endl;
         // Allow continuation, but this is usually an issue.
     }
     return true;
 }
 
 // --- Helper Functions (from original, largely unchanged) ---
 
 bool parseImuLine(const string& line, ImuData& data) {
     stringstream ss(line);
     string segment;
     vector<string> seglist;
     while (getline(ss, segment, ',')) seglist.push_back(segment);
     if (seglist.size() < 7) return false;
     try {
         data.timestamp = stod(seglist[0]) * 1e-9; // ns to s
         data.omega = Vector3(stod(seglist[1]), stod(seglist[2]), stod(seglist[3]));
         data.acc = Vector3(stod(seglist[4]), stod(seglist[5]), stod(seglist[6]));
     } catch (const std::exception& e) { return false; }
     return true;
 }
 
 bool parseGroundTruthLine(const string& line, GroundTruthData& data) {
     stringstream ss(line);
     string segment;
     vector<string> seglist;
     while (getline(ss, segment, ',')) seglist.push_back(segment);
     if (seglist.size() < 17) return false; // EuRoC GT has 17 fields (timestamp, p, q, v, bias_w, bias_a)
     try {
         data.timestamp = stod(seglist[0]) * 1e-9; // ns to s
         Point3 pos(stod(seglist[1]), stod(seglist[2]), stod(seglist[3]));
         Quaternion q(stod(seglist[4]), stod(seglist[5]), stod(seglist[6]), stod(seglist[7])); // w, x, y, z
         Rot3 rot = Rot3(q);
         Velocity3 vel(stod(seglist[8]), stod(seglist[9]), stod(seglist[10]));
         data.navState = NavState(rot, pos, vel);
         Vector3 bias_gyro(stod(seglist[11]), stod(seglist[12]), stod(seglist[13]));
         Vector3 bias_acc(stod(seglist[14]), stod(seglist[15]), stod(seglist[16]));
         data.bias = imuBias::ConstantBias(bias_acc, bias_gyro);
     } catch (const std::exception& e) { return false; }
     return true;
 }
 
 template <typename T>
 bool loadData(const string& filename, vector<T>& data, bool (*parseFunc)(const string&, T&)) {
     ifstream file(filename);
     if (!file.is_open()) {
         cerr << "Error opening file: " << filename << endl;
         return false;
     }
     string line;
     if (!getline(file, line)) { /* Skip header */ } 
     while (getline(file, line)) {
         if (line.empty() || line[0] == '#') continue;
         T entry;
         if (parseFunc(line, entry)) data.push_back(entry);
     }
     cout << "Loaded " << data.size() << " valid entries from " << filename << endl;
     return !data.empty();
 }
 
 template <typename T>
 size_t findIndexBefore(const vector<T>& sorted_data, double timestamp) {
     auto it = lower_bound(sorted_data.begin(), sorted_data.end(), timestamp,
                           [](const T& element, double value) { return element.timestamp < value; });
     if (it == sorted_data.begin()) {
          if (it != sorted_data.end() && it->timestamp >= timestamp) return 0;
          else {
             // cerr << "Warning: Timestamp " << fixed << setprecision(9) << timestamp
             //      << " is before the first data point " << sorted_data.front().timestamp << endl;
             return 0; 
          }
     }
     return distance(sorted_data.begin(), it) - 1;
 }
 
 template <typename T>
 size_t findIndexAtOrAfter(const vector<T>& sorted_data, double timestamp) {
      auto it = lower_bound(sorted_data.begin(), sorted_data.end(), timestamp,
                            [](const T& element, double value) { return element.timestamp < value; });
      return distance(sorted_data.begin(), it);
 }
 
 double calculateMedian(vector<double>& values) { // Note: modifies values vector
     if (values.empty()) return numeric_limits<double>::quiet_NaN();
     size_t n = values.size();
     size_t mid = n / 2;
     nth_element(values.begin(), values.begin() + mid, values.end());
     if (n % 2 != 0) return values[mid];
     else {
         if (n == 0) return numeric_limits<double>::quiet_NaN(); // Should be caught by empty() check
         if (mid == 0) return values[mid]; // Only one element effectively if n=1, covered by n%2 !=0
         // For even size, ensure mid-1 is valid. nth_element for mid is done.
         // Now find element at mid-1 for the other part of average.
         // The values vector is partially sorted up to mid.
         // We need the (mid-1)th element from the original sorted sequence.
         double mid_val1 = values[mid]; // This is the (n/2)-th element (0-indexed)
         // Find the maximum of elements before 'mid' (which is values[mid-1] after sorting up to mid-1)
         nth_element(values.begin(), values.begin() + mid - 1, values.begin() + mid); // Sort within the lower half
         return (values[mid - 1] + mid_val1) / 2.0;
     }
 }
 
 // --- Core Evaluation Logic ---
 
 shared_ptr<PreintegrationCombinedParams> createCombinedImuParams(
     const SensorParams& sensor_params,
     const Vector3& gravity_n_vec,
     double integration_sigma) {
 
     double nominal_imu_dt = 1.0 / sensor_params.rate_hz;
     if (nominal_imu_dt <= 1e-9) { // Avoid division by zero if rate_hz is 0
         cerr << "Warning: IMU rate_hz is " << sensor_params.rate_hz << ", using default nominal_dt of 0.005s for covariance calculation." << endl;
         nominal_imu_dt = 0.005;
     }
 
     shared_ptr<PreintegrationParams> p_base =
         PreintegrationCombinedParams::MakeSharedU(
             gravity_n_vec.norm()
         );
     
     shared_ptr<PreintegrationCombinedParams> p =
         static_pointer_cast<PreintegrationCombinedParams>(p_base);
 
     p->n_gravity = gravity_n_vec; // Set actual gravity vector (overwrites default direction if magnitude was same)
 
     p->gyroscopeCovariance = pow(sensor_params.gyro_noise_density, 2) / nominal_imu_dt * I_3x3;
     p->accelerometerCovariance = pow(sensor_params.accel_noise_density, 2) / nominal_imu_dt * I_3x3;
     p->integrationCovariance = pow(integration_sigma, 2) * I_3x3;
     p->biasAccCovariance = pow(sensor_params.accel_bias_rw_density, 2) * nominal_imu_dt * I_3x3;
     p->biasOmegaCovariance = pow(sensor_params.gyro_bias_rw_density, 2) * nominal_imu_dt * I_3x3;
     p->biasAccOmegaInt = I_6x6 * 1e-5; // Default, as in original script
 
     return p;
 }
 
 IntervalEvaluationResult evaluateCombinedImuForInterval(
     const GroundTruthData& gt_i,
     const GroundTruthData& gt_j,
     const vector<ImuData>& all_imu_data, // Full IMU data vector
     size_t imu_idx_start_global,         // Start index in all_imu_data
     size_t imu_idx_end_global,           // End index (exclusive) in all_imu_data
     const shared_ptr<PreintegrationCombinedParams>& pim_params) {
 
     IntervalEvaluationResult result;
     result.success = false;
 
     PreintegratedCombinedMeasurements pim_combined(pim_params, gt_i.bias);
 
     double previous_imu_t = gt_i.timestamp; // Start integration from gt_i timestamp
 
     for (size_t k = imu_idx_start_global; k < imu_idx_end_global; ++k) {
         const ImuData& imu = all_imu_data[k];
         // Ensure IMU measurement is within [gt_i.timestamp, gt_j.timestamp]
         // This check is mostly handled by findIndexAtOrAfter for start/end, but good for dt calc
         if (imu.timestamp < gt_i.timestamp || imu.timestamp > gt_j.timestamp + 1e-9) { // Add small tolerance for gt_j.timestamp
             // This case should be rare if indices are chosen carefully,
             // but if it happens, adjust previous_imu_t to avoid large dt
             if (imu.timestamp < previous_imu_t && imu.timestamp >= gt_i.timestamp) previous_imu_t = imu.timestamp; 
             continue;
         }
 
 
         double current_imu_t = imu.timestamp;
         double dt = current_imu_t - previous_imu_t;
 
         if (dt < 1e-9) { // Effectively zero dt or out of order (should not happen with sorted data)
              if (dt < 0) { /* cerr << "Warning: Negative IMU dt " << dt << endl; */ }
              previous_imu_t = current_imu_t;
              continue;
         }
         if (dt > 0.1) { // Warn about large dt, might indicate issues
             // cerr << "Warning: Large IMU dt of " << dt << "s at t=" << current_imu_t << endl;
         }
 
 
         pim_combined.integrateMeasurement(imu.acc, imu.omega, dt);
         previous_imu_t = current_imu_t;
     }
 
     // Integrate from the last IMU measurement up to gt_j.timestamp
     double final_dt = gt_j.timestamp - previous_imu_t;
     if (final_dt > 1e-9 && imu_idx_end_global > imu_idx_start_global && imu_idx_end_global <= all_imu_data.size()) {
         // Use the last IMU measurement that was processed in the loop, which is all_imu_data[imu_idx_end_global - 1]
         // if imu_idx_end_global points one past the last element to include.
         const ImuData& last_imu_in_segment = all_imu_data[imu_idx_end_global -1]; 
         pim_combined.integrateMeasurement(last_imu_in_segment.acc, last_imu_in_segment.omega, final_dt);
     }
 
 
     result.actual_dt = pim_combined.deltaTij();
     if (result.actual_dt < 1e-6) { // Too short interval
         return result;
     }
 
     NavState estimated_state_j = pim_combined.predict(gt_i.navState, gt_i.bias);
     Vector9 error_nav = NavState::Logmap(gt_j.navState.inverse() * estimated_state_j); // More robust error for NavState
     Vector6 error_bias = gt_j.bias.vector() - gt_i.bias.vector(); // Bias error is over the interval
 
     Vector15 error15;
     error15 << error_nav, error_bias;
 
     Matrix15x15 P15 = pim_combined.preintMeasCov();
     if (P15.rows() != 15 || P15.cols() != 15) {
         // cerr << "Error: Covariance matrix size mismatch." << endl;
         return result; // Numerical issue
     }
 
     try {
         Matrix15x15 P15_reg = P15 + Matrix15x15::Identity() * 1e-9; // Regularization
         Matrix15x15 P15_inv = P15_reg.inverse();
         double nees_val = error15.transpose() * P15_inv * error15;
 
         if (!isnan(nees_val) && !isinf(nees_val) && nees_val >= 0) {
             result.nees = nees_val;
             result.success = true;
         }
     } catch (const std::exception& e) {
         // cerr << "Exception during NEES calculation: " << e.what() << endl;
         return result; // Numerical issue
     }
     return result;
 }
 
 void aggregateAndStoreResults(
     const string& dataset_name, double deltaTij_target,
     const vector<double>& nees_values_list_const, // const to allow copying
     const vector<double>& actual_dt_list_const,  // const to allow copying
     int num_processed, int num_skipped_gt, int num_skipped_imu, int num_numerical_issues,
     map<string, map<double, AggregatedRunResults>>& final_results_table) {
 
     AggregatedRunResults agg_results;
     agg_results.dataset_name = dataset_name;
     agg_results.deltaTij_target = deltaTij_target;
     agg_results.processed_intervals = num_processed;
     agg_results.skipped_intervals_gt = num_skipped_gt;
     agg_results.skipped_intervals_imu = num_skipped_imu;
     agg_results.numerical_issue_count = num_numerical_issues;
 
     vector<double> nees_values_list_mutable = nees_values_list_const; // mutable copy for median
     vector<double> actual_dt_list_mutable = actual_dt_list_const; // mutable copy for median
 
     if (!nees_values_list_const.empty()) {
         agg_results.median_nees = calculateMedian(nees_values_list_mutable); // Modifies nees_values_list_mutable
 
         double sum_nees = 0;
         double sum_sq_nees = 0;
         for (double n : nees_values_list_const) { 
             sum_nees += n;
             sum_sq_nees += n * n;
         }
         agg_results.mean_nees = sum_nees / nees_values_list_const.size();
         if (nees_values_list_const.size() > 1) {
              // variance = E[X^2] - (E[X])^2
             double variance = (sum_sq_nees / nees_values_list_const.size()) - (agg_results.mean_nees * agg_results.mean_nees);
             // Ensure variance is non-negative due to potential floating point inaccuracies
             agg_results.std_dev_nees = sqrt(max(0.0, variance));
         } else {
             agg_results.std_dev_nees = 0;
         }
     }
 
     if (!actual_dt_list_const.empty()) {
         agg_results.median_actual_dt = calculateMedian(actual_dt_list_mutable); 
         double sum_dt = 0;
         for (double dt_val : actual_dt_list_const) { 
             sum_dt += dt_val;
         }
         agg_results.mean_actual_dt = sum_dt / actual_dt_list_const.size();
     }
     
     final_results_table[dataset_name][deltaTij_target] = agg_results;
     agg_results.print_summary(); 
 }
 
 void printFinalTable(const map<string, map<double, AggregatedRunResults>>& final_results_table,
                      const vector<string>& datasets, const vector<double>& deltas) {
     cout << "\n\n--- Overall Results Table (Median 15-DOF NEES) ---" << endl;
 
     // Header
     cout << setw(20) << left << "Dataset";
     for (double dt_val : deltas) {
         stringstream ss;
         ss << "dT=" << fixed << setprecision(1) << dt_val << "s";
         cout << setw(12) << right << ss.str();
     }
     cout << endl;
     cout << string(20 + deltas.size() * 12, '-') << endl;
 
     // Data rows
     for (const string& dataset_name : datasets) {
         cout << setw(20) << left << dataset_name;
         auto it_dataset = final_results_table.find(dataset_name);
         if (it_dataset != final_results_table.end()) {
             for (double dt_val : deltas) {
                 auto it_delta = it_dataset->second.find(dt_val);
                 if (it_delta != it_dataset->second.end()) {
                     if (it_delta->second.processed_intervals > 0 && isfinite(it_delta->second.median_nees)) {
                         cout << setw(12) << right << fixed << setprecision(3) << it_delta->second.median_nees;
                     } else {
                         cout << setw(12) << right << "N/A";
                     }
                 } else {
                     cout << setw(12) << right << "-"; // No data for this deltaT
                 }
             }
         } else {
              for (size_t i=0; i<deltas.size(); ++i) cout << setw(12) << right << "--"; // No data for this dataset
         }
         cout << endl;
     }
     cout << "----------------------------------------------------" << endl;
 }
 
 
 // --- Main Program ---
 int main(int argc, char* argv[]) {
     cout << "Starting EuRoC Preintegration Evaluation (15-DOF NEES)..." << endl;
     cout << "Base EuRoC Path: " << base_euroc_data_path << endl;
 
     map<string, map<double, AggregatedRunResults>> all_run_results;
 
     for (const string& dataset_name : dataset_sequences) {
         cout << "\n====================================================" << endl;
         cout << "Processing Dataset: " << dataset_name << endl;
         cout << "====================================================" << endl;
 
         string current_dataset_path = base_euroc_data_path + "/" + dataset_name;
         string imu_csv_file = current_dataset_path + "/mav0/imu0/data.csv";
         string ground_truth_csv_file = current_dataset_path + "/mav0/state_groundtruth_estimate0/data.csv";
         string sensor_yaml_file = current_dataset_path + "/mav0/imu0/sensor.yaml";
 
         // 1. Load Sensor Parameters from YAML
         SensorParams sensor_params;
         if (!parseSensorYaml(sensor_yaml_file, sensor_params)) {
             cerr << "Failed to load sensor params for " << dataset_name << ". Skipping dataset." << endl;
             continue;
         }
         cout << "  Loaded Sensor Params for " << dataset_name << ":" << endl;
         cout << "    Gyro Noise Density: " << scientific << sensor_params.gyro_noise_density << endl;
         cout << "    Gyro Bias RW Density: " << sensor_params.gyro_bias_rw_density << endl;
         cout << "    Accel Noise Density: " << sensor_params.accel_noise_density << endl;
         cout << "    Accel Bias RW Density: " << sensor_params.accel_bias_rw_density << endl;
         cout << "    IMU Rate (Hz): " << fixed << setprecision(1) << sensor_params.rate_hz << endl;
         // cout << "    Body_P_Sensor (T_BS): \n" << sensor_params.body_P_sensor.matrix() << endl;
 
 
         // 2. Load IMU and GT Data
         vector<ImuData> imu_data;
         vector<GroundTruthData> gt_data;
         if (!loadData(imu_csv_file, imu_data, parseImuLine) || imu_data.empty()) {
             cerr << "Failed to load or empty IMU data for " << dataset_name << ". Skipping." << endl;
             continue;
         }
         if (!loadData(ground_truth_csv_file, gt_data, parseGroundTruthLine) || gt_data.empty()) {
             cerr << "Failed to load or empty GT data for " << dataset_name << ". Skipping." << endl;
             continue;
         }
         
         if (gt_data.front().bias.gyroscope().norm() < 1e-9 && gt_data.front().bias.accelerometer().norm() < 1e-9 &&
             gt_data.back().bias.gyroscope().norm() < 1e-9 && gt_data.back().bias.accelerometer().norm() < 1e-9) {
              cout << "Warning: Ground truth biases are all zero for " << dataset_name << ". Bias error might be consistently zero if initial bias guess is also zero." << endl;
         }
 
 
         for (double deltaTij_target : deltaTij_values) {
             cout << "\n  --- Processing with Target deltaTij: " << fixed << setprecision(1) << deltaTij_target << " s ---" << endl;
 
             // 3. Setup Preintegration Parameters for this run
             auto p_combined = createCombinedImuParams(sensor_params, gravity_n, integration_noise_sigma);
             // p_combined->print("CombinedParams"); // For debugging
 
             vector<double> current_run_nees_values;
             vector<double> current_run_actual_dt_values;
             int processed_intervals_count = 0;
             int skipped_gt_count = 0;
             int skipped_imu_count = 0;
             int numerical_issues_count = 0;
 
             double min_sync_time = max(imu_data.front().timestamp, gt_data.front().timestamp);
             double max_sync_time = min(imu_data.back().timestamp, gt_data.back().timestamp);
 
             size_t first_valid_gt_idx = findIndexAtOrAfter(gt_data, min_sync_time);
             if (first_valid_gt_idx >= gt_data.size()) {
                 cerr << "  Error: No ground truth data found at or after minimum start time for " << dataset_name << ". Skipping deltaTij " << deltaTij_target << endl;
                 continue;
             }
             double current_t_i = gt_data[first_valid_gt_idx].timestamp;
             
             cout << fixed << setprecision(9); // Higher precision for timestamps in logs
             // cout << "  Processing data from " << current_t_i << " s to " << max_sync_time << " s" << endl;
 
 
             while (current_t_i + deltaTij_target <= max_sync_time + 1e-7) { // Add small tolerance for max_sync_time comparison
                 double t_j_nominal_end = current_t_i + deltaTij_target;
 
                 size_t gt_idx_i = findIndexAtOrAfter(gt_data, current_t_i - 1e-7); // Allow slight mismatch for current_t_i lookup
                 if (gt_idx_i >= gt_data.size() || abs(gt_data[gt_idx_i].timestamp - current_t_i) > 0.01 ) { // If no GT close to current_t_i
                     // Try to advance current_t_i to the next GT sample if it's far off, or to next window
                     size_t next_gt_after_current = findIndexAtOrAfter(gt_data, current_t_i + 1e-7);
                     if(next_gt_after_current < gt_data.size() && gt_data[next_gt_after_current].timestamp < t_j_nominal_end) {
                         current_t_i = gt_data[next_gt_after_current].timestamp;
                     } else {
                         current_t_i = t_j_nominal_end; // Advance to next theoretical window start
                     }
                     skipped_gt_count++;
                     continue;
                 }
                 const GroundTruthData& gt_i_data = gt_data[gt_idx_i];
                 // Sync current_t_i to actual GT timestamp for this interval's start
                 // This is crucial: the interval truly starts at gt_i_data.timestamp
                 double actual_interval_t_i = gt_i_data.timestamp; 
                 double actual_interval_t_j_target = actual_interval_t_i + deltaTij_target;
 
 
                 size_t gt_idx_j = findIndexBefore(gt_data, actual_interval_t_j_target + 1e-7); // Find GT at or just BEFORE t_j_target
                 if (gt_idx_j <= gt_idx_i || gt_idx_j >= gt_data.size() || gt_data[gt_idx_j].timestamp < actual_interval_t_i + 0.1 * deltaTij_target) { // Ensure t_j is meaningfully after t_i
                     current_t_i = t_j_nominal_end; 
                     skipped_gt_count++;
                     continue;
                 }
                 const GroundTruthData& gt_j_data = gt_data[gt_idx_j];
                 double actual_interval_t_j_gt_synced = gt_j_data.timestamp; // This is the actual end time of the interval based on GT availability
 
                 // Find IMU data for [actual_interval_t_i, actual_interval_t_j_gt_synced]
                 size_t imu_idx_start = findIndexAtOrAfter(imu_data, actual_interval_t_i - 1e-7); // inclusive start
                 size_t imu_idx_end = findIndexAtOrAfter(imu_data, actual_interval_t_j_gt_synced + 1e-7); // make it inclusive for last measurement by finding one *after*
 
                 // Ensure imu_idx_end points one past the last IMU measurement *strictly before or at* actual_interval_t_j_gt_synced.
                 // The original findIndexAtOrAfter gets the first element >= timestamp.
                 // If imu_data[imu_idx_end-1].timestamp is what we want as the last IMU data point if its timestamp is <= actual_interval_t_j_gt_synced
                 
                 // Refined imu_idx_end logic: we want IMU data up to and including measurements at actual_interval_t_j_gt_synced.
                 // findIndexAtOrAfter(data, ts) gives index `k` such that data[k].ts >= ts.
                 // So, loop from imu_idx_start up to (but not including) imu_idx_end
                 // This means measurements from imu_data[imu_idx_start]...imu_data[imu_idx_end-1] will be processed.
                 // The last IMU data point whose timestamp is <= actual_interval_t_j_gt_synced.
 
                 if (imu_idx_start >= imu_data.size() || imu_idx_start >= imu_idx_end || (imu_idx_end == imu_idx_start && imu_data[imu_idx_start].timestamp > actual_interval_t_j_gt_synced ) || (imu_idx_end > imu_idx_start && imu_data[imu_idx_start].timestamp > actual_interval_t_j_gt_synced + 1e-7) ) {
                     current_t_i = t_j_nominal_end; 
                     skipped_imu_count++;
                     continue;
                 }
                  // Ensure at least one IMU measurement is processed if imu_idx_start == imu_idx_end
                 if (imu_idx_start == imu_idx_end) { // Only one IMU message could cover the interval
                     if (imu_data[imu_idx_start].timestamp < actual_interval_t_i - 1e-7 || imu_data[imu_idx_start].timestamp > actual_interval_t_j_gt_synced + 1e-7) {
                        current_t_i = t_j_nominal_end; 
                        skipped_imu_count++;
                        continue;
                     }
                     // if imu_idx_start is valid, imu_idx_end should be imu_idx_start + 1 to process this one measurement
                     // This case needs careful handling in integrate loop or here.
                     // For now, let's ensure imu_idx_end > imu_idx_start for the loop.
                     // The integration loop structure might need adjustment if only one IMU point is in range for a very short interval.
                     // But usually deltaTij is large enough for multiple IMU points.
                     // If imu_idx_start == imu_idx_end, it means no IMU data strictly within, or only one point at the boundary.
                     // The current loop `for (size_t k = imu_idx_start; k < imu_idx_end; ++k)` will not run if start == end.
                     // This needs to be at least 1 for the loop to run.
                     if (imu_idx_end <= imu_idx_start) { // ensure at least one measurement can be processed by the loop
                          if (imu_idx_end < imu_data.size() && imu_data[imu_idx_end].timestamp <= actual_interval_t_j_gt_synced + 1e-7) {
                             // This means findIndexAtOrAfter found a point at actual_interval_t_j_gt_synced, so imu_idx_end should be that index + 1
                             // This logic is getting complex, the loop in evaluateCombinedImuForInterval needs to be robust.
                          } else {
                              current_t_i = t_j_nominal_end; 
                              skipped_imu_count++;
                              continue;
                          }
                     }
                 }
 
 
                 // --- Evaluate Preintegration Method(s) for this interval ---
                 // Pass gt_i_data (synced t_i) and gt_j_data (synced t_j)
                 IntervalEvaluationResult combined_eval = evaluateCombinedImuForInterval(
                     gt_i_data, gt_j_data, imu_data, imu_idx_start, imu_idx_end, p_combined);
 
                 if (combined_eval.success) {
                     current_run_nees_values.push_back(combined_eval.nees);
                     current_run_actual_dt_values.push_back(combined_eval.actual_dt);
                     processed_intervals_count++;
                 } else {
                     numerical_issues_count++;
                 }
                 
                 // Advance current_t_i for the next *non-overlapping* window
                 // Start next window based on the *actual start* of the current one + target duration
                 current_t_i = actual_interval_t_i + deltaTij_target; 
             } // End while loop over intervals
 
             // 4. Aggregate and store results for this (dataset, deltaTij) run
             aggregateAndStoreResults(dataset_name, deltaTij_target,
                                      current_run_nees_values, current_run_actual_dt_values,
                                      processed_intervals_count, skipped_gt_count,
                                      skipped_imu_count, numerical_issues_count,
                                      all_run_results);
 
         } // End loop over deltaTij_values
     } // End loop over dataset_names
 
     // 5. Print Final Summary Table
     printFinalTable(all_run_results, dataset_sequences, deltaTij_values);
 
     cout << "\nEvaluation finished." << endl;
     return 0;
 }