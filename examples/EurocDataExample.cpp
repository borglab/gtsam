/**
 * @file EurocDataExample.cpp
 * @brief Example evaluating IMU Preintegration consistency (15-DOF) on EuRoC dataset.
 * @author Matt Kielo, Porter Zach
 */

 #include <gtsam/navigation/NavState.h>
 #include <gtsam/navigation/ImuBias.h>
 #include <gtsam/navigation/CombinedImuFactor.h> 
 #include <gtsam/navigation/PreintegrationCombinedParams.h>
 #include <gtsam/navigation/ImuFactor.h> 
 #include <gtsam/navigation/PreintegrationParams.h> 
 #include <gtsam/navigation/PreintegratedRotation.h>
 
 #include <gtsam/geometry/Pose3.h>
 #include <gtsam/geometry/Rot3.h>
 #include <gtsam/base/Matrix.h> 
 
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
 #include <map>    
 #include <memory>
 
 using namespace gtsam;
 using namespace std;
 
 typedef Eigen::Matrix<double, 15, 1> Vector15;
 typedef Eigen::Matrix<double, 15, 15> Matrix15x15;
 
 // --- Configuration ---
 // Update this path with the location of your EuRoC data, which should contain
 // folders like "MH_01", "V2_02", etc.
 const string base_euroc_data_path = "C:/gtsam/euroc/data";
 const vector<string> dataset_sequences = { "MH_01", "V2_02" };
 const vector<double> deltaTij_values = {0.2, 0.5, 1.0};
 const double integration_noise_sigma = 1e-8;
 const Vector3 gravity_n(0, 0, -9.81);
 
 // --- Data Structures (condensed for brevity) ---
 struct ImuData { double timestamp; Vector3 omega; Vector3 acc; };
 struct GroundTruthData { double timestamp; NavState navState; imuBias::ConstantBias bias; };
 struct SensorParams {
     double gyro_noise_density = 0.0; double gyro_bias_rw_density = 0.0;
     double accel_noise_density = 0.0; double accel_bias_rw_density = 0.0;
     Pose3 body_P_sensor; double rate_hz = 200.0;
 };
 struct IntervalEvaluationResult { double nees = numeric_limits<double>::quiet_NaN(); bool success = false; double actual_dt = 0.0; };
 struct AggregatedRunResults {
     string dataset_name; double deltaTij_target = 0.0;
     double median_nees = numeric_limits<double>::quiet_NaN(); double mean_nees = numeric_limits<double>::quiet_NaN();
     double std_dev_nees = numeric_limits<double>::quiet_NaN();
     double median_actual_dt = numeric_limits<double>::quiet_NaN(); double mean_actual_dt = numeric_limits<double>::quiet_NaN();
     int processed_intervals = 0; int skipped_intervals_gt = 0;
     int skipped_intervals_imu = 0; int numerical_issue_count = 0;
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
         } else { cout << "    No NEES results to report." << endl; }
         cout << "  -----------------------------------" << endl;
     }
 };
 
 // --- Helper Functions (condensed) ---
 string trim(const string& str, const string& whitespace = " \t\n\r\f\v") { 
     const auto strBegin = str.find_first_not_of(whitespace); if (strBegin == string::npos) return ""; 
     const auto strEnd = str.find_last_not_of(whitespace); const auto strRange = strEnd - strBegin + 1; return str.substr(strBegin, strRange);
 }
 bool parseSensorYaml(const string& yaml_path, SensorParams& params) { 
     ifstream file(yaml_path); if (!file.is_open()) { cerr << "Error opening sensor YAML file: " << yaml_path << endl; return false; }
     string line; Matrix4 T_BS_mat = Matrix4::Identity(); bool t_bs_data_mode = false; vector<double> t_bs_coeffs;
     auto extract_value = [&](const string& l, const string& key) { size_t key_pos = l.find(key); if (key_pos != string::npos) { string val_str = l.substr(key_pos + key.length()); return trim(val_str); } return string(""); };
     while (getline(file, line)) {
         line = trim(line); if (line.empty() || line[0] == '#') continue;
         if (t_bs_data_mode) { stringstream ss_line(line); string val_token; while(getline(ss_line, val_token, ',')){ val_token.erase(remove(val_token.begin(), val_token.end(), '['), val_token.end()); val_token.erase(remove(val_token.begin(), val_token.end(), ']'), val_token.end()); val_token = trim(val_token); if(!val_token.empty()){ try { t_bs_coeffs.push_back(stod(val_token)); } catch (const std::exception&) {}}} if (t_bs_coeffs.size() >= 16) { t_bs_data_mode = false; }
         } else {
             string val_str;
             val_str = extract_value(line, "gyroscope_noise_density:"); if (!val_str.empty()) params.gyro_noise_density = stod(val_str);
             val_str = extract_value(line, "gyroscope_random_walk:"); if (!val_str.empty()) params.gyro_bias_rw_density = stod(val_str);
             val_str = extract_value(line, "accelerometer_noise_density:"); if (!val_str.empty()) params.accel_noise_density = stod(val_str);
             val_str = extract_value(line, "accelerometer_random_walk:"); if (!val_str.empty()) params.accel_bias_rw_density = stod(val_str);
             val_str = extract_value(line, "rate_hz:"); if (!val_str.empty()) params.rate_hz = stod(val_str);
             if (line.find("T_BS:") != string::npos) {} else if (line.find("data:") != string::npos && t_bs_coeffs.empty()) { t_bs_data_mode = true; line = line.substr(line.find("data:") + 5); stringstream ss_line(line); string val_token; while(getline(ss_line, val_token, ',')){ val_token.erase(remove(val_token.begin(), val_token.end(), '['), val_token.end()); val_token.erase(remove(val_token.begin(), val_token.end(), ']'), val_token.end()); val_token = trim(val_token); if(!val_token.empty()){ try { t_bs_coeffs.push_back(stod(val_token)); } catch (const std::exception&) {}}}}
         }
     }
     if (t_bs_coeffs.size() >= 16) { for (int i = 0; i < 4; ++i) { for (int j = 0; j < 4; ++j) { T_BS_mat(i, j) = t_bs_coeffs[i * 4 + j]; } } params.body_P_sensor = Pose3(T_BS_mat); } else { params.body_P_sensor = Pose3(); }
     return true;
 }
 bool parseImuLine(const string& line, ImuData& data) { 
     stringstream ss(line); string segment; vector<string> seglist; while (getline(ss, segment, ',')) seglist.push_back(segment); if (seglist.size() < 7) return false;
     try { data.timestamp = stod(seglist[0]) * 1e-9; data.omega = Vector3(stod(seglist[1]), stod(seglist[2]), stod(seglist[3])); data.acc = Vector3(stod(seglist[4]), stod(seglist[5]), stod(seglist[6])); } catch (const std::exception&) { return false; } return true;
 }
 bool parseGroundTruthLine(const string& line, GroundTruthData& data) { 
     stringstream ss(line); string segment; vector<string> seglist; while (getline(ss, segment, ',')) seglist.push_back(segment); if (seglist.size() < 17) return false;
     try { data.timestamp = stod(seglist[0]) * 1e-9; Point3 pos(stod(seglist[1]), stod(seglist[2]), stod(seglist[3])); Quaternion q(stod(seglist[4]), stod(seglist[5]), stod(seglist[6]), stod(seglist[7])); Rot3 rot = Rot3(q); Velocity3 vel(stod(seglist[8]), stod(seglist[9]), stod(seglist[10])); data.navState = NavState(rot, pos, vel); Vector3 bias_gyro(stod(seglist[11]), stod(seglist[12]), stod(seglist[13])); Vector3 bias_acc(stod(seglist[14]), stod(seglist[15]), stod(seglist[16])); data.bias = imuBias::ConstantBias(bias_acc, bias_gyro); } catch (const std::exception&) { return false; } return true;
 }
 template <typename T> bool loadData(const string& filename, vector<T>& data, bool (*parseFunc)(const string&, T&)) {
     ifstream file(filename); if (!file.is_open()) { return false; } string line; if (!getline(file, line)) { return false; }
     while (getline(file, line)) { if (line.empty() || line[0] == '#') continue; T entry; if (parseFunc(line, entry)) data.push_back(entry); } return !data.empty();
 }
 template <typename T> size_t findIndexBefore(const vector<T>& sorted_data, double timestamp) { 
     auto it = lower_bound(sorted_data.begin(), sorted_data.end(), timestamp, [](const T& e, double v) { return e.timestamp < v; }); if (it == sorted_data.begin()) { return 0; } return distance(sorted_data.begin(), it) - 1;
 }
 template <typename T> size_t findIndexAtOrAfter(const vector<T>& sorted_data, double timestamp) {
     auto it = lower_bound(sorted_data.begin(), sorted_data.end(), timestamp, [](const T& e, double v) { return e.timestamp < v; }); return distance(sorted_data.begin(), it);
 }
 double calculateMedian(vector<double>& values) { 
     if (values.empty()) return numeric_limits<double>::quiet_NaN(); size_t n = values.size(); size_t mid = n / 2; nth_element(values.begin(), values.begin() + mid, values.end());
     if (n % 2 != 0) return values[mid]; else { if (mid == 0) return values[mid]; double m1 = values[mid]; nth_element(values.begin(), values.begin() + mid - 1, values.begin() + mid); return (values[mid - 1] + m1) / 2.0; }
 }
 
 // --- IMU Preintegration Technique Abstraction ---
 class ImuPreintegrationTechnique {
 public:
     virtual ~ImuPreintegrationTechnique() = default;
     virtual string name() const = 0;
 
     virtual shared_ptr<PreintegrationParams> createPreintegrationParams(
         const SensorParams& sensor_params, const Vector3& gravity_n_vec,
         double integration_sigma, const Vector3& body_omega_coriolis) const = 0;
 
     virtual IntervalEvaluationResult evaluateInterval(
         const GroundTruthData& gt_i, const GroundTruthData& gt_j,
         const vector<ImuData>& all_imu_data, size_t imu_idx_start_global, size_t imu_idx_end_global,
         const shared_ptr<PreintegrationParams>& pim_params_base,
         const imuBias::ConstantBias& initial_bias, const SensorParams& sensor_params
     ) const = 0;
 
 protected:
     // Sets parameters common to PreintegrationParams and its base PreintegratedRotationParams
     void setupBasePreintegratedRotationParams(PreintegratedRotationParams* p_rot, const SensorParams& sensor_params,
                                             const Vector3& body_omega_coriolis) const {
         double nominal_imu_dt = 1.0 / sensor_params.rate_hz;
         if (nominal_imu_dt <= 1e-9) nominal_imu_dt = 0.005;
 
         p_rot->gyroscopeCovariance = pow(sensor_params.gyro_noise_density, 2) / nominal_imu_dt * I_3x3;
         if (body_omega_coriolis.norm() > 1e-9) { // Only set if non-zero, otherwise keep default std::nullopt
             p_rot->omegaCoriolis = body_omega_coriolis;
         }
         p_rot->body_P_sensor = sensor_params.body_P_sensor; // This is an optional<Pose3>
     }
 };
 
 // --- Combined IMU Factor Technique ---
 class CombinedImuTechnique : public ImuPreintegrationTechnique {
 public:
     string name() const override { return "CombinedIMU"; }
 
     shared_ptr<PreintegrationParams> createPreintegrationParams(
         const SensorParams& sensor_params, const Vector3& gravity_n_vec,
         double integration_sigma, const Vector3& body_omega_coriolis /* body_omega_coriolis effectively unused for this technique */) const override {
         
         // PreintegrationCombinedParams constructor takes gravity magnitude
         auto p_combined = PreintegrationCombinedParams::MakeSharedU(gravity_n_vec.norm());
         
         double nominal_imu_dt = 1.0 / sensor_params.rate_hz;
         if (nominal_imu_dt <= 1e-9) nominal_imu_dt = 0.005;
 
         p_combined->gyroscopeCovariance = pow(sensor_params.gyro_noise_density, 2) / nominal_imu_dt * I_3x3;
         p_combined->accelerometerCovariance = pow(sensor_params.accel_noise_density, 2) / nominal_imu_dt * I_3x3;
         p_combined->integrationCovariance = pow(integration_sigma, 2) * I_3x3; 
         p_combined->biasAccCovariance = pow(sensor_params.accel_bias_rw_density, 2) * nominal_imu_dt * I_3x3;
         p_combined->biasOmegaCovariance = pow(sensor_params.gyro_bias_rw_density, 2) * nominal_imu_dt * I_3x3;
         p_combined->biasAccOmegaInt = I_6x6 * 1e-5; // Default
 
         // body_P_sensor and omegaCoriolis will be std::nullopt by default from 
         // PreintegrationCombinedParams::MakeSharedU. This is consistent with the original
         // script not explicitly setting these on its PreintegrationCombinedParams object.
 
         return p_combined;
     }
 
     IntervalEvaluationResult evaluateInterval(
         const GroundTruthData& gt_i, const GroundTruthData& gt_j,
         const vector<ImuData>& all_imu_data, size_t imu_idx_start_global, size_t imu_idx_end_global,
         const shared_ptr<PreintegrationParams>& pim_params_base,
         const imuBias::ConstantBias& initial_bias, const SensorParams& /* sensor_params - not directly used here after PIM setup */
     ) const override {
         
         auto pim_params = static_pointer_cast<PreintegrationCombinedParams>(pim_params_base);
         IntervalEvaluationResult result; result.success = false;
         PreintegratedCombinedMeasurements pim_combined(pim_params, initial_bias); // initial_bias is gt_i.bias
 
         double previous_imu_t = gt_i.timestamp;
         for (size_t k = imu_idx_start_global; k < imu_idx_end_global; ++k) {
             const ImuData& imu = all_imu_data[k];
             if (imu.timestamp < gt_i.timestamp || imu.timestamp > gt_j.timestamp + 1e-9) {
                 continue;
             }
             double current_imu_t = imu.timestamp; double dt = current_imu_t - previous_imu_t;
             if (dt < 1e-9) { // Effectively zero dt or out of order
                 if (dt < 0 && abs(dt) > 1e-7) { cerr << "Warning: Negative IMU dt " << dt << endl; } 
                 previous_imu_t = current_imu_t; 
                 continue; 
             }
             pim_combined.integrateMeasurement(imu.acc, imu.omega, dt);
             previous_imu_t = current_imu_t;
         }
 
         double final_dt = gt_j.timestamp - previous_imu_t;
         if (final_dt > 1e-9 && imu_idx_end_global > imu_idx_start_global && imu_idx_end_global <= all_imu_data.size()) {
             const ImuData& last_imu_in_segment = all_imu_data[imu_idx_end_global - 1];
             pim_combined.integrateMeasurement(last_imu_in_segment.acc, last_imu_in_segment.omega, final_dt);
         }
         
         result.actual_dt = pim_combined.deltaTij();
         if (result.actual_dt < 1e-6) return result;
 
         // For CombinedIMU, predict returns NavStateAndBias.
         // The error should be calculated against both parts if using pim_combined.preintMeasCov() directly.
         // To match the pattern of error_bias = gt_j.bias - initial_bias (gt_i.bias),
         // ensure pim_combined.preintMeasCov() is consistent with this error definition.
         NavState estimated_state_j = pim_combined.predict(gt_i.navState, initial_bias);
         Vector9 error_nav = NavState::Logmap(gt_j.navState.inverse() * estimated_state_j);
         
         Vector6 error_bias = gt_j.bias.vector() - initial_bias.vector(); // initial_bias is gt_i.bias
 
         Vector15 error15; error15 << error_nav, error_bias;
         Matrix15x15 P15 = pim_combined.preintMeasCov();
         if (P15.rows() != 15 || P15.cols() != 15) return result;
         try {
             Matrix15x15 P15_reg = P15 + Matrix15x15::Identity() * 1e-9; 
             Matrix15x15 P15_inv = P15_reg.inverse();
             double nees_val = error15.transpose() * P15_inv * error15;
             if (!isnan(nees_val) && !isinf(nees_val) && nees_val >= 0) {
                 result.nees = nees_val; result.success = true;
             }
         } catch (const std::exception&) { }
         return result;
     }
 };
 
 // --- Manifold IMU Preintegration Technique ---
 class ManifoldImuTechnique : public ImuPreintegrationTechnique {
 public:
     string name() const override { return "ManifoldIMU"; }
 
     shared_ptr<PreintegrationParams> createPreintegrationParams(
         const SensorParams& sensor_params, const Vector3& gravity_n_vec,
         double /* integration_sigma - not used by standard PIM */, const Vector3& body_omega_coriolis) const override {
         
         auto p_manifold = PreintegrationParams::MakeSharedU(gravity_n_vec.norm());
 
         // For ManifoldIMU, using setupBasePreintegratedRotationParams is standard
         // as it sets body_P_sensor and omegaCoriolis if needed.
         setupBasePreintegratedRotationParams(static_cast<PreintegratedRotationParams*>(p_manifold.get()),
                                             sensor_params, body_omega_coriolis);
 
         double nominal_imu_dt = 1.0 / sensor_params.rate_hz;
         if (nominal_imu_dt <= 1e-9) nominal_imu_dt = 0.005;
         p_manifold->accelerometerCovariance = pow(sensor_params.accel_noise_density, 2) / nominal_imu_dt * I_3x3;
         p_manifold->integrationCovariance = Matrix33::Zero(); // Standard PIM has no integration covariance term like CombinedPIM
         // Note: biasAccCovariance and biasOmegaCovariance are not members of PreintegrationParams.
         // They will be used directly from sensor_params in evaluateInterval for covariance construction.
         return p_manifold;
     }
 
     IntervalEvaluationResult evaluateInterval(
         const GroundTruthData& gt_i, const GroundTruthData& gt_j,
         const vector<ImuData>& all_imu_data, size_t imu_idx_start_global, size_t imu_idx_end_global,
         const shared_ptr<PreintegrationParams>& pim_params_base,
         const imuBias::ConstantBias& initial_bias, const SensorParams& sensor_params
     ) const override {
         
         IntervalEvaluationResult result; result.success = false;
         PreintegratedImuMeasurements pim_manifold(pim_params_base, initial_bias); // initial_bias is gt_i.bias
 
         double previous_imu_t = gt_i.timestamp;
         for (size_t k = imu_idx_start_global; k < imu_idx_end_global; ++k) {
             const ImuData& imu = all_imu_data[k];
             if (imu.timestamp < gt_i.timestamp -1e-9 || imu.timestamp > gt_j.timestamp + 1e-9) {
                 continue;
             }
             double current_imu_t = imu.timestamp; double dt = current_imu_t - previous_imu_t;
             if (dt < 1e-9) { // Effectively zero dt or out of order (assuming data is sorted, dt < 0 should not happen often)
                 if (dt < 0 && abs(dt) > 1e-7) { cerr << "Warning: Negative IMU dt " << dt << endl; }
                 previous_imu_t = current_imu_t; // Advance to skip this measurement for dt calculation of next
                 continue; 
             }
             pim_manifold.integrateMeasurement(imu.acc, imu.omega, dt);
             previous_imu_t = current_imu_t;
         }
 
         double final_dt = gt_j.timestamp - previous_imu_t;
         if (final_dt > 1e-9 && imu_idx_end_global > imu_idx_start_global && imu_idx_end_global <= all_imu_data.size()) {
             const ImuData& last_imu_in_segment = all_imu_data[imu_idx_end_global - 1];
             pim_manifold.integrateMeasurement(last_imu_in_segment.acc, last_imu_in_segment.omega, final_dt);
         }
         
         result.actual_dt = pim_manifold.deltaTij();
         if (result.actual_dt < 1e-6) return result;
 
         NavState estimated_state_j = pim_manifold.predict(gt_i.navState, initial_bias);
         Vector9 error_nav = NavState::Logmap(gt_j.navState.inverse() * estimated_state_j);
         Vector6 error_bias = gt_j.bias.vector() - initial_bias.vector(); // Bias error is (actual bias_j) - (actual bias_i)
 
         Vector15 error15; error15 << error_nav, error_bias;
         Matrix15x15 P15 = Matrix15x15::Zero();
         P15.block<9,9>(0,0) = pim_manifold.preintMeasCov(); // This is 9x9 covariance from standard PIM
         
         double dt_ij = pim_manifold.deltaTij();
         // Use sensor_params directly for bias random walk variances for Manifold PIM's 15-DOF covariance
         double acc_bias_rw_var_comp = pow(sensor_params.accel_bias_rw_density, 2) * dt_ij;
         double gyro_bias_rw_var_comp = pow(sensor_params.gyro_bias_rw_density, 2) * dt_ij;
         P15.block<3,3>(9,9) = Matrix33::Identity() * acc_bias_rw_var_comp;
         P15.block<3,3>(12,12) = Matrix33::Identity() * gyro_bias_rw_var_comp;
         // Off-diagonal blocks between NavState and Bias are zero for this simple construction,
         // assuming independence between IMU measurement noise (affecting error_nav) and bias random walk noise (affecting error_bias).
         
         if (P15.rows() != 15 || P15.cols() != 15) return result;
         try {
             Matrix15x15 P15_reg = P15 + Matrix15x15::Identity() * 1e-9; 
             Matrix15x15 P15_inv = P15_reg.inverse();
             double nees_val = error15.transpose() * P15_inv * error15;
             if (!isnan(nees_val) && !isinf(nees_val) && nees_val >= 0) {
                 result.nees = nees_val; result.success = true;
             }
         } catch (const std::exception&) { }
         return result;
     }
 };
 
 // --- Results Aggregation and Printing (condensed) ---
 void aggregateAndStoreResults(
     const string& d_name, double dT_target, const vector<double>& nees_l, const vector<double>& dt_l,
     int proc, int skip_gt, int skip_imu, int num_iss, map<string, map<double, AggregatedRunResults>>& tbl) { 
     AggregatedRunResults agg_r; agg_r.dataset_name = d_name; agg_r.deltaTij_target = dT_target;
     agg_r.processed_intervals = proc; agg_r.skipped_intervals_gt = skip_gt; agg_r.skipped_intervals_imu = skip_imu; agg_r.numerical_issue_count = num_iss;
     vector<double> nees_m = nees_l; vector<double> dt_m = dt_l;
     if (!nees_l.empty()) { agg_r.median_nees = calculateMedian(nees_m); double s_n = 0, s_sq_n = 0; for (double n : nees_l) { s_n += n; s_sq_n += n * n; } agg_r.mean_nees = s_n / nees_l.size(); if (nees_l.size() > 1) { double var = (s_sq_n / nees_l.size()) - (agg_r.mean_nees * agg_r.mean_nees); agg_r.std_dev_nees = sqrt(max(0.0, var)); } else { agg_r.std_dev_nees = 0; }}
     if (!dt_l.empty()) { agg_r.median_actual_dt = calculateMedian(dt_m); double s_dt = 0; for (double val : dt_l) s_dt += val; agg_r.mean_actual_dt = s_dt / dt_l.size(); }
     tbl[d_name][dT_target] = agg_r; agg_r.print_summary(); 
 }
 void printSingleTechniqueTable(const map<string, map<double, AggregatedRunResults>>& tbl,
                             const vector<string>& dsets, const vector<double>& deltas, const string& t_name) { 
     cout << "\n\n--- Results Table for Technique: " << t_name << " (Median 15-DOF NEES) ---" << endl;
     cout << setw(20) << left << "Dataset"; for (double dv : deltas) { stringstream ss; ss << "dT=" << fixed << setprecision(1) << dv << "s"; cout << setw(12) << right << ss.str(); } cout << endl; cout << string(20 + deltas.size() * 12, '-') << endl;
     for (const string& d_name : dsets) { cout << setw(20) << left << d_name; auto it_d = tbl.find(d_name);
         if (it_d != tbl.end()) { for (double dv : deltas) { auto it_dl = it_d->second.find(dv); if (it_dl != it_d->second.end()) { if (it_dl->second.processed_intervals > 0 && isfinite(it_dl->second.median_nees)) { cout << setw(12) << right << fixed << setprecision(3) << it_dl->second.median_nees; } else { cout << setw(12) << right << "N/A"; }} else { cout << setw(12) << right << "-"; }}}
         else { for (size_t i=0; i<deltas.size(); ++i) cout << setw(12) << right << "--"; } cout << endl;
     } cout << string(20 + deltas.size() * 12, '-') << endl;
 }
 
 // --- Main Program ---
 int main(int argc, char* argv[]) {
     cout << "Starting EuRoC Preintegration Evaluation (15-DOF NEES)..." << endl;
     cout << "Base EuRoC Path: " << base_euroc_data_path << endl;
 
     Vector3 body_omega_Coriolis = Vector3::Zero(); // Set to non-zero if evaluating on rotating frame
 
     // --- List of Preintegration Techniques to Evaluate ---
     // **TO ADD A NEW TECHNIQUE**: 
     // 1. Define YourNewTechniqueClass inheriting from ImuPreintegrationTechnique.
     // 2. Implement its virtual methods (name, createPreintegrationParams, evaluateInterval).
     // 3. Add to this list: techniques.push_back(shared_ptr<ImuPreintegrationTechnique>(new YourNewTechniqueClass()));
     vector<shared_ptr<ImuPreintegrationTechnique>> techniques;
     techniques.push_back(shared_ptr<ImuPreintegrationTechnique>(new CombinedImuTechnique()));
     techniques.push_back(shared_ptr<ImuPreintegrationTechnique>(new ManifoldImuTechnique()));
 
     map<string, map<string, map<double, AggregatedRunResults>>> all_results_by_technique;
 
     for (const auto& tech_ptr : techniques) {
         cout << "\n\n<<<<< Evaluating Technique: " << tech_ptr->name() << " >>>>>" << endl;
         map<string, map<double, AggregatedRunResults>> current_technique_run_results;
 
         for (const string& dataset_name : dataset_sequences) {
             cout << "\n====================================================" << endl;
             cout << "Processing Dataset: " << dataset_name << " for technique: " << tech_ptr->name() << endl;
             cout << "====================================================" << endl;
 
             string current_dataset_path = base_euroc_data_path + "/" + dataset_name;
             string imu_csv = current_dataset_path + "/mav0/imu0/data.csv";
             string gt_csv = current_dataset_path + "/mav0/state_groundtruth_estimate0/data.csv";
             string sensor_yaml = current_dataset_path + "/mav0/imu0/sensor.yaml";
 
             SensorParams sensor_params;
             if (!parseSensorYaml(sensor_yaml, sensor_params)) { 
                 cerr << "Failed to load sensor params for " << dataset_name << ". Skipping." << endl;
                 continue; 
             }
             // Print sensor params only once per dataset
             if (tech_ptr == techniques.front()) { 
                 cout << "  Loaded Sensor Params for " << dataset_name << ":" << endl;
                 cout << "    Gyro Noise Density: " << scientific << sensor_params.gyro_noise_density << endl;
                 cout << "    Gyro Bias RW Density: " << sensor_params.gyro_bias_rw_density << endl;
                 cout << "    Accel Noise Density: " << sensor_params.accel_noise_density << endl;
                 cout << "    Accel Bias RW Density: " << sensor_params.accel_bias_rw_density << endl;
                 cout << "    IMU Rate (Hz): " << fixed << setprecision(1) << sensor_params.rate_hz << endl;
             }
 
             vector<ImuData> imu_data; vector<GroundTruthData> gt_data;
             if (!loadData(imu_csv, imu_data, parseImuLine) || imu_data.empty()) { 
                 cerr << "Failed to load or empty IMU data for " << dataset_name << ". Skipping." << endl;
                 continue; 
             }
             if (!loadData(gt_csv, gt_data, parseGroundTruthLine) || gt_data.empty()){
                 cerr << "Failed to load or empty GT data for " << dataset_name << ". Skipping." << endl;
                 continue;
             }
             
             if (gt_data.front().bias.vector().norm() < 1e-9 && gt_data.back().bias.vector().norm() < 1e-9) {
                 cout << "Warning: Ground truth biases appear to be all zero for " << dataset_name << "." << endl;
             }
 
             for (double deltaTij_target : deltaTij_values) {
                 cout << "\n  --- Processing with Target deltaTij: " << fixed << setprecision(1) << deltaTij_target << " s ---" << endl;
                 shared_ptr<PreintegrationParams> pim_params = 
                     tech_ptr->createPreintegrationParams(sensor_params, gravity_n, integration_noise_sigma, body_omega_Coriolis);
                 
                 vector<double> nees_vals; vector<double> actual_dt_vals;
                 int proc_int = 0; int skip_gt = 0; int skip_imu = 0; int num_iss = 0;
 
                 double min_t = max(imu_data.front().timestamp, gt_data.front().timestamp);
                 double max_t = min(imu_data.back().timestamp, gt_data.back().timestamp);
                 size_t first_gt_idx = findIndexAtOrAfter(gt_data, min_t);
 
                 if (first_gt_idx >= gt_data.size()) { 
                     cerr << "  Error: No ground truth data found at or after minimum start time for " << dataset_name << ". Skipping deltaTij " << deltaTij_target << endl;
                     continue; 
                 }
                 double current_t_i = gt_data[first_gt_idx].timestamp;
 
                 while (current_t_i + deltaTij_target <= max_t + 1e-7) { // Loop over time windows
                     double t_j_nom = current_t_i + deltaTij_target; // Nominal end time for this window
                     size_t gt_i_idx = findIndexAtOrAfter(gt_data, current_t_i - 1e-7); // Find GT at current_t_i
 
                     // Check validity of GT at start of interval
                     if (gt_i_idx >= gt_data.size() || abs(gt_data[gt_i_idx].timestamp - current_t_i) > 0.02) { // If no GT close to current_t_i or too far
                         size_t next_gt = findIndexAtOrAfter(gt_data, current_t_i + 1e-7); // try to find next available GT
                         if(next_gt < gt_data.size() && gt_data[next_gt].timestamp < t_j_nom) { // If next GT is within current nominal window
                             current_t_i = gt_data[next_gt].timestamp; // Advance to it
                         } else {
                             current_t_i = t_j_nom; // Else, advance to next window's start
                         }
                         skip_gt++; continue;
                     }
                     const GroundTruthData& gt_i_d = gt_data[gt_i_idx];
                     double act_t_i = gt_i_d.timestamp; // Actual start time of interval is this GT's timestamp
                     double act_t_j_target = act_t_i + deltaTij_target; // Target end time based on actual start
                     
                     // Find GT at or just BEFORE act_t_j_target
                     size_t gt_j_idx = findIndexBefore(gt_data, act_t_j_target + 1e-7); 
 
                     // Check validity of GT at end of interval
                     if (gt_j_idx <= gt_i_idx || gt_j_idx >= gt_data.size() || gt_data[gt_j_idx].timestamp < act_t_i + 0.05 * deltaTij_target) { // Ensure t_j is meaningfully after t_i
                         current_t_i = t_j_nom; skip_gt++; continue;
                     }
                     const GroundTruthData& gt_j_d = gt_data[gt_j_idx];
                     double act_t_j_gt = gt_j_d.timestamp; // Actual end time of interval, synced to this GT
 
                     // Find IMU data for [act_t_i, act_t_j_gt]
                     size_t imu_start_idx = findIndexAtOrAfter(imu_data, act_t_i - 1e-7); // Inclusive start
                     size_t imu_end_idx = findIndexAtOrAfter(imu_data, act_t_j_gt + 1e-7); // Exclusive end (points to first element >= act_t_j_gt + tol)
                     
                     // Check for valid IMU data range
                     bool no_valid_imu = imu_start_idx >= imu_data.size() || // Start index out of bounds
                                         (imu_end_idx <= imu_start_idx && // No elements in [start, end) range, AND
                                         !(imu_start_idx < imu_data.size() && // start_idx is valid
                                             imu_data[imu_start_idx].timestamp >= act_t_i -1e-7 && // and its timestamp is within the interval
                                             imu_data[imu_start_idx].timestamp <= act_t_j_gt + 1e-7) 
                                         ) ||
                                         (imu_end_idx > imu_start_idx && imu_data[imu_start_idx].timestamp > act_t_j_gt + 1e-7); // First IMU is already past interval end
 
                     if (no_valid_imu){ current_t_i = t_j_nom; skip_imu++; continue; }
                     
                     // Handle case where only one IMU measurement might cover the interval
                     if (imu_end_idx <= imu_start_idx) { 
                         if (imu_start_idx < imu_data.size() && 
                             imu_data[imu_start_idx].timestamp >= act_t_i - 1e-7 && 
                             imu_data[imu_start_idx].timestamp <= act_t_j_gt + 1e-7) {
                             imu_end_idx = imu_start_idx + 1; // Ensure at least one measurement is processed if it's the only one in range
                         } else { 
                             current_t_i = t_j_nom; skip_imu++; continue; 
                         }
                     }
                     
                     // Evaluate for this interval
                     IntervalEvaluationResult eval_res = tech_ptr->evaluateInterval(
                         gt_i_d, gt_j_d, imu_data, imu_start_idx, imu_end_idx, 
                         pim_params, gt_i_d.bias, sensor_params);
 
                     if (eval_res.success) {
                         nees_vals.push_back(eval_res.nees); actual_dt_vals.push_back(eval_res.actual_dt); proc_int++;
                     } else { num_iss++; }
                     
                     // Advance current_t_i for the next non-overlapping window, based on actual start + target duration
                     current_t_i = act_t_i + deltaTij_target; 
                 } 
                 aggregateAndStoreResults(dataset_name, deltaTij_target, nees_vals, actual_dt_vals, proc_int, skip_gt, skip_imu, num_iss, current_technique_run_results); 
             } 
         } 
         all_results_by_technique[tech_ptr->name()] = current_technique_run_results;
     } 
 
     // Print final summary tables for each technique
     for (const auto& tech_ptr : techniques) {
         printSingleTechniqueTable(all_results_by_technique[tech_ptr->name()], dataset_sequences, deltaTij_values, tech_ptr->name());
     }
     cout << "\nEvaluation finished." << endl; return 0;
 }