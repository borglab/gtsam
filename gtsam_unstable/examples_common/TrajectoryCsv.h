/**
 * @file  TrajectoryCsv.h
 * @brief Shared trajectory-CSV format for the tg_eqf / tfg_inekf / mekf
 *        examples: one column layout, one number format, one place to change.
 *
 * Layout: t, ground truth, estimate, tangent error, covariance. Attitude is
 * logged as a quaternion (w, x, y, z); every other block is a 3-vector. The
 * error and covariance columns live in the filter's own tangent space.
 */
#pragma once

#include <gtsam/geometry/Rot3.h>

#include <Eigen/Dense>
#include <iomanip>
#include <ostream>
#include <string>
#include <vector>

namespace imu_scenarios {

/// BlockDiagonal logs only each tangent block's 3x3 diagonal (today's
/// format); FullUpperTriangle also logs the off-block-diagonal terms.
enum class CovarianceFormat { BlockDiagonal, FullUpperTriangle };

/// Column header matching CsvRow below.
/// @param truth_blocks    3-vector blocks logged for ground truth, after the
///                        gt_q* quaternion, e.g. {"v", "p", "bg", "ba"}.
/// @param estimate_blocks same for the estimate; may carry filter-only blocks
///                        the truth has no counterpart for (TG-EqF's "bv").
/// @param tangent_blocks  blocks of the tangent space, driving both the err_*
///                        and P_* groups, e.g. {"R", "v", "p", "bg", "ba"}.
/// @param cov_format      BlockDiagonal (default) or FullUpperTriangle.
inline std::string trajectoryCsvHeader(
    const std::vector<std::string>& truth_blocks,
    const std::vector<std::string>& estimate_blocks,
    const std::vector<std::string>& tangent_blocks,
    CovarianceFormat cov_format = CovarianceFormat::BlockDiagonal) {
  std::string header = "t";

  auto appendQuaternion = [&header](const std::string& prefix) {
    for (const char* component : {"qw", "qx", "qy", "qz"}) {
      header += ',';
      header += prefix;
      header += '_';
      header += component;
    }
  };
  auto appendVectors = [&header](const std::string& prefix,
                                 const std::vector<std::string>& blocks) {
    for (const std::string& block : blocks) {
      for (const char axis : {'x', 'y', 'z'}) {
        header += ',';
        header += prefix;
        header += '_';
        header += block;
        header += axis;
      }
    }
  };

  appendQuaternion("gt");
  appendVectors("gt", truth_blocks);

  appendQuaternion("est");
  appendVectors("est", estimate_blocks);

  appendVectors("err", tangent_blocks);

  if (cov_format == CovarianceFormat::BlockDiagonal) {
    // Upper triangle of each 3x3 diagonal block of the covariance.
    for (const std::string& block : tangent_blocks) {
      for (const char* entry : {"00", "01", "02", "11", "12", "22"}) {
        header += ",P_";
        header += block;
        header += '_';
        header += entry;
      }
    }
  } else {
    // Global scalar tangent indices, not block-pair names: the err_* columns
    // above already declare the 3-wide block layout in order, so a reader
    // can recover any block slice from the index alone.
    const int dim = 3 * static_cast<int>(tangent_blocks.size());
    for (int i = 0; i < dim; ++i) {
      for (int j = i; j < dim; ++j) {
        header += ",P_";
        header += std::to_string(i);
        header += '_';
        header += std::to_string(j);
      }
    }
  }

  return header + "\n";
}

/// Streams one CSV row, in the column order trajectoryCsvHeader() describes.
/// Construct, append the blocks, then end(); the row is not terminated until
/// end() runs.
class CsvRow {
 public:
  CsvRow(std::ostream& out, double t) : out_(out) {
    out_ << std::setprecision(9) << t;
  }

  CsvRow& quat(const gtsam::Rot3& R) {
    const gtsam::Quaternion q = R.toQuaternion();
    out_ << ',' << q.w() << ',' << q.x() << ',' << q.y() << ',' << q.z();
    return *this;
  }

  CsvRow& v3(const Eigen::Vector3d& x) {
    out_ << ',' << x.x() << ',' << x.y() << ',' << x.z();
    return *this;
  }

  /// Every 3-vector block of a tangent vector, in order.
  template <int Dim>
  CsvRow& tangent(const Eigen::Matrix<double, Dim, 1>& eps) {
    static_assert(Dim % 3 == 0, "tangent dimension must be a multiple of 3");
    for (int o = 0; o < Dim; o += 3) {
      v3(eps.template segment<3>(o));
    }
    return *this;
  }

  /// Upper triangle of each 3x3 diagonal block of a covariance, in order.
  template <int Dim>
  CsvRow& covarianceBlocks(const Eigen::Matrix<double, Dim, Dim>& P) {
    static_assert(Dim % 3 == 0, "covariance dimension must be a multiple of 3");
    for (int o = 0; o < Dim; o += 3) {
      out_ << ',' << P(o, o) << ',' << P(o, o + 1) << ',' << P(o, o + 2) << ','
           << P(o + 1, o + 1) << ',' << P(o + 1, o + 2) << ','
           << P(o + 2, o + 2);
    }
    return *this;
  }

  /// Full upper triangle of a covariance, row-major (i, then j >= i), matching
  /// the FullUpperTriangle header.
  template <int Dim>
  CsvRow& covarianceFullUpper(const Eigen::Matrix<double, Dim, Dim>& P) {
    static_assert(Dim % 3 == 0, "covariance dimension must be a multiple of 3");
    for (int i = 0; i < Dim; ++i) {
      for (int j = i; j < Dim; ++j) {
        out_ << ',' << P(i, j);
      }
    }
    return *this;
  }

  void end() { out_ << '\n'; }

 private:
  std::ostream& out_;
};

}  // namespace imu_scenarios
