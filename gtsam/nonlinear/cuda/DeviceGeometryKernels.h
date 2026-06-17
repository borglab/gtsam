#pragma once

#include <gtsam/config.h>
#include <gtsam/nonlinear/cuda/DeviceGeometryTypes.h>
#include <gtsam/slam/cuda/CudaSfmTypes.h>

#include <cuda_runtime_api.h>

#include <math.h>

namespace gtsam::cuda {

struct DeviceProjectionResult {
  double residual[2];
  double cameraJacobian[18];  // Row-major 2 x 9.
  double pointJacobian[6];    // Row-major 2 x 3.
};

namespace internal {

__host__ __device__ inline double cameraR(
    const DevicePinholeCameraCal3Bundler& camera, int row, int col) {
  return camera.R[3 * row + col];
}

__host__ __device__ inline void setIdentity3(double* matrix) {
  for (int i = 0; i < 9; ++i) {
    matrix[i] = 0.0;
  }
  matrix[0] = 1.0;
  matrix[4] = 1.0;
  matrix[8] = 1.0;
}

__host__ __device__ inline void cayleyRetractRotation(const double* omega,
                                                      double* rotation) {
  const double x = omega[0];
  const double y = omega[1];
  const double z = omega[2];
  const double x2 = x * x;
  const double y2 = y * y;
  const double z2 = z * z;
  const double xy = x * y;
  const double xz = x * z;
  const double yz = y * z;
  const double f = 1.0 / (4.0 + x2 + y2 + z2);
  const double twoF = 2.0 * f;

  rotation[0] = (4.0 + x2 - y2 - z2) * f;
  rotation[1] = (xy - 2.0 * z) * twoF;
  rotation[2] = (xz + 2.0 * y) * twoF;
  rotation[3] = (xy + 2.0 * z) * twoF;
  rotation[4] = (4.0 - x2 + y2 - z2) * f;
  rotation[5] = (yz - 2.0 * x) * twoF;
  rotation[6] = (xz - 2.0 * y) * twoF;
  rotation[7] = (yz + 2.0 * x) * twoF;
  rotation[8] = (4.0 - x2 - y2 + z2) * f;
}

__host__ __device__ inline void multiply3x3(const double* A, const double* B,
                                            double* result) {
  for (int row = 0; row < 3; ++row) {
    for (int col = 0; col < 3; ++col) {
      double value = 0.0;
      for (int k = 0; k < 3; ++k) {
        value += A[3 * row + k] * B[3 * k + col];
      }
      result[3 * row + col] = value;
    }
  }
}

__host__ __device__ inline void hat3(const double* omega, double* matrix) {
  matrix[0] = 0.0;
  matrix[1] = -omega[2];
  matrix[2] = omega[1];
  matrix[3] = omega[2];
  matrix[4] = 0.0;
  matrix[5] = -omega[0];
  matrix[6] = -omega[1];
  matrix[7] = omega[0];
  matrix[8] = 0.0;
}

__host__ __device__ inline void axpy3x3(double alpha, const double* A,
                                        double* result) {
  for (int i = 0; i < 9; ++i) {
    result[i] += alpha * A[i];
  }
}

__host__ __device__ inline void so3Expmap(const double* omega,
                                          double* rotation) {
  double W[9];
  double W2[9];
  hat3(omega, W);
  multiply3x3(W, W, W2);
  setIdentity3(rotation);

  const double theta2 =
      omega[0] * omega[0] + omega[1] * omega[1] + omega[2] * omega[2];
  if (theta2 < 1e-12) {
    axpy3x3(1.0, W, rotation);
    axpy3x3(0.5, W2, rotation);
    return;
  }

  const double theta = sqrt(theta2);
  axpy3x3(sin(theta) / theta, W, rotation);
  axpy3x3((1.0 - cos(theta)) / theta2, W2, rotation);
}

__host__ __device__ inline void se3Expmap(const double* xi, double* rotation,
                                          double* translation) {
  double W[9];
  double W2[9];
  double V[9];
  hat3(xi, W);
  multiply3x3(W, W, W2);
  so3Expmap(xi, rotation);
  setIdentity3(V);

  const double theta2 = xi[0] * xi[0] + xi[1] * xi[1] + xi[2] * xi[2];
  if (theta2 < 1e-12) {
    axpy3x3(0.5, W, V);
    axpy3x3(1.0 / 6.0, W2, V);
  } else {
    const double theta = sqrt(theta2);
    axpy3x3((1.0 - cos(theta)) / theta2, W, V);
    axpy3x3((theta - sin(theta)) / (theta2 * theta), W2, V);
  }

  for (int row = 0; row < 3; ++row) {
    translation[row] = V[3 * row] * xi[3] + V[3 * row + 1] * xi[4] +
                       V[3 * row + 2] * xi[5];
  }
}

}  // namespace internal

__host__ __device__ inline DevicePoint3 RetractPoint(const DevicePoint3& point,
                                                     const double* delta3) {
  return {point.x + delta3[0], point.y + delta3[1], point.z + delta3[2]};
}

__host__ __device__ inline DevicePinholeCameraCal3Bundler RetractCamera(
    const DevicePinholeCameraCal3Bundler& camera, const double* delta9) {
  DevicePinholeCameraCal3Bundler result = camera;

  double deltaR[9];
#ifdef GTSAM_POSE3_EXPMAP
  double deltaT[3];
  internal::se3Expmap(delta9, deltaR, deltaT);
#else
  internal::cayleyRetractRotation(delta9, deltaR);
  const double deltaT[3] = {delta9[3], delta9[4], delta9[5]};
#endif
  double composedR[9];
  internal::multiply3x3(camera.R, deltaR, composedR);
  for (int i = 0; i < 9; ++i) {
    result.R[i] = composedR[i];
  }

  for (int row = 0; row < 3; ++row) {
    result.t[row] =
        camera.t[row] + internal::cameraR(camera, row, 0) * deltaT[0] +
        internal::cameraR(camera, row, 1) * deltaT[1] +
        internal::cameraR(camera, row, 2) * deltaT[2];
  }
  result.f = camera.f + delta9[6];
  result.k1 = camera.k1 + delta9[7];
  result.k2 = camera.k2 + delta9[8];
  return result;
}

__host__ __device__ inline DeviceProjectionResult
EvaluatePinholeBundlerProjection(
    const DevicePinholeCameraCal3Bundler& camera, const DevicePoint3& point,
    const CudaSfmObservation& observation) {
  const double dx = point.x - camera.t[0];
  const double dy = point.y - camera.t[1];
  const double dz = point.z - camera.t[2];

  const double qx = internal::cameraR(camera, 0, 0) * dx +
                    internal::cameraR(camera, 1, 0) * dy +
                    internal::cameraR(camera, 2, 0) * dz;
  const double qy = internal::cameraR(camera, 0, 1) * dx +
                    internal::cameraR(camera, 1, 1) * dy +
                    internal::cameraR(camera, 2, 1) * dz;
  const double qz = internal::cameraR(camera, 0, 2) * dx +
                    internal::cameraR(camera, 1, 2) * dy +
                    internal::cameraR(camera, 2, 2) * dz;

  const double d = 1.0 / qz;
  const double x = qx * d;
  const double y = qy * d;
  const double r = x * x + y * y;
  const double rr = r * r;
  const double g = 1.0 + (camera.k1 + camera.k2 * r) * r;
  const double u = camera.f * g * x;
  const double v = camera.f * g * y;

  DeviceProjectionResult result{};
  result.residual[0] = u - observation.measuredU;
  result.residual[1] = v - observation.measuredV;

  double Dpose[12];
  Dpose[0] = x * y;
  Dpose[1] = -1.0 - x * x;
  Dpose[2] = y;
  Dpose[3] = -d;
  Dpose[4] = 0.0;
  Dpose[5] = d * x;
  Dpose[6] = 1.0 + y * y;
  Dpose[7] = -x * y;
  Dpose[8] = -x;
  Dpose[9] = 0.0;
  Dpose[10] = -d;
  Dpose[11] = d * y;

  double Dpoint[6];
  Dpoint[0] = d * (internal::cameraR(camera, 0, 0) -
                   x * internal::cameraR(camera, 0, 2));
  Dpoint[1] = d * (internal::cameraR(camera, 1, 0) -
                   x * internal::cameraR(camera, 1, 2));
  Dpoint[2] = d * (internal::cameraR(camera, 2, 0) -
                   x * internal::cameraR(camera, 2, 2));
  Dpoint[3] = d * (internal::cameraR(camera, 0, 1) -
                   y * internal::cameraR(camera, 0, 2));
  Dpoint[4] = d * (internal::cameraR(camera, 1, 1) -
                   y * internal::cameraR(camera, 1, 2));
  Dpoint[5] = d * (internal::cameraR(camera, 2, 1) -
                   y * internal::cameraR(camera, 2, 2));

  const double a = 2.0 * (camera.k1 + 2.0 * camera.k2 * r);
  const double Dpi[4] = {camera.f * (g + a * x * x),
                         camera.f * (a * x * y),
                         camera.f * (a * x * y),
                         camera.f * (g + a * y * y)};

  for (int col = 0; col < 6; ++col) {
    result.cameraJacobian[col] =
        Dpi[0] * Dpose[col] + Dpi[1] * Dpose[6 + col];
    result.cameraJacobian[9 + col] =
        Dpi[2] * Dpose[col] + Dpi[3] * Dpose[6 + col];
  }
  result.cameraJacobian[6] = g * x;
  result.cameraJacobian[7] = camera.f * r * x;
  result.cameraJacobian[8] = camera.f * rr * x;
  result.cameraJacobian[15] = g * y;
  result.cameraJacobian[16] = camera.f * r * y;
  result.cameraJacobian[17] = camera.f * rr * y;

  for (int col = 0; col < 3; ++col) {
    result.pointJacobian[col] =
        Dpi[0] * Dpoint[col] + Dpi[1] * Dpoint[3 + col];
    result.pointJacobian[3 + col] =
        Dpi[2] * Dpoint[col] + Dpi[3] * Dpoint[3 + col];
  }

  return result;
}

}  // namespace gtsam::cuda
