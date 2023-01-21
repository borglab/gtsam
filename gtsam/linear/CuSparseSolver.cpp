/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file CuSparseSolver.cpp
 *
 * @brief cuSparse based linear solver backend for GTSAM
 *
 * @date Jun 2020
 * @author Fan Jiang
 */

#include "gtsam/linear/CuSparseSolver.h"
#include "gtsam/linear/LinearSolverParams.h"

#ifdef GTSAM_USE_CUSPARSE
#include <cusolverSp.h>
#include <cuda_runtime.h>
#include <cusolverSp.h>
#include <cusparse_v2.h>

#endif

#include <gtsam/linear/GaussianFactorGraph.h>

namespace gtsam {
  CuSparseSolver::CuSparseSolver(const Ordering &ordering)
      : ordering_(ordering) {}

#ifdef GTSAM_USE_CUSPARSE

#define S1(x) #x
#define S2(x) S1(x)
#define ____LOCATION __FILE__ " : " S2(__LINE__)

  void checkCUDAError(cudaError code, const char* location) {
    if(code != cudaSuccess) {
      throw std::runtime_error(std::string("cudaMalloc error ") + std::to_string(code) + std::string(" at ") + std::string(location));
    }
  }

#define CHECK_CUDA_ERROR(code) checkCUDAError(code, ____LOCATION)

  void checkCuSolverError(cusolverStatus_t code, const char* location) {
    if(code != CUSOLVER_STATUS_SUCCESS) {
      throw std::runtime_error(std::string("cuSolver error ") + std::to_string(code) + std::string(" at ") + std::string(location));
    }
  }

#define CHECK_CUSOLVER_ERROR(code) checkCuSolverError(code, ____LOCATION)

  void checkCuSparseError(cusparseStatus_t code, const char* location) {
    if(code != CUSPARSE_STATUS_SUCCESS) {
      throw std::runtime_error(std::string("cuSparse error ") + std::to_string(code) + std::string(" at ") + std::string(location));
    }
  }

#define CHECK_CUSPARSE_ERROR(code) checkCuSparseError(code, ____LOCATION)

  void EigenSparseToCuSparseTranspose(const SparseMatrixEigen &mat, int **row,
                                      int **col, double **val) {
    const int num_non0  = mat.nonZeros();
    const int num_outer = mat.cols() + 1;

    cudaError err;
    err = cudaMalloc(reinterpret_cast<void **>(row), sizeof(int) * num_outer);
    if(err != cudaSuccess) {
      throw std::runtime_error(std::string("cudaMalloc error: out of memory? trying to allocate ") + std::to_string(err));
    }
    if(cudaMalloc(reinterpret_cast<void **>(col), sizeof(int) * num_non0) != cudaSuccess) {
      cudaFree(row);
      throw std::runtime_error("cudaMalloc error: out of memory?");
    }
    if(cudaMalloc(reinterpret_cast<void **>(val), sizeof(double) * num_non0) != cudaSuccess) {
      cudaFree(row);
      cudaFree(col);
      throw std::runtime_error("cudaMalloc error: out of memory?");
    }

    CHECK_CUDA_ERROR(cudaMemcpy(
        *val, mat.valuePtr(),
        num_non0 * sizeof(double),
        cudaMemcpyHostToDevice));
    CHECK_CUDA_ERROR(cudaMemcpy(*row, mat.outerIndexPtr(),
        num_outer * sizeof(int),
        cudaMemcpyHostToDevice));
    CHECK_CUDA_ERROR(cudaMemcpy(
        *col, mat.innerIndexPtr(),
        num_non0 * sizeof(int),
        cudaMemcpyHostToDevice));
  }

  VectorValues CuSparseSolver::solve(
      const gtsam::GaussianFactorGraph &gfg) const {
    gttic_(CuSparseSolver_optimizeEigenCholesky);

    // ordering is used here
    size_t rows, cols;
    SparseMatrixEigen Ab;
    std::tie(rows, cols, Ab) = gfg.sparseJacobian<SparseMatrixEigen>(ordering_);
    SparseMatrixEigen A = Ab.block(0, 0, rows, cols - 1);
//
//      // CSC in Eigen, CSR in CUDA, so A becomes At
//      int *At_row(NULL), *At_col(NULL);
//      double *At_val(NULL);
//
//      int At_num_rows = A.cols();
//      int At_num_cols = A.rows();
//      int At_num_nnz = A.nonZeros();
//
//      std::cout << "Base of At: " << A.outerIndexPtr()[0] << std::endl;
//      EigenSparseToCuSparseTranspose(A, &At_row, &At_col, &At_val);
//
//      cusparseHandle_t     cspHandle = NULL;
//      cusparseSpMatDescr_t matA, matB, matC;
//      void*  dBuffer1    = NULL, *dBuffer2   = NULL;
//      size_t bufferSize1 = 0,    bufferSize2 = 0;
//
//      CHECK_CUSPARSE_ERROR( cusparseCreate(&cspHandle) );
//
//      CHECK_CUSPARSE_ERROR(cusparseCreateCsr(&matA, At_num_rows, At_num_cols, At_num_nnz,
//                                        At_row, At_col, At_val,
//                                        CUSPARSE_INDEX_32I, CUSPARSE_INDEX_32I,
//                                        CUSPARSE_INDEX_BASE_ZERO, CUDA_R_64F) );
//      CHECK_CUSPARSE_ERROR(cusparseCreateCsr(&matB, At_num_rows, At_num_cols, At_num_nnz,
//                                        At_row, At_col, At_val,
//                                        CUSPARSE_INDEX_32I, CUSPARSE_INDEX_32I,
//                                        CUSPARSE_INDEX_BASE_ZERO, CUDA_R_64F) );
//      CHECK_CUSPARSE_ERROR(cusparseCreateCsr(&matC, At_num_rows, At_num_rows, 0,
//                                        NULL, NULL, NULL,
//                                        CUSPARSE_INDEX_32I, CUSPARSE_INDEX_32I,
//                                        CUSPARSE_INDEX_BASE_ZERO, CUDA_R_64F) );
//
//      int AtA_num_rows = A.cols();
//      int AtA_num_cols = A.cols();
//      int AtA_num_nnz = A.nonZeros();
//      int *AtA_row(NULL), *AtA_col(NULL);
//      double *AtA_val(NULL);
//      CHECK_CUDA_ERROR(cudaMalloc(reinterpret_cast<void **>(AtA_row), sizeof(int) * (AtA_num_cols + 1)));
//
//      int baseC, nnzC = 0;
//      // nnzTotalDevHostPtr points to host memory
//      int *nnzTotalDevHostPtr = &nnzC;
//
//      CHECK_CUSPARSE_ERROR(cusparseSetPointerMode(cspHandle, CUSPARSE_POINTER_MODE_HOST));

    //--------------------------------------------------------------------------
    // SpGEMM Computation

    auto At = A.transpose();
    Matrix b = At * Ab.col(cols - 1);

    SparseMatrixEigen AtA(A.cols(), A.cols());
    AtA.selfadjointView<Eigen::Upper>().rankUpdate(At);
    AtA.makeCompressed();

    gttic_(CuSparseSolver_optimizeEigenCholesky_solve);

    // Create the cuSolver object
    cusolverSpHandle_t solverHandle;
    CHECK_CUSOLVER_ERROR(cusolverSpCreate(&solverHandle));

    // Create the matrix descriptor
    cusparseMatDescr_t descrA;
    CHECK_CUSPARSE_ERROR(cusparseCreateMatDescr(&descrA));
    CHECK_CUSPARSE_ERROR(cusparseSetMatType(descrA, CUSPARSE_MATRIX_TYPE_GENERAL));

    int *AtA_row(NULL), *AtA_col(NULL);
    double *AtA_val(NULL);

    EigenSparseToCuSparseTranspose(AtA, &AtA_row, &AtA_col, &AtA_val);

//      std::cout << "Base of AtA: " << AtA.outerIndexPtr()[0] << std::endl;

    double *x_gpu(NULL), *b_gpu(NULL);

    CHECK_CUDA_ERROR(cudaMalloc(&x_gpu, sizeof(double) * AtA.cols()));
    CHECK_CUDA_ERROR(cudaMalloc(&b_gpu, sizeof(double) * AtA.cols()));

    CHECK_CUDA_ERROR(cudaMemcpy(b_gpu, b.data(),
                sizeof(double) * AtA.cols(),
                cudaMemcpyHostToDevice));

    int singularity = 0;
    const double tol = 0.00001;

    // no internal reordering, so only lower part (upper part of CSC) is used
    CHECK_CUSOLVER_ERROR(cusolverSpDcsrlsvchol(
        solverHandle, AtA.rows(), AtA.nonZeros(), descrA,
        AtA_val, AtA_row, AtA_col, b_gpu, tol, 0, x_gpu, &singularity));

    Vector x;
    x.resize(A.cols());
    CHECK_CUDA_ERROR(cudaMemcpy(x.data(), x_gpu, sizeof(double) * A.cols(),
                                cudaMemcpyDeviceToHost));

    cudaFree(AtA_val);
    cudaFree(AtA_row);
    cudaFree(AtA_col);
    cudaFree(b_gpu);
    cudaFree(x_gpu);
    cusolverSpDestroy(solverHandle);

    if (singularity != -1)
      throw std::runtime_error(std::string("ILS in CUDA Solver, singularity: ") + std::to_string(singularity));

    gttoc_(CuSparseSolver_optimizeEigenCholesky_solve);

    // NOTE: b is reordered now, so we need to transform back the order.
    // First find dimensions of each variable
    std::map<Key, size_t> dims;
    for (const boost::shared_ptr<GaussianFactor> &factor : gfg) {
      if (!static_cast<bool>(factor))
        continue;

      for (auto it = factor->begin(); it != factor->end(); ++it) {
        dims[*it] = factor->getDim(it);
      }
    }

    VectorValues vv;

    std::map<Key, size_t> columnIndices;

    {
      size_t currentColIndex = 0;
      for (const auto key : ordering_) {
        columnIndices[key] = currentColIndex;
        currentColIndex += dims[key];
      }
    }

    for (const std::pair<const Key, unsigned long> keyDim : dims) {
      vv.insert(keyDim.first, x.segment(columnIndices[keyDim.first], keyDim.second));
    }

    return vv;
  }
#else
  VectorValues CuSparseSolver::solve(
      const gtsam::GaussianFactorGraph &gfg) const {
    throw std::invalid_argument("This GTSAM is compiled without CUDA support");
  }
#endif
}
