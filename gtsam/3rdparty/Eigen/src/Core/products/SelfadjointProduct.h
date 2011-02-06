// This file is part of Eigen, a lightweight C++ template library
// for linear algebra.
//
// Copyright (C) 2009 Gael Guennebaud <gael.guennebaud@inria.fr>
//
// Eigen is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation; either
// version 3 of the License, or (at your option) any later version.
//
// Alternatively, you can redistribute it and/or
// modify it under the terms of the GNU General Public License as
// published by the Free Software Foundation; either version 2 of
// the License, or (at your option) any later version.
//
// Eigen is distributed in the hope that it will be useful, but WITHOUT ANY
// WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
// FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License or the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public
// License and a copy of the GNU General Public License along with
// Eigen. If not, see <http://www.gnu.org/licenses/>.

#ifndef EIGEN_SELFADJOINT_PRODUCT_H
#define EIGEN_SELFADJOINT_PRODUCT_H

/**********************************************************************
* This file implements a self adjoint product: C += A A^T updating only
* half of the selfadjoint matrix C.
* It corresponds to the level 3 SYRK Blas routine.
**********************************************************************/

// forward declarations (defined at the end of this file)
template<typename Scalar, typename Index, int mr, int nr, bool ConjLhs, bool ConjRhs, int UpLo>
struct ei_sybb_kernel;

/* Optimized selfadjoint product (_SYRK) */
template <typename Scalar, typename Index,
          int RhsStorageOrder,
          int ResStorageOrder, bool AAT, int UpLo>
struct ei_selfadjoint_product;

// as usual if the result is row major => we transpose the product
template <typename Scalar, typename Index, int MatStorageOrder, bool AAT, int UpLo>
struct ei_selfadjoint_product<Scalar, Index, MatStorageOrder, RowMajor, AAT, UpLo>
{
  static EIGEN_STRONG_INLINE void run(Index size, Index depth, const Scalar* mat, Index matStride, Scalar* res, Index resStride, Scalar alpha)
  {
    ei_selfadjoint_product<Scalar, Index, MatStorageOrder, ColMajor, !AAT, UpLo==Lower?Upper:Lower>
      ::run(size, depth, mat, matStride, res, resStride, alpha);
  }
};

template <typename Scalar, typename Index,
          int MatStorageOrder, bool AAT, int UpLo>
struct ei_selfadjoint_product<Scalar, Index, MatStorageOrder, ColMajor, AAT, UpLo>
{

  static EIGEN_DONT_INLINE void run(
    Index size, Index depth,
    const Scalar* _mat, Index matStride,
    Scalar* res,        Index resStride,
    Scalar alpha)
  {
    ei_const_blas_data_mapper<Scalar, Index, MatStorageOrder> mat(_mat,matStride);

//     if(AAT)
//       alpha = ei_conj(alpha);

    typedef ei_gebp_traits<Scalar,Scalar> Traits;

    Index kc = depth; // cache block size along the K direction
    Index mc = size;  // cache block size along the M direction
    Index nc = size;  // cache block size along the N direction
    computeProductBlockingSizes<Scalar,Scalar>(kc, mc, nc);
    // !!! mc must be a multiple of nr:
    if(mc>Traits::nr)
      mc = (mc/Traits::nr)*Traits::nr;

    Scalar* blockA = ei_aligned_stack_new(Scalar, kc*mc);
    std::size_t sizeW = kc*Traits::WorkSpaceFactor;
    std::size_t sizeB = sizeW + kc*size;
    Scalar* allocatedBlockB = ei_aligned_stack_new(Scalar, sizeB);
    Scalar* blockB = allocatedBlockB + sizeW;

    // note that the actual rhs is the transpose/adjoint of mat
    enum {
      ConjLhs = NumTraits<Scalar>::IsComplex && !AAT,
      ConjRhs = NumTraits<Scalar>::IsComplex && AAT
    };

    ei_gebp_kernel<Scalar, Scalar, Index, Traits::mr, Traits::nr, ConjLhs, ConjRhs> gebp_kernel;
    ei_gemm_pack_rhs<Scalar, Index, Traits::nr,MatStorageOrder==RowMajor ? ColMajor : RowMajor> pack_rhs;
    ei_gemm_pack_lhs<Scalar, Index, Traits::mr, Traits::LhsProgress, MatStorageOrder, false> pack_lhs;
    ei_sybb_kernel<Scalar, Index, Traits::mr, Traits::nr, ConjLhs, ConjRhs, UpLo> sybb;

    for(Index k2=0; k2<depth; k2+=kc)
    {
      const Index actual_kc = std::min(k2+kc,depth)-k2;

      // note that the actual rhs is the transpose/adjoint of mat
      pack_rhs(blockB, &mat(0,k2), matStride, actual_kc, size);

      for(Index i2=0; i2<size; i2+=mc)
      {
        const Index actual_mc = std::min(i2+mc,size)-i2;

        pack_lhs(blockA, &mat(i2, k2), matStride, actual_kc, actual_mc);

        // the selected actual_mc * size panel of res is split into three different part:
        //  1 - before the diagonal => processed with gebp or skipped
        //  2 - the actual_mc x actual_mc symmetric block => processed with a special kernel
        //  3 - after the diagonal => processed with gebp or skipped
        if (UpLo==Lower)
          gebp_kernel(res+i2, resStride, blockA, blockB, actual_mc, actual_kc, std::min(size,i2), alpha,
                      -1, -1, 0, 0, allocatedBlockB);

        sybb(res+resStride*i2 + i2, resStride, blockA, blockB + actual_kc*i2, actual_mc, actual_kc, alpha, allocatedBlockB);

        if (UpLo==Upper)
        {
          Index j2 = i2+actual_mc;
          gebp_kernel(res+resStride*j2+i2, resStride, blockA, blockB+actual_kc*j2, actual_mc, actual_kc, std::max(Index(0), size-j2), alpha,
                      -1, -1, 0, 0, allocatedBlockB);
        }
      }
    }
    ei_aligned_stack_delete(Scalar, blockA, kc*mc);
    ei_aligned_stack_delete(Scalar, allocatedBlockB, sizeB);
  }
};

// high level API

template<typename MatrixType, unsigned int UpLo>
template<typename DerivedU>
SelfAdjointView<MatrixType,UpLo>& SelfAdjointView<MatrixType,UpLo>
::rankUpdate(const MatrixBase<DerivedU>& u, Scalar alpha)
{
  typedef ei_blas_traits<DerivedU> UBlasTraits;
  typedef typename UBlasTraits::DirectLinearAccessType ActualUType;
  typedef typename ei_cleantype<ActualUType>::type _ActualUType;
  const ActualUType actualU = UBlasTraits::extract(u.derived());

  Scalar actualAlpha = alpha * UBlasTraits::extractScalarFactor(u.derived());

  enum { IsRowMajor = (ei_traits<MatrixType>::Flags&RowMajorBit) ? 1 : 0 };

  ei_selfadjoint_product<Scalar, Index,
    _ActualUType::Flags&RowMajorBit ? RowMajor : ColMajor,
    MatrixType::Flags&RowMajorBit ? RowMajor : ColMajor,
    !UBlasTraits::NeedToConjugate, UpLo>
    ::run(_expression().cols(), actualU.cols(), &actualU.coeff(0,0), actualU.outerStride(),
          const_cast<Scalar*>(_expression().data()), _expression().outerStride(), actualAlpha);

  return *this;
}


// Optimized SYmmetric packed Block * packed Block product kernel.
// This kernel is built on top of the gebp kernel:
// - the current selfadjoint block (res) is processed per panel of actual_mc x BlockSize
//   where BlockSize is set to the minimal value allowing gebp to be as fast as possible
// - then, as usual, each panel is split into three parts along the diagonal,
//   the sub blocks above and below the diagonal are processed as usual,
//   while the selfadjoint block overlapping the diagonal is evaluated into a
//   small temporary buffer which is then accumulated into the result using a
//   triangular traversal.
template<typename Scalar, typename Index, int mr, int nr, bool ConjLhs, bool ConjRhs, int UpLo>
struct ei_sybb_kernel
{
  enum {
    PacketSize = ei_packet_traits<Scalar>::size,
    BlockSize  = EIGEN_PLAIN_ENUM_MAX(mr,nr)
  };
  void operator()(Scalar* res, Index resStride, const Scalar* blockA, const Scalar* blockB, Index size, Index depth, Scalar alpha, Scalar* workspace)
  {
    ei_gebp_kernel<Scalar, Scalar, Index, mr, nr, ConjLhs, ConjRhs> gebp_kernel;
    Matrix<Scalar,BlockSize,BlockSize,ColMajor> buffer;

    // let's process the block per panel of actual_mc x BlockSize,
    // again, each is split into three parts, etc.
    for (Index j=0; j<size; j+=BlockSize)
    {
      Index actualBlockSize = std::min<Index>(BlockSize,size - j);
      const Scalar* actual_b = blockB+j*depth;

      if(UpLo==Upper)
        gebp_kernel(res+j*resStride, resStride, blockA, actual_b, j, depth, actualBlockSize, alpha,
                    -1, -1, 0, 0, workspace);

      // selfadjoint micro block
      {
        Index i = j;
        buffer.setZero();
        // 1 - apply the kernel on the temporary buffer
        gebp_kernel(buffer.data(), BlockSize, blockA+depth*i, actual_b, actualBlockSize, depth, actualBlockSize, alpha,
                    -1, -1, 0, 0, workspace);
        // 2 - triangular accumulation
        for(Index j1=0; j1<actualBlockSize; ++j1)
        {
          Scalar* r = res + (j+j1)*resStride + i;
          for(Index i1=UpLo==Lower ? j1 : 0;
              UpLo==Lower ? i1<actualBlockSize : i1<=j1; ++i1)
            r[i1] += buffer(i1,j1);
        }
      }

      if(UpLo==Lower)
      {
        Index i = j+actualBlockSize;
        gebp_kernel(res+j*resStride+i, resStride, blockA+depth*i, actual_b, size-i, depth, actualBlockSize, alpha,
                    -1, -1, 0, 0, workspace);
      }
    }
  }
};

#endif // EIGEN_SELFADJOINT_PRODUCT_H
