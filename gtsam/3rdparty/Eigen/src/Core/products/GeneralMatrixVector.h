// This file is part of Eigen, a lightweight C++ template library
// for linear algebra.
//
// Copyright (C) 2008-2009 Gael Guennebaud <gael.guennebaud@inria.fr>
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

#ifndef EIGEN_GENERAL_MATRIX_VECTOR_H
#define EIGEN_GENERAL_MATRIX_VECTOR_H

/* Optimized col-major matrix * vector product:
 * This algorithm processes 4 columns at onces that allows to both reduce
 * the number of load/stores of the result by a factor 4 and to reduce
 * the instruction dependency. Moreover, we know that all bands have the
 * same alignment pattern.
 *
 * Mixing type logic: C += alpha * A * B
 *  |  A  |  B  |alpha| comments
 *  |real |cplx |cplx | no vectorization
 *  |real |cplx |real | alpha is converted to a cplx when calling the run function, no vectorization
 *  |cplx |real |cplx | invalid, the caller has to do tmp: = A * B; C += alpha*tmp
 *  |cplx |real |real | optimal case, vectorization possible via real-cplx mul
 */
template<typename Index, typename LhsScalar, bool ConjugateLhs, typename RhsScalar, bool ConjugateRhs>
struct ei_general_matrix_vector_product<Index,LhsScalar,ColMajor,ConjugateLhs,RhsScalar,ConjugateRhs>
{
typedef typename ei_scalar_product_traits<LhsScalar, RhsScalar>::ReturnType ResScalar;

enum {
  Vectorizable = ei_packet_traits<LhsScalar>::Vectorizable && ei_packet_traits<RhsScalar>::Vectorizable
              && int(ei_packet_traits<LhsScalar>::size)==int(ei_packet_traits<RhsScalar>::size),
  LhsPacketSize = Vectorizable ? ei_packet_traits<LhsScalar>::size : 1,
  RhsPacketSize = Vectorizable ? ei_packet_traits<RhsScalar>::size : 1,
  ResPacketSize = Vectorizable ? ei_packet_traits<ResScalar>::size : 1
};

typedef typename ei_packet_traits<LhsScalar>::type  _LhsPacket;
typedef typename ei_packet_traits<RhsScalar>::type  _RhsPacket;
typedef typename ei_packet_traits<ResScalar>::type  _ResPacket;

typedef typename ei_meta_if<Vectorizable,_LhsPacket,LhsScalar>::ret LhsPacket;
typedef typename ei_meta_if<Vectorizable,_RhsPacket,RhsScalar>::ret RhsPacket;
typedef typename ei_meta_if<Vectorizable,_ResPacket,ResScalar>::ret ResPacket;

EIGEN_DONT_INLINE static void run(
  Index rows, Index cols,
  const LhsScalar* lhs, Index lhsStride,
  const RhsScalar* rhs, Index rhsIncr,
  ResScalar* res, Index
  #ifdef EIGEN_INTERNAL_DEBUGGING
    resIncr
  #endif
  , RhsScalar alpha)
{
  ei_internal_assert(resIncr==1);
  #ifdef _EIGEN_ACCUMULATE_PACKETS
  #error _EIGEN_ACCUMULATE_PACKETS has already been defined
  #endif
  #define _EIGEN_ACCUMULATE_PACKETS(A0,A13,A2) \
    ei_pstore(&res[j], \
      ei_padd(ei_pload<ResPacket>(&res[j]), \
        ei_padd( \
          ei_padd(pcj.pmul(EIGEN_CAT(ei_ploa , A0)<LhsPacket>(&lhs0[j]),    ptmp0), \
                  pcj.pmul(EIGEN_CAT(ei_ploa , A13)<LhsPacket>(&lhs1[j]),   ptmp1)), \
          ei_padd(pcj.pmul(EIGEN_CAT(ei_ploa , A2)<LhsPacket>(&lhs2[j]),    ptmp2), \
                  pcj.pmul(EIGEN_CAT(ei_ploa , A13)<LhsPacket>(&lhs3[j]),   ptmp3)) )))

  ei_conj_helper<LhsScalar,RhsScalar,ConjugateLhs,ConjugateRhs> cj;
  ei_conj_helper<LhsPacket,RhsPacket,ConjugateLhs,ConjugateRhs> pcj;
  if(ConjugateRhs)
    alpha = ei_conj(alpha);

  enum { AllAligned = 0, EvenAligned, FirstAligned, NoneAligned };
  const Index columnsAtOnce = 4;
  const Index peels = 2;
  const Index LhsPacketAlignedMask = LhsPacketSize-1;
  const Index ResPacketAlignedMask = ResPacketSize-1;
  const Index PeelAlignedMask = ResPacketSize*peels-1;
  const Index size = rows;
  
  // How many coeffs of the result do we have to skip to be aligned.
  // Here we assume data are at least aligned on the base scalar type.
  Index alignedStart = ei_first_aligned(res,size);
  Index alignedSize = ResPacketSize>1 ? alignedStart + ((size-alignedStart) & ~ResPacketAlignedMask) : 0;
  const Index peeledSize  = peels>1 ? alignedStart + ((alignedSize-alignedStart) & ~PeelAlignedMask) : alignedStart;

  const Index alignmentStep = LhsPacketSize>1 ? (LhsPacketSize - lhsStride % LhsPacketSize) & LhsPacketAlignedMask : 0;
  Index alignmentPattern = alignmentStep==0 ? AllAligned
                       : alignmentStep==(LhsPacketSize/2) ? EvenAligned
                       : FirstAligned;

  // we cannot assume the first element is aligned because of sub-matrices
  const Index lhsAlignmentOffset = ei_first_aligned(lhs,size);

  // find how many columns do we have to skip to be aligned with the result (if possible)
  Index skipColumns = 0;
  // if the data cannot be aligned (TODO add some compile time tests when possible, e.g. for floats)
  if( (size_t(lhs)%sizeof(LhsScalar)) || (size_t(res)%sizeof(ResScalar)) )
  {
    alignedSize = 0;
    alignedStart = 0;
  }
  else if (LhsPacketSize>1)
  {
    ei_internal_assert(size_t(lhs+lhsAlignmentOffset)%sizeof(LhsPacket)==0 || size<LhsPacketSize);

    while (skipColumns<LhsPacketSize &&
          alignedStart != ((lhsAlignmentOffset + alignmentStep*skipColumns)%LhsPacketSize))
      ++skipColumns;
    if (skipColumns==LhsPacketSize)
    {
      // nothing can be aligned, no need to skip any column
      alignmentPattern = NoneAligned;
      skipColumns = 0;
    }
    else
    {
      skipColumns = std::min(skipColumns,cols);
      // note that the skiped columns are processed later.
    }

    ei_internal_assert(  (alignmentPattern==NoneAligned)
                      || (skipColumns + columnsAtOnce >= cols)
                      || LhsPacketSize > size
                      || (size_t(lhs+alignedStart+lhsStride*skipColumns)%sizeof(LhsPacket))==0);
  }
  else if(Vectorizable)
  {
    alignedStart = 0;
    alignedSize = size;
    alignmentPattern = AllAligned;
  }

  Index offset1 = (FirstAligned && alignmentStep==1?3:1);
  Index offset3 = (FirstAligned && alignmentStep==1?1:3);

  Index columnBound = ((cols-skipColumns)/columnsAtOnce)*columnsAtOnce + skipColumns;
  for (Index i=skipColumns; i<columnBound; i+=columnsAtOnce)
  {
    RhsPacket ptmp0 = ei_pset1<RhsPacket>(alpha*rhs[i*rhsIncr]),
              ptmp1 = ei_pset1<RhsPacket>(alpha*rhs[(i+offset1)*rhsIncr]),
              ptmp2 = ei_pset1<RhsPacket>(alpha*rhs[(i+2)*rhsIncr]),
              ptmp3 = ei_pset1<RhsPacket>(alpha*rhs[(i+offset3)*rhsIncr]);

    // this helps a lot generating better binary code
    const LhsScalar *lhs0 = lhs + i*lhsStride,     *lhs1 = lhs + (i+offset1)*lhsStride,
                    *lhs2 = lhs + (i+2)*lhsStride, *lhs3 = lhs + (i+offset3)*lhsStride;

    if (Vectorizable)
    {
      /* explicit vectorization */
      // process initial unaligned coeffs
      for (Index j=0; j<alignedStart; ++j)
      {
        res[j] = cj.pmadd(lhs0[j], ei_pfirst(ptmp0), res[j]);
        res[j] = cj.pmadd(lhs1[j], ei_pfirst(ptmp1), res[j]);
        res[j] = cj.pmadd(lhs2[j], ei_pfirst(ptmp2), res[j]);
        res[j] = cj.pmadd(lhs3[j], ei_pfirst(ptmp3), res[j]);
      }

      if (alignedSize>alignedStart)
      {
        switch(alignmentPattern)
        {
          case AllAligned:
            for (Index j = alignedStart; j<alignedSize; j+=ResPacketSize)
              _EIGEN_ACCUMULATE_PACKETS(d,d,d);
            break;
          case EvenAligned:
            for (Index j = alignedStart; j<alignedSize; j+=ResPacketSize)
              _EIGEN_ACCUMULATE_PACKETS(d,du,d);
            break;
          case FirstAligned:
            if(peels>1)
            {
              LhsPacket A00, A01, A02, A03, A10, A11, A12, A13;
              ResPacket T0, T1;

              A01 = ei_pload<LhsPacket>(&lhs1[alignedStart-1]);
              A02 = ei_pload<LhsPacket>(&lhs2[alignedStart-2]);
              A03 = ei_pload<LhsPacket>(&lhs3[alignedStart-3]);

              for (Index j = alignedStart; j<peeledSize; j+=peels*ResPacketSize)
              {
                A11 = ei_pload<LhsPacket>(&lhs1[j-1+LhsPacketSize]);  ei_palign<1>(A01,A11);
                A12 = ei_pload<LhsPacket>(&lhs2[j-2+LhsPacketSize]);  ei_palign<2>(A02,A12);
                A13 = ei_pload<LhsPacket>(&lhs3[j-3+LhsPacketSize]);  ei_palign<3>(A03,A13);

                A00 = ei_pload<LhsPacket>(&lhs0[j]);
                A10 = ei_pload<LhsPacket>(&lhs0[j+LhsPacketSize]);
                T0  = pcj.pmadd(A00, ptmp0, ei_pload<ResPacket>(&res[j]));
                T1  = pcj.pmadd(A10, ptmp0, ei_pload<ResPacket>(&res[j+ResPacketSize]));

                T0  = pcj.pmadd(A01, ptmp1, T0);
                A01 = ei_pload<LhsPacket>(&lhs1[j-1+2*LhsPacketSize]);  ei_palign<1>(A11,A01);
                T0  = pcj.pmadd(A02, ptmp2, T0);
                A02 = ei_pload<LhsPacket>(&lhs2[j-2+2*LhsPacketSize]);  ei_palign<2>(A12,A02);
                T0  = pcj.pmadd(A03, ptmp3, T0);
                ei_pstore(&res[j],T0);
                A03 = ei_pload<LhsPacket>(&lhs3[j-3+2*LhsPacketSize]);  ei_palign<3>(A13,A03);
                T1  = pcj.pmadd(A11, ptmp1, T1);
                T1  = pcj.pmadd(A12, ptmp2, T1);
                T1  = pcj.pmadd(A13, ptmp3, T1);
                ei_pstore(&res[j+ResPacketSize],T1);
              }
            }
            for (Index j = peeledSize; j<alignedSize; j+=ResPacketSize)
              _EIGEN_ACCUMULATE_PACKETS(d,du,du);
            break;
          default:
            for (Index j = alignedStart; j<alignedSize; j+=ResPacketSize)
              _EIGEN_ACCUMULATE_PACKETS(du,du,du);
            break;
        }
      }
    } // end explicit vectorization

    /* process remaining coeffs (or all if there is no explicit vectorization) */
    for (Index j=alignedSize; j<size; ++j)
    {
      res[j] = cj.pmadd(lhs0[j], ei_pfirst(ptmp0), res[j]);
      res[j] = cj.pmadd(lhs1[j], ei_pfirst(ptmp1), res[j]);
      res[j] = cj.pmadd(lhs2[j], ei_pfirst(ptmp2), res[j]);
      res[j] = cj.pmadd(lhs3[j], ei_pfirst(ptmp3), res[j]);
    }
  }

  // process remaining first and last columns (at most columnsAtOnce-1)
  Index end = cols;
  Index start = columnBound;
  do
  {
    for (Index k=start; k<end; ++k)
    {
      RhsPacket ptmp0 = ei_pset1<RhsPacket>(alpha*rhs[k*rhsIncr]);
      const LhsScalar* lhs0 = lhs + k*lhsStride;

      if (Vectorizable)
      {
        /* explicit vectorization */
        // process first unaligned result's coeffs
        for (Index j=0; j<alignedStart; ++j)
          res[j] += cj.pmul(lhs0[j], ei_pfirst(ptmp0));
        // process aligned result's coeffs
        if ((size_t(lhs0+alignedStart)%sizeof(LhsPacket))==0)
          for (Index i = alignedStart;i<alignedSize;i+=ResPacketSize)
            ei_pstore(&res[i], pcj.pmadd(ei_ploadu<LhsPacket>(&lhs0[i]), ptmp0, ei_pload<ResPacket>(&res[i])));
        else
          for (Index i = alignedStart;i<alignedSize;i+=ResPacketSize)
            ei_pstore(&res[i], pcj.pmadd(ei_ploadu<LhsPacket>(&lhs0[i]), ptmp0, ei_pload<ResPacket>(&res[i])));
      }

      // process remaining scalars (or all if no explicit vectorization)
      for (Index i=alignedSize; i<size; ++i)
        res[i] += cj.pmul(lhs0[i], ei_pfirst(ptmp0));
    }
    if (skipColumns)
    {
      start = 0;
      end = skipColumns;
      skipColumns = 0;
    }
    else
      break;
  } while(Vectorizable);
  #undef _EIGEN_ACCUMULATE_PACKETS
}
};

/* Optimized row-major matrix * vector product:
 * This algorithm processes 4 rows at onces that allows to both reduce
 * the number of load/stores of the result by a factor 4 and to reduce
 * the instruction dependency. Moreover, we know that all bands have the
 * same alignment pattern.
 *
 * Mixing type logic:
 *  - alpha is always a complex (or converted to a complex)
 *  - no vectorization
 */
template<typename Index, typename LhsScalar, bool ConjugateLhs, typename RhsScalar, bool ConjugateRhs>
struct ei_general_matrix_vector_product<Index,LhsScalar,RowMajor,ConjugateLhs,RhsScalar,ConjugateRhs>
{
typedef typename ei_scalar_product_traits<LhsScalar, RhsScalar>::ReturnType ResScalar;

enum {
  Vectorizable = ei_packet_traits<LhsScalar>::Vectorizable && ei_packet_traits<RhsScalar>::Vectorizable
              && int(ei_packet_traits<LhsScalar>::size)==int(ei_packet_traits<RhsScalar>::size),
  LhsPacketSize = Vectorizable ? ei_packet_traits<LhsScalar>::size : 1,
  RhsPacketSize = Vectorizable ? ei_packet_traits<RhsScalar>::size : 1,
  ResPacketSize = Vectorizable ? ei_packet_traits<ResScalar>::size : 1
};

typedef typename ei_packet_traits<LhsScalar>::type  _LhsPacket;
typedef typename ei_packet_traits<RhsScalar>::type  _RhsPacket;
typedef typename ei_packet_traits<ResScalar>::type  _ResPacket;

typedef typename ei_meta_if<Vectorizable,_LhsPacket,LhsScalar>::ret LhsPacket;
typedef typename ei_meta_if<Vectorizable,_RhsPacket,RhsScalar>::ret RhsPacket;
typedef typename ei_meta_if<Vectorizable,_ResPacket,ResScalar>::ret ResPacket;
  
EIGEN_DONT_INLINE static void run(
  Index rows, Index cols,
  const LhsScalar* lhs, Index lhsStride,
  const RhsScalar* rhs, Index rhsIncr,
  ResScalar* res, Index resIncr,
  ResScalar alpha)
{
  EIGEN_UNUSED_VARIABLE(rhsIncr);
  ei_internal_assert(rhsIncr==1);
  #ifdef _EIGEN_ACCUMULATE_PACKETS
  #error _EIGEN_ACCUMULATE_PACKETS has already been defined
  #endif

  #define _EIGEN_ACCUMULATE_PACKETS(A0,A13,A2) {\
    RhsPacket b = ei_pload<RhsPacket>(&rhs[j]); \
    ptmp0 = pcj.pmadd(EIGEN_CAT(ei_ploa,A0) <LhsPacket>(&lhs0[j]), b, ptmp0); \
    ptmp1 = pcj.pmadd(EIGEN_CAT(ei_ploa,A13)<LhsPacket>(&lhs1[j]), b, ptmp1); \
    ptmp2 = pcj.pmadd(EIGEN_CAT(ei_ploa,A2) <LhsPacket>(&lhs2[j]), b, ptmp2); \
    ptmp3 = pcj.pmadd(EIGEN_CAT(ei_ploa,A13)<LhsPacket>(&lhs3[j]), b, ptmp3); }

  ei_conj_helper<LhsScalar,RhsScalar,ConjugateLhs,ConjugateRhs> cj;
  ei_conj_helper<LhsPacket,RhsPacket,ConjugateLhs,ConjugateRhs> pcj;

  enum { AllAligned=0, EvenAligned=1, FirstAligned=2, NoneAligned=3 };
  const Index rowsAtOnce = 4;
  const Index peels = 2;
  const Index RhsPacketAlignedMask = RhsPacketSize-1;
  const Index LhsPacketAlignedMask = LhsPacketSize-1;
  const Index PeelAlignedMask = RhsPacketSize*peels-1;
  const Index depth = cols;

  // How many coeffs of the result do we have to skip to be aligned.
  // Here we assume data are at least aligned on the base scalar type
  // if that's not the case then vectorization is discarded, see below.
  Index alignedStart = ei_first_aligned(rhs, depth);
  Index alignedSize = RhsPacketSize>1 ? alignedStart + ((depth-alignedStart) & ~RhsPacketAlignedMask) : 0;
  const Index peeledSize  = peels>1 ? alignedStart + ((alignedSize-alignedStart) & ~PeelAlignedMask) : alignedStart;

  const Index alignmentStep = LhsPacketSize>1 ? (LhsPacketSize - lhsStride % LhsPacketSize) & LhsPacketAlignedMask : 0;
  Index alignmentPattern = alignmentStep==0 ? AllAligned
                         : alignmentStep==(LhsPacketSize/2) ? EvenAligned
                         : FirstAligned;

  // we cannot assume the first element is aligned because of sub-matrices
  const Index lhsAlignmentOffset = ei_first_aligned(lhs,depth);

  // find how many rows do we have to skip to be aligned with rhs (if possible)
  Index skipRows = 0;
  // if the data cannot be aligned (TODO add some compile time tests when possible, e.g. for floats)
  if( (sizeof(LhsScalar)!=sizeof(RhsScalar)) || (size_t(lhs)%sizeof(LhsScalar)) || (size_t(rhs)%sizeof(RhsScalar)) )
  {
    alignedSize = 0;
    alignedStart = 0;
  }
  else if (LhsPacketSize>1)
  {
    ei_internal_assert(size_t(lhs+lhsAlignmentOffset)%sizeof(LhsPacket)==0  || depth<LhsPacketSize);

    while (skipRows<LhsPacketSize &&
           alignedStart != ((lhsAlignmentOffset + alignmentStep*skipRows)%LhsPacketSize))
      ++skipRows;
    if (skipRows==LhsPacketSize)
    {
      // nothing can be aligned, no need to skip any column
      alignmentPattern = NoneAligned;
      skipRows = 0;
    }
    else
    {
      skipRows = std::min(skipRows,Index(rows));
      // note that the skiped columns are processed later.
    }
    ei_internal_assert(  alignmentPattern==NoneAligned
                      || LhsPacketSize==1
                      || (skipRows + rowsAtOnce >= rows)
                      || LhsPacketSize > depth
                      || (size_t(lhs+alignedStart+lhsStride*skipRows)%sizeof(LhsPacket))==0);
  }
  else if(Vectorizable)
  {
    alignedStart = 0;
    alignedSize = depth;
    alignmentPattern = AllAligned;
  }

  Index offset1 = (FirstAligned && alignmentStep==1?3:1);
  Index offset3 = (FirstAligned && alignmentStep==1?1:3);

  Index rowBound = ((rows-skipRows)/rowsAtOnce)*rowsAtOnce + skipRows;
  for (Index i=skipRows; i<rowBound; i+=rowsAtOnce)
  {
    EIGEN_ALIGN16 ResScalar tmp0 = ResScalar(0);
    ResScalar tmp1 = ResScalar(0), tmp2 = ResScalar(0), tmp3 = ResScalar(0);

    // this helps the compiler generating good binary code
    const LhsScalar *lhs0 = lhs + i*lhsStride,     *lhs1 = lhs + (i+offset1)*lhsStride,
                    *lhs2 = lhs + (i+2)*lhsStride, *lhs3 = lhs + (i+offset3)*lhsStride;

    if (Vectorizable)
    {
      /* explicit vectorization */
      ResPacket ptmp0 = ei_pset1<ResPacket>(ResScalar(0)), ptmp1 = ei_pset1<ResPacket>(ResScalar(0)),
                ptmp2 = ei_pset1<ResPacket>(ResScalar(0)), ptmp3 = ei_pset1<ResPacket>(ResScalar(0));

      // process initial unaligned coeffs
      // FIXME this loop get vectorized by the compiler !
      for (Index j=0; j<alignedStart; ++j)
      {
        RhsScalar b = rhs[j];
        tmp0 += cj.pmul(lhs0[j],b); tmp1 += cj.pmul(lhs1[j],b);
        tmp2 += cj.pmul(lhs2[j],b); tmp3 += cj.pmul(lhs3[j],b);
      }

      if (alignedSize>alignedStart)
      {
        switch(alignmentPattern)
        {
          case AllAligned:
            for (Index j = alignedStart; j<alignedSize; j+=RhsPacketSize)
              _EIGEN_ACCUMULATE_PACKETS(d,d,d);
            break;
          case EvenAligned:
            for (Index j = alignedStart; j<alignedSize; j+=RhsPacketSize)
              _EIGEN_ACCUMULATE_PACKETS(d,du,d);
            break;
          case FirstAligned:
            if (peels>1)
            {
              /* Here we proccess 4 rows with with two peeled iterations to hide
               * tghe overhead of unaligned loads. Moreover unaligned loads are handled
               * using special shift/move operations between the two aligned packets
               * overlaping the desired unaligned packet. This is *much* more efficient
               * than basic unaligned loads.
               */
              LhsPacket A01, A02, A03, A11, A12, A13;
              A01 = ei_pload<LhsPacket>(&lhs1[alignedStart-1]);
              A02 = ei_pload<LhsPacket>(&lhs2[alignedStart-2]);
              A03 = ei_pload<LhsPacket>(&lhs3[alignedStart-3]);

              for (Index j = alignedStart; j<peeledSize; j+=peels*RhsPacketSize)
              {
                RhsPacket b = ei_pload<RhsPacket>(&rhs[j]);
                A11 = ei_pload<LhsPacket>(&lhs1[j-1+LhsPacketSize]);  ei_palign<1>(A01,A11);
                A12 = ei_pload<LhsPacket>(&lhs2[j-2+LhsPacketSize]);  ei_palign<2>(A02,A12);
                A13 = ei_pload<LhsPacket>(&lhs3[j-3+LhsPacketSize]);  ei_palign<3>(A03,A13);

                ptmp0 = pcj.pmadd(ei_pload<LhsPacket>(&lhs0[j]), b, ptmp0);
                ptmp1 = pcj.pmadd(A01, b, ptmp1);
                A01 = ei_pload<LhsPacket>(&lhs1[j-1+2*LhsPacketSize]);  ei_palign<1>(A11,A01);
                ptmp2 = pcj.pmadd(A02, b, ptmp2);
                A02 = ei_pload<LhsPacket>(&lhs2[j-2+2*LhsPacketSize]);  ei_palign<2>(A12,A02);
                ptmp3 = pcj.pmadd(A03, b, ptmp3);
                A03 = ei_pload<LhsPacket>(&lhs3[j-3+2*LhsPacketSize]);  ei_palign<3>(A13,A03);

                b = ei_pload<RhsPacket>(&rhs[j+RhsPacketSize]);
                ptmp0 = pcj.pmadd(ei_pload<LhsPacket>(&lhs0[j+LhsPacketSize]), b, ptmp0);
                ptmp1 = pcj.pmadd(A11, b, ptmp1);
                ptmp2 = pcj.pmadd(A12, b, ptmp2);
                ptmp3 = pcj.pmadd(A13, b, ptmp3);
              }
            }
            for (Index j = peeledSize; j<alignedSize; j+=RhsPacketSize)
              _EIGEN_ACCUMULATE_PACKETS(d,du,du);
            break;
          default:
            for (Index j = alignedStart; j<alignedSize; j+=RhsPacketSize)
              _EIGEN_ACCUMULATE_PACKETS(du,du,du);
            break;
        }
        tmp0 += ei_predux(ptmp0);
        tmp1 += ei_predux(ptmp1);
        tmp2 += ei_predux(ptmp2);
        tmp3 += ei_predux(ptmp3);
      }
    } // end explicit vectorization

    // process remaining coeffs (or all if no explicit vectorization)
    // FIXME this loop get vectorized by the compiler !
    for (Index j=alignedSize; j<depth; ++j)
    {
      RhsScalar b = rhs[j];
      tmp0 += cj.pmul(lhs0[j],b); tmp1 += cj.pmul(lhs1[j],b);
      tmp2 += cj.pmul(lhs2[j],b); tmp3 += cj.pmul(lhs3[j],b);
    }
    res[i*resIncr]            += alpha*tmp0;
    res[(i+offset1)*resIncr]  += alpha*tmp1;
    res[(i+2)*resIncr]        += alpha*tmp2;
    res[(i+offset3)*resIncr]  += alpha*tmp3;
  }

  // process remaining first and last rows (at most columnsAtOnce-1)
  Index end = rows;
  Index start = rowBound;
  do
  {
    for (Index i=start; i<end; ++i)
    {
      EIGEN_ALIGN16 ResScalar tmp0 = ResScalar(0);
      ResPacket ptmp0 = ei_pset1<ResPacket>(tmp0);
      const LhsScalar* lhs0 = lhs + i*lhsStride;
      // process first unaligned result's coeffs
      // FIXME this loop get vectorized by the compiler !
      for (Index j=0; j<alignedStart; ++j)
        tmp0 += cj.pmul(lhs0[j], rhs[j]);

      if (alignedSize>alignedStart)
      {
        // process aligned rhs coeffs
        if ((size_t(lhs0+alignedStart)%sizeof(LhsPacket))==0)
          for (Index j = alignedStart;j<alignedSize;j+=RhsPacketSize)
            ptmp0 = pcj.pmadd(ei_pload<LhsPacket>(&lhs0[j]), ei_pload<RhsPacket>(&rhs[j]), ptmp0);
        else
          for (Index j = alignedStart;j<alignedSize;j+=RhsPacketSize)
            ptmp0 = pcj.pmadd(ei_ploadu<LhsPacket>(&lhs0[j]), ei_pload<RhsPacket>(&rhs[j]), ptmp0);
        tmp0 += ei_predux(ptmp0);
      }

      // process remaining scalars
      // FIXME this loop get vectorized by the compiler !
      for (Index j=alignedSize; j<depth; ++j)
        tmp0 += cj.pmul(lhs0[j], rhs[j]);
      res[i*resIncr] += alpha*tmp0;
    }
    if (skipRows)
    {
      start = 0;
      end = skipRows;
      skipRows = 0;
    }
    else
      break;
  } while(Vectorizable);

  #undef _EIGEN_ACCUMULATE_PACKETS
}
};

#endif // EIGEN_GENERAL_MATRIX_VECTOR_H
