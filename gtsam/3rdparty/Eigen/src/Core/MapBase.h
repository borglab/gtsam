// This file is part of Eigen, a lightweight C++ template library
// for linear algebra.
//
// Copyright (C) 2007-2010 Benoit Jacob <jacob.benoit.1@gmail.com>
// Copyright (C) 2008 Gael Guennebaud <gael.guennebaud@inria.fr>
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

#ifndef EIGEN_MAPBASE_H
#define EIGEN_MAPBASE_H

/** \class MapBase
  * \ingroup Core_Module
  *
  * \brief Base class for Map and Block expression with direct access
  *
  * \sa class Map, class Block
  */
template<typename Derived> class MapBase
  : public ei_dense_xpr_base<Derived>::type
{
  public:

    typedef typename ei_dense_xpr_base<Derived>::type Base;
    enum {
      RowsAtCompileTime = ei_traits<Derived>::RowsAtCompileTime,
      ColsAtCompileTime = ei_traits<Derived>::ColsAtCompileTime,
      SizeAtCompileTime = Base::SizeAtCompileTime
    };


    typedef typename ei_traits<Derived>::StorageKind StorageKind;
    typedef typename ei_traits<Derived>::Index Index;
    typedef typename ei_traits<Derived>::Scalar Scalar;
    typedef typename ei_packet_traits<Scalar>::type PacketScalar;
    typedef typename NumTraits<Scalar>::Real RealScalar;

    using Base::derived;
//    using Base::RowsAtCompileTime;
//    using Base::ColsAtCompileTime;
//    using Base::SizeAtCompileTime;
    using Base::MaxRowsAtCompileTime;
    using Base::MaxColsAtCompileTime;
    using Base::MaxSizeAtCompileTime;
    using Base::IsVectorAtCompileTime;
    using Base::Flags;
    using Base::IsRowMajor;

    using Base::CoeffReadCost;

//    using Base::derived;
    using Base::const_cast_derived;
    using Base::rows;
    using Base::cols;
    using Base::size;
    using Base::coeff;
    using Base::coeffRef;
    using Base::lazyAssign;
    using Base::eval;
//    using Base::operator=;
    using Base::operator+=;
    using Base::operator-=;
    using Base::operator*=;
    using Base::operator/=;

    using Base::innerStride;
    using Base::outerStride;
    using Base::rowStride;
    using Base::colStride;


    typedef typename Base::CoeffReturnType CoeffReturnType;

    inline Index rows() const { return m_rows.value(); }
    inline Index cols() const { return m_cols.value(); }

    /** Returns a pointer to the first coefficient of the matrix or vector.
      *
      * \note When addressing this data, make sure to honor the strides returned by innerStride() and outerStride().
      *
      * \sa innerStride(), outerStride()
      */
    inline const Scalar* data() const { return m_data; }

    inline const Scalar& coeff(Index row, Index col) const
    {
      return m_data[col * colStride() + row * rowStride()];
    }

    inline Scalar& coeffRef(Index row, Index col)
    {
      return const_cast<Scalar*>(m_data)[col * colStride() + row * rowStride()];
    }

    inline const Scalar& coeff(Index index) const
    {
      ei_assert(Derived::IsVectorAtCompileTime || (ei_traits<Derived>::Flags & LinearAccessBit));
      return m_data[index * innerStride()];
    }

    inline Scalar& coeffRef(Index index)
    {
      ei_assert(Derived::IsVectorAtCompileTime || (ei_traits<Derived>::Flags & LinearAccessBit));
      return const_cast<Scalar*>(m_data)[index * innerStride()];
    }

    template<int LoadMode>
    inline PacketScalar packet(Index row, Index col) const
    {
      return ei_ploadt<PacketScalar, LoadMode>
               (m_data + (col * colStride() + row * rowStride()));
    }

    template<int LoadMode>
    inline PacketScalar packet(Index index) const
    {
      return ei_ploadt<PacketScalar, LoadMode>(m_data + index * innerStride());
    }

    template<int StoreMode>
    inline void writePacket(Index row, Index col, const PacketScalar& x)
    {
      ei_pstoret<Scalar, PacketScalar, StoreMode>
               (const_cast<Scalar*>(m_data) + (col * colStride() + row * rowStride()), x);
    }

    template<int StoreMode>
    inline void writePacket(Index index, const PacketScalar& x)
    {
      ei_pstoret<Scalar, PacketScalar, StoreMode>
        (const_cast<Scalar*>(m_data) + index * innerStride(), x);
    }

    inline MapBase(const Scalar* data) : m_data(data), m_rows(RowsAtCompileTime), m_cols(ColsAtCompileTime)
    {
      EIGEN_STATIC_ASSERT_FIXED_SIZE(Derived)
      checkSanity();
    }

    inline MapBase(const Scalar* data, Index size)
            : m_data(data),
              m_rows(RowsAtCompileTime == Dynamic ? size : Index(RowsAtCompileTime)),
              m_cols(ColsAtCompileTime == Dynamic ? size : Index(ColsAtCompileTime))
    {
      EIGEN_STATIC_ASSERT_VECTOR_ONLY(Derived)
      ei_assert(size >= 0);
      ei_assert(data == 0 || SizeAtCompileTime == Dynamic || SizeAtCompileTime == size);
      checkSanity();
    }

    inline MapBase(const Scalar* data, Index rows, Index cols)
            : m_data(data), m_rows(rows), m_cols(cols)
    {
      ei_assert( (data == 0)
              || (   rows >= 0 && (RowsAtCompileTime == Dynamic || RowsAtCompileTime == rows)
                  && cols >= 0 && (ColsAtCompileTime == Dynamic || ColsAtCompileTime == cols)));
      checkSanity();
    }

    Derived& operator=(const MapBase& other)
    {
      Base::operator=(other);
      return derived();
    }

    using Base::operator=;

  protected:

    void checkSanity() const
    {
      EIGEN_STATIC_ASSERT(EIGEN_IMPLIES(ei_traits<Derived>::Flags&PacketAccessBit,
                                        ei_inner_stride_at_compile_time<Derived>::ret==1),
                          PACKET_ACCESS_REQUIRES_TO_HAVE_INNER_STRIDE_FIXED_TO_1);
      ei_assert(EIGEN_IMPLIES(ei_traits<Derived>::Flags&AlignedBit, (size_t(m_data) % (sizeof(Scalar)*ei_packet_traits<Scalar>::size)) == 0)
        && "data is not aligned");
    }

    const Scalar* EIGEN_RESTRICT m_data;
    const ei_variable_if_dynamic<Index, RowsAtCompileTime> m_rows;
    const ei_variable_if_dynamic<Index, ColsAtCompileTime> m_cols;
};

#endif // EIGEN_MAPBASE_H
