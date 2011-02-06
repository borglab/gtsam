// This file is part of Eigen, a lightweight C++ template library
// for linear algebra.
//
// Copyright (C) 2009 Benoit Jacob <jacob.benoit.1@gmail.com>
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

#ifndef EIGEN_JACOBI_H
#define EIGEN_JACOBI_H

/** \ingroup Jacobi_Module
  * \jacobi_module
  * \class PlanarRotation
  * \brief Represents a rotation in the plane from a cosine-sine pair.
  *
  * This class represents a Jacobi or Givens rotation.
  * This is a 2D rotation in the plane \c J of angle \f$ \theta \f$ defined by
  * its cosine \c c and sine \c s as follow:
  * \f$ J = \left ( \begin{array}{cc} c & \overline s \\ -s  & \overline c \end{array} \right ) \f$
  *
  * You can apply the respective counter-clockwise rotation to a column vector \c v by
  * applying its adjoint on the left: \f$ v = J^* v \f$ that translates to the following Eigen code:
  * \code
  * v.applyOnTheLeft(J.adjoint());
  * \endcode
  *
  * \sa MatrixBase::applyOnTheLeft(), MatrixBase::applyOnTheRight()
  */
template<typename Scalar> class PlanarRotation
{
  public:
    typedef typename NumTraits<Scalar>::Real RealScalar;

    /** Default constructor without any initialization. */
    PlanarRotation() {}

    /** Construct a planar rotation from a cosine-sine pair (\a c, \c s). */
    PlanarRotation(const Scalar& c, const Scalar& s) : m_c(c), m_s(s) {}

    Scalar& c() { return m_c; }
    Scalar c() const { return m_c; }
    Scalar& s() { return m_s; }
    Scalar s() const { return m_s; }

    /** Concatenates two planar rotation */
    PlanarRotation operator*(const PlanarRotation& other)
    {
      return PlanarRotation(m_c * other.m_c - ei_conj(m_s) * other.m_s,
                            ei_conj(m_c * ei_conj(other.m_s) + ei_conj(m_s) * ei_conj(other.m_c)));
    }

    /** Returns the transposed transformation */
    PlanarRotation transpose() const { return PlanarRotation(m_c, -ei_conj(m_s)); }

    /** Returns the adjoint transformation */
    PlanarRotation adjoint() const { return PlanarRotation(ei_conj(m_c), -m_s); }

    template<typename Derived>
    bool makeJacobi(const MatrixBase<Derived>&, typename Derived::Index p, typename Derived::Index q);
    bool makeJacobi(RealScalar x, Scalar y, RealScalar z);

    void makeGivens(const Scalar& p, const Scalar& q, Scalar* z=0);

  protected:
    void makeGivens(const Scalar& p, const Scalar& q, Scalar* z, ei_meta_true);
    void makeGivens(const Scalar& p, const Scalar& q, Scalar* z, ei_meta_false);

    Scalar m_c, m_s;
};

/** Makes \c *this as a Jacobi rotation \a J such that applying \a J on both the right and left sides of the selfadjoint 2x2 matrix
  * \f$ B = \left ( \begin{array}{cc} x & y \\ \overline y & z \end{array} \right )\f$ yields a diagonal matrix \f$ A = J^* B J \f$
  *
  * \sa MatrixBase::makeJacobi(const MatrixBase<Derived>&, Index, Index), MatrixBase::applyOnTheLeft(), MatrixBase::applyOnTheRight()
  */
template<typename Scalar>
bool PlanarRotation<Scalar>::makeJacobi(RealScalar x, Scalar y, RealScalar z)
{
  typedef typename NumTraits<Scalar>::Real RealScalar;
  if(y == Scalar(0))
  {
    m_c = Scalar(1);
    m_s = Scalar(0);
    return false;
  }
  else
  {
    RealScalar tau = (x-z)/(RealScalar(2)*ei_abs(y));
    RealScalar w = ei_sqrt(ei_abs2(tau) + 1);
    RealScalar t;
    if(tau>0)
    {
      t = RealScalar(1) / (tau + w);
    }
    else
    {
      t = RealScalar(1) / (tau - w);
    }
    RealScalar sign_t = t > 0 ? 1 : -1;
    RealScalar n = RealScalar(1) / ei_sqrt(ei_abs2(t)+1);
    m_s = - sign_t * (ei_conj(y) / ei_abs(y)) * ei_abs(t) * n;
    m_c = n;
    return true;
  }
}

/** Makes \c *this as a Jacobi rotation \c J such that applying \a J on both the right and left sides of the 2x2 selfadjoint matrix
  * \f$ B = \left ( \begin{array}{cc} \text{this}_{pp} & \text{this}_{pq} \\ (\text{this}_{pq})^* & \text{this}_{qq} \end{array} \right )\f$ yields
  * a diagonal matrix \f$ A = J^* B J \f$
  *
  * Example: \include Jacobi_makeJacobi.cpp
  * Output: \verbinclude Jacobi_makeJacobi.out
  *
  * \sa PlanarRotation::makeJacobi(RealScalar, Scalar, RealScalar), MatrixBase::applyOnTheLeft(), MatrixBase::applyOnTheRight()
  */
template<typename Scalar>
template<typename Derived>
inline bool PlanarRotation<Scalar>::makeJacobi(const MatrixBase<Derived>& m, typename Derived::Index p, typename Derived::Index q)
{
  return makeJacobi(ei_real(m.coeff(p,p)), m.coeff(p,q), ei_real(m.coeff(q,q)));
}

/** Makes \c *this as a Givens rotation \c G such that applying \f$ G^* \f$ to the left of the vector
  * \f$ V = \left ( \begin{array}{c} p \\ q \end{array} \right )\f$ yields:
  * \f$ G^* V = \left ( \begin{array}{c} r \\ 0 \end{array} \right )\f$.
  *
  * The value of \a z is returned if \a z is not null (the default is null).
  * Also note that G is built such that the cosine is always real.
  *
  * Example: \include Jacobi_makeGivens.cpp
  * Output: \verbinclude Jacobi_makeGivens.out
  *
  * This function implements the continuous Givens rotation generation algorithm
  * found in Anderson (2000), Discontinuous Plane Rotations and the Symmetric Eigenvalue Problem.
  * LAPACK Working Note 150, University of Tennessee, UT-CS-00-454, December 4, 2000.
  *
  * \sa MatrixBase::applyOnTheLeft(), MatrixBase::applyOnTheRight()
  */
template<typename Scalar>
void PlanarRotation<Scalar>::makeGivens(const Scalar& p, const Scalar& q, Scalar* z)
{
  makeGivens(p, q, z, typename ei_meta_if<NumTraits<Scalar>::IsComplex, ei_meta_true, ei_meta_false>::ret());
}


// specialization for complexes
template<typename Scalar>
void PlanarRotation<Scalar>::makeGivens(const Scalar& p, const Scalar& q, Scalar* r, ei_meta_true)
{
  if(q==Scalar(0))
  {
    m_c = ei_real(p)<0 ? Scalar(-1) : Scalar(1);
    m_s = 0;
    if(r) *r = m_c * p;
  }
  else if(p==Scalar(0))
  {
    m_c = 0;
    m_s = -q/ei_abs(q);
    if(r) *r = ei_abs(q);
  }
  else
  {
    RealScalar p1 = ei_norm1(p);
    RealScalar q1 = ei_norm1(q);
    if(p1>=q1)
    {
      Scalar ps = p / p1;
      RealScalar p2 = ei_abs2(ps);
      Scalar qs = q / p1;
      RealScalar q2 = ei_abs2(qs);

      RealScalar u = ei_sqrt(RealScalar(1) + q2/p2);
      if(ei_real(p)<RealScalar(0))
        u = -u;

      m_c = Scalar(1)/u;
      m_s = -qs*ei_conj(ps)*(m_c/p2);
      if(r) *r = p * u;
    }
    else
    {
      Scalar ps = p / q1;
      RealScalar p2 = ei_abs2(ps);
      Scalar qs = q / q1;
      RealScalar q2 = ei_abs2(qs);

      RealScalar u = q1 * ei_sqrt(p2 + q2);
      if(ei_real(p)<RealScalar(0))
        u = -u;

      p1 = ei_abs(p);
      ps = p/p1;
      m_c = p1/u;
      m_s = -ei_conj(ps) * (q/u);
      if(r) *r = ps * u;
    }
  }
}

// specialization for reals
template<typename Scalar>
void PlanarRotation<Scalar>::makeGivens(const Scalar& p, const Scalar& q, Scalar* r, ei_meta_false)
{

  if(q==0)
  {
    m_c = p<Scalar(0) ? Scalar(-1) : Scalar(1);
    m_s = 0;
    if(r) *r = ei_abs(p);
  }
  else if(p==0)
  {
    m_c = 0;
    m_s = q<Scalar(0) ? Scalar(1) : Scalar(-1);
    if(r) *r = ei_abs(q);
  }
  else if(ei_abs(p) > ei_abs(q))
  {
    Scalar t = q/p;
    Scalar u = ei_sqrt(Scalar(1) + ei_abs2(t));
    if(p<Scalar(0))
      u = -u;
    m_c = Scalar(1)/u;
    m_s = -t * m_c;
    if(r) *r = p * u;
  }
  else
  {
    Scalar t = p/q;
    Scalar u = ei_sqrt(Scalar(1) + ei_abs2(t));
    if(q<Scalar(0))
      u = -u;
    m_s = -Scalar(1)/u;
    m_c = -t * m_s;
    if(r) *r = q * u;
  }

}

/****************************************************************************************
*   Implementation of MatrixBase methods
****************************************************************************************/

/** \jacobi_module
  * Applies the clock wise 2D rotation \a j to the set of 2D vectors of cordinates \a x and \a y:
  * \f$ \left ( \begin{array}{cc} x \\ y \end{array} \right )  =  J \left ( \begin{array}{cc} x \\ y \end{array} \right ) \f$
  *
  * \sa MatrixBase::applyOnTheLeft(), MatrixBase::applyOnTheRight()
  */
template<typename VectorX, typename VectorY, typename OtherScalar>
void ei_apply_rotation_in_the_plane(VectorX& _x, VectorY& _y, const PlanarRotation<OtherScalar>& j);

/** \jacobi_module
  * Applies the rotation in the plane \a j to the rows \a p and \a q of \c *this, i.e., it computes B = J * B,
  * with \f$ B = \left ( \begin{array}{cc} \text{*this.row}(p) \\ \text{*this.row}(q) \end{array} \right ) \f$.
  *
  * \sa class PlanarRotation, MatrixBase::applyOnTheRight(), ei_apply_rotation_in_the_plane()
  */
template<typename Derived>
template<typename OtherScalar>
inline void MatrixBase<Derived>::applyOnTheLeft(Index p, Index q, const PlanarRotation<OtherScalar>& j)
{
  RowXpr x(this->row(p));
  RowXpr y(this->row(q));
  ei_apply_rotation_in_the_plane(x, y, j);
}

/** \ingroup Jacobi_Module
  * Applies the rotation in the plane \a j to the columns \a p and \a q of \c *this, i.e., it computes B = B * J
  * with \f$ B = \left ( \begin{array}{cc} \text{*this.col}(p) & \text{*this.col}(q) \end{array} \right ) \f$.
  *
  * \sa class PlanarRotation, MatrixBase::applyOnTheLeft(), ei_apply_rotation_in_the_plane()
  */
template<typename Derived>
template<typename OtherScalar>
inline void MatrixBase<Derived>::applyOnTheRight(Index p, Index q, const PlanarRotation<OtherScalar>& j)
{
  ColXpr x(this->col(p));
  ColXpr y(this->col(q));
  ei_apply_rotation_in_the_plane(x, y, j.transpose());
}


template<typename VectorX, typename VectorY, typename OtherScalar>
void /*EIGEN_DONT_INLINE*/ ei_apply_rotation_in_the_plane(VectorX& _x, VectorY& _y, const PlanarRotation<OtherScalar>& j)
{
  typedef typename VectorX::Index Index;
  typedef typename VectorX::Scalar Scalar;
  enum { PacketSize = ei_packet_traits<Scalar>::size };
  typedef typename ei_packet_traits<Scalar>::type Packet;
  ei_assert(_x.size() == _y.size());
  Index size = _x.size();
  Index incrx = _x.innerStride();
  Index incry = _y.innerStride();

  Scalar* EIGEN_RESTRICT x = &_x.coeffRef(0);
  Scalar* EIGEN_RESTRICT y = &_y.coeffRef(0);

  /*** dynamic-size vectorized paths ***/

  if(VectorX::SizeAtCompileTime == Dynamic &&
    (VectorX::Flags & VectorY::Flags & PacketAccessBit) &&
    ((incrx==1 && incry==1) || PacketSize == 1))
  {
    // both vectors are sequentially stored in memory => vectorization
    enum { Peeling = 2 };

    Index alignedStart = ei_first_aligned(y, size);
    Index alignedEnd = alignedStart + ((size-alignedStart)/PacketSize)*PacketSize;

    const Packet pc = ei_pset1<Packet>(j.c());
    const Packet ps = ei_pset1<Packet>(j.s());
    ei_conj_helper<Packet,Packet,NumTraits<Scalar>::IsComplex,false> pcj;

    for(Index i=0; i<alignedStart; ++i)
    {
      Scalar xi = x[i];
      Scalar yi = y[i];
      x[i] =  j.c() * xi + ei_conj(j.s()) * yi;
      y[i] = -j.s() * xi + ei_conj(j.c()) * yi;
    }

    Scalar* EIGEN_RESTRICT px = x + alignedStart;
    Scalar* EIGEN_RESTRICT py = y + alignedStart;

    if(ei_first_aligned(x, size)==alignedStart)
    {
      for(Index i=alignedStart; i<alignedEnd; i+=PacketSize)
      {
        Packet xi = ei_pload<Packet>(px);
        Packet yi = ei_pload<Packet>(py);
        ei_pstore(px, ei_padd(ei_pmul(pc,xi),pcj.pmul(ps,yi)));
        ei_pstore(py, ei_psub(pcj.pmul(pc,yi),ei_pmul(ps,xi)));
        px += PacketSize;
        py += PacketSize;
      }
    }
    else
    {
      Index peelingEnd = alignedStart + ((size-alignedStart)/(Peeling*PacketSize))*(Peeling*PacketSize);
      for(Index i=alignedStart; i<peelingEnd; i+=Peeling*PacketSize)
      {
        Packet xi   = ei_ploadu<Packet>(px);
        Packet xi1  = ei_ploadu<Packet>(px+PacketSize);
        Packet yi   = ei_pload <Packet>(py);
        Packet yi1  = ei_pload <Packet>(py+PacketSize);
        ei_pstoreu(px, ei_padd(ei_pmul(pc,xi),pcj.pmul(ps,yi)));
        ei_pstoreu(px+PacketSize, ei_padd(ei_pmul(pc,xi1),pcj.pmul(ps,yi1)));
        ei_pstore (py, ei_psub(pcj.pmul(pc,yi),ei_pmul(ps,xi)));
        ei_pstore (py+PacketSize, ei_psub(pcj.pmul(pc,yi1),ei_pmul(ps,xi1)));
        px += Peeling*PacketSize;
        py += Peeling*PacketSize;
      }
      if(alignedEnd!=peelingEnd)
      {
        Packet xi = ei_ploadu<Packet>(x+peelingEnd);
        Packet yi = ei_pload <Packet>(y+peelingEnd);
        ei_pstoreu(x+peelingEnd, ei_padd(ei_pmul(pc,xi),pcj.pmul(ps,yi)));
        ei_pstore (y+peelingEnd, ei_psub(pcj.pmul(pc,yi),ei_pmul(ps,xi)));
      }
    }

    for(Index i=alignedEnd; i<size; ++i)
    {
      Scalar xi = x[i];
      Scalar yi = y[i];
      x[i] =  j.c() * xi + ei_conj(j.s()) * yi;
      y[i] = -j.s() * xi + ei_conj(j.c()) * yi;
    }
  }

  /*** fixed-size vectorized path ***/
  else if(VectorX::SizeAtCompileTime != Dynamic &&
          (VectorX::Flags & VectorY::Flags & PacketAccessBit) &&
          (VectorX::Flags & VectorY::Flags & AlignedBit))
  {
    const Packet pc = ei_pset1<Packet>(j.c());
    const Packet ps = ei_pset1<Packet>(j.s());
    ei_conj_helper<Packet,Packet,NumTraits<Scalar>::IsComplex,false> pcj;
    Scalar* EIGEN_RESTRICT px = x;
    Scalar* EIGEN_RESTRICT py = y;
    for(Index i=0; i<size; i+=PacketSize)
    {
      Packet xi = ei_pload<Packet>(px);
      Packet yi = ei_pload<Packet>(py);
      ei_pstore(px, ei_padd(ei_pmul(pc,xi),pcj.pmul(ps,yi)));
      ei_pstore(py, ei_psub(pcj.pmul(pc,yi),ei_pmul(ps,xi)));
      px += PacketSize;
      py += PacketSize;
    }
  }

  /*** non-vectorized path ***/
  else
  {
    for(Index i=0; i<size; ++i)
    {
      Scalar xi = *x;
      Scalar yi = *y;
      *x =  j.c() * xi + ei_conj(j.s()) * yi;
      *y = -j.s() * xi + ei_conj(j.c()) * yi;
      x += incrx;
      y += incry;
    }
  }
}

#endif // EIGEN_JACOBI_H
