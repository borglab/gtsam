#pragma once

#include <gtsam/base/Lie.h>

/**
 * Adapter to 'hack' VectorSpace traits onto a class that only satisfies Manifold traits
 * This class assumes that a there is no "most sensible" choice of embedding,
 * so composition will always be meaningless regardless of embedding.
 * This class provides rough and ready implementations for expmap and logmap 
 * based on extrapolations of ManifoldTraits::Local and ManifoldTraits::Retract
 * The Identity vector is chosen as the default constructor for the base class.
 */

namespace gtsam {

using namespace gtsam::internal;

template <class Class>
struct AsVectorSpace : public Class
{
  typedef typename traits<Class>::TangentVector TangentVector;

  /**
   * default and copy constructor allows up-cast from Class
   */
  AsVectorSpace(const Class& c = Identity()):Class(c){}

  /**
   * Identity can be anywhere, as long as it's consistent
   * Choose the default constructor for simplicity.
   */
  static Class Identity() { return Class();}

  /**
   * satisfy vector constructor requirement imposed by
   *  static Class VectorSpaceImpl::Expmap(const TangentVector&, ChartJacobian Hv)
   */
  AsVectorSpace(const TangentVector& v):
    Class(Identity().retract(v)) {}
    //Class(traits<Class>::Retract(Identity(), v)) {}

  /**
   * Local and Retract are provided by ManifoldTraits
   * VectorSpace Compose, Between, Inverse require binary +- and unary - operators.
   */
  Class operator+ (const Class& rhs) const {
    auto v_rhs = traits<Class>::Local(Identity(), rhs);
    return traits<Class>::Retract(*this, v_rhs);
  }
  Class operator- (const Class& rhs) const {
    auto v_rhs = traits<Class>::Local(Identity(), rhs);
    return traits<Class>::Retract(*this, -v_rhs);
  }
  Class operator- () const {
    auto v_rhs = traits<Class>::Local(Identity(), *this);
    return traits<Class>::Retract(Identity(), -v_rhs);
  }
  Class operator+ (const TangentVector& rhs) const {
    return traits<Class>::Retract(*this, rhs);
  }
  Class operator- (const TangentVector& rhs) const {
    return traits<Class>::Retract(*this, -rhs);
  }

  /**
   * return a vector describing the difference between this and Identity
   * required by VectorSpaceImpl::Logmap
   */
  TangentVector vector() const {
    return traits<Class>::Local(Identity(), *this);
  }
  

}; // class AsVectorSpace


template <class Class>
struct traits<AsVectorSpace<Class>> :
  public internal::VectorSpaceTraits<AsVectorSpace<Class>> ,
  public Testable<AsVectorSpace<Class>>
{};

template <class Class>
struct traits<const AsVectorSpace<Class>> :
  public internal::VectorSpaceTraits<AsVectorSpace<Class>>,
  public Testable<AsVectorSpace<Class>>
{};

} // namespace gtsam
