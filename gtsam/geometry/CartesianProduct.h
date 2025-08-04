#pragma once

#include <gtsam/base/Lie.h>


/*
Cartesian product to combine unrelated manifolds into one data structure
conceptually similar to a std::pair but you get to keep all the manifold properties
This provides syntactic sugar allowing Lie::interpolate to be applied to multiple objects at once.
For example, a trajectory involving control points for pose and calibration for a motorised zoom lens on a moving camera.
 */


// LieGroup       requires: identity(), inverse(), composition operator*(p), static ExpMap, static LogMap
//                provides: dimension, identity(h), inverse(h), compose(p,h1,h2), between, localCoordinates, retract,
// LieGroupTraits requires: dimension, identity(h), inverse(h), compose(p,h1,h2), between, localCoordinates, retract, Logmap, Expmap, AdjointMap, 

// ExpmapDerivative, LogmapDerivative are normally defined to provide jacobians for expmap and logmap,
// we can just refer to the expmap jacobians of the stored classes



  // XXX needs a compile time flag to check that the stored classes satisfy relevant properties
  // (IsGroup<A> &&  IsGroup<B>)
  // (IsManifold<A> &&  IsManifold<B>)
  // (IsVectorSpace<A> && IsVectorSpace<B>)
  // Except that I'm mostly interested in interpolate, so I only care about stuff with expmap and logmap
  //   camera calibration is a manifold not a group, so it can't encode a guarantee of continuity into an expmap.
  // Manifold at least offers a rudimentary local vector space `Local(origin, other)->vector` and `Retract(origin, vector)->other`
  //   So I might be able to monkey-patch an interpolate method into those,
  //   except that it doesn't expose jacobians so it's useless for Expressions


namespace gtsam {

template <class A, class B>
class CartesianProduct
: public LieGroup<CartesianProduct<A,B>, traits<A>::dimension + traits<B>::dimension>
{
public:
  constexpr static auto a_dim = traits<A>::dimension;
  constexpr static auto b_dim = traits<B>::dimension;
  constexpr static auto dimension = traits<A>::dimension + traits<B>::dimension;
  //using LieGroup<CartesianProduct<A,B>, traits<A>::dimension + traits<B>::dimension>::dimension;

  //using LieGroup<CartesianProduct<A,B>, dimension>::ChartJacobian;
  //using LieGroup<CartesianProduct<A,B>, dimension>::Jacobian;
  //using LieGroup<CartesianProduct<A,B>, dimension>::TangentVector;
  using LieGroup<CartesianProduct<A,B>, dimension>::inverse;


  typedef OptionalJacobian<dimension, dimension> ChartJacobian;
  typedef Eigen::Matrix<double, dimension, dimension> Jacobian;
  typedef Eigen::Matrix<double, dimension, 1> TangentVector;


  // XXX For whatever reason, MatrixLieGroup::Jacobian isn't exposed on Pose3
  typedef Eigen::Matrix<double, a_dim, a_dim> A_Jacobian;
  typedef Eigen::Matrix<double, b_dim, b_dim> B_Jacobian;


private:
  A a_;
  B b_;

public:
  /** Default constructor */
  CartesianProduct(const A& a = traits<A>::Identity(), const B& b = traits<B>::Identity()) : a_(a), b_(b) {}

  /** Copy constructor */
  CartesianProduct(const CartesianProduct& copy_ref) = default;


  static CartesianProduct Create(const A& a, const B& b,
    OptionalJacobian<dimension, a_dim> Ha = {},
    OptionalJacobian<dimension, b_dim> Hb = {})
  {
    // XXX use transpose of getter jacobians
    /*
    ret = CartesianProduct(a, b)
    ret.a(Ha.T())
    ret.b(Hb.T())
    */

    if(Ha)
    {
      *Ha <<  Eigen::Matrix<double, a_dim, a_dim>::Identity(),
              Eigen::Matrix<double, b_dim, a_dim>::Zero();
    }
    if(Hb)
    {
      *Hb <<  Eigen::Matrix<double, a_dim, b_dim>::Zero(),
              Eigen::Matrix<double, b_dim, b_dim>::Identity();
    }
    return CartesianProduct(a, b);
  }

  /** Getter for the first stored value */
  const A& a(OptionalJacobian<a_dim, dimension> H={}) const
  {
    if(H)
    {
      *H << Eigen::Matrix<double, a_dim, a_dim>::Identity(),
            Eigen::Matrix<double, a_dim, b_dim>::Zero();
    }
    return a_;
  }

  /** Getter for the second stored value */
  const B& b(OptionalJacobian<b_dim, dimension> H={}) const
  {
    if(H)
    {
      *H << Eigen::Matrix<double, b_dim, a_dim>::Zero(),
            Eigen::Matrix<double, b_dim, b_dim>::Identity();
    }
    return b_;
  }

  // ## pre-requisites for LieGroup

  /** inverse without derivative is required for LieGroup::inverse(H) */
  const CartesianProduct inverse() const
  {
    return CartesianProduct(traits<A>::Inverse(a_), traits<B>::Inverse(b_));
  }

  /** group composition operator, members of the cartesian product are are treated as fully orthogonal. */
  inline CartesianProduct operator*(const CartesianProduct& p) const
  {
    return CartesianProduct(
      traits<A>::Compose(a_, p.a()),
      traits<B>::Compose(b_, p.b()));
  }


  /// @}
  /// @name Testable
  /// @{

  friend std::ostream& operator<< (std::ostream &os, const CartesianProduct& p)
  {
    os << "a: " << p.a() << "\n";
    os << "b: " << p.b();
    return os;
  }
  
  /// print with optional string
  void print(const std::string& s = "") const {
    std::cout << (s.empty() ? s : s + " ") << *this << std::endl;
  }

  /// assert equality up to a tolerance
  bool equals(const CartesianProduct& p, double tol = 1e-9) const {
    return traits<A>::Equals(a_, p.a_, tol) && traits<B>::Equals(b_, p.b_, tol);
  }



  /// @}
  /// @name Group
  /// @{

  static CartesianProduct Identity() {
    return CartesianProduct(traits<A>::Identity(), traits<B>::Identity());
  }


  static CartesianProduct Expmap(const TangentVector& v, ChartJacobian H = {})
  {
    typename traits<A>::TangentVector va = v.template block<a_dim,1>(0,0);
    typename traits<B>::TangentVector vb = v.template block<b_dim,1>(a_dim,0);
    A a;
    B b;
    if(H) {
      A_Jacobian Ha;
      B_Jacobian Hb;
      a = traits<A>::Expmap(va, Ha);
      b = traits<B>::Expmap(vb, Hb);
      *H <<  Eigen::Matrix<double, dimension, dimension>::Zero();
      (*H).template block<a_dim,a_dim>(0,0) = Ha;
      (*H).template block<b_dim,b_dim>(a_dim,a_dim) = Hb;
    }
    else
    {
      a = traits<A>::Expmap(va);
      b = traits<B>::Expmap(vb);
    }
    return CartesianProduct(a,b);
  }


  static TangentVector Logmap(const CartesianProduct& p, ChartJacobian H = {})
  {
    TangentVector tan;
    if (H) {
      A_Jacobian Ha;
      B_Jacobian Hb;
      tan <<  traits<A>::Logmap(p.a_, Ha),
              traits<B>::Logmap(p.b_, Hb);
      *H << Eigen::Matrix<double, dimension, dimension>::Zero();
      (*H).template block<a_dim,a_dim>(0,0) = Ha;
      (*H).template block<b_dim,b_dim>(a_dim,a_dim) = Hb;
      // XXX check off-diag row/col order
      //*H << A::LogmapDerivative(p.a_), Eigen::Matrix<double, b_dim, a_dim>::Zero(), 
      //      Eigen::Matrix<double, a_dim, b_dim>::Zero(), B::LogmapDerivative(p.b_);
    }
    else
    {
      tan <<  traits<A>::Logmap(p.a_),
              traits<B>::Logmap(p.b_);
    }
    return tan;
  }


  Jacobian AdjointMap() const
  {
    Jacobian H = Eigen::Matrix<double, dimension, dimension>::Zero();
    H.template block<a_dim,a_dim>(0,0) = traits<A>::AdjointMap(a_);
    H.template block<b_dim,b_dim>(a_dim,a_dim) = traits<B>::AdjointMap(b_);
    return H;
  }


  TangentVector localCoordinates(const CartesianProduct& other) const
  {
    TangentVector v;
    v <<  traits<A>::Local(a(), other.a()),
          traits<B>::Local(b(), other.b());
    return v;
  }
  TangentVector localCoordinates(const CartesianProduct& other,
    ChartJacobian H1, ChartJacobian H2 = {}) const
  {
    TangentVector v;
    A_Jacobian H1a, H2a;
    B_Jacobian H1b, H2b;
    v <<  traits<A>::Local(a(), other.a(), H1a, H2a),
          traits<B>::Local(b(), other.b(), H1b, H2b);
    
    if(H1){
      (*H1).template block<a_dim,a_dim>(0,0) = H1a;
      (*H1).template block<b_dim,b_dim>(a_dim,a_dim) = H1b;
    }
    if(H2){
      (*H2).template block<a_dim,a_dim>(0,0) = H2a;
      (*H2).template block<b_dim,b_dim>(a_dim,a_dim) = H2b;
    }

    return v;
  }

  CartesianProduct retract(const TangentVector& v) const
  {
    typename traits<A>::TangentVector va = v.template block<a_dim,1>(0,0);
    typename traits<B>::TangentVector vb = v.template block<b_dim,1>(a_dim,0);
    return CartesianProduct(traits<A>::Retract(a(), va),traits<B>::Retract(b(), vb));
  }

  CartesianProduct retract(const TangentVector& v,
    ChartJacobian H1 = {}, ChartJacobian H2 = {}) const
  {
    typename traits<A>::TangentVector va = v.template block<a_dim,1>(0,0);
    typename traits<B>::TangentVector vb = v.template block<b_dim,1>(a_dim,0);
    A_Jacobian H1a, H2a;
    B_Jacobian H1b, H2b;
    CartesianProduct ret = CartesianProduct(
      traits<A>::Retract(a(), va, H1a, H2a),
      traits<B>::Retract(b(), vb, H1b, H2b));
    if(H1){
      (*H1).template block<a_dim,a_dim>(0,0) = H1a;
      (*H1).template block<b_dim,b_dim>(a_dim,a_dim) = H1b;
    }
    if(H2){
      (*H2).template block<a_dim,a_dim>(0,0) = H2a;
      (*H2).template block<b_dim,b_dim>(a_dim,a_dim) = H2b;
    }

    return ret;
  }


  

  struct ChartAtOrigin {
    static CartesianProduct Retract(const TangentVector& v, ChartJacobian H = {})
    {
      typename traits<A>::TangentVector va = v.template block<a_dim,1>(0,0);
      typename traits<B>::TangentVector vb = v.template block<b_dim,1>(a_dim,0);
      A a;
      B b;
      if(H) {
        A_Jacobian Ha;
        B_Jacobian Hb;
        a = traits<A>::ChartAtOrigin::Retract(va, Ha);
        b = traits<B>::ChartAtOrigin::Retract(vb, Hb);
        *H <<  Eigen::Matrix<double, dimension, dimension>::Zero();
        (*H).template block<a_dim,a_dim>(0,0) = Ha;
        (*H).template block<b_dim,b_dim>(a_dim,a_dim) = Hb;
      }
      else
      {
        a = traits<A>::ChartAtOrigin::Retract(va);
        b = traits<B>::ChartAtOrigin::Retract(vb);
      }  
      return CartesianProduct(a,b);
    }
    static TangentVector Local(const CartesianProduct& p, ChartJacobian H = {})
    {
      TangentVector t;
      if(H){
        A_Jacobian Ha;
        B_Jacobian Hb;
        t <<  traits<A>::ChartAtOrigin::Local(p.a(), Ha),
              traits<B>::ChartAtOrigin::Local(p.b(), Hb);
        *H <<  Eigen::Matrix<double, dimension, dimension>::Zero();
        (*H).template block<a_dim,a_dim>(0,0) = Ha;
        (*H).template block<b_dim,b_dim>(a_dim,a_dim) = Hb;
      }
      else
      {
        t <<  traits<A>::ChartAtOrigin::Local(p.a()),
              traits<B>::ChartAtOrigin::Local(p.b());
      }
      return t;
    }
  };
  

};


// create the Traits structure for the class

template<class A, class B>
struct traits<CartesianProduct<A,B> > :
public internal::LieGroupTraits<CartesianProduct<A,B> >,
public Testable<CartesianProduct<A,B> >
{};




/*
template<class A, class B>
struct traits<CartesianProduct<A,B> > :
public internal::Manifold<CartesianProduct<A,B> >
{};
*/



}