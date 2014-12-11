GTSAM Concepts
==============

As discussed in [Generic Programming Techniques](http://www.boost.org/community/generic_programming.html), concepts define

* associated types
* valid expressions, like functions and values
* invariants
* complexity guarantees

Below we discuss the most important concepts use in GTSAM, and after that we discuss how they are implemented/used/enforced.


Manifold
--------

To optimize over continuous types, we assume they are manifolds. This is central to GTSAM and hence discussed in some more detail below.

[Manifolds](http://en.wikipedia.org/wiki/Manifold#Charts.2C_atlases.2C_and_transition_maps) and [charts](http://en.wikipedia.org/wiki/Manifold#Charts.2C_atlases.2C_and_transition_maps) are intimately linked concepts. We are only interested here in [differentiable manifolds](http://en.wikipedia.org/wiki/Differentiable_manifold#Definition), continuous spaces that can be locally approximated *at any point* using a local vector space, called the [tangent space](http://en.wikipedia.org/wiki/Tangent_space). A *chart* is an invertible map from the manifold to that tangent space.

In GTSAM, all properties and operations needed to use a type must be defined through template specialization of the struct  `gtsam::manifold::traits`. Concept checks are used to check that all required functions are implemented.
In detail, we ask the following are defined in the traits object:

* values:
    * `enum { dimension = D};`, an enum that indicates the dimensionality *n* of the manifold. In Eigen-fashion, we also support manifolds whose dimenionality is only defined at runtime, by specifying the value -1.
* types: 
    * `TangentVector`, type that lives in tangent space. This will almost always be an `Eigen::Matrix<double,n,1>`.
	* `ChartJacobian`, a typedef for `OptionalJacobian<dimension, dimension>`.  
	* `ManifoldType`, a pointer back to the type.
	* `structure_category`, a tag type that defines what requirements the type fulfills, and therefore what requirements this traits class must fulfill. It should be defined to be one of the following: 
		*  `gtsam::traits::manifold_tag` -- Everything in this list is expected
		* `gtsam::traits::group_tag` -- Everything in this list is expected, plus the functions defined under **Groups** below.
		* `gtsam::traits::lie_group_tag` -- Everything in this list is expected, plus the functions defined under **Groups**, and **Lie Groups** below.
		* `gtsam::traits::vector_space_tag` -- Everything in this list is expected, plus the functions defined under **Groups**, and **Lie Groups** below.
* valid expressions:
    * `size_t dim = traits<T>::getDimension(p);` static function should be defined. This is mostly useful if the size is not known at compile time.
    * `v = traits<T>::Local(p,q)`, the chart, from manifold to tangent space, think of it as *q (-) p*, where *p* and *q* are elements of the manifold and the result, *v* is an element of the vector space.
	* `v = traits<T>::Local(p,q, Hp, Hq)`.
    * `p = traits<T>::Retract(p,v)`, the inverse chart, from tangent space to manifold, think of it as *p (+) v*, where *p* is an element of the manifold and the result, *v* is an element of the vector space.
    * `p = traits<T>::Retract(p,v, Hp, Hv)`.

In the functions above, the `H` arguments stand for optional Jacobians. When provided, it is assumed
that the function will return the derivatives of the chart (and inverse) with respect to its arguments.
* invariants
	* `Retract(p, Local(p,q)) == q`
	* `Local(p, Retract(p, v)) == v`

For many differential manifolds, an obvious mapping is the `exponential map`, 
which  associates straight lines in the tangent space with geodesics on the manifold 
(and it's inverse, the log map). However, there are two cases in which we deviate from this:

* Sometimes, most notably for *SO(3)* and *SE(3)*, the exponential map is unnecessarily expensive for use in optimization. Hence, the `Local` and `Retract` refer to a chart that is much cheaper to evaluate.
* While vector spaces (see below) are in principle also manifolds, it is overkill to think about charts etc. Really, we should simply think about vector addition and subtraction. Hence, while a these functions are defined for every vector space, GTSAM will never invoke them. (IS THIS TRUE?)

Group
-----
A [group](http://en.wikipedia.org/wiki/Group_(mathematics)) should be well known from grade school :-), and provides a type with a composition operation that is closed, associative, has an identity element, and an inverse for each element. The following should be added to the traits class for a group:

* valid expressions:
    * `r = traits<M>::Compose(p,q)`, where *p*, *q*, and *r* are elements of the manifold. 
    * `q = traits<M>::Inverse(p)`, where *p* and*q* are elements of the manifold. 
    * `r = traits<M>::Between(p,q)`, where *p*, *q*, and *r* are elements of the manifold. 
* static members:
	* `traits<M>::Identity`, a static const member that represents the group's identity element.
* invariants:
    * `Compose(p,Inverse(p)) == Identity`
    * `Compose(p,Between(p,q)) == q`
    * `Between(p,q) == Compose(Inverse(p),q)`
The `gtsam::group::traits` namespace defines the following:
* values:
	* `traits<M>::Identity` -- The identity element for this group stored as a static const.
	* `traits<M>::group_flavor` -- the flavor of this group's `compose()` operator, either:
		*  `gtsam::traits::group_multiplicative_tag` for multiplicative operator syntax ,or 
		* `gtsam::traits::group_additive_tag` for additive operator syntax.

We do *not* at this time support more than one composition operator per type. Although mathematically possible, it is hardly ever needed, and the machinery to support it would be burdensome and counter-intuitive. 

Also, a type should provide either multiplication or addition operators depending on the flavor of the operation. To distinguish between the two, we will use a tag (see below).

Group Action
------------

Group actions are concepts in and of themselves that can be concept checked (see below).
In particular, a group can *act* on another space.
For example, the [cyclic group of order 6](http://en.wikipedia.org/wiki/Cyclic_group) can rotate 2D vectors around the origin:

    q = R(i)*p
    where R(i) = R(60)^i, where R(60) rotates by 60 degrees
    
Hence, we formalize by the following extension of the concept:

* valid expressions:
    * `q = traits<T>::Act(g,p)`, for some instance, *p*,  of a space *S*, that can be acted upon by the group element *g* to produce *q* in *S*.
    * `q = traits<T>::Act(g,p,Hp)`, if the space acted upon is a continuous differentiable manifold. *
  
In the latter case, if *S* is an n-dimensional manifold, *Hp* is an output argument that should be 
filled with the *nxn* Jacobian matrix of the action with respect to a change in *p*. It typically depends
on the group element *g*, but in most common example will *not* depend on the value of *p*. For example, in 
the cyclic group example above, we simply have

    Hp = R(i)
  
Note there is no derivative of the action with respect to a change in g. That will only be defined
for Lie groups, which we introduce now.

Lie Group
---------

A Lie group is both a manifold *and* a group. Hence, a LIE_GROUP type should implements both MANIFOLD and GROUP concepts. 
However, we now also need to be able to evaluate the derivatives of compose and inverse. 
Hence, we have the following extra valid static functions defined in the struct `gtsam::manifold::traits<M>`:

* `r = traits<M>::Compose(p,q,Hq,Hp)`
* `q = traits<M>::Inverse(p,Hp)`
* `r = traits<M>::Between(p,q,Hq,H2p)`

where above the *H* arguments stand for optional Jacobian arguments. 
That makes it possible to create factors implementing priors (PriorFactor) or relations between 
two instances of a Lie group type (BetweenFactor).

In addition, a Lie group has a Lie algebra, which affords two extra valid expressions:

* `v = Chart::Log(p,Hp)`, the log map, with optional Jacobian
* `p = Chart::Exp(v,Hv)`, the exponential map, with optional Jacobian

Note that in the Lie group case, the usual valid expressions for Retract and Local can be generated automatically, e.g.

    T Retract(p,v,Hp,Hv) {
      T q = Exp(v,Hqv);
      T r = Compose(p,q,Hrp,Hrq);
      Hv = Hrq * Hqv; // chain rule
      return r;
    }

Lie Group Action
----------------

When a Lie group acts on a space, we have two derivatives to care about:

  * `gtasm::manifold::traits<M>::act(g,p,Hg,Hp)`, if the space acted upon is a continuous differentiable manifold.

An example is a *similarity transform* in 3D, which can act on 3D space, like

    q = s*R*p + t
    
Note that again the derivative in *p*,  *Hp* is simply *s R*, which depends on *g* but not on *p*. 
The derivative  in *g*,  *Hg*, is in general more complex.

For now, we won't care about Lie groups acting on non-manifolds.

Matrix Group
------------

Most Lie groups we care about are *Matrix groups*, continuous sub-groups of *GL(n)*, the group of *n x n* invertible matrices.
In this case, a lot of the derivatives calculations needed can be standardized.

Vector Space
------------

Trivial Lie group where

  * `Identity == 0`
  * `Inverse(p) == -p`
  * `Compose(p,q) == p+q`
  * `Between(p,q) == q-p`
  * `Local(q) == p-q`   
  * `Retract(v) == p+v`

This considerably simplifies certain operations.

Testable
--------
Unit tests heavily depend on the following two functions being defined for all types that need to be tested:

* valid expressions:
    * `Print(p,s)` where s is an optional string
    * `Equals(p,q,tol)` where tol is an optional (double) tolerance 

Implementation
==============

GTSAM Types start with Uppercase, e.g., `gtsam::Point2`, and are models of the 
TESTABLE, MANIFOLD, GROUP, LIE_GROUP, and VECTOR_SPACE concepts.

`gtsam::manifold::traits` is our way to associate these concepts with types, 
and we also define a limited number of `gtsam::tags` to select the correct implementation
of certain functions at compile time (tag dispatching). 

Traits
------

However, a base class is not a good way to implement/check the other concepts, as we would like these
to apply equally well to types that are outside GTSAM control, e.g., `Eigen::VectorXd`. This is where
[traits](http://www.boost.org/doc/libs/1_57_0/libs/type_traits/doc/html/boost_typetraits/background.html) come in.

We will use Eigen-style or STL-style traits, that define *many* properties at once. 

Note that not everything that makes a concept is defined by traits. Valid expressions such as traits<T>::Compose are
defined simply as static functions within the traits class.
Finally, for GTSAM types, it is perfectly acceptable (and even desired) to define associated types as internal types, 
rather than having to use traits internally.


** THE EXAMPLES ARE NOT UPDATED YET **

Manifold Example
----------------

An example of implementing a Manifold type is here:

```
#!c++
    // GTSAM type
    class Rot2 {
	  typedef Vector2 TangentVector; // internal typedef, not required
      class Chart : gtsam::Chart<Rot2,Chart>{
	    static TangentVector local(const Rot2& R1, const Rot2& R2);
	    static Rot2 retract(const Rot2& R, const TangentVector& v);
      }
      Rot2 operator*(const Rot2&) const;
      Rot2 transpose() const;
    }

    namespace gtsam { namespace traits {
      
      template<>
      struct dimension<Rot2> {
        static const int value = 2;      
        typedef int value_type;      
      }

      template<>
      struct TangentVector<Rot2> {
        typedef Rot2::TangentVector type;      
      }

      template<>
      struct defaultChart<Rot2> : Rot2::Chart {}

    }}
```

But Rot2 is in fact also a Lie Group, after we define

```
#!c++
namespace manifold { 
    Rot2 inverse(const Rot2& p) { return p.transpose();}
    Rot2 operator*(const Rot2& p, const Rot2& q) { return p*q;}
    Rot2 compose(const Rot2& p, const Rot2& q) { return p*q;}
    Rot2 between(const Rot2& p, const Rot2& q) { return inverse(p)*q;}
}
```

The only traits that needs to be implemented are the tags: 


```
#!c++
namespace gtsam { 
    namespace traits {
      
      template<>
      struct structure<Rot2> : lie_group_tag {}

      template<>
      struct group_flavor<Rot2> : multiplicative_group_tag {}

    }
}
```

Group Example
-------------

As an example of a group, let's do a cyclic group, acting on Point2:

```
#!c++
    // GTSAM type
    class Cyclic {
      CyclicGroup(size_t n) {}
      Cyclic operator+(const Cyclic&) const; // add modulo n
      Cyclic operator-() const; // negate modulo n
      
      // Act on R2, rotate by i*360/n, derivative will simply be 2*2 rotation matrix
      Point2 operator*(cons Point2& p, OptionalJacobian<2,2> H) const;
    }

    namespace group {
      // make Cyclic obey GROUP concept
      Cyclic compose(const Cyclic& g, const Cyclic& h) { return g+h;}
      Cyclic between(const Cyclic& g, const Cyclic& h) { return h-g;}
      Cyclic inverse(const Cyclic& g) { return -p;}
      
      // implement acting on 2D
      Vector2 act(Const Cyclic& g, cons Point2& p) { return g*p;}
    }

```


Lie Group Example
-----------------

As an example of a Lie group, let's do a similarity transform, acting on Point3:

```
#!c++
    // GTSAM type
    class Similarity3 {
    
      ... constructors and Manifold stuff...
      
      Similarity3 compose(const Similarity3&, OptionalJacobian<7,7> H1, OptionalJacobian<7,7> H2) const;
      Similarity3 between(const Similarity3&, OptionalJacobian<7,7> H1, OptionalJacobian<7,7> H2) const;
      Similarity3 inverse(OptionalJacobian<7,7> H) const; // matrix inverse
      
      Similarity3 operator*(const Similarity3& g) const { return compose(h);} // compose sugar

      // Act on R3
      Point3 act(cons Point3& p, OptionalJacobian<3,7> Hg, OptionalJacobian<3,3> Hp) const {
        if (Hg) *Hg << - s * R * [p], s * R, R * p; // TODO check !
        if (Hp) *Hp = s*R;
        return s*R*p + t;
      }
      Point3 operator*(cons Point3& p); // act sugar
    }

    namespace group {
      // make Similarity3 obey GROUP concept
      Similarity3 compose(const Similarity3& g, const Similarity3& h, 
          OptionalJacobian<7,7> H1, OptionalJacobian<7,7> H2) { return g.operator*(p,H1,H2);}
      Similarity3 between(const Similarity3& g, const Similarity3& h, 
          OptionalJacobian<7,7> H1, OptionalJacobian<7,7> H2) { return g.between(h,H1,H2);}
      Similarity3 inverse(const Similarity3& g, OptionalJacobian<7,7> H) { return p.inverse(H);}
      
      // implement acting on 3D
      Vector2 act(Const Similarity3& g, cons Point3& p, 
          OptionalJacobian<3,7> Hg, OptionalJacobian<3,3> Hp) { return g.act(p,Hg,Hp);}
    }

```


Vector Space Example
--------------------

Providing the Vector space concept is easier:


```
#!c++
    // GTSAM type
    class Point2 {
      static const int dimension = 2;
      Point2 operator+(const Point2&) const;
      Point2 operator-(const Point2&) const;
      Point2 operator-() const;
      // Still needs to be defined, unless Point2 *is* Vector2
      class Chart {
	    static Vector2 local(const Point2& p, const Point2& q) const;
	    static Rot2 retract(const Point2& p, const Point2& v) const;
      }
    }
```

 The following macro, called inside the gtsam namespace,
 
	DEFINE_VECTOR_SPACE_TRAITS(Point2)
	
 should automatically define 
 
```
#!c++
namespace traits {
      
      template<>
      struct dimension<Point2> {
        static const int value = Point2::dimension;      
        typedef int value_type;      
      }

      template<>
      struct TangentVector<Point2> {
        typedef Matrix::Eigen<double,Point2::dimension,1> type;      
      }

      template<>
      struct defaultChart<Point2> : Point2::Chart {}

      template<>
      struct Manifold<Point2::Chart> {
        typedef Point2 type;      
      }

      template<>
      struct structure<Point2> : vector_space_tag {}

      template<>
      struct group_flavor<Point2> : additive_group_tag {}
}
```
and

```
#!c++
namespace manifold { 
    Point2 inverse(const Point2& p) { return -p;}
    Point2 operator+(const Point2& p, const Point2& q) { return p+q;}
    Point2 compose(const Point2& p, const Point2& q) { return p+q;}
    Point2 between(const Point2& p, const Point2& q) { return q-p;}
}
```

Concept Checks
--------------

Boost provides a nice way to check whether a given type satisfies a concept. For example, the following

    BOOST_CONCEPT_ASSERT(ChartConcept<gtsam::traits::defaultChart<Point2> >)
    
Using the following from Mike Bosse's prototype:


```
#!c++
template<class C>
struct ChartConcept {

  typedef gtsam::traits::Manifold<C>::type type;
  typedef gtsam::traits::TangentVector<type>::type vector;

  BOOST_CONCEPT_USAGE(ChartConcept) {
  
    // Returns Retraction update of val_
    type retract_ret = C::retract(val_, vec_);

    // Returns local coordinates of another object
    vec_ = C::local(val_, retract_ret);
  }

private:
  type val_;
  vector vec_;
  int dim_;
};

```
