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

In GTSAM we assume that a manifold type can yield such a *Chart* at any point, and we require that a functor `defaultChart` is available that, when called for any point on the manifold, returns a Chart type. Hence, the functor itself can be seen as an *Atlas*.

In detail, we ask the following are defined for a MANIFOLD type:

* values:
    * `dimension`, an int that indicates the dimensionality *n* of the manifold. In Eigen-fashion, we also support manifolds whose dimenionality is only defined at runtime, by specifying the value -1.
* functors:
	* `defaultChart`, returns the default chart at a point p
* types: 
    * `TangentVector`, type that lives in tangent space. This will almost always be an `Eigen::Matrix<double,n,1>`.
* valid expressions:
    * `size_t dim = getDimension(p);` free function should be defined in case the dimension is not known at compile time.

Anything else?

Chart
-----
A given chart is implemented using a small class that defines the chart itself (from manifold to tangent space) and its inverse.

* types:
  * `ManifoldType`, a pointer back to the type
* valid expressions: 
  * `v = Chart::Local(p,q)`, the chart, from manifold to tangent space, think of it as *q (-) p*
  * `p = Chart::Retract(p,v)`, the inverse chart, from tangent space to manifold, think of it as *p (+) v*

For many differential manifolds, an obvious mapping is the `exponential map`, which  associates straight lines in the tangent space with geodesics on the manifold (and it's inverse, the log map). However, there are two cases in which we deviate from this:

  * Sometimes, most notably for *SO(3)* and *SE(3)*, the exponential map is unnecessarily expensive for use in optimization. Hence, the `defaultChart` functor returns a chart that is much cheaper to evaluate.
  * While vector spaces (see below) are in principle also manifolds, it is overkill to think about charts etc. Really, we should simply think about vector addition and subtraction. Hence, while a `defaultChart` functor is defined by default for every vector space, GTSAM will never call it.


Group
-----
A [group](http://en.wikipedia.org/wiki/Group_(mathematics)) should be well known from grade school :-), and provides a type with a composition operation that is closed, associative, has an identity element, and an inverse for each element.

* values:
  * `group::identity<G>()`
* valid expressions:
  * `group::compose(p,q)`
  * `group::inverse(p)`
  * `group::between(p,q)`
* invariants (using namespace group):
  * `compose(p,inverse(p)) == identity`
  * `compose(p,between(p,q)) == q`
  * `between(p,q) == compose(inverse(p),q)`
  
We do *not* at this time support more than one composition operator per type. Although mathematically possible, it is hardly ever needed, and the machinery to support it would be burdensome and counter-intuitive. 

Also, a type should provide either multiplication or addition operators depending on the flavor of the operation. To distinguish between the two, we will use a tag (see below).

Group Action
------------

A group can *act* on another space. For example, a *similarity transform* in 3D can act on 3D space, like

    q = s*R*p + t
    
Even finite groups can act on continuous entities. For example, the [cyclic group of order 6](http://en.wikipedia.org/wiki/Cyclic_group) can rotate 2D vectors around the origin:

    q = R(i)*p
    where R(i) = R(60)^i, where R(60) rotates by 60 degrees
    
Hence, we formalize by the following extension of the concept:

* valid expressions:
  * `group::act(g,t)`, for some instance of a space T, that can be acted upon by the group
  * `group::act(g,t,H)`, if the space acted upon is a continuous differentiable manifold
  
Group actions are concepts in and of themselves that can be concept checked (see below).
  
Lie Group
---------

A Lie group is both a manifold *and* a group. Hence, a LIE_GROUP type should implements both MANIFOLD and GROUP concepts. However, we now also need to be able to evaluate the derivatives of compose and inverse. Hence, we have the following extra valid expressions:

* `compose(p,q,H1,H2)`
* `inverse(p,H)`
* `between(p,q,H1,H2)`

where above the `H` arguments stand for optional Jacobian arguments. That makes it possible to create factors implementing priors (PriorFactor) or relations between two instances of a Lie group type (BteweenFactor).

Lie Group Action
----------------

When a Lie group acts on a space, we have two derivatives to care about:

  * `group::act(g,t,Hg,Ht)`, if the space acted upon is a continuous differentiable manifold

For now, we won't care about Lie groups acting on non-manifolds.

Matrix Group
------------

Most Lie groups we care about are *Matrix groups*, continuous sub-groups of *GL(n)*, the group of nxn invertible matrices.
In this case, a lot of the derivatives calculations needed can be standardized.

Vector Space
------------

Trivial Lie group where

  * `identity<T> == 0`
  * `inverse(p) == -p`
  * `compose(p,q) == p+q`
  * `between(p,q) == q-p`
  * `chart::retract(q) == p-q`   
  * `chart::retract(v) == p+v`

This considerably simplifies certain operations.

Testable
--------
Unit tests heavily depend on the following two functions being defined for all types that need to be tested:

* valid expressions:
  * `print(p,s)` where s is an optional string
  * `equals(p,q,tol)` where tol is an optional tolerance 

Implementation
==============

GTSAM Types start with Uppercase, e.g., `gtsam::Point2`, and are models of the 
TESTABLE, MANIFOLD, GROUP, LIE_GROUP, and VECTOR_SPACE concepts.

`gtsam::traits` is our way to associate these concepts with types, 
and we also define a limited number of `gtsam::tags` to select the correct implementation
of certain functions at compile time (tag dispatching). Charts are done more conventionally, so we start there...

Traits
------

However, a base class is not a good way to implement/check the other concepts, as we would like these
to apply equally well to types that are outside GTSAM control, e.g., `Eigen::VectorXd`. This is where
[traits](http://www.boost.org/doc/libs/1_57_0/libs/type_traits/doc/html/boost_typetraits/background.html) come in.

We will not use Eigen-style or STL-style traits, that define *many* properties at once. 
Rather, we use boost::mpl style meta-programming functions to facilitate meta-programming, 
which return a single type or value for every trait. Some rationale/history can be 
found [here](http://www.boost.org/doc/libs/1_55_0/libs/type_traits/doc/html/boost_typetraits/background.html).
as well. 

Note that not everything that makes a concept is defined by traits. Valid expressions such as group::compose are
defined simply as free functions.
Finally, for GTSAM types, it is perfectly acceptable (and even desired) to define associated types as internal types, 
rather than having to use traits internally.

The conventions for `gtsam::traits` are as follows:

* Types: `gtsam::traits::SomeAssociatedType<T>::type`, i.e., they are MixedCase and define a *single* `type`, for example:
    
      template<>
      gtsam::traits::TangentVector<Point2> {
        typedef Vector2 type;
      }
    
* Values: `gtsam::traits::someValue<T>::value`, i.e., they are mixedCase starting with a lowercase letter and define a `value`, *and* a `value_type`. For example:
    
      template<>
      gtsam::traits::dimension<Point2> {
        static const int value = 2;
        typedef const int value_type; // const ?
      }
    
* Functors: `gtsam::traits::someFunctor<T>::type`, i.e., they are mixedCase starting with a lowercase letter and define a functor (i.e., no *type*). The functor itself should define a `result_type`. A contrived example
    
     struct Point2::manhattan {
        typedef double result_type;
        Point2 p_;
        manhattan(const Point2& p) : p_(p) {}
        Point2 operator()(const Point2& q) {
          return abs(p_.x()-q.x()) + abs(p_.y()-q.x());
        }
      }
      
    template<> gtsam::traits::manhattan<Point2> : Point2::manhattan {}

  By *inherting* the trait from the functor, we can just use the [currying](http://en.wikipedia.org/wiki/Currying) style `gtsam::traits::manhattan<Point2>℗(q)`. Note that, although technically a functor is a type, in spirit it is a free function and hence starts with a lowercase letter.
  
* Tags: `gtsam::traits::some_category<T>::type`, i.e., they are lower_case and define a *single* `type`, for example:
    
      template<>
      gtsam::traits::structure_category<Point2> {
        typedef vector_space_tag type;
      }
   
   See below for the tags defined within GTSAM.

Tags
----
  
Algebraic structure concepts are associated with the following tags

* `gtsam::traits::manifold_tag`
* `gtsam::traits::group_tag`
* `gtsam::traits::lie_group_tag`
* `gtsam::traits::vector_space_tag`

which should be queryable by `gtsam::traits::structure_category<T>::type`

The group composition operation can be of two flavors:

* `gtsam::traits::additive_group_tag`
* `gtsam::traits::multiplicative_group_tag`

which should be queryable by `gtsam::traits::group_flavor<T>::type`

A tag can be used for [tag dispatching](http://www.boost.org/community/generic_programming.html#tag_dispatching),
e.g., below is a generic compose:

```
#!c++
  namespace detail {
    template <class T>
    T compose(const T& p, const T& q, additive_group_tag) {
      return p + q;
    }

    template <class T>
    T compose(const T& p, const T& q, multiplicative_group_tag) {
      return p * q;
    }
  }

  template <T>
  T compose(const T& p, const T& q) {
    return detail::compose(p, q, traits::group_flavor<T>::type);
  }
```

Tags also facilitate meta-programming. Taking a leaf from [The boost Graph library](http://www.boost.org/doc/libs/1_40_0/boost/graph/graph_traits.hpp),
tags can be used to create useful meta-functions, like `is_lie_group`, below.

```
#!c++
    template <typename T>
    struct is_lie_group
        : mpl::bool_<
            is_convertible<
                typename structure_category<T>::type,
                lie_group_tag
            >::value
        >
    { };
```

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
