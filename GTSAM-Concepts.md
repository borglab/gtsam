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

[Manifolds](http://en.wikipedia.org/wiki/Manifold#Charts.2C_atlases.2C_and_transition_maps) and [charts](http://en.wikipedia.org/wiki/Manifold#Charts.2C_atlases.2C_and_transition_maps) are intimately linked concepts. We are only interested here in [differentiable manifolds](http://en.wikipedia.org/wiki/Differentiable_manifold#Definition), continuous spaces that can be locally approximated *at any point* using a local vector space, called the [tangent space](http://en.wikipedia.org/wiki/Tangent_space). A *chart* is an invertible map from the manifold to the vector space.

In GTSAM we assume that a manifold type can yield such a chart at any point, and we require that a functor `defaultChart` is available that, when called for any point on the manifold, returns a Chart type. Hence, the functor itself can be seen as an *Atlas*.

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
  * `Manifold`, a pointer back to the type
* valid expressions: 
  * `Chart chart(p)` constructor
  * `v = chart.local(q)`, the chart, from manifold to tangent space, think of it as *p (-) q*
  * `p = chart.retract(v)`, the inverse chart, from tangent space to manifold, think of it as *p (+) v*

For many differential manifolds, an obvious mapping is the `exponential map`, which  associates straight lines in the tangent space with geodesics on the manifold (and it's inverse, the log map). However, there are two cases in which we deviate from this:

  * Sometimes, most notably for *SO(3)* and *SE(3)*, the exponential map is unnecessarily expensive for use in optimization. Hence, the `defaultChart` functor returns a chart that is much cheaper to evaluate.
  * While vector spaces (see below) are in principle also manifolds, it is overkill to think about charts etc. Really, we should simply think about vector addition and subtraction. Hence, while a `defaultChart` functor is defined by default for every vector space, GTSAM will never call it.


Group
-----
A [group](http://en.wikipedia.org/wiki/Group_(mathematics)) should be well known from grade school :-), and provides a type with a composition operation that is closed, associative, has an identity element, and an inverse for each element.

* values:
  * `identity`
* valid expressions:
  * `compose(p,q)`
  * `inverse(p)`
  * `between(p,q)`
* invariants:
  * `compose(p,inverse(p)) == identity`
  * `compose(p,between(p,q)) == q`
  * `between(p,q) == compose(inverse(p),q)`
  
We do *not* at this time support more than one composition operator per type. Although mathematically possible, it is hardly ever needed, and the machinery to support it would be burdensome and counter-intuitive. 

Also, a type should provide either multiplication or addition operators depending on the flavor of the operation. To distinguish between the two, we will use a tag (see below).

Lie Group
---------

Implements both MANIFOLD and GROUP

Vector Space
------------

Trivial Lie Group where

  * `identity == 0`
  * `inverse(p) == -p`
  * `compose(p,q) == p+q`
  * `between(p,q) == q-p`
  * `chart.retract(q) == p-q`   
  * `chart.retract(v) == p+v`

This considerably simplifies certain operations.

Testable
--------
Unit tests heavily depend on the following two functions being defined for all types that need to be tested:

* valid expressions:
  * `print(p,s)` where s is an optional string
  * `equals(p,q,tol)` where tol is an optional tolerance 

Implementation
==============

GTSAM Types start with Uppercase, e.g., `gtsam::Point2`, and are models of the TESTABLE, MANIFOLD, GROUP, LIE_GROUP, and VECTOR_SPACE concepts.

`gtsam::traits` is our way to associate these concepts with types, and we also define a limited number of `gtsam::tags` to select the correct implementation of certain functions at compile time (tag dispatching).

Traits
------

We will not use Eigen-style or STL-style traits, that define *many* properties at once. Rather, we use boost::mpl style meta-programming functions to facilitate meta-programming, which return a single type or value for every trait.

Traits allow us to play with types that are outside GTSAM control, e.g., `Eigen::VectorXd`. However, for GTSAM types, it is perfectly acceptable (and even desired) to define associated types as internal types, as well, rather than having to use traits internally.

Finally, note that not everything that makes a concept is defined by traits. For example, although a CHART type is supposed to have a `retract` function, there is no trait for this: rather, the  

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
    
      template<>
      gtsam::traits::manhattan<Point2> : Point2::manhattan {}
  
  By *inherting* the trait from the functor, we can just use the [currying](http://en.wikipedia.org/wiki/Currying) style `gtsam::traits::manhattan<Point2>℗(q)`. Note that, although technically a functor is a type, in spirit it is a free function and hence starts with a lowercase letter.
  
Tags
----
  
Algebraic structure concepts are associated with the following tags

* `gtsam::traits::manifold_tag`
* `gtsam::traits::group_tag`
* `gtsam::traits::lie_group_tag`
* `gtsam::traits::vector_space_tag`

which should be queryable by `gtsam::traits::structure<T>`

The group composition operation can be of two flavors:

* `gtsam::traits::additive_group_tag`
* `gtsam::traits::multiplicative_group_tag`

which should be queryable by `gtsam::traits::group_flavor<T>`


Manifold Example
----------------

An example of implementing a Manifold type is here:

```
#!c++
    // GTSAM type
    class Rot2 {
	  typedef Vector2 TangentVector;
      class Chart {
	    Chart(const Rot2& R);
	    TangentVector local(const Rot2& R) const;
	    Rot2 retract(const TangentVector& v) const;
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

      template<>
      struct Manifold<Rot2::Chart> {
        typedef Rot2 type;      
      }

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
	    Chart(const Rot2& R);
	    Vector2 local(const Rot2& R) const;
	    Rot2 retract(const Vector2& v) const;
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

    BOOST_CONCEPT_ASSERT(ChartConcept<defaultChart>)
    
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
