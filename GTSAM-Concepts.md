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

In GTSAM, all properties and operations needed to use a type must be defined through template specialization of the struct `gtsam::traits`. Concept checks are used to check that all required functions are implemented.
In detail, we ask that the following items are defined in the traits object (although, not all are needed for optimization):

* values:
   * `enum { dimension = D};`, an enum that indicates the dimensionality *n* of the manifold. In Eigen-fashion, we also support manifolds whose dimensionality is only defined at runtime, by specifying the value -1.
* types:
    * `TangentVector`, type that lives in tangent space. This will almost always be an `Eigen::Matrix<double,n,1>`.
    * `ChartJacobian`, a typedef for `OptionalJacobian<dimension, dimension>`.
    * `ManifoldType`, a pointer back to the type.
    * `structure_category`, a tag type that defines what requirements the type fulfills, and therefore what requirements this traits class must fulfill. It should be defined to be one of the following: 
        * `gtsam::traits::manifold_tag` -- Everything in this list is expected
        * `gtsam::traits::group_tag` -- The functions defined under **Groups** below.
        * `gtsam::traits::lie_group_tag` -- Everything in this list is expected, plus the functions defined under **Groups**, and **Lie Groups** below.
        * `gtsam::traits::vector_space_tag` -- Everything in this list is expected, plus the functions defined under **Groups**, and **Lie Groups** below.
* valid expressions:
    * `size_t dim = traits<T>::GetDimension(p);` static function should be defined. This is mostly useful if the size is not known at compile time.
    * `v = traits<T>::Local(p,q)`, the chart, from manifold to tangent space, think of it as *q (-) p*, where *p* and *q* are elements of the manifold and the result, *v* is an element of the vector space.
    * `p = traits<T>::Retract(p,v)`, the inverse chart, from tangent space to manifold, think of it as *p (+) v*, where *p* is an element of the manifold and the result, *v* is an element of the vector space.
* invariants
    * `Retract(p, Local(p,q)) == q`
    * `Local(p, Retract(p, v)) == v`

Group
-----
A [group]("http://en.wikipedia.org/wiki/Group_(mathematics)"") should be well known from grade school :-), and provides a type with a composition operation that is closed, associative, has an identity element, and an inverse for each element. The following should be added to the traits class for a group:

* valid expressions:
    * `r = traits<T>::Compose(p,q)`, where *p*, *q*, and *r* are elements of the manifold. 
    * `q = traits<T>::Inverse(p)`, where *p* and*q* are elements of the manifold. 
    * `r = traits<T>::Between(p,q)`, where *p*, *q*, and *r* are elements of the manifold. 
* static members:
    * `traits<T>::Identity`, a static const member that represents the group's identity element.
* invariants:
    * `Compose(p,Inverse(p)) == Identity`
    * `Compose(p,Between(p,q)) == q`
    * `Between(p,q) == Compose(Inverse(p),q)`
The `gtsam::group::traits` namespace defines the following:
* values:
    * `traits<T>::Identity` -- The identity element for this group stored as a static const.
    * `traits<T>::group_flavor` -- the flavor of this group's `compose()` operator, either:
         *  `gtsam::traits::group_multiplicative_tag` for multiplicative operator syntax ,or 
         * `gtsam::traits::group_additive_tag` for additive operator syntax.

We do *not* at this time support more than one composition operator per type. Although mathematically possible, it is hardly ever needed, and the machinery to support it would be burdensome and counter-intuitive. 

Also, a type should provide either multiplication or addition operators depending on the flavor of the operation. To distinguish between the two, we will use a tag (see below).

Lie Group
---------

A Lie group is both a manifold *and* a group. Hence, a LIE_GROUP type should implements both MANIFOLD and GROUP concepts. 
However, we now also need to be able to evaluate the derivatives of compose and inverse. 
Hence, we have the following extra valid static functions defined in the struct `gtsam::traits<T>`:

* `r = traits<T>::Compose(p,q,Hp,Hq)`
* `q = traits<T>::Inverse(p,Hp)`
* `r = traits<T>::Between(p,q,Hp,Hq)`

where above the *H* arguments stand for optional Jacobian arguments. 
That makes it possible to create factors implementing priors (PriorFactor) or relations between two instances of a Lie group type (BetweenFactor).

In addition, a Lie group has a Lie algebra, which affords two extra valid expressions:

* `v = traits<T>::Logmap(p,Hp)`, the log map, with optional Jacobian
* `p = traits<T>::Expmap(v,Hv)`, the exponential map, with optional Jacobian

Note that in the Lie group case, the usual valid expressions for Retract and Local can be generated automatically, e.g.

```
    T Retract(p,v,Hp,Hv) {
      T q = Expmap(v,Hqv);
      T r = Compose(p,q,Hrp,Hrq);
      Hv = Hrq * Hqv; // chain rule
      return r;
    }
```

For Lie groups, the `exponential map` above is the most obvious mapping: it 
associates straight lines in the tangent space with geodesics on the manifold 
(and it's inverse, the log map). However, there are several cases in which we deviate from this:

However, the exponential map is unnecessarily expensive for use in optimization. Hence, in GTSAM there is the option to provide a cheaper chart by means of the `ChartAtOrigin` struct in a class. This is done for *SE(2)*, *SO(3)* and *SE(3)* (see `Pose2`, `Rot3`, `Pose3`)

Most Lie groups we care about are *Matrix groups*, continuous sub-groups of *GL(n)*, the group of *n x n* invertible matrices. In this case, a lot of the derivatives calculations needed can be standardized, and this is done by the `LieGroup` superclass. You only need to provide an `AdjointMap` method.

A CRTP helper class `LieGroup` is available that can take a class and create some of the Lie group methods automatically. The class needs:

* operator* : implements group operator
* inverse: implements group inverse
* AdjointMap: maps tangent vectors according to group element
* Expmap/Logmap: exponential map and its inverse
* ChartAtOrigin: struct where you define Retract/Local at origin

To use, simply derive, but also say `using LieGroup<Class,N>::inverse` so you get an inverse with a derivative.

Finally, to create the traits automatically you can use `internal::LieGroupTraits<Class>`

Vector Space
------------

While vector spaces are in principle also manifolds, it is overkill to think about charts etc. Really, we should simply think about vector addition and subtraction. I.e.where

  * `Identity == 0`
  * `Inverse(p) == -p`
  * `Compose(p,q) == p+q`
  * `Between(p,q) == q-p`
  * `Local(q) == p-q`   
  * `Retract(v) == p+v`

This considerably simplifies certain operations. A `VectorSpace` superclass is available to implement the traits. Types that are vector space models include `Matrix`, `Vector`, any fixed or dynamic Eigen Matrix, `Point2`, and `Point3`.

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

`gtsam::traits` is our way to associate these concepts with types, 
and we also define a limited number of `gtsam::tags` to select the correct implementation
of certain functions at compile time (tag dispatching). 

Traits
------

However, a base class is not a good way to implement/check the other concepts, as we would like these
to apply equally well to types that are outside GTSAM control, e.g., `Eigen::VectorXd`. This is where
[traits](http://www.boost.org/doc/libs/1_57_0/libs/type_traits/doc/html/boost_typetraits/background.html) come in.

We use Eigen-style or STL-style traits, that define *many* properties at once. 

Note that not everything that makes a concept is defined by traits. Valid expressions such as traits<T>::Compose are
defined simply as static functions within the traits class.
Finally, for GTSAM types, it is perfectly acceptable (and even desired) to define associated types as internal types, 
rather than having to use traits internally.

Concept Checks
--------------

Boost provides a nice way to check whether a given type satisfies a concept. For example, the following

    GTSAM_CONCEPT_ASSERT(IsVectorSpace<Point2>)
    
asserts that Point2 indeed is a model for the VectorSpace concept.

Future Concepts
===============


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


Lie Group Action
----------------

When a Lie group acts on a space, we have two derivatives to care about:

  * `gtasm::manifold::traits<T>::act(g,p,Hg,Hp)`, if the space acted upon is a continuous differentiable manifold.

An example is a *similarity transform* in 3D, which can act on 3D space, like

    q = s*R*p + t
    
Note that again the derivative in *p*,  *Hp* is simply *s R*, which depends on *g* but not on *p*. 
The derivative  in *g*,  *Hg*, is in general more complex.

For now, we won't care about Lie groups acting on non-manifolds.
