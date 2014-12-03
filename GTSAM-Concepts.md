GTSAM Concepts
==============

Concepts define (see [Generic Programming Techniques](http://www.boost.org/community/generic_programming.html))

* associated types
* valid expressions
* invariants
* complexity guarantees

GTSAM Types start with Uppercase, e.g., `gtsam::Point2`, and are models of the concepts MANIFOLD, GROUP, LIE_GROUP, VECTOR_SPACE

traits
------

`gtsam::traits` is our way to associate these concepts with types. We will not use Eigen-style or STL-style traits, that define many properties at once. Rather, we use boost::mpl style meta-programming functions to facilitate meta-programming.

Traits allow us to play with types that are outside GTSAM control, e.g., `Eigen::VectorXd`.

The naming conventions are as follows:

* Types: `gtsam::traits::SomeAssociatedType<T>::type`, i.e., they are MixedCase and define a `type`, for example:
    
      template<>
      gtsam::traits::TangentVector<Point2> {
        typedef Vector2 type;
      }
    
* Values: `gtsam::traits::someValue<T>::value`, i.e., they are mixedCase starting with a lowercase letter and define a `value`, but also a `value_type`. For example:
    
      template<>
      gtsam::traits::dimension<Point2> {
        static const int value = 2;
        typedef const int value_type; // const ?
      }
    
* Functors: `gtsam::traits::someFunctor<T>::type`, i.e., they are mixedCase starting with a lowercase letter and define a functor `type`. Example
    
      struct Point2::retract {
        Point2 p_;
        retract(const Point2& p) : p_(p) {}
        Point2 operator()(const Vector2& v) {
          return Point2(p.x()+v[0], p.y()+v[1]);
        }
      }
    
      template<>
      gtsam::traits::retract<Point2> {
        typedef Point2::retract type;
      }
    
  The above is still up in the air. Do we need the type indirection?
  
tags
----
  
Concepts are associated with a tag.

* `gtsam::tags::manifold_tag`
* `gtsam::tags::group_tag`
* `gtsam::tags::lie_group_tag`
* `gtsam::tags::vector_space_tag`

Can be queried `gtsam::traits::structure_tag<T>`


Manifold
--------

* types: `DefaultChart`
* values: `dimension`

Anything else?

Chart
-----

* types: `Manifold`, `Vector`
* values: `retract`, `local`

Are these values? They are just methods. Anything else?


Group
-----

* values: `identity`
* values: `compose`, `inverse`, (`between`)

Lie Group
---------

Implements both MANIFOLD and GROUP

Vector Space
------------

Lie Group where compose == `+`

Examples
--------

An example of implementing a Manifold is here:


    namespace gtsam {
      namespace traits {
      
      // types:
      template<typename T>
      struct SomeAssociatedType<T>::type
      
      }
    }



