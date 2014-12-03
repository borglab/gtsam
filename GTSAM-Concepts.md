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
    
* Functors: `gtsam::traits::someFunctor<T>::type`, i.e., they are mixedCase starting with a lowercase letter and define a functor `type`. The funcor itself should define a `result_type`. Example
    
      struct Point2::retract {
        typedef Point2 result_type;
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
    
  The above is still up in the air. Do we need the type indirection? Could we just inherit the trait like so
    
      template<>
      gtsam::traits::retract<Point2> : Point2::retract {
      }
  
  In which case we could just say `gtsam::traits::retract<Point2>(p)(v)`.
  
tags
----
  
Concepts are associated with a tag.

* `gtsam::tags::manifold_tag`
* `gtsam::tags::group_tag`
* `gtsam::tags::lie_group_tag`
* `gtsam::tags::vector_space_tag`

Can be queried `gtsam::traits::structure_tag<T>`


Testable
--------

* values: `print`, `equals`

Anything else?

Manifold
--------

[Manifolds](http://en.wikipedia.org/wiki/Manifold#Charts.2C_atlases.2C_and_transition_maps) and [charts](http://en.wikipedia.org/wiki/Manifold#Charts.2C_atlases.2C_and_transition_maps) are intimately linked concepts. We are only interested here in [differentiable manifolds](http://en.wikipedia.org/wiki/Differentiable_manifold#Definition), continuous spaces that can be locally approximated *at any point* using a local vector space, called the [tangent space](http://en.wikipedia.org/wiki/Tangent_space). A chart is an invertible map from the manifold to the vector space.

In GTSAM we assume that a manifold type can yield such a chart at any point, and we require that a functor `defaultChart` is available that 

* values: `dimension`
* functors `defaultChart`
* types: `DefaultChart` is the *type* of chart returned by the functor `defaultChart`
* invariants: `defaultChart::result_type == DefaultChart::type`

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

    // GTSAM type
    class Rot2 {
	  ...
      class Chart {
	    ...
      }
    }

    namespace gtsam {
      namespace traits {
      
      template<>
      struct DefaultChart<Rot2> {
        typedef Rot2::Chart type;      
      }

      template<>
      struct Manifold<Rot2::Chart> {
        typedef Rot2 type;      
      }

      template<>
      struct Vector<Rot2::Chart> {
        typedef Vector2 type;      
      }
    }



