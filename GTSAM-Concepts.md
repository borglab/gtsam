GTSAM Concepts
==============

Concepts define (see [Generic Programming Techniques](http://www.boost.org/community/generic_programming.html))

* associated types
* valid expressions
* invariants
* complexity guarantees

GTSAM Types start with Uppercase, e.g., `gtsam::Point2`, and are models of the concepts MANIFOLD, GROUP, LIE_GROUP, VECTOR_SPACE

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
    
* Functors: `gtsam::traits::someFunctor<T>`, i.e., they are mixedCase starting with a lowercase letter and define a functor type
    
      template<>
      gtsam::traits::retract<Point2> {
        Point2 operator()(const Point2& p, const Vector2& v) {
        	return Point2(p.x()+v[0], p.y()+v[1]);
        }
      }
    
  The above is still up in the air, can we not point to a function somewhere? Or a method?
  
Concepts are associated with a tag.

Manifold
--------

`gtsam::tags::manifold_tag`

`gtsam::traits::structure_tag<T>`

* associated types: DefaultChart
* valid expressions: dimension

Chart
-----


Group
-----

Lie Group
---------

Vector Space
------------

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



