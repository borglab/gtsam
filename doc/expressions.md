# Expressions
## Motivation
GTSAM is an optimization library for objective functions expressed as a factor graph over a set of unknown variables. In the continuous case, the variables are typically vectors or elements on a manifold (such as the 3D rotation manifold). The factors compute vector-valued errors that need to be minimized, and are typically only connected to a handful of unknowns.

In the continuous case, the main optimization methods we have implemented are variants of Gauss-Newton non-linear optimization or conjugate gradient methods. Let us assume there are m factors over n unknowns. For either optimization method, we need to evaluate the sparse Jacobian matrix of the entire factor graph, which is a sparse block-matrix of m block-rows and n-block columns.

The sparse Jacobian is built up factor by factor, corresponding to the block-rows. A typical non-linear least-square term is $|h(x)-z|^2$ where $h(x)$ is a measurement function, which we need to be able to linearize as
$$
h(x) \approx h(x_0+dx)+H(x_0)dx
$$
Note the above is for vector unknowns, for Lie groups and manifold variables, see [doc/math.pdf](https://github.com/borglab/gtsam/blob/develop/doc/math.pdf) for details.

## Expressions
In many cases one can use GTSAM 4 Expressions to implement factors. Expressions are objects of type Expression<T>, and there are three main expression flavors:

- constants, e.g., `Expression<Point2> kExpr(Point2(3,4))`
- unknowns, e.g., `Expression<Point3> pExpr(123)` where 123 is a key.
- functions, e.g., `Expression<double> sumExpr(h, kexpr, pExpr)`

The latter case is an example of wrapping a binary measurement function `h`. To be able to wrap `h`, it needs to be able to compute its local derivatives, i.e., it has to have the signature
```c++
double h(const Point2& a, const Point3& b, 
         OptionalJacobian<1, 2> Ha, OptionalJacobian<1, 3> Hb)
```
In this case the output type 'T' is 'double', the two arguments have type Point2 and Point3 respectively, and the two remaining arguments provide a way to compute the function Jacobians, if needed. The templated type `OptionalJacobian<M,N>` behaves very much like `std::optional<Eigen::Matrix<double,M,N>`. If an actual matrix is passed in, the function is expected to treat it as an output argument in which to write the Jacobian for the result wrp. the corresponding input argument. *The matrix to write in will be allocated before the call.*

Expression constructors exist for both methods and functions with different arities. Note that an expression is templated with the output type T, not with the argument types. However, the constructor will infer the argument types from inspecting the signature of the function f, and will in this example expect two additional arguments of type Expression<Point2> and Expression<Point3>, respectively.

As an example, here is the constructor declaration for wrapping unary functions:
```c++
template<typename A>
Expression(typename UnaryFunction<A>::type function,
    const Expression<A>& expression);
```
where (in this case) the function type is defined by
```c++
template<class A1>
struct UnaryFunction {
typedef boost::function<
    T(const A1&, typename MakeOptionalJacobian<T, A1>::type)> type;
};
```
## Some measurement function examples
An example of a simple unary function is `gtsam::norm3` in [Point3.cpp](https://github.com/borglab/gtsam/blob/develop/gtsam/geometry/Point3.cpp#L41):
```c++
double norm3(const Point3 & p, OptionalJacobian<1, 3> H = {}) {
  double r = sqrt(p.x() * p.x() + p.y() * p.y() + p.z() * p.z());
  if (H) *H << p.x() / r, p.y() / r, p.z() / r;
  return r;
}
```
The key new concept here is OptionalJacobian, which acts like a std::optional: if it evaluates to true, you should write the Jacobian of the function in it. It acts as a fixed-size Eigen matrix.

As we said above, expressions also support binary functions, ternary functions, and methods. An example of a binary function is 'Point3::cross':

```c++
Point3 cross(const Point3 &p, const Point3 & q,
    OptionalJacobian<3, 3> H1 = {}, OptionalJacobian<3, 3> H2 = {}) {
  if (H1) *H1 << skewSymmetric(-q.x(), -q.y(), -q.z());
  if (H2) *H2 << skewSymmetric(p.x(), p.y(), p.z());
  return Point3(p.y() * q.z() - p.z() * q.y(), p.z() * q.x() - p.x() * q.z(),  p.x() * q.y() - p.y() * q.x());
}
```
Example of using cross:
```c++
using namespace gtsam;
Matrix3 H1, H2;
Point3 p(1,2,3), q(4,5,6), r = cross(p,q,H1,H2);
```
## Using Expressions for Inference
The way expressions are used is by creating unknown Expressions for the unknown variables we are optimizing for:
```c++
Expression<Point3> x(‘x’,1);
auto h = Expression<Point3>(& norm3, x);
```
For convenient creation of factors with expressions, we provide a new factor graph type `ExpressionFactorGraph`, which is just a `NonlinearFactorGraph` with an extra method addExpressionFactor(h, z, n) that takes a measurement expression h, an actual measurement z, and a measurement noise model R. With this, we can add a GTSAM nonlinear factor $|h(x)-z|^2$ to a `NonlinearFactorGraph` by
```c++
graph.addExpressionFactor(h, z, R)
```
In the above, the unknown in the example can be retrieved by the `gtsam::Symbol(‘x’,1)`, which evaluates to a uint64 identifier.

## Composing Expressions
The key coolness behind expressions, however, is that you can compose them into expression trees, as long as the leaves know how to do their own derivatives:
```c++
Expression<Point3> x1(‘x’1), x2(‘x’,2);
auto h = Expression<Point3>(& cross, x1, x2);
auto g = Expression<Point3>(& norm3, h);
```
Because we typedef Point3_ to Expression<Point3>, we can write this very concisely as
```c++
auto g = Point3_(& norm3, Point3_(& cross, x1(‘x’1), x2(‘x’,2)));
```
## PoseSLAM Example
Using expressions, it is simple to quickly create a factor graph corresponding to a PoseSLAM problem, where our only measurements are relative poses between a series of unknown 2D or 3D poses. The following code snippet from [Pose2SLAMExampleExpressions.cpp](https://github.com/borglab/gtsam/blob/develop/examples/Pose2SLAMExampleExpressions.cpp) is used to create a simple Pose2 example (where the robot is moving on a plane):
```c++
1  ExpressionFactorGraph graph;
2  Expression<Pose2> x1(1), x2(2), x3(3), x4(4), x5(5);
3  graph.addExpressionFactor(x1, Pose2(0, 0, 0), priorNoise);
4  graph.addExpressionFactor(between(x1,x2), Pose2(2, 0, 0     ), model);
5  graph.addExpressionFactor(between(x2,x3), Pose2(2, 0, M_PI_2), model);
6  graph.addExpressionFactor(between(x3,x4), Pose2(2, 0, M_PI_2), model);
7  graph.addExpressionFactor(between(x4,x5), Pose2(2, 0, M_PI_2), model);
8  graph.addExpressionFactor(between(x5,x2), Pose2(2, 0, M_PI_2), model);
```
This is what is going on:
- In line 1, we create an empty factor graph.
- In line 2 we create the 5 unknown poses, of type `Expression<Pose2>`, with keys 1 to 5. These are what we will optimize over.
- Line 3 then creates a simple factor that gives a prior on `x1` (the first argument), namely that it is at the origin `Pose2(0, 0, 0)` (the second argument), with a particular probability density given by `priorNoise` (the third argument).
- Lines 4-7 adds factors for the odometry constraints, i.e., the movement between successive poses of the robot. The function `between(t1,t2)` is implemented in [nonlinear/expressions.h](https://github.com/borglab/gtsam/blob/develop/gtsam/nonlinear/expressions.h) and is equivalent to calling the constructor Expression<T>(traits<T>::Between, t1, t2).
- Finally, line 8 creates a loop closure constraint between poses x2 and x5.

Another good example of its use is in 
[SFMExampleExpressions.cpp](https://github.com/borglab/gtsam/blob/develop/examples/SFMExampleExpressions.cpp).