#include <gtsam/geometry/Pose2.h>
#include <functional>


/* x \in X == R^n
 * xdot \in Xdot == R^n
 * u \in U = R^p
 * theta \in Theta == R^m
 */

//n = 3
//p = 2
//m = 5


namespace gtsam{

Matrix<3> Xdot(Matrix<3> x, Matrix<5> theta) {

}

typedef std::function<Matrix<3>(Matrix<3>,Matrix<5>)> RBF(Xdot);
std:vector<RBF> rbf;

struct Predictor {
Predictor(x,u,dt) {
  for (i in range(m)) {
    dt_rbf.col(i) = dt * rbf[i](x,u);
}
}
X operator(theta, OptionalJacobian<n,m> H) {
// calculate f(x,y) as sum of RBF:
// i.e., \sum w_i rbf(x,u)
if (H) *H = dt_rbf_;
return x + dt_rbf_*theta; // nxm * mx1
}
private:
    Matrix<n,m> dt_rbf_;
}

}
