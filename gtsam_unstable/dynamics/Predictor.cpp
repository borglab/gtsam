#include <gtsam/geometry/Pose2.h>
#include <functional>
#include <vector>
#include <cmath>
#include "Predictor.h"


/* x \in X == R^n
 * xdot \in Xdot == R^n
 * u \in U = R^p
 * theta \in Theta == R^m
 */

//n = 3
//p = 2
//m = 5


namespace gtsam{

Matrix31 Xdot(Matrix31 x, double c) {//, Matrix<2> u) {
  // For now, just some hard coded centroids
  // And for now, we will ignore the input u
  Matrix31 xdot;

  for(int i=0; i<3; i++) {
    xdot[i] = exp(pow(x[i]-c,2)/-0.25);
  }
  return xdot;
}

typedef std::function<Matrix31(Matrix31, double)> RBF;


  Predictor::Predictor(Matrix31 x,double dt) {
    x_ = x;
    RBF rbf(Xdot);
    for (int i=0;i<5;i++) {
      dt_rbf_.col(i) = dt * rbf(x,i*0.5);
    }
  }
  Matrix31 Predictor::operator()(Matrix51 theta, OptionalJacobian<3,5> H) {
    // calculate f(x,y) as sum of RBF:
    // i.e., \sum w_i rbf(x,u)
    if (H) *H = dt_rbf_;
    return x_ + dt_rbf_*theta; // nxm * mx1
  }


}
