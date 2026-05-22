//-*- mode: c++ -*-
#ifndef _SOLVER_SQMR_H_
#define _SOLVER_SQMR_H_

#include <algorithm>
#include <cmath>
#include <string>

template <class el_type, class mat_type>
int solver<el_type, mat_type>::sqmr(int max_iter, double stop_tol) {
  // Zero out solution vector
  int n = A.n_rows();
  sol_vec.resize(n, 0);

  // ---------- set initial values and allocate memory for variables ---------//

  // temporary vectors for calcluations
  vector<el_type> x(n), q(n), t(n), r(n), tmp(n);

  // residual = b - A*x0
  r = rhs;

  // search direction
  vector<el_type> d(n);

  // set up initial values for variables above
  double eps = A.eps;
  double norm_rhs = norm(rhs, 2.0);

  double res = norm_rhs;
  double resmin = res;

  // Our preconditioner M = LDL'.
  auto Minv = [&](vector<el_type> &in, vector<el_type> &out) {
    L.backsolve(in, out);
    D.solve(out, tmp);
    L.forwardsolve(tmp, out);
  };

  // compute t = M^(-1) * r
  Minv(r, t);
  double tau = norm(t, 2.0);

  q = t;
  double thet = 0;
  double rho = dot_product(r, q);

  double sigma, alpha, thet1, c2, rho1, beta;

  // -------------- begin sqmr iterations --------------//
  int k = 1; // iteration number
  while (res / norm_rhs > stop_tol && k <= max_iter) {
    // t = A * q
    // sigma = q'*t
    A.multiply(q, t);
    sigma = dot_product(q, t);
    alpha = rho / sigma;

    // r = r - alpha * t
    vector_sum(1, r, -alpha, t, r);

    // t = Minv(r)
    Minv(r, t);

    thet1 = thet;
    thet = norm(t, 2.0) / tau;

    c2 = 1.0 / (1 + thet * thet);

    tau = tau * thet * sqrt(c2);
    if (k == 1) {
      // d = c^2 * alpha * q
      for (int i = 0; i < n; i++) {
        d[i] = c2 * alpha * q[i];
      }
    } else {
      // d = c^2 * thet1^2 * d + c^2 * alpha * q
      vector_sum(c2 * thet1 * thet1, d, c2 * alpha, q, d);
    }

    // update x
    vector_sum(1, x, 1, d, x);

    // update residual and norms
    res = norm(r, 2.0);
    /*
    // the true residual
    A.multiply(x, tmp);
    vector_sum(1, rhs, -1, tmp, tmp);
    res = norm(tmp, 2.0);
    */

    if (res < resmin) {
      resmin = res;
      sol_vec = x;
    }

    rho1 = rho;
    rho = dot_product(r, t);
    beta = rho / rho1;
    vector_sum(1, t, beta, q, q);

    k++;
    // ------------- end update ------------- //
  }

  std::string iter_str = "iterations";
  if (k - 1 == 1)
    iter_str = "iteration";

  if (msg_lvl)
    printf("SQMR took %i %s and got down to relative residual %e.\n", k - 1,
           iter_str.c_str(), resmin / norm_rhs);
  return k - 1;
}

#endif // _SOLVER_SQMR_H_
