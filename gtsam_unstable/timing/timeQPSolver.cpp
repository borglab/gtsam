/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file timeQPSolver.cpp
 * @brief Benchmark comparing the old active-set QPSolver (re-eliminates each
 *        iteration) vs the new QPSolver (sparse Cholesky + Schur complement,
 *        single elimination at construction).
 *
 * Run from the gtsam_unstable/timing build directory after CMake build.
 *
 * @date  May 2026
 * @author Frank Dellaert
 */

#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/HessianFactor.h>
#include <gtsam_unstable/linear/LinearEquality.h>
#include <gtsam_unstable/linear/QP.h>
#include <gtsam_unstable/linear/ActiveSetSolver.h>
#include <gtsam_unstable/linear/ActiveSetSolver-inl.h>
#include <gtsam_unstable/linear/QPInitSolver.h>
#include <gtsam_unstable/linear/QPSolver.h>

#include <chrono>
#include <iomanip>
#include <iostream>
#include <random>
#include <stdexcept>
#include <string>
#include <vector>

using namespace gtsam;
using namespace gtsam::symbol_shorthand;
using namespace std::chrono;

/// Policy for the old active-set QPSolver (re-eliminates KKT graph per iter).
struct OldQPPolicy {
  /// For QP, alpha=1 is the minimum of the quadratic along the step direction.
  static constexpr double maxAlpha = 1.0;

  /// Returns the cost graph (with constant term stripped) as the cost function.
  static GaussianFactorGraph buildCostFunction(
      const QP& qp, const VectorValues& /*xk*/ = VectorValues()) {
    GaussianFactorGraph graph;
    for (const auto& factor : qp.cost) {
      HessianFactor hf = static_cast<HessianFactor>(*factor);
      graph.push_back(std::make_shared<HessianFactor>(hf));
    }
    return graph;
  }
};

/// Old solver: re-eliminates the full KKT graph each active-set iteration.
using OldQPSolver = ActiveSetSolver<QP, OldQPPolicy, QPInitSolver>;

// =============================================================================
// Problem generators
// =============================================================================

/**
 * Build a random convex QP with @p n scalar variables and @p m inequality
 * constraints from a fixed random seed for reproducibility.
 *
 * H = A'A + I (random SPD), η = random, constraints: a_i'x ≤ b_i with b_i
 * chosen so the origin is feasible.
 *
 * Variables are named X(1)…X(n), each 1-dimensional.
 */
static QP makeRandomQP(int n, int m, unsigned seed = 42) {
  std::mt19937 rng(seed);
  std::normal_distribution<double> normal(0.0, 1.0);

  // Hessian: one HessianFactor per pair + diagonal, but for simplicity we
  // build one big HessianFactor by accumulating.  With scalar keys (dim=1),
  // we add individual factors.

  QP qp;

  // Cost: H = A'A + I (sum of outer products + regularizer).
  // Add individual diagonal HessianFactors (gradient terms) and off-diagonal.
  // For simplicity, use separate 2-key Hessian factors to build a random SPD H.
  // We only add upper-triangular pairs to avoid double-counting.
  const int kCholesky = std::min(n, 5);  // rank of random perturbation
  for (int r = 0; r < kCholesky; ++r) {
    // Random unit vector v → rank-1 outer product v v'
    std::vector<double> v(n);
    double norm2 = 0;
    for (int i = 0; i < n; ++i) { v[i] = normal(rng); norm2 += v[i] * v[i]; }
    double norm = std::sqrt(norm2);
    for (int i = 0; i < n; ++i) v[i] /= norm;

    // Add v_i v_i diagonal blocks
    for (int i = 0; i < n; ++i) {
      double gii = v[i] * v[i];
      // diagonal linear term η_i: use a fresh random gradient
      double gi = (r == 0) ? normal(rng) : 0.0;
      qp.cost.add(X(i + 1), gii * I_1x1, gi * I_1x1);
    }
    // Off-diagonal blocks v_i v_j (i < j)
    for (int i = 0; i < n; ++i) {
      for (int j = i + 1; j < n; ++j) {
        double gij = v[i] * v[j];
        if (std::abs(gij) > 1e-10) {
          // HessianFactor for keys X(i+1), X(j+1) with G12 = gij, G11=G22=0
          // (diagonal was added above; only off-diagonal here)
          qp.cost.push_back(HessianFactor(X(i + 1), X(j + 1),
                                          0.0 * I_1x1, gij * I_1x1, Z_1x1,
                                          0.0 * I_1x1, Z_1x1, 0.0));
        }
      }
    }
  }
  // Add regularizer I to ensure positive definiteness
  for (int i = 0; i < n; ++i) qp.cost.add(X(i + 1), I_1x1, Z_1x1);

  // Inequality constraints: a_i'x ≤ b_i with a_i random, b_i = |a_i|_1 + 1
  // so the origin is feasible.
  for (int k = 0; k < m; ++k) {
    Key dualKey = static_cast<Key>(n + k + 1);
    // Constraint with two variables to create structure
    int i1 = rng() % n;
    int i2 = rng() % n;
    double a1 = normal(rng), a2 = normal(rng);
    double b = std::abs(a1) + std::abs(a2) + 1.0;
    if (i1 == i2) {
      qp.inequalities.add(X(i1 + 1), (a1 + a2) * I_1x1, b, dualKey);
    } else {
      qp.inequalities.add(X(i1 + 1), a1 * I_1x1, X(i2 + 1), a2 * I_1x1, b,
                          dualKey);
    }
  }

  return qp;
}

/**
 * Build a chain QP with @p n scalar variables.
 * Hessian is tridiagonal (adjacent pairs share a cost), like a Kalman smoother.
 * Two inequality constraints per variable (lower and upper box bounds).
 */
static QP makeChainQP(int n) {
  QP qp;

  // Cost: sum_i (x_i - 1)^2 + 0.5 * sum_i (x_i - x_{i+1})^2
  for (int i = 0; i < n; ++i) {
    qp.cost.add(X(i + 1), 2.0 * I_1x1, 2.0 * I_1x1);  // 2*(x-1)^2 term
    if (i < n - 1) {
      // 0.5 * (x_i - x_{i+1})^2 → H_ii += 1, H_ij -= 1, H_jj += 1
      qp.cost.add(X(i + 1), I_1x1, Z_1x1);
      qp.cost.add(X(i + 2), I_1x1, Z_1x1);
      qp.cost.push_back(HessianFactor(X(i + 1), X(i + 2),
                                      0.0 * I_1x1, -I_1x1, Z_1x1,
                                      0.0 * I_1x1, Z_1x1, 0.0));
    }
  }

  // Box constraints: -0.5 ≤ x_i ≤ 2.5 (all active at lower bound initially)
  int dualKey = n + 1;
  for (int i = 0; i < n; ++i) {
    qp.inequalities.add(X(i + 1), -I_1x1, 0.5, dualKey++);  // -x_i ≤ 0.5
    qp.inequalities.add(X(i + 1), I_1x1, 2.5, dualKey++);   //  x_i ≤ 2.5
  }

  return qp;
}

/**
 * Build an MPC-style QP: horizon steps of a double integrator.
 * State = [position, velocity], input = acceleration.
 * n_state = 2*horizon, n_input = horizon.
 * Returns (qp, initial_x) where initial_x is a feasible starting point.
 */
static std::pair<QP, VectorValues> makeMpcQP(int horizon) {
  QP qp;

  const double dt = 0.1;
  const double Q_p = 1.0, Q_v = 0.1, R = 0.01;

  // Stage cost: Q_p * p^2 + Q_v * v^2 + R * u^2
  for (int k = 0; k <= horizon; ++k) {
    qp.cost.add(Symbol('p', k), 2 * Q_p * I_1x1, Z_1x1);
    qp.cost.add(Symbol('v', k), 2 * Q_v * I_1x1, Z_1x1);
  }
  for (int k = 0; k < horizon; ++k) {
    qp.cost.add(Symbol('u', k), 2 * R * I_1x1, Z_1x1);
  }

  // Dynamics: p_{k+1} = p_k + dt*v_k,  v_{k+1} = v_k + dt*u_k
  // As equalities.
  int dualKey = 10000;
  for (int k = 0; k < horizon; ++k) {
    // p_{k+1} - p_k - dt*v_k = 0
    qp.equalities.add(Symbol('p', k + 1), I_1x1, Symbol('p', k), -I_1x1,
                      Symbol('v', k), -dt * I_1x1, Z_1x1, dualKey++);
    // v_{k+1} - v_k - dt*u_k = 0
    qp.equalities.add(Symbol('v', k + 1), I_1x1, Symbol('v', k), -I_1x1,
                      Symbol('u', k), -dt * I_1x1, Z_1x1, dualKey++);
  }

  // Input limits: -1 ≤ u_k ≤ 1
  for (int k = 0; k < horizon; ++k) {
    qp.inequalities.add(Symbol('u', k), I_1x1, 1.0, dualKey++);
    qp.inequalities.add(Symbol('u', k), -I_1x1, 1.0, dualKey++);
  }

  // Initial state constraints (as equalities): p_0 = 1, v_0 = 0
  qp.equalities.add(Symbol('p', 0), I_1x1, Vector::Ones(1), dualKey++);
  qp.equalities.add(Symbol('v', 0), I_1x1, Z_1x1, dualKey++);

  // Build feasible initial point (all zeros is feasible for inputs;
  // propagate dynamics to get consistent states)
  VectorValues initialValues;
  double p = 1.0, v = 0.0;
  for (int k = 0; k <= horizon; ++k) {
    initialValues.insert(Symbol('p', k), (Vector1() << p).finished());
    initialValues.insert(Symbol('v', k), (Vector1() << v).finished());
    if (k < horizon) {
      initialValues.insert(Symbol('u', k), Z_1x1);
      // p and v unchanged (u=0)
    }
  }

  return {qp, initialValues};
}

// =============================================================================
// Timing helper
// =============================================================================

/// Return wall-clock time in seconds for a single call to functor().
template <typename Functor>
static double timeOnce(Functor&& functor) {
  auto t0 = high_resolution_clock::now();
  functor();
  auto t1 = high_resolution_clock::now();
  return duration<double>(t1 - t0).count();
}

/**
 * Run @p functor @p repeats times and return (mean_seconds, min_seconds).
 * The first call is a warm-up and is excluded from statistics.
 */
template <typename Functor>
static std::pair<double, double> timeit(Functor&& functor, int repeats = 20) {
  // Warm-up
  functor();

  double total = 0.0, minTime = 1e18;
  for (int i = 0; i < repeats; ++i) {
    double t = timeOnce(functor);
    total += t;
    minTime = std::min(minTime, t);
  }
  return {total / repeats, minTime};
}

// =============================================================================
// Printing helpers
// =============================================================================

static void printHeader(const std::string& title) {
  std::cout << "\n" << std::string(72, '=') << "\n";
  std::cout << "  " << title << "\n";
  std::cout << std::string(72, '=') << "\n";
  std::cout << std::left << std::setw(22) << "Solver"
            << std::right << std::setw(12) << "mean (ms)"
            << std::setw(12) << "min (ms)"
            << std::setw(10) << "speedup"
            << "\n";
  std::cout << std::string(72, '-') << "\n";
}

static void printRow(const std::string& label, double mean, double minTime,
                     double referenceMin) {
  double speedup = (referenceMin > 0) ? referenceMin / minTime : 1.0;
  std::cout << std::left << std::setw(22) << label
            << std::right << std::fixed << std::setprecision(3)
            << std::setw(12) << mean * 1e3
            << std::setw(12) << minTime * 1e3
            << std::setw(10) << speedup
            << "x\n";
}

// =============================================================================
// Benchmark: single-solve timing
// =============================================================================

static void benchmarkSingleSolve(const std::string& title, const QP& qp,
                                  const VectorValues& initial, int repeats) {
  printHeader(title);

  // Old: active-set solver that re-eliminates KKT graph each iteration
  auto [oldMean, oldMin] = timeit(
      [&] { OldQPSolver(qp).optimize(initial); }, repeats);
  printRow("QPSolver (old, ActiveSet)", oldMean, oldMin, oldMin);

  // New: single sparse Cholesky at construction + Schur complement per iter
  auto [newMean, newMin] = timeit(
      [&] { QPSolver(qp).optimize(initial); }, repeats);
  printRow("QPSolver (new, Schur)", newMean, newMin, oldMin);
}

// =============================================================================
// Benchmark: repeated (MPC-style) solves with warm start
// =============================================================================

static void benchmarkWarmStart(const std::string& title, const QP& qp,
                                const VectorValues& initial, int nSolves,
                                int repeats) {

  printHeader(title + "  (warm-start, " + std::to_string(nSolves) + " solves)");

  // Old: cold-start every solve
  auto [oldMean, oldMin] = timeit(
      [&] {
        OldQPSolver solver(qp);
        for (int i = 0; i < nSolves; ++i) solver.optimize(initial);
      },
      repeats);
  printRow("QPSolver (old, cold)", oldMean, oldMin, oldMin);

  // New: warm-start subsequent solves
  auto [newMean, newMin] = timeit(
      [&] {
        QPSolver solver(qp);
        auto [x0, d0, s0] = solver.optimizeWithState(initial);
        for (int i = 1; i < nSolves; ++i)
          solver.optimizeWithState(s0.values, s0.duals, true);
      },
      repeats);
  printRow("QPSolver (new, warm)", newMean, newMin, oldMin);
}

// =============================================================================
// main
// =============================================================================

int main() {
  std::cout << "QP Solver Benchmark\n";
  std::cout << "Old QPSolver (ActiveSet, re-eliminates per iter) vs\n";
  std::cout << "New QPSolver (sparse Cholesky at construction + Schur complement)\n";
  std::cout << "Times in milliseconds.  Speedup = oldMin / solverMin.\n";

  const int kRepeats = 30;

  // -----------------------------------------------------------------------
  // 1. Random dense QPs of increasing size
  // -----------------------------------------------------------------------
  for (int n : {5, 10, 20, 50}) {
    int m = n;  // equal number of inequality constraints
    QP qp = makeRandomQP(n, m);

    // Initial point: origin is feasible (by construction)
    VectorValues initial;
    for (int i = 0; i < n; ++i) initial.insert(X(i + 1), Z_1x1);

    std::string title =
        "Random dense QP  n=" + std::to_string(n) + "  m=" + std::to_string(m);
    benchmarkSingleSolve(title, qp, initial, kRepeats);
  }

  // -----------------------------------------------------------------------
  // 2. Chain (tridiagonal) QPs
  // -----------------------------------------------------------------------
  for (int n : {5, 10, 20}) {
    QP qp = makeChainQP(n);

    VectorValues initial;
    for (int i = 0; i < n; ++i) initial.insert(X(i + 1), Z_1x1);

    std::string title = "Chain QP  n=" + std::to_string(n) +
                        "  m=" + std::to_string(2 * n);
    benchmarkSingleSolve(title, qp, initial, kRepeats);
  }

  // -----------------------------------------------------------------------
  // 3. MPC warm-start sweeps
  // -----------------------------------------------------------------------
  for (int horizon : {5, 10}) {
    try {
      auto [qp, initial] = makeMpcQP(horizon);
      std::string title = "MPC QP  horizon=" + std::to_string(horizon);
      benchmarkWarmStart(title, qp, initial, 20 /*nSolves*/, kRepeats);
    } catch (const std::exception& e) {
      std::cout << "\nMPC QP  horizon=" << horizon
                << " SKIPPED (ill-conditioned with generic ordering): "
                << e.what() << "\n";
    }
  }

  return 0;
}
