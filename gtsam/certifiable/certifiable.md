# Certifiable optimization

The certifiable module provides explicit semidefinite relaxations and a
solver-independent Burer–Monteiro Riemannian Staircase for QCQP-representable
factor graphs.

- [`LiftedSDPProblem`](doc/LiftedSDPProblem.ipynb) introduces the shared lifted
  SDP and recovery contract.
- [`MosekMonolithicSDP`](doc/MosekMonolithicSDP.ipynb) uses one PSD cone.
- [`MosekChordalSDP`](doc/MosekChordalSDP.ipynb) uses chordal clique cones.
- [`RiemannianStaircaseOptimizer`](doc/RiemannianStaircaseOptimizer.ipynb)
  explains the Burer–Monteiro solver and certificate.
- [`RiemannianStaircaseParams`](doc/RiemannianStaircaseParams.ipynb) documents
  rank, verification, saddle-escape, and inner-solver parameters.
- [`RiemannianStaircaseResult`](doc/RiemannianStaircaseResult.ipynb) documents
  rounded values and per-level diagnostics.
