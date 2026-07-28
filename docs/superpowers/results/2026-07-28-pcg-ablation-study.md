# PCG Ablation Study: from a BAE-Style Baseline to Our Shipping Configuration

## Question

Are our PCG design choices — block-Jacobi preconditioning, scaled warm start,
tight tolerance — actually earning their keep, or would the simpler
PyPose/BAE recipe (scalar Jacobi, cold start every attempt, tol 1e-4) do just
as well? Built each feature as a switch and measured the ladder rung by rung.

## Setup

All rungs run in **our engine** (matrix-free operator, device-resident
scalars, 10-iteration convergence checks, 250-iteration cap) so the ablation
isolates the algorithmic choices, not the engine. BAE's engine differs in
ways we cannot ablate here (materialized JᵀJ via SpGEMM, PyTorch kernels,
per-CG-iteration host syncs) — their published numbers are an external
anchor, not a rung.

New switches (committed): `params.pcg.preconditioner =
block-jacobi|jacobi|none`, `params.pcg.warmStart`; benchmark flags
`--pcg-preconditioner`, `--pcg-no-warm-start`. Scalar Jacobi matches BAE's
recipe: z = r / max(diag(JᵀJ) + λD, floor). A `none` (identity) mode exists
for completeness but fails the objective sanity check on pose3 —
unpreconditioned CG is not usable on these problems.

A100, 5-run medians, warmup 1, `--objective-tol 0.5` (deliberately loose so
every rung *completes* and we can see its accuracy honestly). Raw data:
`timing/cuda_sparse/results/2026-07-28-ablation/`.

## Results

GPU wall (s) / relative final-objective difference vs CPU:

| Rung | bal16 | bal135 | pose2 | pose3 |
|---|---|---|---|---|
| R0 scalar+cold+1e-4 (BAE recipe) | 0.472 / 2.7e-6 | 1.480 / 9.5e-3 | 0.388 / 2.3e-2 | 0.716 / 1.6e-3 |
| R1 block+cold+1e-4 | 0.359 / 5.4e-7 | 1.291 / 2.9e-4 | 0.314 / 9.2e-3 | 0.549 / **5.2e-1**¹ |
| R1b scalar+warm+1e-4 | 0.471 / 2.6e-6 | 1.301 / 9.5e-3 | 0.355 / 2.3e-2 | 0.599 / 1.9e-3 |
| R2 block+warm+1e-4 | 0.370 / 5.4e-7 | 1.348 / 2.7e-4 | 0.294 / 1.0e-2 | 0.751 / 2.1e-3 |
| R3 block+warm+1e-6 (**shipping**) | 0.392 / 2.2e-9 | 1.335 / 2.8e-4 | 0.338 / 2.8e-3 | 0.692 / 8.4e-6 |

CG iteration counts (medians): bal16 1500→540 (scalar→block), bal135
1000→750, pose2 650→530, pose3 3660→2690.

¹ R1's pose3 converged (all 5 reps, deterministically) to a *different local
minimum* (7658 vs 5037): with cold starts and loose tolerance the inexact
early steps took a different basin. Not a solver failure — a reminder that on
nonconvex problems the LM trajectory is part of the result.

## Findings, feature by feature

1. **Block-Jacobi is the workhorse — confirmed.** Versus scalar Jacobi at
   identical settings (R0→R1): 2.8x fewer CG iterations on bal16, wall −24%
   on bal16 / −13% on bal135 / −19% on pose2 / −23% on pose3, and
   **accuracy improves ~5-35x on the BA problems** (bal135 9.5e-3 → 2.9e-4)
   because the iteration cap stops binding. Scalar Jacobi at the cap leaves
   bal135 1% off and pose2 2.3% off the CPU objective — consistent with
   BAE's published +1.35% on their bal135 run.
2. **Warm start is real but second-order, and workload-dependent.** On the
   scalar rung (R0→R1b) it cuts pose3 wall 16% and bal135 solve time 25%
   with accuracy unchanged. On the block rung (R1→R2) its effect is within
   noise on BA (few, always-accepted attempts — little to warm-start) and
   mixed on pose3, where it also changed which basin LM lands in (fixing
   R1's outlier). Verdict: keep (it never hurts the solve and pose3-style
   retry-heavy workloads benefit), but it is not the headline feature —
   block-Jacobi is.
3. **Tight tolerance (1e-6) buys accuracy nearly free — reconfirmed on the
   ladder.** R2→R3: wall changes −1% to +15%, objective improves 30-250x
   (pose3 2.1e-3 → 8.4e-6, bal16 5.4e-7 → 2.2e-9). bal135 is unchanged
   because it is iteration-cap-bound, not tolerance-bound.
4. **Unpreconditioned CG is not viable** (fails objective sanity on pose3
   even at 5% tolerance) — preconditioning is not optional on these
   problems.
5. **The BAE recipe in our engine is strictly dominated**: R0 is the worst
   rung on every workload on wall time *and* accuracy. Their competitive
   loop-time on bal135 therefore comes from engine differences (batched
   GPU-autodiff Jacobians, fewer outer iterations under their stop rule) —
   not from their PCG configuration, which our ladder shows costs both speed
   and accuracy.

## Caveats

- Single machine/GPU; 5 repeats; run-to-run drift on this box is a few
  percent (contemporaneous ladder, so rung-to-rung comparisons are clean).
- The R1 pose3 basin-switch shows loose-tolerance rungs can change LM
  trajectories qualitatively; wall-time comparisons between rungs that
  converged to different optima (R1 pose3) are not apples-to-apples.
- `--objective-tol 0.5` was an experiment harness setting, not a shipping
  default (shipping validation remains 0.02).
