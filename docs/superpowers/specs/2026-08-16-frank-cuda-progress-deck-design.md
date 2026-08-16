# Frank CUDA Progress Deck Design

Date: 2026-08-16

## Purpose

Prepare a concise progress deck for the August 17 meeting with Frank. The
deck should answer two questions: what changed since the August 11 discussion,
and what the current benchmark evidence says. It should not reteach the prior
general or SFM implementations.

## Audience and tone

- Primary audience: Frank Dellaert.
- Six main slides plus a data appendix.
- Direct technical language; no promotional filler.
- Use the prior deck's navy/green visual identity, with larger text and less
  content per slide.

## Main deck

1. **CUDA LM update** — title, presenter, and meeting date.
2. **What is implemented** — summarize the shared linear core, backend
   switching, ordering support, and integration with both optimizers as four
   current capabilities. Do not frame the slide as a request/response.
3. **Architecture change** — one before/after diagram. The current state has
   the general hybrid frontend and specialized SFM frontend producing dense,
   sparse, or operator systems for one shared `CudaLinearSolverSession`.
4. **General CUDA LM: 16 workloads** — compact speedup charts for BAL,
   Pose2/mixed, and Pose3 across cuDSS automatic ordering, cuDSS with GTSAM
   ordering, and PCG.
5. **What the measurements show** — setup crossover, ordering dependence,
   PCG dependence on topology/conditioning, and the specialized SFM advantage.
6. **Next work** — only future work, grouped as benchmark coverage, release
   cleanup, and performance follow-up. Do not repeat current status.

## Appendix

- Complete general benchmark table, split across two slides.
- Specialized SFM versus general CUDA versus CPU on BAL16/88/135.
- Specialized SFM formulation/backend matrix.
- Pose2 raw versus FastSync control.
- Benchmark protocol, hardware, solver tolerances, and correctness caveats.

## Data policy

- Use the final 16-workload campaign at revision `c949b0736` for the complete
  general table and charts.
- Use the separately paired three-way BAL campaign for CPU/specialized/general
  comparisons and label it as a separate harness.
- Keep direct-solver and PCG correctness claims separate: direct rows use a
  `1e-8` objective gate; PCG uses relative residual `1e-6`, at most 5,000
  iterations per solve, block-Jacobi, warm starts, and a `1e-3` endpoint gate.
- Label induced Pose2/Pose3 prefixes as controlled subgraphs.
- Treat iteration-limit rows as same-budget comparisons, not convergence
  claims.

## Visual design

- 16:9 layout.
- Dark navy title and closing slides; warm off-white content slides.
- CUDA green for completed/shared architecture and cuDSS automatic ordering;
  amber for GTSAM ordering; violet for PCG.
- Use diagrams, charts, and compact tables rather than paragraph text.
- Minimum body text target: 16 pt in the main deck and 11–12 pt in appendix
  tables.

## Acceptance checks

- The PowerPoint opens and contains all expected slides.
- The six main slides can be presented without reading the appendix.
- Every one of the 16 workloads and three GPU configurations appears in the
  appendix.
- No text overlaps or overflows after PDF rendering.
- A content extraction pass shows no placeholders or missing sections.
