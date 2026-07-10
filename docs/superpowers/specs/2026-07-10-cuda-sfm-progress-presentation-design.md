# CUDA SFM Progress Presentation Design

## Purpose

Create a ten-minute research progress presentation for Prof. Frank. The deck
reports the current CUDA SFM Levenberg-Marquardt implementation and its detailed
timing breakdown, then introduces CPU nonlinear processing plus GPU linear
solving as a simple baseline for a more general solver.

The deck is a progress update, not a pitch. It should distinguish completed
work, measured evidence, estimates, and proposed next work.

## Audience

The audience has a broad understanding of GTSAM and bundle adjustment but does
not know the current CUDA implementation internals. Every implementation term
must therefore be explained by its role in the pipeline rather than by class or
function names alone.

## Main Deck

The main deck contains nine slides:

1. **CUDA SFM LM progress**
   Introduce the two topics: detailed timing and a baseline toward generality.

2. **Current implementation**
   Show `NonlinearFactorGraph + Values -> SFM array conversion -> CUDA LM ->
   Values`. State that the current backend is specialized for SFM and dense
   Schur but exposed through a GTSAM graph API.

3. **Whole graph API: 0.426 s**
   Show optimizer construction, `optimize()`, and result/error queries with
   times and percentages of the 0.426223 s total.

4. **Inside `optimize()`: 0.376 s**
   Show graph conversion, CUDA backend, backend result return/assignment,
   Values merge/state update, and converted-data destruction. Keep the full
   component values visible, not only grouped totals.

5. **Inside the CUDA backend: 0.138 s**
   Show setup, solve loop, and download/rebuild. Add a second level of detail:
   projection host build, dense-Schur solve, Hessian diagonal, raw D2H copy,
   and host Values rebuild.

6. **What the profile says**
   Contrast the 29.968 ms solve loop and 1.936 ms total raw H2D+D2H copies with
   the larger host representation costs. Conclude that current bottlenecks are
   graph conversion, packing/building array representations, rebuilding
   `Values`, and state copying, not PCIe bandwidth or the dense-Schur kernels.

7. **General-solver baseline**
   Show a Ceres-style separation: CPU/TBB residual and Jacobian evaluation,
   reusable sparse `J`/`H` layout and numeric packing, GPU linear solve,
   downloaded delta, and CPU manifold retraction/trial evaluation. Describe it
   as a baseline architecture, not the only or final design.

8. **Baseline feasibility timing**
   Report CPU/TBB linearization on the same 135-camera problem. Add measured CPU
   retraction and trial-error timing. Show first-call and steady-state behavior
   where material. Include the approximately 115.1 MB Jacobian payload and
   approximately 11.3 ms H2D estimate as estimates, clearly labeled.

9. **Current assessment and next work**
   State whether CPU nonlinear work appears manageable relative to the 0.426 s
   current API total. Identify the remaining unknowns: direct sparse packing,
   repeated numeric H2D upload, GPU normal-equation construction/solve for a
   general pattern, and end-to-end convergence behavior.

## Appendix

Appendix slides may contain the complete optimize breakdown table, setup and
solve-loop sub-timers, transfer bytes/bandwidth, raw feasibility samples, and
the first-call CPU linearization variability. The main deck must remain
readable without the appendix.

## Benchmark Definitions

All feasibility measurements use `dubrovnik-135-90642-pre.txt` on the current
AMD EPYC Milan 32-vCPU host and NVIDIA A100 80 GB PCIe GPU.

- **CPU linearization:** time `NonlinearFactorGraph::linearize(values)` with
  TBB enabled. Graph loading/construction and output destruction are excluded.
- **CPU retraction:** time creation of a new `Values` through manifold
  retraction using a prebuilt, dimension-compatible delta. Report dense-delta
  conversion separately if measured.
- **Trial error:** time `NonlinearFactorGraph::error(trialValues)` with the
  current TBB-enabled implementation.
- **GPU references:** keep projection linearization, Hessian-diagonal
  generation, and combined dense-Schur timing separately labeled. The combined
  dense-Schur timer uses a precomputed damping diagonal.

Warm-up policy and repeat counts must be printed in raw logs. Main-slide values
should use steady-state means while also disclosing materially slower startup
behavior.

## Ceres Comparison

Use the official Ceres documentation only. Phrase the comparison narrowly:
Ceres separates nonlinear residual/Jacobian evaluation from selectable linear
algebra backends and can use CUDA/cuDSS to accelerate Gauss-Newton linear
systems. Do not claim that Ceres always performs every nonlinear operation on
the CPU or that its internal data flow exactly matches the proposed GTSAM
prototype.

## Visual System

Reuse the existing deck's dark navy, teal, green, amber, and muted red palette.
Use dark slides for the opening and closing, light slides for technical content,
and one consistent pipeline motif. Detailed timing slides should use stacked
bars, compact tables, and large number callouts rather than paragraphs.

The main deck should use 16:9 widescreen layout, 36-44 pt slide titles, 14-16 pt
body text, and at least 0.5 inch margins. Technical labels may use 11-12 pt only
when paired with a visual and kept away from the slide edge.

## Deliverables

- Editable PowerPoint deck generated from the existing JavaScript source.
- Rendered PDF.
- Individual rendered slide images for visual QA.
- Raw retraction/error benchmark logs and a concise Markdown summary.

## Verification

- Rebuild and run the focused CUDA SFM tests after benchmark instrumentation.
- Run each benchmark in three independent processes with repeated inner
  samples.
- Extract deck text and scan for placeholders, stale values, and unsupported
  claims.
- Render all slides and complete at least one visual fix-and-reverify cycle.
- Have an independent reviewer inspect the rendered slides for overflow,
  collisions, low contrast, and unclear information hierarchy.
