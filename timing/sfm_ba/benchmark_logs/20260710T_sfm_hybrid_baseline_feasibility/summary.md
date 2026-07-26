# SFM Hybrid-Baseline Feasibility Benchmark

## Benchmark

Three independent, sequential processes ran on `examples/Data/dubrovnik-135-90642-pre.txt`:

```sh
./build-cuda-cudss-on/timing/sfm_ba/timeCudaSFMBAL \
  --linearization-benchmark --linearization-repeats 10 \
  examples/Data/dubrovnik-135-90642-pre.txt
```

Each process used one untimed warm-up per path and then recorded ten repeats. The graph has 553,336 factors and 90,777 values; all logs report `GTSAM_USE_TBB: yes` and a finite trial error of 58,055,917. Hardware: AMD EPYC Milan (32 available CPUs) and NVIDIA A100 80 GB PCIe.

Raw output path: `timing/sfm_ba/benchmark_logs/20260710T_sfm_hybrid_baseline_feasibility/`. Git HEAD during collection was `d21004c11b81242eb17206ce53895b18c9ee2929`; the initial artifact-recording commit was `6549622b8b1d0ba47d3970a1f3959cb0b6d9660e`. CUDA version, driver version, and the actual TBB thread configuration were not recorded in the raw logs.

## Timing Results

All timing units are milliseconds. Statistics are computed from individual samples; standard deviation is population standard deviation.

| Section, all samples | n | Mean | Min | Max | SD | CV |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| CPU/TBB graph.linearize() | 30 | 30.851 | 25.119 | 57.299 | 8.140 | 26.39% |
| CPU Values retract() | 30 | 14.260 | 12.320 | 18.152 | 1.253 | 8.79% |
| CPU trial graph.error() | 30 | 11.644 | 8.518 | 16.235 | 2.329 | 20.00% |
| GPU projection residual/Jacobian generation | 30 | 0.384280 | 0.378943 | 0.391316 | 0.003031 | 0.79% |
| GPU Hessian diagonal generation | 30 | 0.918988 | 0.908664 | 0.923662 | 0.003584 | 0.39% |
| GPU dense Schur combined | 30 | 7.513482 | 7.467875 | 7.575702 | 0.031868 | 0.42% |

| Section, repeat-0-excluded sensitivity view | n | Mean | Min | Max | SD | CV |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| CPU/TBB graph.linearize() | 27 | 28.316 | 25.119 | 38.918 | 2.756 | 9.73% |
| CPU Values retract() | 27 | 14.332 | 12.591 | 18.152 | 1.249 | 8.71% |
| CPU trial graph.error() | 27 | 11.440 | 8.518 | 14.845 | 2.204 | 19.26% |
| GPU projection residual/Jacobian generation | 27 | 0.384203 | 0.378943 | 0.391316 | 0.003145 | 0.82% |
| GPU Hessian diagonal generation | 27 | 0.919007 | 0.908664 | 0.923662 | 0.003764 | 0.41% |
| GPU dense Schur combined | 27 | 7.515787 | 7.467875 | 7.575702 | 0.032627 | 0.43% |

## Hybrid Interpretation

| Derived quantity | All 30 samples | Repeat-0-excluded sensitivity view (27 samples) |
| --- | ---: | ---: |
| Index-aligned illustrative CPU subtotal: linearization + retraction + trial error | 56.755 ms | 54.088 ms |
| Synthetic GPU reference: Hessian diagonal + dense Schur combined - GPU projection | 8.048 ms | 8.051 ms |
| Synthetic hybrid lower bound: CPU subtotal + GPU reference | 64.803 ms | 62.139 ms |

Each operation is timed in a separate loop, not as one sequential hybrid candidate call. Derived rows align repeat indices only for illustrative arithmetic, so they are not sequential hybrid-call measurements. Repeat 0 is systematically slower for the index-aligned illustrative CPU subtotal: its cross-process mean is 80.752 ms, versus 56.881 ms for repeat 1 and 57.024 ms for repeat 2. The index-aligned illustrative subtotal across repeat indices 0 through 2 is 194.657 ms per process on average (64.886 ms per aligned index). Both aggregates are therefore material: 62.139 ms is the repeat-0-excluded sensitivity-view lower bound, while 64.803 ms retains the first measured index.

The estimated generic Jacobian transfer is `553,336 x (24 Jacobian + 2 residual) x 8 B = 115,093,888 B` (115.1 decimal MB). Dividing by `9.50 GiB/s` gives 11.283 ms. Adding that estimate to the repeat-0-excluded sensitivity-view lower bound gives 73.422 ms, but neither 62.139 ms nor 73.422 ms is an end-to-end timing. Sparse packing, the actual generic J/H numeric layout, repeated upload, delta download/mapping, and convergence remain unmeasured.

For context, a prior full Graph API run covering three LM iterations measured 426.223 ms total, including 114.246 ms graph conversion; its complete backend solve loop is 29.968 ms across three attempts. Pure H2D+D2H copies are 1.936 ms only for the specialized representation, not the estimated generic Jacobian transfer above. The 62.139 ms figure is a per-candidate synthetic lower bound, not total runtime, and is therefore not directly comparable to the 426.223 ms three-iteration Graph API runtime.

## Raw Logs

- `hybrid_135_rep1.log`
- `hybrid_135_rep2.log`
- `hybrid_135_rep3.log`
