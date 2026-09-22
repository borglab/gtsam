# Unstable Timing Benchmarks

## Sparse Hybrid AllDiff Multiplication

`timeHybridAllDiff` separates the two performance improvements involved in
multiplying a `TableFactor` by `AllDiff`:

- `decision_tree` reproduces the original conversion through a
  `DecisionTreeFactor`;
- `copied_table` uses sparse `AllDiff` conversion but reproduces the former
  generic dispatch, including its copy of the input table;
- `direct_table` uses the current virtual `toTableFactor()` dispatch and
  borrows the input table by reference.

The benchmark checks that all three products are numerically identical before
timing them. It reports full-scope and partial-scope multiplication separately,
including both the speedup over decision-tree conversion and the incremental
speedup from removing the input-table copy.

Build and run it from the build directory:

```bash
make -j6 timeHybridAllDiff
./gtsam_unstable/timing/timeHybridAllDiff \
  --minimum-objects 3 --maximum-objects 7 --warmup 1 --repeats 5
```

Use `--output hybrid_all_diff.json` to retain the medians in Benchmark Action
JSON format.
