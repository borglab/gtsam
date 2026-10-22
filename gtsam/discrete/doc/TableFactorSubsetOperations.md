# Table operations with a contained scope

Discrete elimination normalizes a joint factor by dividing it by a separator
factor. Every separator key already belongs to the joint. Multiplication also
frequently combines a joint with a factor over a subset of its keys.

`TableFactor::apply` uses a direct lookup for this case. The general hash join
remains available when the right operand introduces any new key or has a
different cardinality for a shared key. There is no public API or table layout
change.

## Reading the implementation

1. `apply` checks containment on the cached sorted `(key, cardinality)` pairs.
   Successful checks avoid constructing the general join's scope vectors and
   per-entry assignment maps.
2. `applySubset` constructs an integer index projection once, then visits the
   left operand's stored entries. Each entry has exactly one possible match in
   the right operand.
3. A fully stored right operand permits direct array access. A sparse right
   operand uses binary search over its stored indices.
4. The result retains the existing sorted-key layout. Entries append directly
   when the left storage already has that layout; otherwise their indices are
   converted to the sorted layout before insertion.

## Index projection

For a key with cardinality `c` and source stride `s`, the value encoded by a
linear source index `i` is

```text
value = (i / s) % c
```

If that key has target stride `t`, its contribution to the target index is
`value * t`. Summing these contributions projects the joint index into the
separator. All arithmetic stays integral, including indices above `2^53`.

Consecutive dimensions can be decoded together when they are adjacent in both
storage layouts. For a previous block followed by a dimension `(s, c, t)`, the
conditions are

```text
previous.sourceStride == s * c
previous.targetStride == t * c
```

The combined block has source stride `s`, cardinality
`previous.cardinality * c`, and target stride `t`. This is the usual
mixed-radix identity: decoding several adjacent digits together gives the same
integer as decoding and recombining them separately.

For example, a joint over `(A:2, B:3, C:2)` uses strides `(6, 2, 1)`.
A separator over `(B:3, C:2)` uses `(2, 1)`. Its two contributions

```text
((i / 2) % 3) * 2 + i % 2
```

collapse to `i % 6`. Reordered or nonadjacent dimensions remain separate blocks.

## Behavior that must remain unchanged

- Containment compares cardinalities as well as keys.
- A missing sparse entry is different from an explicitly stored zero. The
  generic binary operation is called only for stored matches, including zeros;
  an output zero remains stored. This matters for operations beyond multiplication
  and division, and for `nrValues()`.
- Operand order remains `op(leftValue, rightValue)`, including for division and
  other noncommutative operations.
- Metadata-only constructors can reorder `keys_` without rearranging storage.
  Projection and append eligibility therefore use the actual stride maps.
- Equal-scope output allocation is bounded by the smaller stored-entry count.
  A sparse mask does not require a dense-sized allocation.
- Existing treatment of default-constructed empty factors remains in `apply`
  before the fast-path dispatch.

If the left operand has `L` stored entries, the right has `R`, and the projection
has `B` coalesced blocks, lookup work is `O(L * (B + 1))` for a fully stored right
operand and `O(L * (B + 1 + log R))` for a sparse one. The extra constant covers
scalar or unit-cardinality subsets with no projection blocks. No right-hand hash map or
per-entry assignment map is allocated. Reordering an unsorted result retains
the existing sparse insertion cost.

## Validation and measurement

`testTableFactor` compares products and quotients with decision trees over all
permutations and subsets of a three-key factor. Additional cases cover generic
binary operations, stored and missing zeros, metadata-only reordering, empty
tables, and exact sparse indices above `2^53`. Discrete elimination and hybrid
inference suites exercise the caller paths.

The focused benchmark `timeTableFactorSubset` and the existing
`timeHybridInference` measure table operations and end-to-end hybrid inference,
respectively. Validate both revisions with the same compiler and build flags;
run timing processes serially after builds finish.

```sh
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release \
  -DGTSAM_USE_BOOST_FEATURES=OFF -DGTSAM_ENABLE_BOOST_SERIALIZATION=OFF \
  -DGTSAM_WITH_TBB=OFF -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF \
  -DGTSAM_BUILD_UNSTABLE=OFF -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF \
  -DGTSAM_BUILD_TIMING_ALWAYS=OFF
cmake --build build --parallel 6 --target \
  testTableFactor.run testDiscreteFactorGraph.run \
  testDecisionTreeFactor.run testDiscreteConditional.run \
  testHybridGaussianFactorGraph.run testHybridBayesNet.run \
  timeTableFactorSubset timeHybridInference
build/timing/timeTableFactorSubset --warmup 3 --repeats 31
build/timing/timeHybridInference --objects 6 --warmup 3 \
  --repeats 11 --iterations 10
```

For the parent revision, compile the focused benchmark source from this branch
against the parent's library: it uses only existing public APIs. Alternate
baseline/optimized process order across at least three rounds and report the
median of those round medians. The PR records the exact baseline SHA, build
configuration, and measured results; timings are workload- and machine-specific.

## Recorded comparison

Baseline: PR #2868 at `b5c566dd6f2b30f581779152d55e5199ec099aed`.
Both libraries used GCC 14.2.0, Release, `GTSAM_DT_MERGING=ON`, and the flags above
(Boost and TBB disabled). Each value below is the median of three alternating
process rounds; all times are microseconds per call. The focused benchmark
validates every assignment against independent dense arithmetic before timing.
The hybrid benchmark verifies that sequential and multifrontal solutions agree.
Checksums matched across revisions. These numbers are not directly comparable
to the different GCC/TBB configuration used in the parent PR's description.

### Hybrid data association

| Objects | Operation | Parent | Optimized | Speedup |
|---:|---|---:|---:|---:|
| 3 | Sequential construction | 34.39 | 32.91 | 1.04× |
| 3 | Sequential optimization | 7.65 | 7.01 | 1.09× |
| 3 | Multifrontal construction | 33.00 | 33.01 | 1.00× |
| 3 | Multifrontal optimization | 3.30 | 3.33 | 0.99× |
| 4 | Sequential construction | 74.26 | 63.91 | 1.16× |
| 4 | Sequential optimization | 19.82 | 14.80 | 1.34× |
| 4 | Multifrontal construction | 58.61 | 51.44 | 1.14× |
| 4 | Multifrontal optimization | 4.83 | 4.73 | 1.02× |
| 5 | Sequential construction | 195.34 | 156.89 | 1.25× |
| 5 | Sequential optimization | 89.95 | 55.08 | 1.63× |
| 5 | Multifrontal construction | 114.35 | 82.29 | 1.39× |
| 5 | Multifrontal optimization | 7.38 | 7.28 | 1.01× |
| 6 | Sequential construction | 1090.65 | 823.81 | 1.32× |
| 6 | Sequential optimization | 660.19 | 419.80 | 1.57× |
| 6 | Multifrontal construction | 421.05 | 159.87 | 2.63× |
| 6 | Multifrontal optimization | 20.18 | 19.60 | 1.03× |

Small cases and multifrontal optimization change little. The largest measured
end-to-end gain is in six-object multifrontal construction; the table-only
speedups below are not whole-pipeline speedups.

### Table division, 14 binary keys

| Joint storage | Right-hand scope | Parent | Optimized | Speedup |
|---|---|---:|---:|---:|
| Dense | suffix | 5410.22 | 161.87 | 33.42× |
| Dense | middle | 4329.52 | 184.84 | 23.42× |
| Dense | equal | 7153.62 | 99.82 | 71.67× |
| Sparse | suffix | 1366.66 | 164.25 | 8.32× |
| Sparse | middle | 1254.16 | 158.22 | 7.93× |
| Sparse | equal | 1384.51 | 140.43 | 9.86× |
