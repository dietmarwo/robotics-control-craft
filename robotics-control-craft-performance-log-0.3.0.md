# robotics-control-craft performance implementation log

4 November 2026. Version 0.3.0, based on the 0.2.0 delivery imported as
commit `9c4633e`. The ZIP contains the updated source, a new
[user manual](docs/user-manual.md), application and optimization examples
in [`examples/python/`](examples/python/README.md) with recorded outputs,
the updated [architecture guide](docs/architecture.md) with measured
performance, the ordered patch series in `patches/0.3.0/`, raw evidence in
`evidence/performance-0.3.0/` and `evidence/release-0.3.0/`, and the 0.3.0
crate, sdist and wheel in `packages/`. The 0.2.0 log is kept unchanged as
[`robotics-control-craft-performance-log-0.2.0.md`](robotics-control-craft-performance-log-0.2.0.md);
its patch files moved to `patches/0.2.0/`.

All measurements in this log come from one host: 2 vCPUs, Intel(R)
Xeon(R) Processor @ 2.10GHz under KVM (AVX-512F available), Rust 1.97.0
and CPython 3.12.3. Four other projects shared the host; every build and
timing ran under a shared CPU lock. The 0.2.0 log's numbers came from
another host (AMD EPYC 9V74, 4 CPUs, Rust 1.88) and are not comparable
with these.

## Ten incremental changes

Each row compares a round with the previous kept round on the workload
declared in `evidence/performance-0.3.0/plan.md` before the timing run,
using `validation/performance_pair.py --samples 11` with the two saved
benchmark binaries. Times are median milliseconds per prepared call. The
decision rule was a median speedup of at least 1.03x with a passing paired
output check. Host: 2 vCPUs, Intel Xeon @ 2.10GHz (KVM), Rust 1.97.0.

| Round | Commit | Change | Case | Before ms | After ms | Before / after |
| ---: | --- | --- | --- | ---: | ---: | ---: |
| 1 | `79ea5fa` | Add the typed `score_matrix` API in Rust and Python, a matrix benchmark mode and `--matrix` in the optimization driver. The row compares 0.2.0 lean `scores_json` with the new matrix API. | `serial`, lean → matrix | 0.2595 | 0.2091 | 1.24x |
| 2 | `87eb946` | Integrate untraced candidates with valid parameters in lockstep RK4 lanes (four) on the shared time grid when the budget has no deadline or cancel file. | `domain` | 6.2568 | 1.7126 | 3.65x |
| 3 | `8c3e23e` | Compile the lane kernel for AVX2, selected at preparation, and widen groups to eight lanes. | `domain2` | 1.8097 | 0.9947 | 1.82x |
| 4 | `a2ff3df` | Serialize a typed metrics struct, with fields in the former map's key order, instead of a `serde_json::Value` map. | `serial` | 0.1834 | 0.1062 | 1.73x |
| 5 | `2b9ddff` | Serialize traces directly from typed rows and reserve them up front. | `trace` | 0.2343 | 0.1047 | 2.24x |
| 6 | `1e07a68` | Run one batch worker on the calling thread. | `parallel` | 0.2775 | 0.2169 | 1.28x |
| 7 | `1997816` | Make the lane kernel generic over its width and add a sixteen-lane AVX-512F build. | `domain` | 1.0381 | 0.7046 | 1.47x |
| 8 | `8044a10` | Decode parameter arrays in one pass with Clinger's exact fast path and fall back to the reference decoder on any failure. | `invalid` | 1.6150 | 1.1976 | 1.35x |
| 9 | `216ad8b` | Reuse the per-lane effort increment while the shared step repeats. | `domain` | 0.7316 | 0.6655 | 1.10x |
| 10 | `8b2eda9` | Write batch JSON with pre-encoded keys and static strings. | `serial` | 0.1071 | 0.0876 | 1.22x |

Every round passed `cargo fmt --check`, Clippy with the `python` feature
and warnings denied, the release tests and the paired output check on all
seven workloads in full and lean mode, plus matrix mode from round 1 on.
The paired check uses `equal()` with its existing tolerances and covers
record order, status, detail, steps, provenance, scores and metrics; lean
comparisons normalize metrics to null, and matrix comparisons normalize
metrics and detail. In addition, the probe output of every saved round
binary was compared byte for byte with its predecessor's on all seven
workloads in full and lean mode (`evidence/performance-0.3.0/byte-identity.json`).
All ten pairs, and the 0.2.0 baseline against round 10, are identical, so
no round changed a floating-point result and none relied on a tolerance. The independent domain validation (`validation/validate.py`
against the release CLI) passed for the release and regenerated
`evidence/validation.json` byte for byte. Each round's timing JSON is
`evidence/performance-0.3.0/roundNN-<workload>.json`; the benchmark
binaries are saved outside the project as `round00/performance` to
`round10/performance`. Rounds 2, 3 and 7 added unit tests that compare the
lockstep, AVX2, AVX-512F and baseline kernels with the scalar kernel bit
for bit; round 9 changed the lane kernel under those tests. Rounds 1, 4,
5, 8 and 10 added tests for the matrix API, metrics key order, the trace
limit, decoder equivalence and writer byte equality. Round 6 changed only
which thread runs a range and is covered by the existing worker tests.

## Rejected attempts

| Attempt | Case | Before ms | After ms | Before / after | Evidence |
| --- | --- | ---: | ---: | ---: | --- |
| Validated fast decoding of parameter arrays through a raw slice and `str::parse::<f64>` | `invalid` | 1.8048 | 1.8890 | 0.96x | `evidence/performance-0.3.0/rejected/attempt-fast-parameter-decoding-invalid.json` |

The fast path was taken, but the raw-slice scan and the standard library
parser cost about as many instructions per short number as serde_json's
own parser. The code was reverted. Round 8 kept the same structure with a
single-pass scanner and Clinger's exact fast path. Informal diagnostics
that were never declared as rounds (eight SSE2 lanes, cheaper failure
screening, AVX-512F with eight lanes, a build without divisions) are
described in the plan.

## Cumulative native results

0.2.0 baseline benchmark binary (built from the import commit) against
the round 10 binary, `--samples 11`, alternating order, model preparation
and process start excluded. The `parallel` workload keeps its four
workers on this 2-vCPU host. The matrix rows compare 0.2.0 lean scores,
because 0.2.0 has no matrix API. Host: 2 vCPUs, Intel Xeon @ 2.10GHz
(KVM), Rust 1.97.0. Evidence: `evidence/performance-0.3.0/cumulative/`.

| Workload | 0.2.0 ms | 0.3.0 ms | Before / after |
| --- | ---: | ---: | ---: |
| Single prepared evaluation | 0.0040 | 0.0031 | 1.32x |
| 96 full records, one worker | 0.3549 | 0.0849 | 4.18x |
| 96 full records, four workers | 0.4450 | 0.1869 | 2.38x |
| 12 traced records | 0.2388 | 0.1012 | 2.36x |
| 1,024 invalid candidates | 1.3765 | 1.1183 | 1.23x |
| 48 domain cases | 6.1882 | 0.7384 | 8.38x |
| 48 secondary domain cases | 6.3913 | 0.8587 | 7.44x |
| 96 lean scores | 0.2203 | 0.0580 | 3.80x |
| 48 lean domain scores | 6.2682 | 0.6744 | 9.29x |
| 48 lean secondary scores | 6.1974 | 0.8306 | 7.46x |
| 96 matrix rows (0.2.0 lean) | 0.2371 | 0.0306 | 7.75x |
| 48 matrix domain rows (0.2.0 lean) | 6.3298 | 0.6165 | 10.27x |

The domain workloads use a 4 s horizon on a 1 ms grid (4,000 RK4 steps per
candidate) under sampled PD and held actuation. Ratios from individual
rounds do not multiply into the cumulative ratio because every pair was
timed separately under varying host load. The phase breakdown, per-call
costs, worker scaling and the cost of monitored budgets are in the
[architecture guide](docs/architecture.md#performance) and
`evidence/performance-0.3.0/breakdown.json`.

## Installed Python optimization

Seven runs per configuration of `validation/optimize_performance.py` with
the same seed, bounds, starting point, optimizer budget and final traced
replay, on one logical CPU (`taskset -c 0`). The 0.3.0 wheel was built
with maturin 1.9.6 from the extracted 0.3.0 sdist and installed into a
fresh venv; the 0.2.0 wheel from the 0.2.0 delivery went into another
fresh venv. Times include preparation, objective evaluation and the traced
winner replay, excluding imports. The total median is calculated per run.
Host: 2 vCPUs, Intel Xeon @ 2.10GHz (KVM), CPython 3.12.3, SciPy 1.17.0.

| Configuration | Prepare ms | Search ms | Replay ms | Total ms | Baseline / total |
| --- | ---: | ---: | ---: | ---: | ---: |
| v0.2.0-full | 0.101 | 8.316 | 0.181 | 8.632 | 1.00x |
| v0.2.0-scores | 0.094 | 5.146 | 0.156 | 5.386 | 1.60x |
| v0.3.0-full | 0.151 | 6.014 | 0.136 | 6.317 | 1.37x |
| v0.3.0-scores | 0.151 | 5.325 | 0.128 | 5.707 | 1.51x |
| v0.3.0-matrix | 0.142 | 4.583 | 0.151 | 4.905 | 1.76x |

The run made **48** objective calls and classified **0** failures. The
selected vector was `[5.574900894396944, 2.6972219501353756]` with replay
score **0.24281542939165648**. All 35 runs agreed on evaluation and
failure counts, final parameters and replay score exactly. Search times
of single runs varied by up to a factor of two with host load, and SciPy's
own work (about 95–175 µs per evaluation) dominates the search, so only
large differences are meaningful. Preparation in a fresh process is about
0.05 ms slower in 0.3.0 because of one-time CPU feature detection and the
larger binary; repeated preparations cost the same in both versions
(`evidence/performance-0.3.0/python-call-cost.json`, which also has the
per-call costs: `score_matrix` of one row 4.9 µs against 10.7 µs for
`scores` and 13.8 µs for `evaluate`). Evidence:
`evidence/performance-0.3.0/python-optimization.json`.

## Release checks

All checks ran under the CPU lock on the host above. Summary:
`evidence/release-0.3.0/checks.json`.

- `cargo fmt --check`, `cargo clippy --release --locked --all-targets
  --features python -- -D warnings` (with `PYO3_PYTHON=/usr/bin/python3.12`)
  and `cargo test --release --locked` (11 passed, 1 ignored timing
  harness) passed.
- The CLI request (`run models/request.json`, exit 0) and the native sweep
  passed; `evidence/cli.json` differs from 0.2.0 only in the version
  string, and `evidence/native-sweep.json` is unchanged.
- The independent domain validation `validation/validate.py` passed
  against the release CLI.
- The sdist built, the wheel built from the extracted sdist and installed
  into a fresh CPython 3.12 venv. Its ABI is CPython 3.10+ and its target
  is Linux x86-64.
- `validation/installed.py` passed: ordered replay with one and four
  workers, rejection of unknown model fields, a fixed work limit, and
  SQLite cancellation and resume that retained 21 attempts and recovered
  12 completed rows.
- `validation/release_checks.py` passed: lean and full records agree, and
  the matrix API agrees with lean scores (statuses, steps and scores, NaN
  for failures, NumPy and list input, ragged rows rejected) with mixed
  valid and invalid candidates, four workers, a step limit and a cancel
  file; the traced 10,001-step limit holds; the tutorial's 5 and the
  manual's 26 Python blocks ran.
- Two full campaign manifest runs against one database produced identical
  output. The record equals 0.2.0's; only the build hash and cache key
  changed.
- `cargo package --locked` packaged 39 files and verified the crate by
  building it.
- The final packages in `packages/` were built from the committed release
  tree: the crate by `cargo package`, the sdist by `maturin sdist`, and the
  wheel from the extracted sdist. The final wheel passed
  `validation/installed.py`, `validation/release_checks.py` and the
  identical campaign replay again in another fresh venv
  (`evidence/release-0.3.0/final-wheel-*.json`).

## Reproduction

Rebuild the benchmark binaries from Git: the import commit `9c4633e` for
0.2.0 and the round commits listed above, each with
`cargo build --release --locked --examples`. Then pair two of them, for
example:

```sh
python validation/performance_pair.py BEFORE/performance AFTER/performance models/example.json domain --samples 11
python validation/performance_pair.py BEFORE/performance AFTER/performance models/example.json serial --samples 11 --before-scores --after-matrix
```

Run the phase breakdown with
`cargo test --release --locked --lib breakdown -- --ignored --nocapture`,
and the Python optimization with
`taskset -c 0 python validation/optimize_performance.py robotics_control_craft [--scores | --matrix]`
in a venv with the installed wheel. To reconstruct the history, apply
`patches/0.3.0/*.patch` in filename order with `git am` to the 0.2.0
source tree committed as a baseline; the series covers the ten rounds,
the timing harness, the documentation and the release-check commit, and
reproduces the release tree exactly. The 0.2.0 series is in
`patches/0.2.0/`. The packages, the 0.3.0 patch files, this log and
`SHA256SUMS` were added after the series.
