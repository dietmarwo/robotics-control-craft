# robotics-control-craft performance implementation log

3 November 2026. Version 0.2.0, based on the prior 0.1.0 delivery
at `125bfba09b42`. The ZIP contains the updated source, runnable
examples, an ordered patch series, [tutorial](docs/tutorial.md),
[architecture](docs/architecture.md), raw evidence and build artifacts.
The [earlier implementation record](docs/implementation-0.1.0.md)
documents the original research scope and remaining external work.

## Ten incremental changes

Each row compares the indicated stage with its immediate predecessor.
Times are median milliseconds per prepared call. A ratio below 1.00×
means the new median was slower for that measurement. Small differences
vary with thread scheduling, code layout and host noise.

| Round | Commit | Change | Case | Before ms | After ms | Before / after |
| ---: | --- | --- | --- | ---: | ---: | ---: |
| 1 | `296f98b` | Evaluate singleton and one-worker batches on the calling thread, removing worker startup and join. | `one` | 0.040 | 0.006 | 6.53× |
| 2 | `17b668f` | Replace worker-local row collections and final sorting with one result slot per candidate index. Keep dynamic work assignment. | `parallel` | 0.552 | 0.525 | 1.05× |
| 3 | `ad4ce0b` | Validate unique candidate IDs through a pre-sized hash set. The large invalid batch isolates ID checks and response overhead from most domain computation. | `invalid` | 2.244 | 2.462 | 0.91× |
| 4 | `dec0abb` | Serialize a typed prepared batch directly and keep the public CLI result adapter. | `trace` | 0.385 | 0.272 | 1.42× |
| 5 | `f4fff5a` | Borrow candidate IDs until the batch is serialized, eliminating output-side string clones. | `invalid` | 1.595 | 1.529 | 1.04× |
| 6 | `33569f9` | Inline the unmonitored work counter. Keep cancellation, deadline and exhausted-budget checks on the guarded path. | `domain` | 8.557 | 8.463 | 1.01× |
| 7 | `95614f2` | Add Rust and Python score-only entry points, preserving status, step counts, score and record order while skipping diagnostic construction. This row compares the earlier full path with the new lean path. | `lean-serial` | 0.409 | 0.308 | 1.33× |
| 8 | `d4f579b` | Compute reciprocal mass at preparation and multiply in acceleration stages. | `domain` | 8.629 | 6.301 | 1.37× |
| 9 | `096de38` | Compute held-force RK4 weights once per control segment and accumulate effort separately from the stage state. | `domain` | 6.155 | 6.498 | 0.95× |
| 10 | `a4b32da` | Specialize the integration loop for traced and untraced candidates. | `domain` | 6.617 | 5.834 | 1.13× |

Each stage passed locked Rust release tests and paired output checks
for the supplied workloads. The result comparisons
ignore the package-version string and require matching record order,
status, detail, steps, provenance and numeric values within tolerance.
Lean comparisons also normalize metrics to null. The benchmark commit
precedes the ten rounds. Documentation and version changes follow them.

## Cumulative native results

| Workload | 0.1.0 ms | Final ms | Before / after |
| --- | ---: | ---: | ---: |
| Single prepared evaluation | 0.088 | 0.005 | 16.21× |
| 96 full records, one worker | 0.765 | 0.399 | 1.92× |
| 96 full records, four workers | 0.612 | 0.395 | 1.55× |
| 12 traced records | 0.415 | 0.210 | 1.98× |
| 1,024 invalid candidates | 2.966 | 1.766 | 1.68× |
| 48 domain cases | 9.240 | 5.677 | 1.63× |
| 48 secondary domain cases | 8.839 | 5.431 | 1.63× |
| 96 lean scores | 0.709 | 0.219 | 3.24× |
| 48 lean domain scores | 9.967 | 6.102 | 1.63× |
| 48 lean secondary scores | 9.821 | 5.698 | 1.72× |

The lean API was introduced in 0.2.0. The baseline uses full
records for the three lean rows. The paired comparisons alternate process order,
exclude startup and model preparation, warm up each prepared evaluator,
and measure JSON decode, evaluation, checks and JSON encoding. They
use five observations per side on an AMD EPYC 9V74 with Rust 1.88.0,
using logical CPUs 0–3. These are fixed test workloads, with four
workers only in the parallel row. Ratios from individual stages do
not multiply into the cumulative ratio because the timings were rerun.

The supplemental workloads use a 4 s, 1 ms nominal grid under sampled PD and held actuation. The reciprocal can alter the last bits of the trajectory and decisions at exact position thresholds. The matrix-exponential oracle and refined replay passed. The model remains the bounded oscillator.

## Installed Python optimization

Seven runs per configuration used the same seed, bounds, starting point,
optimizer budget and final traced replay. CPython 3.12 and SciPy 1.17.0
ran on one logical CPU. Times include preparation, objective evaluation
and traced winner replay, excluding Python imports and wheel compilation.
The total median is calculated per run, separately from the column medians.

| Configuration | Prepare ms | Search ms | Replay ms | Total ms | Baseline / total |
| --- | ---: | ---: | ---: | ---: | ---: |
| v0.1.0-full | 0.073 | 12.211 | 0.142 | 12.516 | 1.00× |
| v0.2.0-full | 0.068 | 11.808 | 0.102 | 12.008 | 1.04× |
| v0.2.0-scores | 0.070 | 10.913 | 0.111 | 11.089 | 1.13× |

The run made **48** objective calls and classified
**0** failures. The selected vector was
`[5.574900894396944, 2.6972219501353756]` with replay score **0.242815429392**.
All configurations agreed on evaluation and failure counts, final
parameters and full replay score within the recorded tolerance.

## Release checks

- Rust formatting, Clippy with Python enabled and warnings denied,
  release tests, CLI request and native sweep passed.
- The independent domain validation passed against the release CLI.
- The wheel built from the extracted source distribution and installed
  into a fresh CPython 3.12 environment. Its ABI is CPython 3.10+ and
  its bundled target is Linux x86-64.
- The installed lean and full paths agreed on results, including mixed
  valid/invalid candidates, worker count, fixed work limits and
  cancellation. The tutorial's 4 Python blocks ran.
- SQLite cancellation/resume retained 21
  attempts and recovered 12 completed rows.
  Two full manifest runs against the same database produced identical output.
- The Rust crate archive passed Cargo package verification.

Run the packaged `validation/performance_pair.py` with saved native
benchmark binaries to reproduce comparisons. To reconstruct the ten
rounds in Git, apply `patches/*.patch` in filename order to the source
from the 0.1.0 ZIP. A baseline may first be committed locally. The patch
series includes the benchmark and release documentation commits around
the ten implementation commits.
