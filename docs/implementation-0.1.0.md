# robotics-control-craft implementation log

3 November 2026. Version 0.1.0. This delivery uses the bounded research
scope recorded below. The independent references and optional comparisons
are identified separately for each increment.

## Increment results

| Increment | Status | Implementation and evidence |
| --- | --- | --- |
| 0 | Oscillator fallback | The pinned source and legacy pushing environment could not be reconstructed. Specified an independent bounded oscillator and matrix-exponential oracle. |
| 1 | Implemented | Installable Rust and Python oscillator evaluator with held and sampled-PD control, validation, cooperative cancellation and result records. |
| 2 | Implemented | RK4 aligns with control switches and horizon. Under-, critically and overdamped examples converge under refinement and match the SciPy reference. |
| 3 | Blocked | The Rapier dependency was unavailable offline. No rigid-body adapter was validated. |
| 4 | Blocked | The matched planar pushing task and backend comparison require the missing runtime and legacy inputs. The package exposes oscillator capabilities. |
| 5 | Implemented for oscillator | Immutable prepared inputs, complete state reset and bounded native batches. Candidate order and worker count preserve records. |
| 6 | Implemented for oscillator | Twenty gain combinations plus optional in-process SciPy DE. Winners are checked at a smaller step and on fresh scenarios. SQP refinement was deferred. |
| 7 | Implemented for oscillator | Bounded mass and damping scenarios are sampled independently of control. Training and holdout use separate seeds. Held-out score and failures are retained. |
| 8 | Local fallback | An error/effort archive built from the gain grid is refined-time rechecked. A local SQLite campaign exercises resume. An external worker and contact-rich quality-diversity campaign remain open. |
| 9 | Oscillator release prepared | Source, Rust crate, sdist and wheel pass installation checks. This release does not include pushing or contact mechanics. |

## Numerical results

At a 0.02 s step the largest terminal state component error versus the matrix exponential was **1.91e-09** in the corresponding SI component. Mean holdout score was **0.25754** versus 0.56852 for zero gains. The grid produced 14 occupied descriptor cells that passed refined replay.

The optional in-process SciPy differential-evolution demonstration made
48 evaluations and replayed its selected candidate.
It used the packaged example and its own local bounds. The domain-specific
grid and holdout evidence are in `evidence/validation.json`.

## Installed performance and replay

The following are medians of three 96-candidate batch observations,
measured in a clean CPython 3.12 environment with the installed wheel.
The scenarios and candidate inputs are identical within a row pair.
Times include native crossing and compact JSON result conversion.

| Boundary | Time |
| --- | ---: |
| Cold preparation and batch, one worker | 1.428 ms |
| Prepared batch, one worker | 1.161 ms |
| Prepared batch, four workers | 0.933 ms |

The replay test reversed candidate order and changed worker count. It
recovered 12 completed SQLite records across cancellation and retained
21 attempts. This campaign uses a local database and package-build
identity. Cross-machine determinism and distributed worker behavior
were outside the test scope.

## Release evidence

- `cargo fmt`, Clippy with warnings denied and locked Rust release tests passed.
- The native sweep and numerical validation ran against a release build.
- A wheel built from the extracted sdist and installed offline into a fresh
  environment. The wheel target is Linux x86-64, CPython 3.10+ ABI. Actual
  verification used CPython 3.12 on x86_64.
- The standalone Rust crate passed Cargo package verification.
- Model and source manifests are in `docs/`. Raw timing, numerical and
  clean-install results are in `evidence/`.

The source was staged in successive local commits. Each increment above
records its own outcome, while related implementation changes share
commits.

- `8281ee4` Define versioned evaluation and packaging boundaries
- `e27130a` Implement bounded domain evaluator and resumable local execution
- `193b65c` Validate model laws and add bounded search and installation examples
- `a61c76a` Prepare source distributions and document validated research scope
- `125bfba` Canonicalize campaign replay output
