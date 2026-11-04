# 0.3.0 performance plan

Host: 2 vCPUs, Intel(R) Xeon(R) Processor @ 2.10GHz (KVM), Rust 1.97.0,
CPython 3.12. Five projects share the host, and every build and timing
run holds `/home/claude/craft5/cpu.lock`.

Each round is declared here before its timing run. The decision rule is
fixed for every round: keep the round if the declared workload's median
speedup from `validation/performance_pair.py --samples 11` against the
previous kept round's binary is at least 1.03x and the paired output
check passes. Otherwise revert it and record it under `rejected/`.

## Baseline profile (0.2.0 binary, callgrind instruction counts)

| Workload | Main costs |
| --- | --- |
| `domain` | 96.6% in `domain::evaluate`, about 151 instructions per RK4 step on one serial dependency chain per candidate |
| `serial` full | domain 35%; metrics `Value` map construction, allocation and string/float serialization most of the rest |
| `serial` lean | domain 61%; candidate JSON decode and record encode the rest |
| `trace` | float formatting 23%, `Value` tree serialization and drop about 20%, allocation 7%, domain 7% |
| `invalid` | JSON number parsing about 45%, record serialization most of the rest |

## Rounds

### Round 1: typed score-matrix API

- Target workload: `serial` (96 untraced candidates, one worker). Before:
  round 0 binary in lean `scores_json` mode (`--before-scores`). After:
  round 1 binary in matrix mode (`--after-matrix`).
- Change: add `Prepared::score_matrix` (row-major `f64` parameters, no
  candidate JSON, no record JSON), its Python method and wrapper, a
  matrix API mode in `examples/performance.rs` (`CRAFT_MATRIX=1`), matrix
  normalization in `performance_pair.py` and `--matrix` in
  `validation/optimize_performance.py`.
- Expected effect: removes candidate decoding and record encoding, which
  are about 40% of lean `serial` instructions; expect about 1.4x to 1.7x.
- Floating-point results: unchanged (same kernel).

### Round 2: lockstep RK4 lanes for untraced candidates

- Target workload: `domain` (48 full records, 4,000 RK4 steps each, one
  worker), round 1 binary before, round 2 binary after, both full mode.
- Change: when a budget has no deadline and no cancel file, the engine
  groups consecutive untraced candidates with valid parameters into
  groups of four. The domain integrates each group in lockstep on the
  shared control and time grid with one shared step counter. Each lane
  performs the same floating-point operations in the same order as the
  scalar kernel, so results are bit-identical. A lane that fails is
  recorded with its own step count and masked. Traced or monitored
  candidates and a single leftover candidate use the scalar kernel.
- Expected effect: the scalar step is latency-bound on one dependency
  chain; four independent chains should give about 1.8x to 3x on
  `domain`.
- Floating-point results: bit-identical by construction (same operation
  order per lane, no fused operations).

### Round 3: AVX2 build of the lockstep lane kernel

- Target workload: `domain2` (48 full records, held actuation with four
  switches, 4,000 RK4 steps each, one worker), round 2 binary before,
  round 3 binary after.
- Profile after round 2: the lane kernel is vectorized two-wide with
  SSE2. Twelve percent fewer instructions (cheaper failure screening)
  and eight lanes both left the time unchanged, which points at the FP
  execution ports rather than instruction count or latency.
- Change: compile the same lane kernel body a second time with
  `#[target_feature(enable = "avx2")]` and select it at preparation with
  `is_x86_feature_detected!("avx2")`. AVX2 does not enable FMA, and Rust
  does not contract multiplies and adds, so each lane performs the same
  IEEE operations. Hosts without AVX2 keep the SSE2 kernel.
- Expected effect: about half the FP micro-ops per lane-step; expect
  1.3x to 1.8x on `domain2`, limited by the divider (four divisions per
  lane-step).
- Floating-point results: bit-identical (same operations and order).
- Amendment before the formal timing: informal matrix-mode pair runs
  (not kept as evidence) gave 1.15x for AVX2 with four lanes, 2.0x for
  AVX2 with eight lanes and 1.9x with twelve. With AVX2, four lanes no
  longer cover the step's dependency chain (about 87 cycles of
  multiply, add and divide latency). The round therefore also widens
  `domain::LANES` from four to eight, for both builds. The target
  workload and the decision rule are unchanged.

### Round 4: typed metrics records

- Target workload: `serial` (96 full records, 62 steps each, one
  worker), round 3 binary before, round 4 binary after, full mode.
- Profile after round 3 (`serial` full, callgrind): string serialization
  13%, allocation and free about 22%, `BTreeMap` inserts 5%, the lane
  kernel 11%. Most of this is the per-record metrics `Value` map: eight
  owned key strings, a four-string column list and boxed numbers.
- Change: the domain returns a typed `Metrics` struct that serializes
  directly. Its fields are declared in alphabetical order, the order a
  `serde_json::Value` map produces, so the JSON bytes are unchanged. The
  optional trace stays a `Value` in this round. Failure metrics keep the
  `Value` form behind an untagged enum.
- Expected effect: 1.3x to 1.6x on `serial` full; small gains on `one`
  and the full domain workloads.
- Floating-point results: unchanged (same kernel; serde_json formats the
  same `f64` values with the same formatter).

### Round 5: typed trace output

- Target workload: `trace` (12 traced full records, 62 steps each, one
  worker), round 4 binary before, round 5 binary after, full mode.
- Profile after round 3 (`trace`, callgrind): float formatting 24%,
  `Value` serialization 13%, allocation and free about 12%, `Value` drop
  6%, `to_value` of trace rows 5%, the scalar kernel 8%.
- Change: store the trace as `Option<Vec<[f64; 4]>>` in `Metrics` and
  serialize it directly, instead of converting it into a `Value` tree of
  arrays. Reserve the trace vector for the expected step count (capped
  at the 10,000-step trace limit) to avoid repeated growth.
- Expected effect: 1.3x to 1.6x on `trace` (float formatting remains).
- Floating-point results: unchanged; the same `f64` values go through
  the same formatter.

### Round 6: the calling thread works as one of the batch workers

- Target workload: `parallel` (96 full records, 62 steps each, four
  workers on this 2-vCPU host), round 5 binary before, round 6 binary
  after, full mode.
- Profile after round 5: `parallel` takes about 0.25 ms versus about
  0.11 ms for `serial` with the same candidates, so thread creation and
  joining dominate. The calling thread only waits inside the scope.
- Change: spawn `workers - 1` scoped threads (bounded by the number of
  index ranges) and run the same claim loop on the calling thread.
  Records are written to the same per-index slots, so output order and
  content do not depend on which thread evaluates a range.
- Expected effect: one fewer thread spawn and join per batch; expect
  1.15x to 1.3x on `parallel`.
- Floating-point results: unchanged.

### Round 7: validated fast decoding of parameter arrays

- Target workload: `invalid` (1,024 full records with 63 parameters
  each, one worker), round 6 binary before, round 7 binary after, full
  mode.
- Profile after round 3 (`invalid`, callgrind): serde_json number
  parsing about 45% (`parse_number`, `parse_integer`, the
  `float_roundtrip` lexical path), sequence handling 15%, record
  serialization most of the rest.
- Change: decode candidate batches first with a twin of `Candidate`
  whose `parameters` field is borrowed as a raw JSON slice, checked
  against the strict JSON number-array grammar and parsed with
  `str::parse::<f64>`. Both serde_json's `float_roundtrip` parser and
  the standard library round correctly, so every finite value is the
  same `f64`, including signed zeros and subnormals. If the fast decode
  fails for any reason (other JSON, an infinite value, or any serde
  error), the batch is decoded again with the original types, so
  results and error messages are exactly the previous ones. The CLI
  request path uses the same scheme.
- Expected effect: 1.4x to 2x on `invalid`; smaller gains on lean and
  full `serial`.
- Floating-point results: unchanged (identical parsed values); unit
  tests compare both decoders on edge cases and on 20,000 random bit
  patterns in several notations.
- Outcome: rejected at 0.955x (1.805 ms before, 1.890 ms after;
  `rejected/attempt-fast-parameter-decoding-invalid.json`). The fast
  path was taken, but the raw-slice scan plus `str::parse` costs more
  instructions per short token than serde_json's own number parser. The
  code was reverted; the next round reuses the number 7.

### Round 7 (second attempt): AVX-512 lane kernel with sixteen lanes

- Target workload: `domain` (48 full records, PD control, 4,000 RK4
  steps each, one worker), round 6 binary before, round 7 binary after.
- Diagnostics after round 3 (11-sample medians, informal): removing the
  divisions saved only 8%, and AVX-512 with eight lanes (one register
  per quantity, one dependency chain) matched AVX2 with eight lanes (two
  chains). The kernel is bound by the step's dependency-chain latency.
  AVX-512 with sixteen lanes keeps two chains of eight in 32 registers;
  an informal build measured 1.25x on `domain` against round 6.
- Change: make the lane kernel generic over its width. On hosts with
  AVX-512F, preparation selects a sixteen-lane build compiled with
  `avx512f`; otherwise eight lanes with AVX2 or the baseline build. The
  engine groups and distributes work in ranges of the selected width.
  Rust does not contract multiplies and adds even though `avx512f`
  implies FMA support, so lanes perform the same IEEE operations.
- Expected effect: 1.15x to 1.3x on `domain`; batches smaller than
  sixteen run as padded groups.
- Floating-point results: bit-identical; a unit test compares the
  AVX-512, AVX2 and baseline kernels bit for bit.

### Round 8: exact fast-path decoding of parameter arrays

- Target workload: `invalid` (1,024 full records with 63 parameters
  each, one worker), round 7 binary before, round 8 binary after, full
  mode.
- Profile after round 7 (callgrind): candidate decoding is 69% of
  `invalid` and 22% of lean `serial`. The rejected round 7 attempt spent
  about 100 instructions per number in its scanner and about 170 in
  `str::parse`, roughly serde_json's own cost.
- Change: as in the rejected attempt, decode `parameters` as a borrowed
  raw slice with fallback to the reference decoder on any failure, but
  parse each number in one tight pass: accumulate up to 19 significant
  digits into an integer `m` and a decimal exponent `e`. When
  `m <= 2^53` and `|e| <= 22`, both `m` and `10^|e|` are exact doubles,
  so one IEEE multiply or divide gives the correctly rounded value
  (Clinger's fast path), the same value serde_json's correctly rounded
  parser returns. Otherwise use `str::parse::<f64>` on the token, and
  decline non-finite results.
- Expected effect: 1.4x to 2x on `invalid`; 1.05x to 1.15x on lean
  `serial`.
- Floating-point results: unchanged (identical parsed values); unit
  tests compare the decoders on edge cases and on random values in
  several notations.

### Round 9: reuse the effort increment while the step repeats

- Target workload: `domain` (48 full records, PD control, 4,000 RK4
  steps each, one worker), round 8 binary before, round 9 binary after.
- Profile after round 7: with sixteen AVX-512 lanes the kernel issues
  eight 512-bit divisions per step (four per eight lanes: position,
  velocity, tracking error and effort).
- Change: in the lane kernel, the effort increment `h * w / 6` depends
  only on the shared step `h` and the per-segment force weight `w`. Keep
  the increment per lane and recompute it only when `h` differs from the
  cached step or a new segment starts. The grid step repeats exactly
  over long stretches, so most steps skip one division per lane. The
  reused value comes from the same operations, so it is bit-identical.
- Informal comparison before this declaration (11 alternating matrix
  samples, not kept as evidence): about 1.12x on `domain` and 1.035x on
  `domain2`.
- Expected effect: 1.05x to 1.15x on `domain`.
- Floating-point results: bit-identical.
