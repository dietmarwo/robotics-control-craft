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
