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
