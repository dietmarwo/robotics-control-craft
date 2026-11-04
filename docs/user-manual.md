# Robotics Control Craft user manual

Version 0.3.0. This manual is for researchers and engineers who use the
bounded oscillator model to study, compare and optimize controllers. It
covers installation, the model and its JSON reference, every API (Python,
Rust and command line), budgets, statuses and errors, reproducibility,
performance and troubleshooting. Every Python block in this manual runs as
shown, in order, in one Python session; `validation/release_checks.py`
executes all of them against the installed package. Outputs shown after a
block come from an actual run.

Contents:

1. [Install](#1-install)
2. [Concepts](#2-concepts)
3. [Model JSON reference](#3-model-json-reference)
4. [Candidates, requests and results](#4-candidates-requests-and-results)
5. [Statuses and error messages](#5-statuses-and-error-messages)
6. [Budgets](#6-budgets)
7. [Python API](#7-python-api)
8. [Application scenarios](#8-application-scenarios)
9. [Optimization](#9-optimization)
10. [Campaigns](#10-campaigns)
11. [Rust API](#11-rust-api)
12. [Command line](#12-command-line)
13. [Reproducibility and provenance](#13-reproducibility-and-provenance)
14. [Performance tips](#14-performance-tips)
15. [Troubleshooting](#15-troubleshooting)

## 1. Install

### Wheel

The release ZIP contains a wheel for Linux x86-64 with the CPython 3.10+
stable ABI, tested with CPython 3.12. Install it into a virtual environment
from the package root:

```sh
python3 -m venv .venv
. .venv/bin/activate
python -m pip install --no-deps packages/robotics_control_craft-0.3.0-cp310-abi3-manylinux_2_34_x86_64.whl
python -m pip install numpy scipy fcmaes   # only for the examples and validation
```

The package itself has no Python dependencies. NumPy is needed only to pass
2-D arrays to `score_matrix`; SciPy and fcmaes are used by the examples and
the validation scripts.

### From source

The source distribution `packages/robotics_control_craft-0.3.0.tar.gz`
builds with Maturin 1.9.6 and Rust 1.88 or later (`rust-version` in
`Cargo.toml`; this release was built and tested with Rust 1.97). Cargo
needs the locked dependencies from its registry or cache.

```sh
python -m pip install 'maturin==1.9.6'
tar -xzf packages/robotics_control_craft-0.3.0.tar.gz
cd robotics_control_craft-0.3.0
maturin build --release --locked --out dist
python -m pip install --no-deps dist/*.whl
```

### Command line and Rust crate

```sh
cargo build --release --locked --bins --examples
target/release/robotics-control-craft describe
```

The crate archive `packages/robotics-control-craft-0.3.0.crate` can be used
as a path or vendored dependency. The `python` feature builds the PyO3
extension and is off by default.

Check the installation:

```python
import robotics_control_craft
from robotics_control_craft import Prepared, load_example

case = load_example()
prepared = Prepared(case["model"])
row = prepared.evaluate(case["parameters"])
print(row["status"], row["steps"], round(row["score"], 6))
```

```text
complete 62 0.261834
```

## 2. Concepts

- **Model.** A one-dimensional mass–spring–damper with position `x` (m) and
  velocity `v` (m/s): `m·dv/dt = F − k·x − c·v`. The force `F` (N) comes from
  the controller and is limited to `±force_limit_n`. The model also fixes
  the initial state, the target position, the integration step, the horizon
  and a position limit.
- **Control kind.** `held` applies one constant force per segment between
  fixed switch times; the candidate supplies the forces. `pd` samples the
  state every `interval_s`, computes `F = kp·(target − x) − kd·v`, saturates
  it and holds it until the next sample; the candidate supplies `[kp, kd]`.
- **Candidate.** A parameter vector to evaluate, with an ID in the JSON APIs.
- **Rollout.** Classical RK4 integration from the initial state to the
  horizon. The step shortens to land exactly on control boundaries and on
  the horizon. Each RK4 step is one logical *step* of work.
- **Score.** A value to minimize:
  `(x_T − target)² + 0.1·v_T² + (∫(x − target)² dt)/T + 0.001·∫F² dt`.
  It is reported only for complete rollouts.
- **Status.** Every candidate ends as `complete`, `invalid`, `infeasible`,
  `work_limit`, `wall_limit` or `cancelled` (section 5). A rollout is
  infeasible when `|x|` exceeds the position limit at a step endpoint.
- **Prepared model.** `Prepared(model)` validates the model once and is then
  reused for any number of calls. Each call evaluates candidates
  independently and returns them in input order.
- **Full, lean and matrix calls.** `batch` returns full records with metrics
  and optional traces, `scores` returns the same records with `metrics: null`,
  and `score_matrix` takes a numeric matrix and returns scores, statuses and
  steps without JSON. All three give identical statuses, steps and scores.
- **Budget.** Per-call settings: worker threads, a per-candidate step limit,
  an optional wall-clock deadline and an optional cancel file (section 6).

## 3. Model JSON reference

A model is a JSON object with exactly these fields; all are required, and
unknown fields are rejected. JSON integers are accepted for numbers.

| Field | Type | Unit | Allowed values | Meaning |
| --- | --- | --- | --- | --- |
| `mass_kg` | number | kg | 0.001 to 10,000 | Moving mass `m` |
| `spring_n_per_m` | number | N/m | 0 to 100,000 | Spring stiffness `k` |
| `damping_ns_per_m` | number | N·s/m | 0 to 100,000 | Viscous damping `c` |
| `initial_x_m` | number | m | `|x0| ≤ position_limit_m` | Initial position |
| `initial_v_m_s` | number | m/s | `|v0| ≤ 100,000` | Initial velocity |
| `target_m` | number | m | `|target| ≤ position_limit_m` | Target position for the error terms and the PD law |
| `dt_s` | number | s | 1e-6 to 10, and the stability envelope below | Nominal RK4 step |
| `horizon_s` | number | s | 1e-6 to 100,000 | Rollout length `T` |
| `force_limit_n` | number | N | 0.001 to 100,000 | Saturation of the PD force; bound on held forces |
| `position_limit_m` | number | m | 0.001 to 100,000 | `|x|` above this at a step endpoint is infeasible |
| `control` | object | – | see below | Control kind and its settings |

The stability envelope requires `dt_s × (sqrt(k/m) + c/m) ≤ 0.5`.

`control` is one of:

| `kind` | Other field | Type | Allowed values |
| --- | --- | --- | --- |
| `"held"` | `switches_s` | array of numbers (s) | at most 63 entries, strictly increasing, each strictly between 0 and `horizon_s` |
| `"pd"` | `interval_s` | number (s) | 1e-6 to `horizon_s` |

There are no defaults. A model that breaks a rule raises `ValueError` in
Python (or rejects the request in the CLI) with one of these messages:

| Message | Cause |
| --- | --- |
| `invalid oscillator state, coefficients, limits or time grid` | A field outside its range in the first table |
| `time step outside declared stability envelope` | `dt_s × (sqrt(k/m) + c/m) > 0.5` |
| `switch times must increase inside horizon` | Held switches not increasing, outside `(0, horizon_s)` or more than 63 |
| `invalid controller sampling interval` | PD `interval_s` outside `[1e-6, horizon_s]` |
| `unknown field ...`, `missing field ...`, `unknown variant ...` | JSON shape errors reported by the decoder |

The packaged example is a PD model:

```python
import json
print(json.dumps(case["model"], indent=1))
```

```text
{
 "mass_kg": 1,
 "spring_n_per_m": 1,
 "damping_ns_per_m": 0.4,
 "initial_x_m": 1,
 "initial_v_m_s": 0.2,
 "target_m": 0,
 "dt_s": 0.04,
 "horizon_s": 2.05,
 "force_limit_n": 3,
 "position_limit_m": 2,
 "control": {
  "kind": "pd",
  "interval_s": 0.1
 }
}
```

A held model with three segments, and the validation messages:

```python
held_model = dict(case["model"], control={"kind": "held", "switches_s": [0.4, 1.1]})
Prepared(held_model)

def rejected(model):
    try:
        Prepared(model)
    except ValueError as error:
        return str(error)
    return "accepted"

print(rejected(case["model"] | {"mass_kg": 0}))
print(rejected(case["model"] | {"dt_s": 0.5}))
print(rejected(held_model | {"control": {"kind": "held", "switches_s": [1.1, 0.4]}}))
print(rejected(case["model"] | {"control": {"kind": "pd", "interval_s": 5.0}}))
print(rejected(case["model"] | {"friction": 0.1}))
print(rejected({k: v for k, v in case["model"].items() if k != "target_m"}))
print(rejected(case["model"] | {"control": {"kind": "lqr"}}))
```

```text
invalid oscillator state, coefficients, limits or time grid
time step outside declared stability envelope
switch times must increase inside horizon
invalid controller sampling interval
unknown field `friction`, expected one of `mass_kg`, `spring_n_per_m`, `damping_ns_per_m`, `initial_x_m`, `initial_v_m_s`, `target_m`, `dt_s`, `horizon_s`, `force_limit_n`, `position_limit_m`, `control` at line 1 column 105
missing field `target_m` at line 1 column 200
unknown variant `lqr`, expected `held` or `pd` at line 1 column 24
```

## 4. Candidates, requests and results

### Candidate parameters

| Control kind | `parameters` | Rule; otherwise the status is `invalid` with this detail |
| --- | --- | --- |
| `held` | one force (N) per segment: `len(switches_s) + 1` values | every force finite with `|F| ≤ force_limit_n`; detail `one force within limits per held-control segment` |
| `pd` | `[kp, kd]`: proportional gain (N/m) and derivative gain (N·s/m) | exactly two values, each from 0 to 100,000; detail `expected nonnegative proportional and derivative gains` |

A candidate that fails these rules takes 0 steps. The batch is still
evaluated; only that candidate is invalid.

### Candidate objects (JSON APIs)

| Field | Type | Rules |
| --- | --- | --- |
| `id` | string | 1 to 128 bytes, unique within a call |
| `parameters` | array of numbers | at most 64 values; the control rules above decide validity |
| `trace` | boolean | optional, default `false`; at most 16 traced candidates per call; not allowed in `scores` |

### Request (command line)

```json
{
  "schema": "robotics-control-craft.request/0.1",
  "model": {"...": "a model object"},
  "candidates": [{"id": "a", "parameters": [4, 2], "trace": false}],
  "budget": {"workers": 1}
}
```

`budget` is optional. Unknown fields are rejected at every level.

### Result

| Field | Type | Meaning |
| --- | --- | --- |
| `model_sha256` | string | SHA-256 of the exact model bytes that were prepared |
| `records` | array | One record per candidate, in input order |
| `schema` | string | `robotics-control-craft.result/0.1` |
| `version` | string | Package version, `0.3.0` |

Each record has:

| Field | Type | Meaning |
| --- | --- | --- |
| `id` | string | Candidate ID |
| `status` | string | See section 5 |
| `steps` | integer | RK4 steps charged to the candidate |
| `score` | number or null | Objective; a finite number only for `complete` |
| `metrics` | object or null | Full calls, complete rows only; `null` in `scores` |
| `detail` | string or null | Failure explanation; `null` for `complete` |

Metrics of a complete full record:

| Key | Unit | Meaning |
| --- | --- | --- |
| `terminal_position_m` | m | `x` at the horizon |
| `terminal_velocity_m_s` | m/s | `v` at the horizon |
| `integrated_error` | m²·s | `∫(x − target)² dt` (RK4 quadrature) |
| `integrated_force_squared` | N²·s | `∫F² dt` |
| `max_abs_position_m` | m | largest `|x|` at step endpoints, including the initial state |
| `max_abs_force_n` | N | largest applied `|F|` |
| `trace_columns` | strings | always `["t_s", "x_m", "v_m_s", "held_force_n"]` |
| `trace` | array or null | rows `[t, x, v, F]` after each step when `trace` was requested; `null` otherwise |

```python
result = prepared.batch([{"id": "packaged", "parameters": case["parameters"]}])
print(sorted(result))
record = result["records"][0]
print(sorted(record))
print({k: round(v, 6) for k, v in record["metrics"].items() if isinstance(v, float)})
score_terms = (record["metrics"]["terminal_position_m"] ** 2
               + 0.1 * record["metrics"]["terminal_velocity_m_s"] ** 2
               + record["metrics"]["integrated_error"] / case["model"]["horizon_s"]
               + 0.001 * record["metrics"]["integrated_force_squared"])
print(abs(score_terms - record["score"]) < 1e-12)
```

```text
['model_sha256', 'records', 'schema', 'version']
['detail', 'id', 'metrics', 'score', 'status', 'steps']
{'integrated_error': 0.499964, 'integrated_force_squared': 4.143846, 'max_abs_force_n': 3.0, 'max_abs_position_m': 1.004752, 'terminal_position_m': -0.101846, 'terminal_velocity_m_s': 0.185272}
True
```

## 5. Statuses and error messages

| Status | Detail | Steps | Meaning |
| --- | --- | --- | --- |
| `complete` | `null` | all steps | Rollout reached the horizon; `score` is finite |
| `invalid` | `one force within limits per held-control segment` | 0 | Held parameters have the wrong count or a force outside the limit |
| `invalid` | `expected nonnegative proportional and derivative gains` | 0 | PD parameters are not two gains from 0 to 100,000 |
| `invalid` | `trace requires at most 10,000 integration steps` | 10,001 | A traced rollout needs more than 10,000 steps |
| `invalid` | `nonfinite trajectory` | step reached | The state or effort became infinite or NaN |
| `invalid` | `nonfinite objective` | all steps | The score overflowed |
| `invalid` | `time step does not advance` | step reached | The time grid stopped advancing in floating point |
| `infeasible` | `position limit exceeded` | step reached | `|x|` exceeded `position_limit_m` |
| `work_limit` | `candidate step budget reached` | `max_steps` | The candidate needed more than `max_steps` steps |
| `wall_limit` | `batch deadline reached` | step reached | The call's `wall_ms` deadline passed |
| `cancelled` | `cancel file exists` | step reached | The `cancel_file` existed when a step was about to start |

Each status from the model alone:

```python
import tempfile
from pathlib import Path

def status(model, parameters, **options):
    row = Prepared(model).evaluate(parameters, **options)
    return row["status"], row["steps"], row["detail"]

near_limit = case["model"] | {"target_m": 1.8, "initial_x_m": 0.0, "initial_v_m_s": 0.0}
print(status(case["model"], [4, 2]))
print(status(case["model"], [-1, 2]))
print(status(held_model, [-1.0, 0.0]))
print(status(held_model, [-1.0, 0.0, 4.0]))
print(status(near_limit, [30.0, 0.0]))
print(status(case["model"], [4, 2], max_steps=10))
print(status(case["model"] | {"dt_s": 1e-4}, [4, 2], trace=True))
with tempfile.TemporaryDirectory() as folder:
    flag = Path(folder) / "stop"
    flag.touch()
    print(status(case["model"], [4, 2], cancel_file=str(flag)))
long_model = case["model"] | {"dt_s": 1e-4, "horizon_s": 50.0}
print(status(long_model, [4, 2], wall_ms=1)[0])
```

```text
('complete', 62, None)
('invalid', 0, 'expected nonnegative proportional and derivative gains')
('invalid', 0, 'one force within limits per held-control segment')
('invalid', 0, 'one force within limits per held-control segment')
('infeasible', 41, 'position limit exceeded')
('work_limit', 10, 'candidate step budget reached')
('invalid', 10001, 'trace requires at most 10,000 integration steps')
('cancelled', 0, 'cancel file exists')
wall_limit
```

Errors that reject a whole call raise `ValueError` in Python, return `Err`
in Rust and exit with status 1 in the CLI:

| Message | Cause |
| --- | --- |
| `invalid budget or batch size` | Budget out of range, or 0 or more than 4,096 candidates |
| `invalid or duplicate candidate id, or parameter vector exceeds limit` | Empty, long or duplicate ID, or more than 64 parameters |
| `at most 16 traced candidates per batch` | Too many traced candidates |
| `scores mode accepts untraced candidates only` | `trace: true` in `scores` |
| `matrix rows must be between 1 and 4096` | `score_matrix` with 0 or more than 4,096 rows |
| `parameter matrix length is not a multiple of rows` | Rust `score_matrix` with a ragged flat buffer |
| `parameter vector exceeds limit` | `score_matrix` rows longer than 64 |
| `parameter rows must all have the same length` | Python `score_matrix` with ragged rows |
| `score_matrix expects a 2-D array of parameter rows` | Python `score_matrix` with a NumPy array that is not 2-D |
| `unsupported request schema` | CLI request with another schema tag |
| `request exceeds 16 MiB` | CLI input too large |
| JSON decoder messages | Malformed JSON, missing or unknown fields, wrong types |

```python
def call_error(function, *args, **budget):
    try:
        function(*args, **budget)
    except ValueError as error:
        return str(error)
    return "accepted"

one = {"id": "a", "parameters": [4, 2]}
print(call_error(prepared.batch, [one], workers=0))
print(call_error(prepared.batch, [one, one]))
print(call_error(prepared.batch, [{"id": f"t{i}", "parameters": [4, 2], "trace": True} for i in range(17)]))
print(call_error(prepared.scores, [one | {"trace": True}]))
print(call_error(prepared.score_matrix, []))
print(call_error(prepared.score_matrix, [[4, 2], [4]]))
print(call_error(prepared.batch, [{"id": "a", "parameters": [4, 2], "weight": 1}]))
```

```text
invalid budget or batch size
invalid or duplicate candidate id, or parameter vector exceeds limit
at most 16 traced candidates per batch
scores mode accepts untraced candidates only
matrix rows must be between 1 and 4096
parameter rows must all have the same length
unknown field `weight`, expected one of `id`, `parameters`, `trace` at line 1 column 38
```

## 6. Budgets

Every call accepts the same budget, as keyword arguments in Python or a JSON
object elsewhere. Unknown keys are rejected.

| Key | Type | Default | Allowed | Effect |
| --- | --- | --- | --- | --- |
| `workers` | integer | 1 | 1 to 64 | Threads for the call, including the calling thread |
| `max_steps` | integer | 1,000,000 | 1 to 100,000,000 | Step limit per candidate; reaching it gives `work_limit` |
| `wall_ms` | integer or null | none | 1 to 86,400,000 | Deadline for the whole call; later steps give `wall_limit` |
| `cancel_file` | path string or null | none | any path | If the file exists before a step, the candidate is `cancelled` |

Results do not depend on `workers`. `max_steps` is deterministic. `wall_ms`
and `cancel_file` are cooperative and checked before every step, so which
candidates finish depends on timing. These two settings also switch the call
to the one-candidate-at-a-time kernel and add a clock read or a file-system
check per step, which makes long rollouts many times slower (section 14).

```python
rows = [{"id": f"c{i}", "parameters": [4 + i, 2]} for i in range(20)]
serial = prepared.batch(rows)
parallel = prepared.batch(rows, workers=4)
print(serial["records"] == parallel["records"])
limited = prepared.scores(rows[:3], max_steps=30)
print([(r["status"], r["steps"]) for r in limited["records"]])
generous = prepared.scores(rows[:3], wall_ms=60_000)
print([r["status"] for r in generous["records"]])
```

```text
True
[('work_limit', 30), ('work_limit', 30), ('work_limit', 30)]
['complete', 'complete', 'complete']
```

## 7. Python API

```python
from robotics_control_craft import Prepared, load_example
from robotics_control_craft import campaign
```

| Name | Signature | Returns |
| --- | --- | --- |
| `load_example()` | – | `{"model": ..., "parameters": ...}`, the packaged PD example |
| `Prepared(model)` | `model`: dict | Prepared model; raises `ValueError` for an invalid model |
| `Prepared.batch` | `(candidates, **budget)` | Result dict with full records |
| `Prepared.scores` | `(candidates, **budget)` | Result dict with `metrics: null`; untraced candidates only |
| `Prepared.score_matrix` | `(rows, **budget)` | `(scores, statuses, steps)` lists; NaN score for non-complete rows |
| `Prepared.evaluate` | `(parameters, *, trace=False, **budget)` | One full record (ID `"one"`) |
| `Prepared.model_sha256` | property | Hex SHA-256 of the encoded model |
| `campaign.run` | `(database, scenarios, candidates, *, workers=1, chunk_size=16, **budget)` | Rows in manifest order (section 10) |
| `campaign.digest` | `(value)` | SHA-256 of the canonical JSON encoding |

All native work releases the GIL, so other Python threads keep running.
`batch`, `scores` and `evaluate` encode their arguments as JSON with sorted
keys and finite numbers; NaN or infinite parameters raise `ValueError` from
the encoder. `score_matrix` passes numbers directly, so NaN or infinite
parameters reach the model check and give `invalid` rows.

### `batch` and `evaluate`

```python
traced = prepared.evaluate(case["parameters"], trace=True)
print(traced["status"], traced["steps"], len(traced["metrics"]["trace"]))
print(traced["metrics"]["trace_columns"])
print([round(value, 6) for value in traced["metrics"]["trace"][-1]])
records = prepared.batch([
    {"id": "packaged", "parameters": case["parameters"]},
    {"id": "stiff", "parameters": [16, 5.5], "trace": True},
])["records"]
print([(r["id"], r["status"], r["metrics"]["trace"] is not None) for r in records])
```

```text
complete 62 62
['t_s', 'x_m', 'v_m_s', 'held_force_n']
[2.05, -0.101846, 0.185272, 0.085176]
[('packaged', 'complete', False), ('stiff', 'complete', True)]
```

### `scores`

```python
lean = prepared.scores([{"id": "packaged", "parameters": case["parameters"]},
                        {"id": "bad", "parameters": [1, 2, 3]}])
print(lean["records"])
```

```text
[{'id': 'packaged', 'status': 'complete', 'steps': 62, 'score': 0.26183385953817806, 'metrics': None, 'detail': None}, {'id': 'bad', 'status': 'invalid', 'steps': 0, 'score': None, 'metrics': None, 'detail': 'expected nonnegative proportional and derivative gains'}]
```

### `score_matrix`

Rows can be a list of lists or a 2-D NumPy array; a row may be empty, which
the PD model reports as `invalid`. The three returned lists are in row
order.

```python
import math
import numpy as np

grid = np.array([[kp, kd] for kp in (0.0, 4.0, 16.0) for kd in (0.0, 2.0, 5.5)])
scores, statuses, steps = prepared.score_matrix(grid)
print([round(s, 6) for s in scores])
print(set(statuses), set(steps))
same = prepared.scores([{"id": str(i), "parameters": row.tolist()} for i, row in enumerate(grid)])
print(all(s == r["score"] for s, r in zip(scores, same["records"])))
mixed = prepared.score_matrix([[4, 2], [-1, 2], [math.nan, 2], [1e6, 0]])
print(mixed[1], [math.isnan(s) for s in mixed[0]])
print(prepared.score_matrix([[], []]))
```

```text
[0.571764, 0.901199, 1.322546, 0.869466, 0.261834, 0.368517, 0.906258, 0.407311, 0.230627]
{'complete'} {62}
True
['complete', 'invalid', 'invalid', 'invalid'] [False, True, True, True]
([nan, nan], ['invalid', 'invalid'], [0, 0])
```

### `load_example` and `model_sha256`

```python
import hashlib

print(load_example()["parameters"])
encoded = json.dumps(case["model"], sort_keys=True, allow_nan=False, separators=(",", ":"))
print(prepared.model_sha256 == hashlib.sha256(encoded.encode()).hexdigest())
print(prepared.model_sha256 == prepared.batch([one])["model_sha256"])
```

```text
[4, 2]
True
True
```

## 8. Application scenarios

The scripts in [`examples/python/`](../examples/python/README.md) answer
complete questions; their recorded outputs are in
`examples/python/outputs/`. The blocks below are short versions.

### PD gain study

Score a gain grid in one call and inspect the trade-off between tracking
error and effort with full records
([`pd_gain_study.py`](../examples/python/pd_gain_study.py)).

```python
kp_values, kd_values = np.linspace(0, 20, 11), np.linspace(0, 8, 9)
gains = np.array([[kp, kd] for kp in kp_values for kd in kd_values])
scores, statuses, _ = prepared.score_matrix(gains)
best = gains[int(np.argmin(scores))]
print("best gains", best.tolist(), "score", round(min(scores), 6))
records = prepared.batch([{"id": str(i), "parameters": g.tolist()} for i, g in enumerate(gains)])["records"]
for i in np.argsort(scores)[:3]:
    m = records[i]["metrics"]
    print(gains[i].tolist(), round(m["integrated_error"], 4), round(m["integrated_force_squared"], 4))
```

```text
best gains [14.0, 5.0] score 0.230648
[14.0, 5.0] 0.4555 8.4195
[18.0, 6.0] 0.4538 9.4014
[20.0, 7.0] 0.4546 9.0815
```

### Held actuation schedules

Compare force schedules for the same switch times
([`held_actuation_and_convergence.py`](../examples/python/held_actuation_and_convergence.py)).

```python
schedule_model = case["model"] | {
    "dt_s": 0.01, "horizon_s": 4.0,
    "control": {"kind": "held", "switches_s": [0.5, 1.0, 2.0, 3.0]},
}
schedules = np.array([[-3.0, -1.0, 0.0, 0.0, 0.0], [-3.0, 3.0, -1.0, 0.5, 0.0], [-1.0, -0.5, 0.0, 0.25, 0.0]])
scores, statuses, steps = Prepared(schedule_model).score_matrix(schedules)
print([round(s, 6) for s in scores], statuses, steps)
```

```text
[1.104367, 0.315276, 0.407857] ['complete', 'complete', 'complete'] [402, 402, 402]
```

### Step-size convergence and refined replay

RK4 converges with fourth order; the terminal state approaches the exact
segment-wise solution. A study should confirm that its conclusions hold at
a smaller step.

```python
from scipy.linalg import expm

def exact_terminal(model, forces):
    state = np.array([model["initial_x_m"], model["initial_v_m_s"], 1.0])
    times = [0.0, *model["control"]["switches_s"], model["horizon_s"]]
    m, k, c = model["mass_kg"], model["spring_n_per_m"], model["damping_ns_per_m"]
    for lo, hi, force in zip(times[:-1], times[1:], forces):
        state = expm(np.array([[0, 1, 0], [-k / m, -c / m, force / m], [0, 0, 0]]) * (hi - lo)) @ state
    return state[:2]

reference = exact_terminal(schedule_model, schedules[0])
for dt in (0.04, 0.02, 0.01):
    m = Prepared(schedule_model | {"dt_s": dt}).evaluate(schedules[0].tolist())["metrics"]
    error = max(abs(reference - [m["terminal_position_m"], m["terminal_velocity_m_s"]]))
    print(dt, f"{error:.2e}")
```

```text
0.04 7.19e-08
0.02 4.49e-09
0.01 2.78e-10
```

### Scenario robustness

Evaluate controllers over sampled plants; prepare each scenario once
([`scenario_robustness.py`](../examples/python/scenario_robustness.py)).

```python
import random

rng = random.Random(3)
scenarios = [case["model"] | {"mass_kg": rng.uniform(0.8, 1.2), "damping_ns_per_m": rng.uniform(0.25, 0.65)}
             for _ in range(12)]
controllers = [[4.0, 2.0], [16.0, 5.5]]
table = np.array([Prepared(s).score_matrix(controllers)[0] for s in scenarios])
print("mean", table.mean(axis=0).round(6).tolist(), "worst", table.max(axis=0).round(6).tolist())
```

```text
mean [0.266021, 0.23358] worst [0.300985, 0.256365]
```

### Feasibility near the position limit

```python
gains = np.array([[kp, kd] for kp in (2.0, 8.0, 30.0) for kd in (0.0, 1.0, 3.0)])
_, statuses, steps = Prepared(near_limit).score_matrix(gains)
for g, s, n in zip(gains.tolist(), statuses, steps):
    print(g, s, n)
```

```text
[2.0, 0.0] infeasible 49
[2.0, 1.0] complete 62
[2.0, 3.0] complete 62
[8.0, 0.0] infeasible 41
[8.0, 1.0] infeasible 43
[8.0, 3.0] complete 62
[30.0, 0.0] infeasible 41
[30.0, 1.0] infeasible 41
[30.0, 3.0] infeasible 43
```

### Sensitivity to plant parameters

Central differences over perturbed models
([`parameter_sensitivity.py`](../examples/python/parameter_sensitivity.py)).

```python
base_model = case["model"] | {"dt_s": 0.01}
nominal_score = Prepared(base_model).evaluate([8.0, 3.0])["score"]
for name in ("mass_kg", "damping_ns_per_m", "force_limit_n"):
    value = base_model[name]
    up = Prepared(base_model | {name: value * 1.01}).evaluate([8.0, 3.0])["score"]
    down = Prepared(base_model | {name: value * 0.99}).evaluate([8.0, 3.0])["score"]
    print(name, round((up - down) / (0.02 * nominal_score), 4))
```

```text
mass_kg 0.6475
damping_ns_per_m 0.006
force_limit_n -0.3527
```

### Traces for replay

Request traces only for selected candidates. A traced rollout can have at
most 10,000 steps; shorten the horizon or enlarge `dt_s` for inspection.

```python
replay = Prepared(near_limit | {"horizon_s": 1.0, "dt_s": 0.1}).evaluate([8.0, 3.0], trace=True)
for t, x, v, force in replay["metrics"]["trace"][:4]:
    print(round(t, 3), round(x, 5), round(v, 5), round(force, 5))
```

```text
0.1 0.01479 0.29359 3.0
0.2 0.05824 0.57279 3.0
0.3 0.1288 0.83544 3.0
0.4 0.22471 1.0796 3.0
```

## 9. Optimization

Return a penalty for every non-complete row so that optimizers move away
from infeasible and invalid regions. Score populations in one call where
the optimizer allows it.

### SciPy, whole populations

`differential_evolution(vectorized=True)` passes an array of shape
`(dim, population)`; one `score_matrix` call scores it
([`scipy_de_tuning.py`](../examples/python/scipy_de_tuning.py)).

```python
from scipy.optimize import differential_evolution

near_prepared = Prepared(near_limit)

def population_objective(x):
    scores, statuses, _ = near_prepared.score_matrix(x.T)
    return np.where(np.array(statuses) == "complete", scores, 10.0)

result = differential_evolution(population_objective, [(0, 100), (0, 30)], seed=1, popsize=8,
                                maxiter=20, vectorized=True, updating="deferred", polish=False)
print(np.round(result.x, 3).tolist(), round(result.fun, 6))
```

```text
[43.597, 11.369] 1.012918
```

### SciPy, one candidate per call

```python
from scipy.optimize import minimize

def objective(x):
    scores, statuses, _ = prepared.score_matrix([x.tolist()])
    return scores[0] if statuses[0] == "complete" else 1e3

local = minimize(objective, [4.0, 2.0], method="Nelder-Mead", bounds=[(0, 40), (0, 20)])
print(np.round(local.x, 3).tolist(), round(local.fun, 6))
```

```text
[15.55, 5.435] 0.230608
```

### fcmaes ask/tell

fcmaes optimizers with an ask/tell interface hand out a whole population;
score it with one `score_matrix` call
([`fcmaes_cmaes_held_schedule.py`](../examples/python/fcmaes_cmaes_held_schedule.py)).

```python
from loguru import logger
from scipy.optimize import Bounds
from fcmaes import cmaes

logger.remove()
generator = np.random.default_rng(4)
es = cmaes.Cmaes(bounds=Bounds([-3.0] * 5, [3.0] * 5), x0=np.zeros(5), input_sigma=1.0, popsize=16,
                 rg=generator, randn=lambda *shape: generator.standard_normal(shape))
held_prepared = Prepared(schedule_model)
for generation in range(40):
    xs = es.ask()
    scores, statuses, _ = held_prepared.score_matrix(xs)
    if es.tell(np.where(np.array(statuses) == "complete", scores, 10.0)):
        break
print(round(float(min(scores)), 6), np.round(xs[int(np.argmin(scores))], 3).tolist())
```

```text
0.1447 [-3.0, 3.0, 0.062, 0.289, -0.051]
```

### fcmaes standard minimizers

`bitecpp.minimize`, `cmaescpp.minimize` and `de.minimize` call a scalar
objective ([`fcmaes_bite_robust_pd.py`](../examples/python/fcmaes_bite_robust_pd.py)).

```python
from fcmaes import bitecpp

robust_models = [Prepared(s | {"target_m": 1.8, "initial_x_m": 0.0, "initial_v_m_s": 0.0})
                 for s in scenarios[:4]]

def robust(x):
    total = 0.0
    for p in robust_models:
        scores, statuses, _ = p.score_matrix([x.tolist()])
        total += scores[0] if statuses[0] == "complete" else 5.0
    return total / len(robust_models)

found = bitecpp.minimize(robust, Bounds([0.0, 0.0], [40.0, 4.0]), x0=[5.0, 2.0],
                         max_evaluations=400, rg=np.random.default_rng(2))
print(np.round(found.x, 3).tolist(), round(found.fun, 6))
```

```text
[14.871, 4.0] 0.995672
```

After any search, replay the winner with full metrics, at a smaller step
and on held-out scenarios before drawing conclusions.

## 10. Campaigns

`campaign.run(database, scenarios, candidates, *, workers=1, chunk_size=16,
**budget)` evaluates every candidate on every scenario and stores results in
SQLite. `scenarios` is a list of `{"id", "model"}` and `candidates` a list
of candidate objects; IDs must be unique within each list, and `chunk_size`
is 1 to 256. It returns one row per scenario and candidate, scenario-major,
with `scenario`, `key`, `build_sha256`, `model_sha256` and `record`.

The cache key is the SHA-256 of the extension build, the scenario ID, the
model, the candidate and the budget without `cancel_file`. Complete,
invalid and infeasible records are reused; canceled and time-limited
attempts stay in the `attempts` journal and are retried on the next run.

```python
with tempfile.TemporaryDirectory() as folder:
    database = Path(folder) / "campaign.sqlite"
    study = [{"id": "nominal", "model": case["model"]}, {"id": "heavy", "model": case["model"] | {"mass_kg": 1.2}}]
    trials = [{"id": "packaged", "parameters": [4, 2]}, {"id": "stiff", "parameters": [16, 5.5]}]
    first = campaign.run(database, study, trials, workers=2)
    again = campaign.run(database, study, trials)
    print(first == again, [(r["scenario"], r["record"]["id"], round(r["record"]["score"], 6)) for r in first])
    flag = Path(folder) / "stop"
    flag.touch()
    more = trials + [{"id": "soft", "parameters": [2, 1]}]
    interrupted = campaign.run(database, study, more, cancel_file=str(flag))
    print([r["record"]["status"] for r in interrupted])
    flag.unlink()
    resumed = campaign.run(database, study, more)
    print([r["record"]["status"] for r in resumed])
```

```text
True [('nominal', 'packaged', 0.261834), ('nominal', 'stiff', 0.230627), ('heavy', 'packaged', 0.303248), ('heavy', 'stiff', 0.256491)]
['complete', 'complete', 'cancelled', 'complete', 'complete', 'cancelled']
['complete', 'complete', 'complete', 'complete', 'complete', 'complete']
```

The manifest runner uses the same function:

```sh
python validation/run_campaign.py robotics_control_craft models/campaign.json campaign.sqlite > first.json
python validation/run_campaign.py robotics_control_craft models/campaign.json campaign.sqlite > replay.json
cmp first.json replay.json
```

## 11. Rust API

The crate `robotics_control_craft` exposes:

| Item | Signature | Notes |
| --- | --- | --- |
| `Prepared::new` | `fn new(model: &str) -> Result<Prepared, String>` | Model JSON text; hashes the exact bytes |
| `Prepared::evaluate_json` | `fn evaluate_json(&self, candidates: &str, budget: &str) -> Result<String, String>` | Full result JSON |
| `Prepared::scores_json` | `fn scores_json(&self, candidates: &str, budget: &str) -> Result<String, String>` | Lean result JSON |
| `Prepared::score_matrix` | `fn score_matrix(&self, rows: usize, parameters: &[f64], budget: &str) -> Result<Vec<MatrixScore>, String>` | Row-major parameters |
| `Prepared::model_sha256` | `fn model_sha256(&self) -> &str` | Hex hash of the model bytes |
| `MatrixScore` | `struct { status: &'static str, steps: u64, score: Option<f64> }` | `score` is `Some` only for `"complete"` |
| `run` | `fn run(text: &str) -> Result<serde_json::Value, String>` | Whole request, as the CLI uses it |
| `capabilities` | `fn capabilities() -> serde_json::Value` | The `describe` document |

A `Prepared` is `Sync`; share it between threads by reference. The
example [`examples/matrix.rs`](../examples/matrix.rs) runs a gain grid:

```rust
use robotics_control_craft::Prepared;

let prepared = Prepared::new(&model_json)?;
let gains: Vec<[f64; 2]> = (0..=20)
    .flat_map(|p| (0..=16).map(move |d| [f64::from(p), f64::from(d) * 0.5]))
    .collect();
let parameters: Vec<f64> = gains.iter().flatten().copied().collect();
let rows = prepared.score_matrix(gains.len(), &parameters, r#"{"workers":2}"#)?;
```

```sh
cargo run --release --locked --example matrix -- models/example.json
```

```text
357 rows, 357 complete; best kp=16 kd=5.5 score=0.23062720947966966 steps=62
model sha256 f272fedc71a1f36d95e7ced0fd32ad04255f1d82f2b67044a2de4e50fa9510f7
```

`examples/sweep.rs` scales the packaged gains and prints the best full
record with `evaluate_json`:

```sh
target/release/examples/sweep models/example.json
```

```text
{"detail":null,"id":"4","metrics":{"integrated_error":0.4846115392329917,"integrated_force_squared":4.683341872923213,"max_abs_force_n":3.0,"max_abs_position_m":1.004751648768,"terminal_position_m":-0.05998901457910427,"terminal_velocity_m_s":0.14412306659006174,"trace":null,"trace_columns":["t_s","x_m","v_m_s","held_force_n"]},"score":0.24675504237200888,"status":"complete","steps":62}
```

## 12. Command line

```text
robotics-control-craft describe
robotics-control-craft run FILE
robotics-control-craft run -        (read the request from standard input)
```

`describe` prints the capability document:

```sh
target/release/robotics-control-craft describe
```

```text
{"batch":true,"models":["bounded_mass_spring_damper","piecewise_constant_actuation","sampled_pd","scenario_replay"],"schema":"robotics-control-craft.capabilities/0.1","version":"0.3.0","worker_limit":64}
```

`run` evaluates one request (at most 16 MiB) and prints the result. The
exit status is 0 when every record is complete, 2 when the request was valid
but at least one record is not complete, and 1 when the request is rejected;
a rejection prints `{"detail": ..., "status": "rejected"}` on standard
error. The CLI prints maps with sorted keys.

```sh
target/release/robotics-control-craft run - <<'EOF'
{"schema": "robotics-control-craft.request/0.1",
 "model": {"mass_kg": 1, "spring_n_per_m": 1, "damping_ns_per_m": 0.4, "initial_x_m": 1,
           "initial_v_m_s": 0.2, "target_m": 0, "dt_s": 0.04, "horizon_s": 2.05,
           "force_limit_n": 3, "position_limit_m": 2, "control": {"kind": "pd", "interval_s": 0.1}},
 "candidates": [{"id": "packaged", "parameters": [4, 2]},
                {"id": "negative", "parameters": [-1, 2]}],
 "budget": {"workers": 1}}
EOF
echo "exit $?"
```

```text
{"model_sha256":"e3ceef7647bb96de775ac0d7ddfef9f4680a090f0e48461a158d66a3d23bd06e","records":[{"detail":null,"id":"packaged","metrics":{"integrated_error":0.49996410758532783,"integrated_force_squared":4.143845804875482,"max_abs_force_n":3.0,"max_abs_position_m":1.004751648768,"terminal_position_m":-0.10184556515463423,"terminal_velocity_m_s":0.18527180201551174,"trace":null,"trace_columns":["t_s","x_m","v_m_s","held_force_n"]},"score":0.26183385953817806,"status":"complete","steps":62},{"detail":"expected nonnegative proportional and derivative gains","id":"negative","metrics":null,"score":null,"status":"invalid","steps":0}],"schema":"robotics-control-craft.result/0.1","version":"0.3.0"}
exit 2
```

The hash differs from the Python one for the same model because the bytes
differ: the CLI hashes the model text exactly as written in the request.

A rejected request and the usage message:

```sh
target/release/robotics-control-craft run - <<'EOF'
{"schema": "robotics-control-craft.request/9.9", "model": {}, "candidates": []}
EOF
echo "exit $?"
target/release/robotics-control-craft
echo "exit $?"
```

```text
{"detail":"unsupported request schema","status":"rejected"}
exit 1
{"detail":"usage: robotics-control-craft describe | run FILE (use - for stdin)","status":"rejected"}
exit 1
```

The benchmark binary prints seconds per prepared call after warm-up for one
of the workloads `one`, `serial`, `parallel`, `trace`, `invalid`, `domain`
and `domain2`. `CRAFT_SCORES=1` selects the lean API and `CRAFT_MATRIX=1`
the matrix API. Times depend on the host; these are from the release host:

```sh
target/release/examples/performance models/example.json serial
CRAFT_MATRIX=1 target/release/examples/performance models/example.json serial
```

```text
0.0000929835
0.000029997
```

`validation/performance_pair.py BEFORE AFTER models/example.json WORKLOAD`
checks that two benchmark binaries give the same records and times them
alternately; `--before-scores`, `--after-scores`, `--before-matrix` and
`--after-matrix` select the API per side.

## 13. Reproducibility and provenance

- **Model identity.** `model_sha256` is the SHA-256 of the exact model bytes
  prepared. The Python wrapper encodes models with sorted keys and compact
  separators, so equal dictionaries give equal hashes; the CLI hashes the
  request's model text as written.
- **Versions.** Results carry `schema` (`robotics-control-craft.result/0.1`)
  and `version` (`0.3.0`). The schema tag changes only when the record shape
  changes.
- **Determinism.** With a fixed model, candidate and `max_steps`, results are
  identical across calls, worker counts, batch compositions and API
  (`batch`, `scores`, `score_matrix`). The lockstep lane kernels (AVX-512F,
  AVX2 and baseline builds) perform the same IEEE operations as the scalar
  kernel without fused multiply-add, so the choice of kernel on a host does
  not change any bit of the result; the unit tests compare the kernels bit
  for bit. Version 0.3.0 reproduces 0.2.0 records byte for byte on every
  benchmark workload.
- **Timing-dependent settings.** `wall_ms` and `cancel_file` can change which
  candidates finish and at which step.
- **Campaigns.** Campaign keys include the SHA-256 of the installed extension,
  so a new build re-evaluates instead of reusing results across builds.
- **Validation.** `validation/validate.py BINARY` checks terminal states
  against a matrix-exponential oracle for under-, critically and overdamped
  plants, refinement order, switch landing, a held-out scenario comparison
  and a refined replay of an error/effort archive. It writes
  `evidence/validation.json` and regenerates `models/example.json`.

```python
again = Prepared(json.loads(json.dumps(case["model"])))
print(again.model_sha256 == prepared.model_sha256)
print(prepared.score_matrix(grid) == prepared.score_matrix(grid, workers=3))
```

```text
True
True
```

## 14. Performance tips

Measured numbers and their evidence are in the
[architecture guide](architecture.md#performance).

- **Prefer `score_matrix` in optimization loops.** It skips JSON in both
  directions. Through Python it costs about 0.8 µs per row in batches of
  96 short rollouts, against 3.5 µs for `scores` and 4.4 µs for `batch`; a
  single row costs about 5 µs against 11–14 µs.
- **Send populations, not single candidates.** Untraced rows are integrated
  in lockstep groups of 16 (hosts with AVX-512F) or 8. Batches of at least
  one group use the fast kernel; a single row uses the scalar kernel.
- **Avoid `wall_ms` and `cancel_file` in tight loops.** They force the scalar
  kernel and add a per-step check. On the release host 48 rollouts of 4,000
  steps took 0.63 ms without them, 13 ms with `wall_ms` and 97 ms with a
  `cancel_file`.
- **Choose workers by work per call.** Thread start-up costs tens of
  microseconds. Keep `workers=1` for cheap rollouts; use two workers or more
  only for calls with hundreds of thousands of steps and at least two groups
  per worker. More workers than CPUs do not help.
- **Trace only for replay.** Trace output is dominated by number formatting.
- **Prepare once.** `Prepared` validates and hashes the model; reuse it for
  all candidates of one scenario. The first preparation in a process takes
  about 0.15 ms, later ones a few microseconds.

```python
from time import perf_counter

population = np.column_stack([np.linspace(0, 20, 64), np.linspace(0, 8, 64)])
start = perf_counter()
for _ in range(20):
    prepared.score_matrix(population)
matrix_seconds = perf_counter() - start
candidates = [{"id": str(i), "parameters": row.tolist()} for i, row in enumerate(population)]
start = perf_counter()
for _ in range(20):
    prepared.batch(candidates)
batch_seconds = perf_counter() - start
print("score_matrix is faster than batch:", matrix_seconds < batch_seconds)
```

```text
score_matrix is faster than batch: True
```

## 15. Troubleshooting

| Symptom | Cause and fix |
| --- | --- |
| `ValueError: invalid oscillator state, coefficients, limits or time grid` | A model field is out of range or the initial or target position exceeds `position_limit_m`; see section 3 |
| `ValueError: time step outside declared stability envelope` | Reduce `dt_s` until `dt_s × (sqrt(k/m) + c/m) ≤ 0.5` |
| Every row is `invalid` with a PD message | Parameters must be exactly `[kp, kd]`, both from 0 to 100,000 |
| Every row is `invalid` with a held message | Supply `len(switches_s) + 1` forces within `force_limit_n` |
| Traced rows are `invalid` with 10,001 steps | The rollout needs more than 10,000 steps; trace a shorter horizon or a larger step |
| Rows are `work_limit` | Raise `max_steps`; a rollout needs about `horizon_s / dt_s` steps plus one per control boundary |
| `ValueError: Out of range float values are not JSON compliant` | NaN or infinity in `batch`, `scores` or `evaluate`; use finite numbers, or `score_matrix`, which reports such rows as `invalid` |
| `ValueError: scores mode accepts untraced candidates only` | Remove `trace` from candidates passed to `scores`, or use `batch` |
| `ValueError: parameter rows must all have the same length` | `score_matrix` needs equal-length rows; pad or split by control kind |
| Optimization is slow | Batch populations through `score_matrix` and remove `wall_ms` and `cancel_file` from the loop (section 14) |
| Results differ between runs | Only `wall_ms` and `cancel_file` make results timing-dependent; also check that the model dictionaries are equal (`model_sha256`) |
| `ImportError` for `_native` | The wheel targets Linux x86-64 and CPython 3.10+; build from the source distribution on other platforms |
| A campaign re-evaluates everything | The extension build changed (new wheel), or the model, candidates or budget changed; all are part of the cache key |
