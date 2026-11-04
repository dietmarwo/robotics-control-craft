# Robotics Control Craft tutorial

Optimize sampled control and held actuation for a bounded mass–spring–damper oscillator. This guide uses the examples shipped with version 0.2.0.

## 1. Install and run the example

Open the package root after extracting the ZIP. The bundled wheel was
built for Linux x86-64 with a CPython 3.10+ stable ABI and tested with
CPython 3.12. Create a virtual environment with a compatible Python.

```sh
python3 -m venv .venv
. .venv/bin/activate
python -m pip install --no-deps packages/*.whl
```

If the wheel tag is incompatible, build the supplied Rust/Python source
on the target system with Rust 1.88+ and Maturin 1.9.6. Cargo needs the
locked dependencies available from its registry or cache.

```sh
python -m pip install 'maturin==1.9.6'
maturin build --release --locked --out dist
python -m pip install --no-deps dist/*.whl
```

```python
from robotics_control_craft import Prepared, load_example

case = load_example()
prepared = Prepared(case["model"])
row = prepared.evaluate(case["parameters"], trace=True)
assert row["status"] == "complete", row
print(row["id"], row["steps"], row["score"])
print(list(row["metrics"]))
```

The model is validated when you create `Prepared`. A malformed model
raises `ValueError`. A well-formed batch can still contain an invalid,
infeasible, or interrupted candidate. Its row carries a status and
detail. The optimizer convention is to minimize `score`.

## 2. Run the domain workflow

The following examples continue in the same Python session.

### Inspect a sampled controller

The fixture starts at 1 m with velocity 0.2 m/s. Parameters are
proportional and derivative gains. The controller samples at its own
interval, computes a force from state, clips to the force limit and
holds that command until its next sample. The time integration step
can shorten at those boundaries and at the horizon.

```python
for gains in [[0, 0], [2, 1], [4, 2]]:
    row = prepared.evaluate(gains, trace=True)
    if row["status"] == "complete":
        m = row["metrics"]
        print(gains, row["score"], m["terminal_position_m"],
              m["integrated_force_squared"])
        print(m["trace_columns"], m["trace"][-1])
```

Search on the example grid, then replay the winner at half the step
with the controller interval fixed. Compare terminal position and
velocity, max absolute displacement, tracking integral and effort.
Change mass and damping in copies of the model to run held-out
scenarios. Persist the scenario identities and all score terms.

### Use held actuation

```python
model = dict(case["model"], control={"kind": "held", "switches_s": [0.4, 1.1]})
row = Prepared(model).evaluate([-1.0, -0.5, 0.0], trace=True)
assert row["status"] == "complete", row
print(row["metrics"]["trace"][-1])
```

Supply one force in newtons per interval. The last force applies
until the horizon. Use increasing switch times strictly inside the
horizon. For longer trajectories, lower the reporting horizon or use
an untraced run when more than 10,000 integration steps would be needed.
The independent validation compares terminal states to a matrix
exponential for under-, critically and overdamped configurations.

## 3. Batch candidates and replay the winner

```python
candidates = [
    {"id": "first", "parameters": case["parameters"]},
    {"id": "second", "parameters": case["parameters"]},
]
scores = prepared.scores(candidates, workers=2)["records"]
assert [row["id"] for row in scores] == ["first", "second"]
winner = min((r for r in scores if r["status"] == "complete"),
             key=lambda r: r["score"])
candidate = next(c for c in candidates if c["id"] == winner["id"])
replay = prepared.evaluate(candidate["parameters"], trace=True)
assert replay["status"] == "complete", replay
print(replay["score"], replay["metrics"])
```

Replace the duplicate vectors with distinct choices for a real search.
Use `batch` if you need the full metrics for every row. `scores` returns
ordered records with score, status, steps and detail, and `metrics: null`.
It rejects any candidate with `trace: true`. A one-worker batch or a
singleton runs inline. Larger batches can try two or four workers.
Measure before setting a worker count for a real workload.

The optional differential-evolution driver times preparation, seeded
search and traced replay as one run. Install SciPy for this example.

```sh
python -m pip install scipy
python validation/optimize_performance.py robotics_control_craft --scores
python validation/optimize_performance.py robotics_control_craft
```

The driver uses small local bounds and a short fixed search budget.
For a scientific result, use problem-specific bounds, an explicit
feasibility penalty or constraint policy, and a held-out evaluation.

## 4. Run and resume a local campaign

```sh
python validation/run_campaign.py robotics_control_craft models/campaign.json campaign.sqlite > first.json
python validation/run_campaign.py robotics_control_craft models/campaign.json campaign.sqlite > replay.json
cmp first.json replay.json
```

The manifest holds scenario models and candidate vectors. Keep IDs
unique inside each list. Reuse the database to resume identical work.
The native build, model, scenario, candidate and work settings are
part of cache identity. A new wheel re-evaluates cached work under
its own build identity. A cancel file passed as `cancel_file` or a
`wall_ms` budget can stop new evaluations cooperatively. Inspect
statuses in the attempt journal before rerunning.

## 5. Use the Rust CLI and benchmark

```sh
cargo build --release --locked --bins --examples
target/release/robotics-control-craft describe
target/release/robotics-control-craft run models/request.json
target/release/examples/sweep models/example.json
target/release/examples/performance models/example.json serial
CRAFT_SCORES=1 target/release/examples/performance models/example.json serial
```

The CLI reads a request file or standard input with `run -`. Exit code
0 indicates complete rows, 1 indicates a rejected request and 2
indicates one or more unsuccessful rows in a well-formed request.
The benchmark prints elapsed seconds per prepared batch after warm-up.
Use `parallel` for four workers, `trace` for traced evaluations,
or `domain` and `domain2` for the package's supplemental cases.
With a previous benchmark executable built from the baseline and this
one, `validation/performance_pair.py BEFORE AFTER models/example.json serial`
checks output parity and times both. See the included performance log
and raw evidence for the exact workloads and comparison boundaries.

## 6. Verify results

```sh
cargo test --release --locked
cargo clippy --locked --all-targets --features python -- -D warnings
python validation/validate.py target/release/robotics-control-craft
```

Consult the [model card](model.md) for supported inputs, the
[architecture guide](architecture.md) for data flow and numerical
assumptions, and the [source notes](sources.md) for references.
