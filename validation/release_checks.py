"""Check the installed lean and matrix APIs and run the Python snippets in the docs.

Usage: release_checks.py MODULE TUTORIAL [MANUAL]. MANUAL defaults to
user-manual.md next to the tutorial when that file exists. The blocks of each
document run in order in one namespace.
"""

import importlib
import json
import math
from pathlib import Path
import re
import sys
from tempfile import TemporaryDirectory

module, tutorial, *rest = sys.argv[1:]
manual = Path(rest[0]) if rest else Path(tutorial).with_name("user-manual.md")
package = importlib.import_module(module)
example = package.load_example()
prepared = package.Prepared(example["model"])
candidates = [{"id": "ok", "parameters": example["parameters"]},
              {"id": "bad", "parameters": [1] * 63}]
with TemporaryDirectory() as tmp:
    flag = Path(tmp) / "cancel"
    flag.touch()
    for budget in ({}, {"workers": 4}, {"max_steps": 1}, {"cancel_file": str(flag)}):
        full = prepared.batch(candidates, **budget)
        lean = prepared.scores(candidates, **budget)
        assert lean["model_sha256"] == full["model_sha256"]
        assert lean["records"] == [r | {"metrics": None} for r in full["records"]]
        if "cancel_file" in budget:
            assert lean["records"][0]["status"] == "cancelled"
    base = example["parameters"]
    matrix_rows = [base, [1e9] * len(base), [x * 1.1 for x in base]]
    matrix_candidates = [{"id": f"row-{i}", "parameters": r} for i, r in enumerate(matrix_rows)]
    for budget in ({}, {"workers": 4}, {"max_steps": 1}, {"cancel_file": str(flag)}):
        lean = prepared.scores(matrix_candidates, **budget)["records"]
        scores, statuses, steps = prepared.score_matrix(matrix_rows, **budget)
        assert statuses == [r["status"] for r in lean], (budget, statuses)
        assert steps == [r["steps"] for r in lean], (budget, steps)
        for value, record in zip(scores, lean, strict=True):
            assert math.isnan(value) if record["score"] is None else value == record["score"]
    try:
        import numpy
    except ImportError:
        numpy = None
    if numpy is not None:
        from_array = prepared.score_matrix(numpy.asarray(matrix_rows))
        from_lists = prepared.score_matrix(matrix_rows)
        assert from_array[1:] == from_lists[1:]
        assert all(a == b or (math.isnan(a) and math.isnan(b))
                   for a, b in zip(from_array[0], from_lists[0], strict=True))
    try:
        prepared.score_matrix([base, base + [1.0]])
    except ValueError:
        pass
    else:
        raise AssertionError("score_matrix accepted ragged rows")
    if module == "population_epidemiology":
        calendar = package.Prepared({"kind": "calendar", "initial_on_probability": 0.5,
                                     "segments": [{"duration": 1, "on_rate": 0, "off_rate": 0}] * 3})
        query = [{"id": "calendar", "parameters": []}]
        limited = calendar.scores(query, max_steps=2)["records"][0]
        assert (limited["status"], limited["steps"]) == ("work_limit", 2)
        canceled = calendar.scores(query, cancel_file=str(flag))["records"][0]
        assert (canceled["status"], canceled["steps"]) == ("cancelled", 0)
        complete = calendar.scores(query, max_steps=3)["records"][0]
        assert (complete["status"], complete["steps"], complete["score"]) == ("complete", 3, 0.5)
    if module == "rail_craft":
        closed = package.Prepared({
            "resources": [{"name": "one", "clearance": 0}], "horizon": 20,
            "trains": [{"name": "train", "release": 0, "planned": 0, "priority": 0,
                        "route": [{"resource": 0, "duration": 1}]}],
            "closures": [{"resource": 0, "start": a, "end": b} for a, b in ((0, 4), (2, 7), (7, 8))],
        })
        row = closed.evaluate([0, 0], trace=True)
        assert row["status"] == "complete"
        assert row["metrics"]["intervals"][0]["entry"] == 8
    if module == "robotics_control_craft":
        long = package.Prepared(example["model"] | {"dt_s": 1e-4})
        assert long.evaluate(example["parameters"])["status"] == "complete"
        traced = long.evaluate(example["parameters"], trace=True)
        assert traced["status"] == "invalid" and traced["steps"] == 10_001
try:
    prepared.scores([candidates[0] | {"trace": True}])
except ValueError:
    pass
else:
    raise AssertionError("scores accepted a traced candidate")

def run_blocks(path, name):
    snippets = re.findall(r"^```python\n(.*?)^```", Path(path).read_text(), re.M | re.S)
    namespace = {}
    for i, snippet in enumerate(snippets, 1):
        exec(compile(snippet, f"{name}-{i}", "exec"), namespace)
    return len(snippets)


tutorial_blocks = run_blocks(tutorial, "tutorial")
assert tutorial_blocks >= 4
report = {"status": "passed", "tutorial_python_blocks": tutorial_blocks,
          "lean_api_and_budget_parity": True, "matrix_api_parity": True}
if manual.exists():
    report["manual_python_blocks"] = run_blocks(manual, "manual")
    assert report["manual_python_blocks"] >= 20
print(json.dumps(report))
