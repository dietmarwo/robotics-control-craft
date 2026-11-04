"""PD gain study: which gains settle the oscillator with acceptable effort?

The packaged example starts the mass at 1 m with 0.2 m/s and asks a sampled
PD controller (0.1 s sample interval, 3 N saturation) to bring it to rest at
the origin within 2.05 s. This script scores a 21 x 17 gain grid in one native
call with `score_matrix`, then fetches full metrics for the same grid with one
`batch` call to show the trade-off between tracking error and actuator effort.
Finally it replays the best gains at half the integration step to check that
the ranking is not a discretization artifact.

Run: python examples/python/pd_gain_study.py
"""

import numpy as np

from robotics_control_craft import Prepared, load_example

case = load_example()
model = case["model"]
prepared = Prepared(model)
print("model sha256:", prepared.model_sha256)

kp = np.linspace(0.0, 20.0, 21)
kd = np.linspace(0.0, 8.0, 17)
grid = np.array([[p, d] for p in kp for d in kd])
scores, statuses, steps = prepared.score_matrix(grid)
scores = np.asarray(scores)
print(f"grid rows: {len(grid)}, statuses: { {s: statuses.count(s) for s in sorted(set(statuses))} }")
print(f"steps per rollout: {sorted(set(steps))}")

order = np.argsort(scores)
print("\nfive best gain pairs by score")
print("   kp     kd     score")
for i in order[:5]:
    print(f"{grid[i, 0]:5.1f}  {grid[i, 1]:5.2f}  {scores[i]:.6f}")
baseline = int(np.flatnonzero((grid == case["parameters"]).all(axis=1))[0])
print(f"packaged gains {case['parameters']}: score {scores[baseline]:.6f}")

# Full records carry the score terms; one batch call returns them for all rows.
candidates = [{"id": f"kp{p:g}-kd{d:g}", "parameters": [p, d]} for p, d in grid]
records = prepared.batch(candidates)["records"]
error = np.array([r["metrics"]["integrated_error"] for r in records])
effort = np.array([r["metrics"]["integrated_force_squared"] for r in records])
assert all(r["score"] == s for r, s in zip(records, scores))

# Pareto front: no other gain pair has both lower tracking error and lower effort.
front = [
    i for i in range(len(grid))
    if not np.any((error < error[i]) & (effort < effort[i]))
]
front.sort(key=lambda i: effort[i])
print(f"\nPareto front of integrated error versus effort: {len(front)} gain pairs")
print("   kp     kd   error(m^2 s)  effort(N^2 s)  peak |x| (m)")
for i in front[:: max(1, len(front) // 8)]:
    m = records[i]["metrics"]
    print(f"{grid[i, 0]:5.1f}  {grid[i, 1]:5.2f}  {error[i]:12.6f}  {effort[i]:13.6f}"
          f"  {m['max_abs_position_m']:.6f}")

# Refined-step replay: halve dt and keep the controller interval fixed.
best = grid[order[0]].tolist()
fine = Prepared(model | {"dt_s": model["dt_s"] / 2})
coarse_row = prepared.evaluate(best)
fine_row = fine.evaluate(best)
print(f"\nbest gains {best}")
print(f"score at dt={model['dt_s']}: {coarse_row['score']:.9f} ({coarse_row['steps']} steps)")
print(f"score at dt={model['dt_s'] / 2}: {fine_row['score']:.9f} ({fine_row['steps']} steps)")
fine_scores = np.asarray(fine.score_matrix(grid[order[:5]])[0])
print("top-five ranking at half step:", np.argsort(fine_scores).tolist())
