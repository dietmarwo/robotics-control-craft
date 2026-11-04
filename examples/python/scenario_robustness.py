"""Scenario robustness: does a tuned controller survive plant uncertainty?

A controller tuned on the nominal plant may degrade when the payload mass or
the damping differ. This script samples 40 seeded scenarios (mass within
+/-20%, damping between 0.25 and 0.65 N s/m), prepares each scenario once and
scores four gain pairs per scenario with one `score_matrix` call. It reports
the mean, worst case and spread of the score, and the number of failed
rollouts. A second study moves the target close to the position limit, where
aggressive gains overshoot and become infeasible.

Run: python examples/python/scenario_robustness.py
"""

import random

import numpy as np

from robotics_control_craft import Prepared, load_example

case = load_example()
nominal = case["model"]
controllers = {
    "zero gains": [0.0, 0.0],
    "packaged": case["parameters"],
    "nominal grid best": [16.0, 5.5],
    "soft": [2.0, 1.0],
}
rows = list(controllers.values())

rng = random.Random(2026)
scenarios = [
    nominal | {"mass_kg": rng.uniform(0.8, 1.2), "damping_ns_per_m": rng.uniform(0.25, 0.65)}
    for _ in range(40)
]
table = []
for scenario in scenarios:
    scores, statuses, _ = Prepared(scenario).score_matrix(rows)
    table.append([s if st == "complete" else np.nan for s, st in zip(scores, statuses)])
table = np.array(table)

print(f"{len(scenarios)} scenarios, mass {min(s['mass_kg'] for s in scenarios):.3f}"
      f"-{max(s['mass_kg'] for s in scenarios):.3f} kg, damping"
      f" {min(s['damping_ns_per_m'] for s in scenarios):.3f}"
      f"-{max(s['damping_ns_per_m'] for s in scenarios):.3f} N s/m")
print("controller           mean      worst     std       failed")
for j, name in enumerate(controllers):
    column = table[:, j]
    ok = column[~np.isnan(column)]
    print(f"{name:18s} {ok.mean():9.6f} {ok.max():9.6f} {ok.std():9.6f} {np.isnan(column).sum():5d}")

# Stress test: target 1.8 m with the position limit at 2 m, starting at rest.
stress = nominal | {"target_m": 1.8, "initial_x_m": 0.0, "initial_v_m_s": 0.0}
gains = np.array([[p, d] for p in (2.0, 8.0, 16.0, 30.0) for d in (0.0, 1.0, 3.0)])
counts = {}
for scenario in scenarios[:10]:
    _, statuses, _ = Prepared(stress | {k: scenario[k] for k in ("mass_kg", "damping_ns_per_m")}).score_matrix(gains)
    for g, status in zip(map(tuple, gains), statuses):
        counts.setdefault(g, []).append(status)
print("\nstress target 1.8 m, limit 2 m: infeasible rollouts out of 10 scenarios")
print("   kp     kd  infeasible")
for g, statuses in counts.items():
    print(f"{g[0]:5.1f}  {g[1]:5.1f}  {statuses.count('infeasible'):10d}")
first = Prepared(stress).evaluate([30.0, 0.0])
print("example failure record:", {k: first[k] for k in ("status", "steps", "score", "detail")})
