"""SciPy: tune PD gains near the position limit, then polish and replay.

The task moves the mass from rest at 0 m to a target at 1.8 m while the
position limit is 2 m, over 4 s with a 1 ms integration step. High
proportional gains with little damping overshoot the limit and the rollout is
infeasible, so the objective adds a penalty for every non-complete status.

SciPy's differential evolution runs with `vectorized=True`: each generation
arrives as one (dim, population) array and is scored with a single
`score_matrix` call on two native workers. Nelder-Mead then polishes the
winner with a one-row objective. The result is replayed with a trace and at
half the step size.

Run: python examples/python/scipy_de_tuning.py
"""

import numpy as np
from scipy.optimize import differential_evolution, minimize

from robotics_control_craft import Prepared, load_example

model = load_example()["model"] | {
    "dt_s": 0.001, "horizon_s": 4.0, "target_m": 1.8, "initial_x_m": 0.0, "initial_v_m_s": 0.0,
}
prepared = Prepared(model)
PENALTY = 10.0
calls = {"native": 0, "rows": 0, "failed": 0}


def population_objective(x):
    """x has shape (2, S); return S penalized scores."""
    scores, statuses, _ = prepared.score_matrix(x.T, workers=2)
    calls["native"] += 1
    calls["rows"] += len(statuses)
    scores = np.asarray(scores)
    failed = np.array([s != "complete" for s in statuses])
    calls["failed"] += int(failed.sum())
    scores[failed] = PENALTY
    return scores


bounds = [(0.0, 100.0), (0.0, 30.0)]
result = differential_evolution(
    population_objective, bounds, seed=7, popsize=16, maxiter=60, tol=1e-10,
    vectorized=True, updating="deferred", polish=False,
)
print(f"DE: best gains {np.round(result.x, 6).tolist()}, score {result.fun:.9f}")
print(f"DE: {calls['rows']} rows scored in {calls['native']} native calls,"
      f" {calls['failed']} failed rollouts penalized")


def one(x):
    return population_objective(np.asarray(x)[:, None])[0]


polished = minimize(one, result.x, method="Nelder-Mead", bounds=bounds,
                    options={"xatol": 1e-8, "fatol": 1e-12, "maxiter": 400})
print(f"Nelder-Mead: gains {np.round(polished.x, 6).tolist()}, score {polished.fun:.9f},"
      f" {polished.nfev} evaluations")

best = polished.x.tolist() if polished.fun <= result.fun else result.x.tolist()
replay = prepared.evaluate(best, trace=False)
m = replay["metrics"]
print(f"replay: status {replay['status']}, steps {replay['steps']},"
      f" peak position {m['max_abs_position_m']:.6f} m (limit {model['position_limit_m']} m)")
print(f"        terminal x {m['terminal_position_m']:.6f} m, v {m['terminal_velocity_m_s']:.6f} m/s")
fine = Prepared(model | {"dt_s": model["dt_s"] / 2}).evaluate(best)
print(f"half-step replay score {fine['score']:.9f} (difference {fine['score'] - replay['score']:.2e})")
reduced = Prepared(model | {"horizon_s": 1.0}).evaluate(best, trace=True)
print(f"traced first second: {len(reduced['metrics']['trace'])} rows,"
      f" last row {np.round(reduced['metrics']['trace'][-1], 6).tolist()}")
