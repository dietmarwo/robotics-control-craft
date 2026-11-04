"""fcmaes CMA-ES ask/tell: optimize a held-force schedule population-wise.

The mass starts at 1 m with 0.2 m/s and must come to rest at the origin within
4 s (1 ms step). Actuation is held: six constant forces between switch times
0.5, 1, 1.5, 2.5 and 3.25 s, each within the 3 N force limit. CMA-ES from
fcmaes asks for a population of 24 schedules per generation; the whole
population is scored with one `score_matrix` call on two native workers,
which integrates the rows in lockstep lanes. Rows that fail (for example a
force outside the limit) get a penalty.

Run: python examples/python/fcmaes_cmaes_held_schedule.py
"""

import numpy as np
from loguru import logger
from scipy.optimize import Bounds

from fcmaes import cmaes
from robotics_control_craft import Prepared, load_example

logger.remove()
switches = [0.5, 1.0, 1.5, 2.5, 3.25]
model = load_example()["model"] | {
    "dt_s": 0.001, "horizon_s": 4.0, "control": {"kind": "held", "switches_s": switches},
}
prepared = Prepared(model)
limit = model["force_limit_n"]
PENALTY = 10.0

rng = np.random.default_rng(11)
optimizer = cmaes.Cmaes(
    bounds=Bounds([-limit] * 6, [limit] * 6),
    x0=np.zeros(6),
    input_sigma=1.0,
    popsize=24,
    rg=rng,
    randn=lambda *shape: rng.standard_normal(shape),
)
best_score, best_x, evaluations, failures = np.inf, None, 0, 0
for generation in range(1, 151):
    xs = optimizer.ask()
    scores, statuses, _ = prepared.score_matrix(xs, workers=2)
    scores = np.asarray(scores)
    failed = np.array([s != "complete" for s in statuses])
    scores[failed] = PENALTY
    evaluations += len(xs)
    failures += int(failed.sum())
    i = int(np.argmin(scores))
    if scores[i] < best_score:
        best_score, best_x = float(scores[i]), xs[i].copy()
    stop = optimizer.tell(scores)
    if generation % 25 == 0 or stop:
        print(f"generation {generation:3d}: best score {best_score:.9f}")
    if stop:
        print(f"CMA-ES stop code {stop}")
        break

print(f"{evaluations} schedules evaluated, {failures} penalized")
print("best forces (N):", np.round(best_x, 4).tolist())
row = prepared.evaluate(best_x.tolist())
m = row["metrics"]
print(f"replay score {row['score']:.9f}, terminal x {m['terminal_position_m']:.3e} m,"
      f" v {m['terminal_velocity_m_s']:.3e} m/s, effort {m['integrated_force_squared']:.4f} N^2 s")
hand = prepared.evaluate([-3.0, -1.0, 0.0, 0.0, 0.0, 0.0])
print(f"hand-made brake-then-hold schedule: score {hand['score']:.6f}")
coarse = Prepared(model | {"dt_s": 0.1}).evaluate(best_x.tolist(), trace=True)
segments = list(dict.fromkeys(round(r[3], 4) for r in coarse["metrics"]["trace"]))
print(f"coarse traced replay at dt=0.1 s: {len(coarse['metrics']['trace'])} rows,"
      f" forces in order {segments}")
