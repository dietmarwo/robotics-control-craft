"""fcmaes BiteOpt: robust PD gains across sampled plants, with failure penalties.

The controller must move the mass from rest at 0 m to 1.9 m, close to the 2 m
position limit, for any payload mass between 0.6 and 1.6 kg and damping between
0.1 and 0.7 N s/m. The derivative gain is limited to 4 N s/m (for example by
sensor noise). The objective is the mean score over eight training scenarios;
every non-complete rollout (infeasible overshoot or invalid gains) adds a
penalty.
`bitecpp.minimize` is a standard fcmaes minimizer that calls the objective one
candidate at a time, so each call scores one row on the eight prepared
scenario models. The winner is then checked on 32 held-out scenarios against
gains tuned on the nominal plant only.

Run: python examples/python/fcmaes_bite_robust_pd.py
"""

import random

import numpy as np
from loguru import logger
from scipy.optimize import Bounds

from fcmaes import bitecpp
from robotics_control_craft import Prepared, load_example

logger.remove()
base = load_example()["model"] | {
    "dt_s": 0.01, "horizon_s": 4.0, "target_m": 1.9, "initial_x_m": 0.0, "initial_v_m_s": 0.0,
}


def sample(seed, count):
    rng = random.Random(seed)
    return [base | {"mass_kg": rng.uniform(0.6, 1.6), "damping_ns_per_m": rng.uniform(0.1, 0.7)}
            for _ in range(count)]


def objective_for(models, penalty=5.0):
    prepared = [Prepared(m) for m in models]
    counter = {"calls": 0, "failed": 0}

    def objective(x):
        counter["calls"] += 1
        total = 0.0
        for p in prepared:
            scores, statuses, _ = p.score_matrix([x.tolist()])
            if statuses[0] == "complete":
                total += scores[0]
            else:
                counter["failed"] += 1
                total += penalty
        return total / len(prepared)

    return objective, counter


bounds = Bounds([0.0, 0.0], [40.0, 4.0])
robust, robust_count = objective_for(sample(1, 8))
r = bitecpp.minimize(robust, bounds, x0=[5.0, 2.0], max_evaluations=1500,
                     rg=np.random.default_rng(5))
print(f"robust gains {np.round(r.x, 5).tolist()}, training mean {r.fun:.6f},"
      f" {robust_count['calls']} calls, {robust_count['failed']} failed rollouts penalized")

nominal, nominal_count = objective_for([base])
n = bitecpp.minimize(nominal, bounds, x0=[5.0, 2.0], max_evaluations=1500,
                     rg=np.random.default_rng(5))
print(f"nominal gains {np.round(n.x, 5).tolist()}, nominal score {n.fun:.6f},"
      f" {nominal_count['failed']} failed rollouts penalized")

holdout = sample(99, 32)
print("\nheld-out evaluation over 32 scenarios (scores over completed rollouts)")
print("gains      mean score  worst score  infeasible")
for name, x in (("robust", r.x), ("nominal", n.x)):
    scores, failed = [], 0
    for m in holdout:
        s, st, _ = Prepared(m).score_matrix([x.tolist()])
        if st[0] == "complete":
            scores.append(s[0])
        else:
            failed += 1
    print(f"{name:9s} {np.mean(scores):11.6f} {np.max(scores):12.6f} {failed:11d}")
