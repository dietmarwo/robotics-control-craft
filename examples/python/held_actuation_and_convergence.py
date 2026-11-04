"""Held actuation: schedules, step-size convergence and traces.

Held control applies one constant force per segment between fixed switch
times. This script answers three questions:

1. Which of three braking schedules brings the mass closest to rest at the
   origin over a 4 s horizon, and at what effort?
2. How fast does the RK4 terminal state converge as the step shrinks? The
   reference is the exact solution of the linear system on each segment,
   computed with a matrix exponential (SciPy), as in validation/validate.py.
3. What does a trace contain, and how does the position limit show up?

Run: python examples/python/held_actuation_and_convergence.py
"""

import numpy as np
from scipy.linalg import expm

from robotics_control_craft import Prepared, load_example

base = load_example()["model"] | {"dt_s": 0.01, "horizon_s": 4.0}
switches = [0.5, 1.0, 2.0, 3.0]
model = base | {"control": {"kind": "held", "switches_s": switches}}
prepared = Prepared(model)
schedules = {
    "brake then hold": [-3.0, -1.0, 0.0, 0.0, 0.0],
    "bang-bang": [-3.0, 3.0, -1.0, 0.5, 0.0],
    "gentle": [-1.0, -0.5, 0.0, 0.25, 0.0],
}
print("schedule          score     terminal x (m)  terminal v (m/s)  effort (N^2 s)")
for name, forces in schedules.items():
    row = prepared.evaluate(forces)
    m = row["metrics"]
    print(f"{name:16s} {row['score']:9.6f} {m['terminal_position_m']:14.6f}"
          f" {m['terminal_velocity_m_s']:16.6f} {m['integrated_force_squared']:14.6f}")


def exact(model, forces):
    """Terminal position and velocity from the segment-wise matrix exponential."""
    state = np.array([model["initial_x_m"], model["initial_v_m_s"], 1.0])
    times = [0.0, *model["control"]["switches_s"], model["horizon_s"]]
    m, k, c = model["mass_kg"], model["spring_n_per_m"], model["damping_ns_per_m"]
    for lo, hi, force in zip(times[:-1], times[1:], forces, strict=True):
        a = np.array([[0, 1, 0], [-k / m, -c / m, force / m], [0, 0, 0]])
        state = expm(a * (hi - lo)) @ state
    return state[:2]


forces = schedules["brake then hold"]
reference = exact(model, forces)
print(f"\nexact terminal state: x={reference[0]:.12f} m, v={reference[1]:.12f} m/s")
print("    dt (s)   steps   max error     ratio")
previous = None
for dt in (0.08, 0.04, 0.02, 0.01, 0.005):
    row = Prepared(model | {"dt_s": dt}).evaluate(forces)
    m = row["metrics"]
    error = float(np.max(np.abs(reference - [m["terminal_position_m"], m["terminal_velocity_m_s"]])))
    ratio = "" if previous is None else f"{previous / error:8.2f}"
    print(f"{dt:10.3f} {row['steps']:7d}   {error:.3e}  {ratio}")
    previous = error
print("halving dt divides the error by about 16, as expected for fourth order RK4")

traced = Prepared(model | {"dt_s": 0.1}).evaluate(forces, trace=True)
trace = traced["metrics"]["trace"]
print(f"\ntrace columns {traced['metrics']['trace_columns']}, {len(trace)} rows")
for t, x, v, f in trace[:3] + trace[-2:]:
    print(f"t={t:6.3f}  x={x:9.6f}  v={v:9.6f}  force={f:5.2f}")
assert {t for t, *_ in trace} >= set(switches)

push = Prepared(model | {"position_limit_m": 1.2}).evaluate([3.0, 3.0, 3.0, 3.0, 3.0])
print("\npushing outward with 3 N against a 1.2 m limit:",
      {k: push[k] for k in ("status", "steps", "detail")})
