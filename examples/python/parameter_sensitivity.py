"""Parameter sensitivity: which plant and controller settings matter most?

For a fixed PD controller this script perturbs each model parameter by +/-1%
and reports the central-difference elasticity of the score,
(dScore / Score) / (dParam / Param), together with the change in tracking error
and effort. Each perturbed model is prepared once; the same candidate is scored
on all of them. A second table repeats the analysis for the controller gains
with `score_matrix` on the nominal model.

Run: python examples/python/parameter_sensitivity.py
"""

from robotics_control_craft import Prepared, load_example

case = load_example()
model = case["model"] | {"dt_s": 0.01}
gains = [8.0, 3.0]
nominal = Prepared(model).evaluate(gains)
print(f"nominal score {nominal['score']:.9f} with gains {gains}, dt={model['dt_s']} s")
print("parameter            value   elasticity   d error/dp     d effort/dp")
for name in ("mass_kg", "spring_n_per_m", "damping_ns_per_m", "force_limit_n",
             "initial_x_m", "initial_v_m_s"):
    value = model[name]
    step = 0.01 * value
    up = Prepared(model | {name: value + step}).evaluate(gains)
    down = Prepared(model | {name: value - step}).evaluate(gains)
    assert up["status"] == down["status"] == "complete", (up, down)
    slope = (up["score"] - down["score"]) / (2 * step)
    d_error = (up["metrics"]["integrated_error"] - down["metrics"]["integrated_error"]) / (2 * step)
    d_effort = (up["metrics"]["integrated_force_squared"]
                - down["metrics"]["integrated_force_squared"]) / (2 * step)
    print(f"{name:18s} {value:7.3f} {slope * value / nominal['score']:12.4f}"
          f" {d_error:14.6f} {d_effort:15.6f}")

interval = model["control"]["interval_s"]
for scale in (0.5, 1.0, 2.0):
    control = {"kind": "pd", "interval_s": interval * scale}
    row = Prepared(model | {"control": control}).evaluate(gains)
    print(f"controller interval {control['interval_s']:.3f} s: score {row['score']:.6f},"
          f" peak force {row['metrics']['max_abs_force_n']:.3f} N")

prepared = Prepared(model)
rows = []
for i in range(2):
    for sign in (1, -1):
        g = list(gains)
        g[i] *= 1 + sign * 0.01
        rows.append(g)
scores, statuses, _ = prepared.score_matrix(rows)
assert set(statuses) == {"complete"}
for i, name in enumerate(("kp", "kd")):
    slope = (scores[2 * i] - scores[2 * i + 1]) / (0.02 * gains[i])
    print(f"gain {name}: elasticity {slope * gains[i] / nominal['score']:.4f}")
