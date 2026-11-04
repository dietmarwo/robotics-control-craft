# Python examples

Each script answers one question with the installed `robotics_control_craft`
package and runs in well under a minute. Run them from the package root, for
example `python examples/python/pd_gain_study.py`. The recorded output of each
run is in [`outputs/`](outputs/). The optimization examples need SciPy and
fcmaes (`pip install scipy fcmaes`); all scripts need NumPy.

## Application examples

| Script | Question | APIs |
| --- | --- | --- |
| [`pd_gain_study.py`](pd_gain_study.py) | Which PD gains settle the packaged oscillator best, and what is the trade-off between tracking error and effort? Does the ranking survive a halved step? | `score_matrix`, `batch`, `evaluate`, `model_sha256` |
| [`scenario_robustness.py`](scenario_robustness.py) | How do four controllers degrade over 40 sampled mass and damping scenarios, and which gains overshoot a target near the position limit? | `score_matrix` per scenario, failure records |
| [`held_actuation_and_convergence.py`](held_actuation_and_convergence.py) | Which held braking schedule works best? How fast does RK4 converge against a matrix-exponential reference? What does a trace contain? | held control, `evaluate(trace=True)`, SciPy `expm` |
| [`parameter_sensitivity.py`](parameter_sensitivity.py) | Which plant, initial-state and controller settings change the score most for a fixed controller? | perturbed `Prepared` models, `score_matrix` |

## Optimization examples

| Script | Optimizer | Problem | Evaluation pattern |
| --- | --- | --- | --- |
| [`scipy_de_tuning.py`](scipy_de_tuning.py) | SciPy `differential_evolution(vectorized=True)` and bounded Nelder-Mead | PD gains for a target near the position limit, 1 ms step | one `score_matrix` call per generation on two native workers; penalty for failed rows |
| [`fcmaes_cmaes_held_schedule.py`](fcmaes_cmaes_held_schedule.py) | fcmaes `cmaes.Cmaes` ask/tell | six held forces over 4 s at a 1 ms step | each population of 24 in one `score_matrix` call (lockstep lanes, two workers); penalty for failed rows |
| [`fcmaes_bite_robust_pd.py`](fcmaes_bite_robust_pd.py) | fcmaes `bitecpp.minimize` | robust PD gains over eight sampled plants, checked on 32 held-out plants | one-row `score_matrix` calls inside a standard minimizer; penalty per failed rollout |

The Rust example [`../matrix.rs`](../matrix.rs) runs a gain grid through
`Prepared::score_matrix`:
`cargo run --release --locked --example matrix -- models/example.json`
(output in [`outputs/rust_matrix.txt`](outputs/rust_matrix.txt)).

See the [user manual](../../docs/user-manual.md) for the model reference and
the APIs these scripts use.
