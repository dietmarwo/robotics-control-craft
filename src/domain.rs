use crate::{Candidate, Evaluation, Work, infeasible, invalid};
use serde::Deserialize;
use serde_json::json;
pub const CAPABILITIES: &[&str] = &[
    "bounded_mass_spring_damper",
    "piecewise_constant_actuation",
    "sampled_pd",
    "scenario_replay",
];

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Model {
    mass_kg: f64,
    #[serde(skip)]
    inverse_mass: f64,
    spring_n_per_m: f64,
    damping_ns_per_m: f64,
    initial_x_m: f64,
    initial_v_m_s: f64,
    target_m: f64,
    dt_s: f64,
    horizon_s: f64,
    force_limit_n: f64,
    position_limit_m: f64,
    control: Control,
}
#[derive(Deserialize)]
#[serde(tag = "kind", rename_all = "snake_case", deny_unknown_fields)]
enum Control {
    Held { switches_s: Vec<f64> },
    Pd { interval_s: f64 },
}

pub fn prepare(mut model: Model) -> Result<Model, String> {
    if !(0.001..=1e4).contains(&model.mass_kg)
        || !(0.0..=1e5).contains(&model.spring_n_per_m)
        || !(0.0..=1e5).contains(&model.damping_ns_per_m)
        || !(1e-6..=10.0).contains(&model.dt_s)
        || !(1e-6..=1e5).contains(&model.horizon_s)
        || !(0.001..=1e5).contains(&model.force_limit_n)
        || !(0.001..=1e5).contains(&model.position_limit_m)
        || ![model.initial_x_m, model.target_m]
            .iter()
            .all(|v| v.abs() <= model.position_limit_m)
        || model.initial_v_m_s.abs() > 1e5
    {
        return Err("invalid oscillator state, coefficients, limits or time grid".into());
    }
    let stiffness =
        (model.spring_n_per_m / model.mass_kg).sqrt() + model.damping_ns_per_m / model.mass_kg;
    if model.dt_s * stiffness > 0.5 {
        return Err("time step outside declared stability envelope".into());
    }
    match &model.control {
        Control::Held { switches_s }
            if switches_s.len() > 63
                || switches_s
                    .iter()
                    .any(|t| !(*t > 0.0 && *t < model.horizon_s))
                || switches_s.windows(2).any(|w| w[0] >= w[1]) =>
        {
            return Err("switch times must increase inside horizon".into());
        }
        Control::Pd { interval_s } if !(*interval_s >= 1e-6 && *interval_s <= model.horizon_s) => {
            return Err("invalid controller sampling interval".into());
        }
        _ => {}
    }
    model.inverse_mass = 1.0 / model.mass_kg;
    Ok(model)
}

pub fn evaluate(m: &Model, c: &Candidate, work: &mut Work<'_>) -> Evaluation {
    let fmax = m.force_limit_n;
    match &m.control {
        Control::Held { switches_s }
            if c.parameters.len() != switches_s.len() + 1
                || c.parameters
                    .iter()
                    .any(|v| v.abs() > fmax || !v.is_finite()) =>
        {
            return Err(invalid("one force within limits per held-control segment"));
        }
        Control::Pd { .. }
            if c.parameters.len() != 2 || c.parameters.iter().any(|v| !(0.0..=1e5).contains(v)) =>
        {
            return Err(invalid(
                "expected nonnegative proportional and derivative gains",
            ));
        }
        _ => {}
    }
    if c.trace {
        integrate::<true>(m, &c.parameters, work)
    } else {
        integrate::<false>(m, &c.parameters, work)
    }
}

fn integrate<const TRACE: bool>(m: &Model, parameters: &[f64], work: &mut Work<'_>) -> Evaluation {
    let fmax = m.force_limit_n;
    let mut t = 0.0_f64;
    let mut y = [m.initial_x_m, m.initial_v_m_s, 0.0];
    let mut effort = 0.0_f64;
    let mut trace = Vec::new();
    let mut max_abs_x = y[0].abs();
    let mut max_force = 0.0_f64;
    let mut segment = 0usize;
    while t < m.horizon_s {
        let (boundary, force) = match &m.control {
            Control::Held { switches_s } => (
                switches_s.get(segment).copied().unwrap_or(m.horizon_s),
                parameters[segment],
            ),
            Control::Pd { interval_s } => (
                ((segment + 1) as f64 * interval_s).min(m.horizon_s),
                (parameters[0] * (m.target_m - y[0]) - parameters[1] * y[1]).clamp(-fmax, fmax),
            ),
        };
        max_force = max_force.max(force.abs());
        let force_squared = force * force;
        let force_weight =
            force_squared + 2.0 * force_squared + 2.0 * force_squared + force_squared;
        let derivative = |s: [f64; 3]| {
            [
                s[1],
                (force - m.spring_n_per_m * s[0] - m.damping_ns_per_m * s[1]) * m.inverse_mass,
                (s[0] - m.target_m).powi(2),
            ]
        };
        while t < boundary {
            work.step()?;
            let next = (t + m.dt_s).min(boundary);
            let h = next - t;
            if h <= 0.0 {
                return Err(invalid("time step does not advance"));
            }
            let a = derivative(y);
            let b = derivative(std::array::from_fn(|i| y[i] + 0.5 * h * a[i]));
            let d = derivative(std::array::from_fn(|i| y[i] + 0.5 * h * b[i]));
            let e = derivative(std::array::from_fn(|i| y[i] + h * d[i]));
            for i in 0..3 {
                y[i] += h * (a[i] + 2.0 * b[i] + 2.0 * d[i] + e[i]) / 6.0;
            }
            effort += h * force_weight / 6.0;
            t = next;
            max_abs_x = max_abs_x.max(y[0].abs());
            if !effort.is_finite() || y.iter().any(|v| !v.is_finite()) {
                return Err(invalid("nonfinite trajectory"));
            }
            if max_abs_x > m.position_limit_m {
                return Err(infeasible("position limit exceeded"));
            }
            if TRACE && trace.len() == 10_000 {
                return Err(invalid("trace requires at most 10,000 integration steps"));
            }
            if TRACE {
                trace.push([t, y[0], y[1], force]);
            }
        }
        segment += 1;
    }
    let terminal = (y[0] - m.target_m).powi(2) + 0.1 * y[1].powi(2);
    Ok((
        terminal + y[2] / m.horizon_s + 0.001 * effort,
        work.metrics(|| {
            json!({
        "terminal_position_m":y[0],"terminal_velocity_m_s":y[1],"integrated_error":y[2],
        "integrated_force_squared":effort,"max_abs_position_m":max_abs_x,"max_abs_force_n":max_force,
        "trace_columns":["t_s","x_m","v_m_s","held_force_n"],"trace":TRACE.then_some(trace)})
        }),
    ))
}
