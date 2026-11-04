use crate::{Evaluation, Failure, Work, infeasible, invalid};
use serde::{Deserialize, Serialize};
pub const CAPABILITIES: &[&str] = &[
    "bounded_mass_spring_damper",
    "piecewise_constant_actuation",
    "sampled_pd",
    "scenario_replay",
];

/// Diagnostics of a complete rollout. The fields are declared in alphabetical
/// order, the key order of a `serde_json::Value` map, so records keep their bytes.
#[derive(Serialize)]
pub struct Metrics {
    integrated_error: f64,
    integrated_force_squared: f64,
    max_abs_force_n: f64,
    max_abs_position_m: f64,
    terminal_position_m: f64,
    terminal_velocity_m_s: f64,
    trace: Option<Vec<[f64; 4]>>,
    trace_columns: [&'static str; 4],
}

/// Most trace rows a traced candidate may produce.
const TRACE_LIMIT: usize = 10_000;

const TRACE_COLUMNS: [&str; 4] = ["t_s", "x_m", "v_m_s", "held_force_n"];

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
    /// The host supports AVX2, detected once at preparation.
    #[serde(skip)]
    avx2: bool,
    /// The host supports AVX-512F, detected once at preparation.
    #[serde(skip)]
    avx512: bool,
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
    #[cfg(target_arch = "x86_64")]
    {
        model.avx2 = std::arch::is_x86_feature_detected!("avx2");
        model.avx512 = std::arch::is_x86_feature_detected!("avx512f");
    }
    Ok(model)
}

/// Lockstep width of the AVX2 and baseline lane kernels.
const LANES: usize = 8;
/// Lockstep width of the AVX-512 lane kernel, the largest width.
pub const MAX_LANES: usize = 16;

/// Number of untraced candidates `evaluate_lanes` integrates together on this host.
pub fn lane_width(m: &Model) -> usize {
    if m.avx512 { MAX_LANES } else { LANES }
}

pub fn evaluate(m: &Model, parameters: &[f64], trace: bool, work: &mut Work<'_>) -> Evaluation {
    check(m, parameters)?;
    if trace {
        integrate::<true>(m, parameters, work)
    } else {
        integrate::<false>(m, parameters, work)
    }
}

/// Reject a parameter vector before integration; such a candidate takes no steps.
pub fn check(m: &Model, parameters: &[f64]) -> Result<(), Failure> {
    let fmax = m.force_limit_n;
    match &m.control {
        Control::Held { switches_s }
            if parameters.len() != switches_s.len() + 1
                || parameters.iter().any(|v| v.abs() > fmax || !v.is_finite()) =>
        {
            return Err(invalid("one force within limits per held-control segment"));
        }
        Control::Pd { .. }
            if parameters.len() != 2 || parameters.iter().any(|v| !(0.0..=1e5).contains(v)) =>
        {
            return Err(invalid(
                "expected nonnegative proportional and derivative gains",
            ));
        }
        _ => {}
    }
    Ok(())
}

/// Integrate up to `lane_width(m)` untraced candidates that passed `check` in
/// lockstep and emit each lane's evaluation and steps.
///
/// All lanes share the model's control and time grid, so one step counter
/// serves the group. Every lane performs the scalar kernel's floating-point
/// operations in the same order, which keeps results bit-identical. A lane
/// that fails keeps the step count reached at that point. Unused lanes repeat
/// the last row and are not emitted. `work` must be unmonitored and unused.
pub fn evaluate_lanes(
    m: &Model,
    parameters: &[&[f64]],
    work: &mut Work<'_>,
    mut emit: impl FnMut(usize, Evaluation, u64),
) {
    let filled = parameters.len();
    assert!((1..=lane_width(m)).contains(&filled), "lane group size");
    #[cfg(target_arch = "x86_64")]
    if m.avx512 {
        let rows = std::array::from_fn(|l| parameters[l.min(filled - 1)]);
        // SAFETY: `prepare` detected AVX-512F support on this host.
        let results = unsafe { lanes_avx512(m, rows, work) };
        for (lane, (result, steps)) in results.into_iter().enumerate().take(filled) {
            emit(lane, result, steps);
        }
        return;
    }
    let rows = std::array::from_fn(|l| parameters[l.min(filled - 1)]);
    #[cfg(target_arch = "x86_64")]
    let results = if m.avx2 {
        // SAFETY: `prepare` detected AVX2 support on this host.
        unsafe { lanes_avx2(m, rows, work) }
    } else {
        lanes::<LANES>(m, rows, work)
    };
    #[cfg(not(target_arch = "x86_64"))]
    let results = lanes::<LANES>(m, rows, work);
    for (lane, (result, steps)) in results.into_iter().enumerate().take(filled) {
        emit(lane, result, steps);
    }
}

/// The eight-lane kernel compiled for AVX2. AVX2 does not enable fused
/// multiply-add, and Rust does not contract operations, so results equal the
/// baseline build.
#[cfg(target_arch = "x86_64")]
#[target_feature(enable = "avx2")]
fn lanes_avx2(
    m: &Model,
    parameters: [&[f64]; LANES],
    work: &mut Work<'_>,
) -> [(Evaluation, u64); LANES] {
    lanes(m, parameters, work)
}

/// The sixteen-lane kernel compiled for AVX-512F. Rust does not contract
/// multiplies and adds into fused operations, so results equal the baseline.
#[cfg(target_arch = "x86_64")]
#[target_feature(enable = "avx512f")]
fn lanes_avx512(
    m: &Model,
    parameters: [&[f64]; MAX_LANES],
    work: &mut Work<'_>,
) -> [(Evaluation, u64); MAX_LANES] {
    lanes(m, parameters, work)
}

#[inline(always)]
fn lanes<const N: usize>(
    m: &Model,
    parameters: [&[f64]; N],
    work: &mut Work<'_>,
) -> [(Evaluation, u64); N] {
    let fmax = m.force_limit_n;
    let (k, c, inverse_mass, target) = (
        m.spring_n_per_m,
        m.damping_ns_per_m,
        m.inverse_mass,
        m.target_m,
    );
    let mut done: [Option<(Evaluation, u64)>; N] = std::array::from_fn(|_| None);
    let mut alive = [true; N];
    let mut remaining = N;
    let mut t = 0.0_f64;
    let mut x = [m.initial_x_m; N];
    let mut v = [m.initial_v_m_s; N];
    let mut error = [0.0_f64; N];
    let mut effort = [0.0_f64; N];
    let mut max_abs_x = [m.initial_x_m.abs(); N];
    let mut max_force = [0.0_f64; N];
    let mut segment = 0usize;
    'run: while t < m.horizon_s {
        let mut force = [0.0_f64; N];
        let boundary = match &m.control {
            Control::Held { switches_s } => {
                for l in 0..N {
                    force[l] = parameters[l][segment];
                }
                switches_s.get(segment).copied().unwrap_or(m.horizon_s)
            }
            Control::Pd { interval_s } => {
                for l in 0..N {
                    force[l] = (parameters[l][0] * (target - x[l]) - parameters[l][1] * v[l])
                        .clamp(-fmax, fmax);
                }
                ((segment + 1) as f64 * interval_s).min(m.horizon_s)
            }
        };
        let mut force_weight = [0.0_f64; N];
        for l in 0..N {
            max_force[l] = max_force[l].max(force[l].abs());
            let force_squared = force[l] * force[l];
            force_weight[l] =
                force_squared + 2.0 * force_squared + 2.0 * force_squared + force_squared;
        }
        // `h * force_weight / 6` for the step `cached_h`. The weight is fixed for the
        // segment and lanes share `h`, which repeats exactly over long stretches of the
        // grid, so the increment is reused; it is the same value the scalar kernel adds.
        let mut cached_h = f64::NAN;
        let mut increment = [0.0_f64; N];
        while t < boundary {
            if let Err(failure) = work.step() {
                for l in 0..N {
                    if alive[l] {
                        done[l] = Some((Err(failure.clone()), work.used));
                    }
                }
                break 'run;
            }
            let next = (t + m.dt_s).min(boundary);
            let h = next - t;
            if h <= 0.0 {
                for l in 0..N {
                    if alive[l] {
                        done[l] = Some((Err(invalid("time step does not advance")), work.used));
                    }
                }
                break 'run;
            }
            let half = 0.5 * h;
            if h != cached_h {
                for l in 0..N {
                    increment[l] = h * force_weight[l] / 6.0;
                }
                cached_h = h;
            }
            let mut failed = false;
            for l in 0..N {
                let (x0, v0) = (x[l], v[l]);
                let a1 = (force[l] - k * x0 - c * v0) * inverse_mass;
                let a2 = (x0 - target).powi(2);
                let (bx, bv) = (x0 + half * v0, v0 + half * a1);
                let b1 = (force[l] - k * bx - c * bv) * inverse_mass;
                let b2 = (bx - target).powi(2);
                let (dx, dv) = (x0 + half * bv, v0 + half * b1);
                let d1 = (force[l] - k * dx - c * dv) * inverse_mass;
                let d2 = (dx - target).powi(2);
                let (ex, ev) = (x0 + h * dv, v0 + h * d1);
                let e1 = (force[l] - k * ex - c * ev) * inverse_mass;
                let e2 = (ex - target).powi(2);
                x[l] = x0 + h * (v0 + 2.0 * bv + 2.0 * dv + ev) / 6.0;
                v[l] = v0 + h * (a1 + 2.0 * b1 + 2.0 * d1 + e1) / 6.0;
                error[l] += h * (a2 + 2.0 * b2 + 2.0 * d2 + e2) / 6.0;
                effort[l] += increment[l];
                max_abs_x[l] = max_abs_x[l].max(x[l].abs());
                let finite = effort[l].is_finite()
                    & x[l].is_finite()
                    & v[l].is_finite()
                    & error[l].is_finite();
                failed |= alive[l] & (!finite | (max_abs_x[l] > m.position_limit_m));
            }
            t = next;
            if failed {
                for l in 0..N {
                    if !alive[l] {
                        continue;
                    }
                    let finite = effort[l].is_finite()
                        && x[l].is_finite()
                        && v[l].is_finite()
                        && error[l].is_finite();
                    let failure = if !finite {
                        invalid("nonfinite trajectory")
                    } else if max_abs_x[l] > m.position_limit_m {
                        infeasible("position limit exceeded")
                    } else {
                        continue;
                    };
                    done[l] = Some((Err(failure), work.used));
                    alive[l] = false;
                    remaining -= 1;
                }
                if remaining == 0 {
                    break 'run;
                }
            }
        }
        segment += 1;
    }
    std::array::from_fn(|l| {
        done[l].take().unwrap_or_else(|| {
            let terminal = (x[l] - target).powi(2) + 0.1 * v[l].powi(2);
            let score = terminal + error[l] / m.horizon_s + 0.001 * effort[l];
            let metrics = work.metrics(|| Metrics {
                integrated_error: error[l],
                integrated_force_squared: effort[l],
                max_abs_force_n: max_force[l],
                max_abs_position_m: max_abs_x[l],
                terminal_position_m: x[l],
                terminal_velocity_m_s: v[l],
                trace: None,
                trace_columns: TRACE_COLUMNS,
            });
            (Ok((score, metrics)), work.used)
        })
    })
}

/// Upper estimate of a rollout's steps: grid steps plus one per control boundary.
fn expected_steps(m: &Model) -> usize {
    let boundaries = match &m.control {
        Control::Held { switches_s } => switches_s.len() as f64,
        Control::Pd { interval_s } => m.horizon_s / interval_s,
    };
    (m.horizon_s / m.dt_s + boundaries + 2.0) as usize
}

fn integrate<const TRACE: bool>(m: &Model, parameters: &[f64], work: &mut Work<'_>) -> Evaluation {
    let fmax = m.force_limit_n;
    let mut t = 0.0_f64;
    let mut y = [m.initial_x_m, m.initial_v_m_s, 0.0];
    let mut effort = 0.0_f64;
    let mut trace = if TRACE {
        Vec::with_capacity(expected_steps(m).min(TRACE_LIMIT))
    } else {
        Vec::new()
    };
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
            if TRACE && trace.len() == TRACE_LIMIT {
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
        work.metrics(|| Metrics {
            integrated_error: y[2],
            integrated_force_squared: effort,
            max_abs_force_n: max_force,
            max_abs_position_m: max_abs_x,
            terminal_position_m: y[0],
            terminal_velocity_m_s: y[1],
            trace: TRACE.then_some(trace),
            trace_columns: TRACE_COLUMNS,
        }),
    ))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::Budget;

    /// Render lane results with exact score bits and serialized metrics.
    fn text(results: &[(Evaluation, u64)]) -> Vec<String> {
        results
            .iter()
            .map(|(e, s)| match e {
                Ok((score, metrics)) => format!(
                    "{s} {:x} {}",
                    score.to_bits(),
                    serde_json::to_string(metrics).unwrap()
                ),
                Err(f) => format!("{s} {} {}", f.status, f.detail),
            })
            .collect()
    }

    #[test]
    fn vector_lane_kernels_equal_baseline_lanes() {
        let budget = Budget::default();
        let fresh = || Work {
            budget: &budget,
            deadline: None,
            used: 0,
            monitored: false,
            metrics_enabled: true,
        };
        let base = r#"{"mass_kg":1,"spring_n_per_m":1,"damping_ns_per_m":0.4,
            "initial_x_m":0,"initial_v_m_s":0.2,"target_m":1.8,"dt_s":0.01,
            "horizon_s":3,"force_limit_n":3,"position_limit_m":2,"control":"#;
        let pd = prepare(
            serde_json::from_str(&format!(r#"{base}{{"kind":"pd","interval_s":0.1}}}}"#)).unwrap(),
        )
        .unwrap();
        let held = prepare(
            serde_json::from_str(&format!(
                r#"{base}{{"kind":"held","switches_s":[0.4,1.1]}}}}"#
            ))
            .unwrap(),
        )
        .unwrap();
        let gains: Vec<[f64; 2]> = (0..MAX_LANES)
            .map(|l| [0.5 + 1.25 * l as f64, 1.5 - 0.08 * l as f64])
            .collect();
        let forces: Vec<[f64; 3]> = (0..MAX_LANES)
            .map(|l| {
                [
                    3.0 - 0.35 * l as f64,
                    0.2 * l as f64 - 1.0,
                    1.0 + 0.1 * l as f64,
                ]
            })
            .collect();
        for (model, rows) in [
            (&pd, gains.iter().map(|g| &g[..]).collect::<Vec<_>>()),
            (&held, forces.iter().map(|f| &f[..]).collect()),
        ] {
            let wide: [&[f64]; MAX_LANES] = std::array::from_fn(|l| rows[l]);
            let baseline = lanes(model, wide, &mut fresh());
            let statuses: Vec<_> = baseline
                .iter()
                .map(|(e, _)| e.as_ref().map_or_else(|f| f.status, |_| "complete"))
                .collect();
            assert!(statuses.contains(&"complete") && statuses.contains(&"infeasible"));
            let narrow: [&[f64]; LANES] = std::array::from_fn(|l| rows[l]);
            assert_eq!(
                text(&lanes(model, narrow, &mut fresh())),
                text(&baseline[..LANES])
            );
            #[cfg(target_arch = "x86_64")]
            {
                if std::arch::is_x86_feature_detected!("avx2") {
                    // SAFETY: AVX2 support was detected above.
                    let avx2 = unsafe { lanes_avx2(model, narrow, &mut fresh()) };
                    assert_eq!(text(&avx2), text(&baseline[..LANES]));
                }
                if std::arch::is_x86_feature_detected!("avx512f") {
                    // SAFETY: AVX-512F support was detected above.
                    let avx512 = unsafe { lanes_avx512(model, wide, &mut fresh()) };
                    assert_eq!(text(&avx512), text(&baseline));
                }
            }
        }
    }
}
