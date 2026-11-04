mod domain;

use serde::{Deserialize, Deserializer, Serialize, de::DeserializeOwned};
use serde_json::{Value, json, value::RawValue};
use sha2::{Digest, Sha256};
use std::{
    collections::HashSet,
    ops::Range,
    path::PathBuf,
    sync::{
        OnceLock,
        atomic::{AtomicUsize, Ordering},
    },
    time::{Duration, Instant},
};

/// A domain result: objective and optional typed metrics, or a classified failure.
pub(crate) type Evaluation = Result<(f64, Option<domain::Metrics>), Failure>;

#[derive(Debug, Clone)]
pub struct Failure {
    status: &'static str,
    detail: String,
    metrics: Option<Value>,
}
pub fn invalid(detail: impl Into<String>) -> Failure {
    Failure {
        status: "invalid",
        detail: detail.into(),
        metrics: None,
    }
}
pub fn infeasible(detail: impl Into<String>) -> Failure {
    Failure {
        status: "infeasible",
        detail: detail.into(),
        metrics: None,
    }
}

impl Failure {
    pub fn with_metrics(mut self, metrics: Value) -> Self {
        self.metrics = Some(metrics);
        self
    }
}

#[derive(Clone, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Candidate {
    pub id: String,
    pub parameters: Vec<f64>,
    #[serde(default)]
    pub trace: bool,
}

/// `Candidate` with `parameters` decoded by `fast_parameters`; see `decode`.
#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct FastCandidate {
    id: String,
    #[serde(deserialize_with = "fast_parameters")]
    parameters: Vec<f64>,
    #[serde(default)]
    trace: bool,
}

impl From<FastCandidate> for Candidate {
    fn from(c: FastCandidate) -> Self {
        Self {
            id: c.id,
            parameters: c.parameters,
            trace: c.trace,
        }
    }
}

/// Borrow a parameter array as raw JSON and parse it with `parse_numbers`.
fn fast_parameters<'de, D: Deserializer<'de>>(deserializer: D) -> Result<Vec<f64>, D::Error> {
    let raw = <&'de RawValue>::deserialize(deserializer)?;
    parse_numbers(raw.get()).ok_or_else(|| serde::de::Error::custom("use the reference decoder"))
}

/// Powers of ten that are exact doubles.
const EXACT_POWERS_OF_TEN: [f64; 23] = [
    1e0, 1e1, 1e2, 1e3, 1e4, 1e5, 1e6, 1e7, 1e8, 1e9, 1e10, 1e11, 1e12, 1e13, 1e14, 1e15, 1e16,
    1e17, 1e18, 1e19, 1e20, 1e21, 1e22,
];

/// Parse a JSON array of finite numbers under the strict JSON grammar, or decline.
///
/// A number with at most 19 digits is read as an integer `m` and a decimal
/// exponent `e`. When `m <= 2^53` and `|e| <= 22`, both `m` and `10^|e|` are
/// exact doubles, so one multiply or divide yields the correctly rounded value
/// (Clinger's fast path). Other numbers use `str::parse`, which also rounds
/// correctly. serde_json's `float_roundtrip` parser rounds correctly too, so
/// both decoders agree on every finite value, including signed zeros.
/// Non-numbers, nesting and out-of-range values return `None`.
fn parse_numbers(text: &str) -> Option<Vec<f64>> {
    let b = text.as_bytes();
    let space = |mut i: usize| {
        while let Some(b' ' | b'\t' | b'\n' | b'\r') = b.get(i) {
            i += 1;
        }
        i
    };
    if b.first() != Some(&b'[') {
        return None;
    }
    let mut values = Vec::with_capacity((b.len() / 4).min(64));
    let mut i = space(1);
    if b.get(i) == Some(&b']') {
        return (i + 1 == b.len()).then_some(values);
    }
    loop {
        let start = i;
        let negative = b.get(i) == Some(&b'-');
        i += usize::from(negative);
        let mut mantissa = 0u64;
        let first = i;
        match b.get(i) {
            Some(b'0') => i += 1,
            Some(b'1'..=b'9') => {
                while let Some(&c @ b'0'..=b'9') = b.get(i) {
                    mantissa = mantissa.wrapping_mul(10).wrapping_add(u64::from(c - b'0'));
                    i += 1;
                }
            }
            _ => return None,
        }
        let mut digits = i - first;
        let mut exponent = 0i64;
        if b.get(i) == Some(&b'.') {
            i += 1;
            let fraction = i;
            while let Some(&c @ b'0'..=b'9') = b.get(i) {
                mantissa = mantissa.wrapping_mul(10).wrapping_add(u64::from(c - b'0'));
                i += 1;
            }
            if i == fraction {
                return None;
            }
            digits += i - fraction;
            exponent = -((i - fraction) as i64);
        }
        if let Some(b'e' | b'E') = b.get(i) {
            i += 1;
            let negative_exponent = b.get(i) == Some(&b'-');
            if let Some(b'+' | b'-') = b.get(i) {
                i += 1;
            }
            let magnitude = i;
            let mut power = 0i64;
            while let Some(&c @ b'0'..=b'9') = b.get(i) {
                power = (power * 10 + i64::from(c - b'0')).min(1 << 40);
                i += 1;
            }
            if i == magnitude {
                return None;
            }
            exponent += if negative_exponent { -power } else { power };
        }
        let value = if digits <= 19 && mantissa <= 1 << 53 && (-22..=22).contains(&exponent) {
            let magnitude = mantissa as f64;
            let magnitude = if exponent >= 0 {
                magnitude * EXACT_POWERS_OF_TEN[exponent as usize]
            } else {
                magnitude / EXACT_POWERS_OF_TEN[(-exponent) as usize]
            };
            if negative { -magnitude } else { magnitude }
        } else {
            text[start..i].parse::<f64>().ok()?
        };
        if !value.is_finite() {
            return None;
        }
        values.push(value);
        i = space(i);
        match b.get(i) {
            Some(b',') => i = space(i + 1),
            Some(b']') => return (i + 1 == b.len()).then_some(values),
            _ => return None,
        }
    }
}

/// Decode with fast parameter arrays. On any failure, decode again with the
/// reference types, so results and error messages are exactly the reference ones.
fn decode<'a, Fast: Deserialize<'a>, Reference: DeserializeOwned>(
    text: &'a str,
    convert: impl FnOnce(Fast) -> Reference,
) -> Result<Reference, String> {
    match serde_json::from_str::<Fast>(text) {
        Ok(fast) => Ok(convert(fast)),
        Err(_) => serde_json::from_str(text).map_err(|e| e.to_string()),
    }
}

fn decode_candidates(text: &str) -> Result<Vec<Candidate>, String> {
    decode(text, |fast: Vec<FastCandidate>| {
        fast.into_iter().map(Candidate::from).collect()
    })
}

#[derive(Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct Budget {
    workers: usize,
    max_steps: u64,
    wall_ms: Option<u64>,
    cancel_file: Option<PathBuf>,
}
impl Default for Budget {
    fn default() -> Self {
        Self {
            workers: 1,
            max_steps: 1_000_000,
            wall_ms: None,
            cancel_file: None,
        }
    }
}

pub struct Work<'a> {
    budget: &'a Budget,
    deadline: Option<Instant>,
    used: u64,
    monitored: bool,
    metrics_enabled: bool,
}
impl Work<'_> {
    pub(crate) fn metrics(
        &self,
        make: impl FnOnce() -> domain::Metrics,
    ) -> Option<domain::Metrics> {
        self.metrics_enabled.then(make)
    }

    #[inline]
    pub fn step(&mut self) -> Result<(), Failure> {
        if self.used < self.budget.max_steps && !self.monitored {
            self.used += 1;
            return Ok(());
        }
        self.checked_step()
    }

    #[cold]
    fn checked_step(&mut self) -> Result<(), Failure> {
        if self.budget.cancel_file.as_ref().is_some_and(|p| p.exists()) {
            return Err(Failure {
                status: "cancelled",
                detail: "cancel file exists".into(),
                metrics: None,
            });
        }
        if self.deadline.is_some_and(|d| Instant::now() >= d) {
            return Err(Failure {
                status: "wall_limit",
                detail: "batch deadline reached".into(),
                metrics: None,
            });
        }
        if self.used == self.budget.max_steps {
            return Err(Failure {
                status: "work_limit",
                detail: "candidate step budget reached".into(),
                metrics: None,
            });
        }
        self.used += 1;
        Ok(())
    }
}

/// Record metrics: typed domain metrics for complete rows, JSON for failures.
#[derive(Serialize)]
#[serde(untagged)]
enum RecordMetrics {
    Domain(domain::Metrics),
    Json(Value),
}

#[derive(Serialize)]
struct Record<'a> {
    id: &'a str,
    status: &'static str,
    steps: u64,
    score: Option<f64>,
    metrics: Option<RecordMetrics>,
    detail: Option<String>,
}

#[derive(Serialize)]
struct Batch<'a> {
    model_sha256: &'a str,
    records: Vec<Record<'a>>,
    schema: &'static str,
    version: &'static str,
}

/// Append a static string that needs no JSON escaping, with quotes.
fn write_plain(out: &mut Vec<u8>, text: &'static str) {
    debug_assert!(
        text.bytes()
            .all(|b| b.is_ascii_alphanumeric() || b"_-./".contains(&b))
    );
    out.push(b'"');
    out.extend_from_slice(text.as_bytes());
    out.push(b'"');
}

impl Batch<'_> {
    /// Append exactly the JSON text of `serde_json::to_string(self)`. Keys and
    /// static strings are written as pre-encoded bytes in serde's field order;
    /// ids, details, failure metrics and numbers go through serde_json.
    fn write_json(&self, out: &mut Vec<u8>) -> serde_json::Result<()> {
        out.extend_from_slice(b"{\"model_sha256\":");
        serde_json::to_writer(&mut *out, self.model_sha256)?;
        out.extend_from_slice(b",\"records\":[");
        for (i, record) in self.records.iter().enumerate() {
            if i > 0 {
                out.push(b',');
            }
            out.extend_from_slice(b"{\"id\":");
            serde_json::to_writer(&mut *out, record.id)?;
            out.extend_from_slice(b",\"status\":");
            write_plain(out, record.status);
            out.extend_from_slice(b",\"steps\":");
            serde_json::to_writer(&mut *out, &record.steps)?;
            out.extend_from_slice(b",\"score\":");
            serde_json::to_writer(&mut *out, &record.score)?;
            out.extend_from_slice(b",\"metrics\":");
            match &record.metrics {
                None => out.extend_from_slice(b"null"),
                Some(RecordMetrics::Domain(metrics)) => metrics.write_json(out)?,
                Some(RecordMetrics::Json(value)) => serde_json::to_writer(&mut *out, value)?,
            }
            out.extend_from_slice(b",\"detail\":");
            serde_json::to_writer(&mut *out, &record.detail)?;
            out.push(b'}');
        }
        out.extend_from_slice(b"],\"schema\":");
        write_plain(out, self.schema);
        out.extend_from_slice(b",\"version\":");
        write_plain(out, self.version);
        out.push(b'}');
        Ok(())
    }
}

/// One untraced candidate's outcome from `Prepared::score_matrix`.
#[derive(Clone, Debug, PartialEq)]
pub struct MatrixScore {
    /// Same status strings as the JSON records.
    pub status: &'static str,
    /// Logical domain steps charged to the candidate.
    pub steps: u64,
    /// Objective value; `Some` only for `"complete"`.
    pub score: Option<f64>,
}

pub struct Prepared {
    model: domain::Model,
    model_sha256: String,
}
impl Prepared {
    pub fn new(model: &str) -> Result<Self, String> {
        let model_sha256 = format!("{:x}", Sha256::digest(model.as_bytes()));
        let model = domain::prepare(serde_json::from_str(model).map_err(|e| e.to_string())?)?;
        Ok(Self {
            model,
            model_sha256,
        })
    }

    /// SHA-256 of the exact model bytes passed to `Prepared::new`.
    pub fn model_sha256(&self) -> &str {
        &self.model_sha256
    }

    pub fn evaluate_json(&self, candidates: &str, budget: &str) -> Result<String, String> {
        let candidates = decode_candidates(candidates)?;
        let budget = serde_json::from_str(budget).map_err(|e| e.to_string())?;
        self.evaluate_encoded(candidates, budget, true)
    }

    /// Evaluate untraced candidates with full feasibility checks and null metrics.
    pub fn scores_json(&self, candidates: &str, budget: &str) -> Result<String, String> {
        let candidates = decode_candidates(candidates)?;
        let budget = serde_json::from_str(budget).map_err(|e| e.to_string())?;
        if candidates.iter().any(|c| c.trace) {
            return Err("scores mode accepts untraced candidates only".into());
        }
        self.evaluate_encoded(candidates, budget, false)
    }

    /// Evaluate `rows` untraced candidates whose parameters are stored row-major in
    /// `parameters`. Each row has `parameters.len() / rows` values; dimension 0 is
    /// allowed. Budget JSON, limits, worker dispatch, deadlines, cancellation and
    /// failure classification are the same as `scores_json`.
    pub fn score_matrix(
        &self,
        rows: usize,
        parameters: &[f64],
        budget: &str,
    ) -> Result<Vec<MatrixScore>, String> {
        if !(1..=4096).contains(&rows) {
            return Err("matrix rows must be between 1 and 4096".into());
        }
        if !parameters.len().is_multiple_of(rows) {
            return Err("parameter matrix length is not a multiple of rows".into());
        }
        let dimension = parameters.len() / rows;
        if dimension > 64 {
            return Err("parameter vector exceeds limit".into());
        }
        let budget: Budget = serde_json::from_str(budget).map_err(|e| e.to_string())?;
        check_budget(&budget, rows)?;
        let deadline = deadline(&budget);
        let row = |index: usize| {
            (
                &parameters[index * dimension..(index + 1) * dimension],
                false,
            )
        };
        Ok(self.dispatch(rows, budget.workers, |range, emit| {
            self.outcomes(
                range,
                row,
                &budget,
                deadline,
                false,
                |index, result, steps| {
                    emit(
                        index,
                        match result {
                            Ok((score, _)) => MatrixScore {
                                status: "complete",
                                steps,
                                score: Some(score),
                            },
                            Err(e) => MatrixScore {
                                status: e.status,
                                steps,
                                score: None,
                            },
                        },
                    )
                },
            )
        }))
    }

    fn evaluate_encoded(
        &self,
        candidates: Vec<Candidate>,
        budget: Budget,
        metrics_enabled: bool,
    ) -> Result<String, String> {
        let batch = self.evaluate(&candidates, budget, metrics_enabled)?;
        let mut out = Vec::with_capacity(256 + batch.records.len() * 256);
        batch.write_json(&mut out).map_err(|e| e.to_string())?;
        String::from_utf8(out).map_err(|e| e.to_string())
    }

    /// Run one candidate's domain evaluation and return its classified result and steps.
    fn outcome(
        &self,
        parameters: &[f64],
        trace: bool,
        budget: &Budget,
        deadline: Option<Instant>,
        metrics_enabled: bool,
    ) -> (Evaluation, u64) {
        let mut work = Work {
            budget,
            deadline,
            used: 0,
            metrics_enabled,
            monitored: deadline.is_some() || budget.cancel_file.is_some(),
        };
        let result = domain::evaluate(&self.model, parameters, trace, &mut work);
        (finite_objective(result), work.used)
    }

    /// Emit the classified outcome of every row in `range`, keyed by row index.
    ///
    /// Without a deadline or cancel file, untraced rows whose parameters pass the
    /// domain check are integrated `domain::lane_width` at a time in lockstep.
    /// Traced rows, monitored budgets and a single leftover row use the scalar
    /// kernel.
    fn outcomes<'p>(
        &self,
        range: Range<usize>,
        row: impl Fn(usize) -> (&'p [f64], bool),
        budget: &Budget,
        deadline: Option<Instant>,
        metrics_enabled: bool,
        mut emit: impl FnMut(usize, Evaluation, u64),
    ) {
        let lockstep = deadline.is_none() && budget.cancel_file.is_none();
        let width = domain::lane_width(&self.model);
        let mut group = [0usize; domain::MAX_LANES];
        let mut filled = 0;
        for index in range {
            let (parameters, trace) = row(index);
            if !lockstep || trace {
                let (result, steps) =
                    self.outcome(parameters, trace, budget, deadline, metrics_enabled);
                emit(index, result, steps);
                continue;
            }
            if let Err(failure) = domain::check(&self.model, parameters) {
                emit(index, Err(failure), 0);
                continue;
            }
            group[filled] = index;
            filled += 1;
            if filled == width {
                self.lanes(&group[..filled], &row, budget, metrics_enabled, &mut emit);
                filled = 0;
            }
        }
        match filled {
            0 => {}
            1 => {
                let (result, steps) =
                    self.outcome(row(group[0]).0, false, budget, None, metrics_enabled);
                emit(group[0], result, steps);
            }
            _ => self.lanes(&group[..filled], &row, budget, metrics_enabled, &mut emit),
        }
    }

    /// Integrate the rows of `group` in lockstep.
    fn lanes<'p>(
        &self,
        group: &[usize],
        row: &impl Fn(usize) -> (&'p [f64], bool),
        budget: &Budget,
        metrics_enabled: bool,
        emit: &mut impl FnMut(usize, Evaluation, u64),
    ) {
        let mut parameters: [&[f64]; domain::MAX_LANES] = [&[]; domain::MAX_LANES];
        for (slot, &index) in parameters.iter_mut().zip(group) {
            *slot = row(index).0;
        }
        let mut work = Work {
            budget,
            deadline: None,
            used: 0,
            metrics_enabled,
            monitored: false,
        };
        domain::evaluate_lanes(
            &self.model,
            &parameters[..group.len()],
            &mut work,
            |lane, result, steps| emit(group[lane], finite_objective(result), steps),
        );
    }

    fn record<'a>(
        &self,
        c: &'a Candidate,
        result: Evaluation,
        steps: u64,
        metrics_enabled: bool,
    ) -> Record<'a> {
        match result {
            Ok((score, metrics)) => Record {
                id: &c.id,
                status: "complete",
                steps,
                score: Some(score),
                metrics: metrics.map(RecordMetrics::Domain),
                detail: None,
            },
            Err(e) => Record {
                id: &c.id,
                status: e.status,
                steps,
                score: None,
                metrics: if metrics_enabled {
                    e.metrics.map(RecordMetrics::Json)
                } else {
                    None
                },
                detail: Some(e.detail),
            },
        }
    }

    /// Produce one item per index in index order. `fill` emits the items of an
    /// index range; it runs inline, or on the calling thread plus scoped workers
    /// that claim ranges of one lockstep group width.
    fn dispatch<T: Send + Sync>(
        &self,
        count: usize,
        workers: usize,
        fill: impl Fn(Range<usize>, &mut dyn FnMut(usize, T)) + Sync,
    ) -> Vec<T> {
        if workers == 1 || count == 1 {
            let mut items: Vec<Option<T>> = (0..count).map(|_| None).collect();
            fill(0..count, &mut |index, item| items[index] = Some(item));
            return items
                .into_iter()
                .map(|item| item.expect("missing evaluation"))
                .collect();
        }
        let chunk = domain::lane_width(&self.model);
        let next = AtomicUsize::new(0);
        let slots: Vec<_> = (0..count).map(|_| OnceLock::new()).collect();
        let claim = || {
            loop {
                let start = next.fetch_add(chunk, Ordering::Relaxed);
                if start >= count {
                    break;
                }
                fill(start..(start + chunk).min(count), &mut |index, item| {
                    assert!(slots[index].set(item).is_ok(), "duplicate work index");
                });
            }
        };
        // The calling thread is one of the workers.
        std::thread::scope(|scope| {
            for _ in 1..workers.min(count.div_ceil(chunk)) {
                scope.spawn(claim);
            }
            claim();
        });
        slots
            .into_iter()
            .map(|slot| slot.into_inner().expect("missing evaluation"))
            .collect()
    }

    fn evaluate<'a>(
        &'a self,
        candidates: &'a [Candidate],
        budget: Budget,
        metrics_enabled: bool,
    ) -> Result<Batch<'a>, String> {
        check_budget(&budget, candidates.len())?;
        let mut seen = HashSet::with_capacity(candidates.len());
        for c in candidates {
            if c.id.is_empty() || c.id.len() > 128 || !seen.insert(&c.id) || c.parameters.len() > 64
            {
                return Err(
                    "invalid or duplicate candidate id, or parameter vector exceeds limit".into(),
                );
            }
        }
        if candidates.iter().filter(|c| c.trace).count() > 16 {
            return Err("at most 16 traced candidates per batch".into());
        }
        let deadline = deadline(&budget);
        let row = |index: usize| {
            let c = &candidates[index];
            (c.parameters.as_slice(), c.trace)
        };
        let records = self.dispatch(candidates.len(), budget.workers, |range, emit| {
            self.outcomes(
                range,
                row,
                &budget,
                deadline,
                metrics_enabled,
                |index, result, steps| {
                    emit(
                        index,
                        self.record(&candidates[index], result, steps, metrics_enabled),
                    )
                },
            )
        });
        Ok(Batch {
            model_sha256: &self.model_sha256,
            records,
            schema: concat!(env!("CARGO_PKG_NAME"), ".result/0.1"),
            version: env!("CARGO_PKG_VERSION"),
        })
    }
}

/// Classify a nonfinite objective as an invalid candidate.
fn finite_objective(result: Evaluation) -> Evaluation {
    result.and_then(|(score, metrics)| {
        if score.is_finite() {
            Ok((score, metrics))
        } else {
            Err(invalid("nonfinite objective"))
        }
    })
}

fn check_budget(budget: &Budget, count: usize) -> Result<(), String> {
    if !(1..=64).contains(&budget.workers)
        || !(1..=100_000_000).contains(&budget.max_steps)
        || budget.wall_ms.is_some_and(|n| n == 0 || n > 86_400_000)
        || count == 0
        || count > 4096
    {
        return Err("invalid budget or batch size".into());
    }
    Ok(())
}

fn deadline(budget: &Budget) -> Option<Instant> {
    budget
        .wall_ms
        .map(|n| Instant::now() + Duration::from_millis(n))
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct Request<C> {
    schema: String,
    model: Box<RawValue>,
    candidates: Vec<C>,
    #[serde(default)]
    budget: Budget,
}

pub fn run(text: &str) -> Result<Value, String> {
    let req: Request<Candidate> = decode(text, |fast: Request<FastCandidate>| Request {
        schema: fast.schema,
        model: fast.model,
        candidates: fast.candidates.into_iter().map(Candidate::from).collect(),
        budget: fast.budget,
    })?;
    if req.schema != concat!(env!("CARGO_PKG_NAME"), ".request/0.1") {
        return Err("unsupported request schema".into());
    }
    let prepared = Prepared::new(req.model.get())?;
    serde_json::to_value(prepared.evaluate(&req.candidates, req.budget, true)?)
        .map_err(|e| e.to_string())
}

pub fn capabilities() -> Value {
    json!({"schema": concat!(env!("CARGO_PKG_NAME"), ".capabilities/0.1"),
           "version": env!("CARGO_PKG_VERSION"), "models": domain::CAPABILITIES,
           "batch": true, "worker_limit": 64})
}

#[cfg(feature = "python")]
mod python {
    use super::*;
    use pyo3::{exceptions::PyValueError, prelude::*};

    #[pyclass(name = "Prepared")]
    struct PyPrepared(Prepared);

    #[pymethods]
    impl PyPrepared {
        #[new]
        fn new(model: &str) -> PyResult<Self> {
            Prepared::new(model)
                .map(Self)
                .map_err(PyValueError::new_err)
        }
        fn scores(&self, py: Python<'_>, candidates: String, budget: String) -> PyResult<String> {
            py.detach(|| self.0.scores_json(&candidates, &budget))
                .map_err(PyValueError::new_err)
        }
        fn batch(&self, py: Python<'_>, candidates: String, budget: String) -> PyResult<String> {
            py.detach(|| self.0.evaluate_json(&candidates, &budget))
                .map_err(PyValueError::new_err)
        }
        /// Score untraced rows; returns (scores, statuses, steps) with NaN scores
        /// for every non-complete row.
        fn score_matrix(
            &self,
            py: Python<'_>,
            rows: Vec<Vec<f64>>,
            budget: String,
        ) -> PyResult<(Vec<f64>, Vec<&'static str>, Vec<u64>)> {
            let dimension = rows.first().map_or(0, Vec::len);
            if rows.iter().any(|row| row.len() != dimension) {
                return Err(PyValueError::new_err(
                    "parameter rows must all have the same length",
                ));
            }
            let parameters = rows.concat();
            let count = rows.len();
            let scores = py
                .detach(|| self.0.score_matrix(count, &parameters, &budget))
                .map_err(PyValueError::new_err)?;
            let mut values = Vec::with_capacity(scores.len());
            let mut statuses = Vec::with_capacity(scores.len());
            let mut steps = Vec::with_capacity(scores.len());
            for row in scores {
                values.push(row.score.unwrap_or(f64::NAN));
                statuses.push(row.status);
                steps.push(row.steps);
            }
            Ok((values, statuses, steps))
        }
        #[getter]
        fn model_sha256(&self) -> &str {
            self.0.model_sha256()
        }
    }

    #[pymodule]
    fn _native(m: &Bound<'_, PyModule>) -> PyResult<()> {
        m.add_class::<PyPrepared>()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    fn fixture() -> Value {
        serde_json::from_str(include_str!("../models/example.json")).unwrap()
    }
    #[test]
    fn prepared_workers_return_same_completed_records() {
        let example = fixture();
        let prepared = Prepared::new(&example["model"].to_string()).unwrap();
        let candidates = json!([
            {"id":"first","parameters":example["parameters"]},
            {"id":"second","parameters":example["parameters"]},
        ]);
        let serial = prepared
            .evaluate_json(&candidates.to_string(), r#"{"workers":1}"#)
            .unwrap();
        let parallel = prepared
            .evaluate_json(&candidates.to_string(), r#"{"workers":4}"#)
            .unwrap();
        assert_eq!(serial, parallel);
        assert!(serial.contains(r#""status":"complete""#));
    }
    #[test]
    fn scores_keep_order_status_steps_and_objective() {
        let example = fixture();
        let prepared = Prepared::new(&example["model"].to_string()).unwrap();
        let mut candidates = json!([
            {"id":"valid","parameters":example["parameters"]},
            {"id":"invalid","parameters": vec![1.0; 63]},
        ]);
        for budget in ["{}", r#"{"workers":4}"#, r#"{"max_steps":1}"#] {
            let mut full: Value = serde_json::from_str(
                &prepared
                    .evaluate_json(&candidates.to_string(), budget)
                    .unwrap(),
            )
            .unwrap();
            for row in full["records"].as_array_mut().unwrap() {
                row["metrics"] = Value::Null;
            }
            let scores: Value = serde_json::from_str(
                &prepared
                    .scores_json(&candidates.to_string(), budget)
                    .unwrap(),
            )
            .unwrap();
            assert_eq!(full, scores);
        }
        candidates[0]["trace"] = json!(true);
        assert!(prepared.scores_json(&candidates.to_string(), "{}").is_err());
    }

    /// Check that every matrix row reproduces the lean JSON status, steps and score bits.
    fn assert_matrix_matches_scores(prepared: &Prepared, rows: &[Vec<f64>], budget: &str) {
        let candidates: Vec<Value> = rows
            .iter()
            .enumerate()
            .map(|(i, p)| json!({"id": format!("row-{i}"), "parameters": p}))
            .collect();
        let lean: Value = serde_json::from_str(
            &prepared
                .scores_json(&Value::from(candidates).to_string(), budget)
                .unwrap(),
        )
        .unwrap();
        let matrix = prepared
            .score_matrix(rows.len(), &rows.concat(), budget)
            .unwrap();
        let records = lean["records"].as_array().unwrap();
        assert_eq!(records.len(), matrix.len());
        for (record, row) in records.iter().zip(&matrix) {
            assert_eq!(record["status"], row.status, "{budget}");
            assert_eq!(record["steps"].as_u64(), Some(row.steps), "{budget}");
            assert_eq!(
                record["score"].as_f64().map(f64::to_bits),
                row.score.map(f64::to_bits),
                "{budget}"
            );
            assert_eq!(row.score.is_some(), row.status == "complete");
        }
    }

    #[test]
    fn score_matrix_matches_scores_json() {
        let example = fixture();
        let prepared = Prepared::new(&example["model"].to_string()).unwrap();
        let mut rows: Vec<Vec<f64>> = (0..9)
            .map(|i| vec![4.0 * (1.0 + f64::from(i) * 0.05), 2.0 - f64::from(i) * 0.1])
            .collect();
        rows.push(vec![-1.0, 2.0]);
        rows.push(vec![1e6, 0.0]);
        let cancel = std::env::temp_dir().join(format!("matrix-cancel-{}", std::process::id()));
        std::fs::write(&cancel, b"").unwrap();
        let cancel_budget = json!({"cancel_file": cancel}).to_string();
        for budget in [
            "{}",
            r#"{"workers":4}"#,
            r#"{"workers":3,"max_steps":30}"#,
            r#"{"max_steps":1}"#,
            r#"{"wall_ms":60000}"#,
            &cancel_budget,
        ] {
            assert_matrix_matches_scores(&prepared, &rows, budget);
        }
        let cancelled = prepared.score_matrix(1, &rows[0], &cancel_budget).unwrap();
        assert_eq!(
            cancelled,
            vec![MatrixScore {
                status: "cancelled",
                steps: 0,
                score: None
            }]
        );
        std::fs::remove_file(&cancel).unwrap();
        assert_matrix_matches_scores(&prepared, &[vec![], vec![]], "{}");
        let nan = prepared.score_matrix(1, &[f64::NAN, 1.0], "{}").unwrap();
        assert_eq!(
            (nan[0].status, nan[0].steps, nan[0].score),
            ("invalid", 0, None)
        );

        let mut held = example["model"].clone();
        held["control"] = json!({"kind": "held", "switches_s": [0.4, 1.1]});
        let held = Prepared::new(&held.to_string()).unwrap();
        let rows = [
            vec![-1.0, -0.5, 0.0],
            vec![3.0, 3.0, 3.0],
            vec![5.0, 0.0, 0.0],
            vec![0.5, 0.25, -0.25],
        ];
        for budget in ["{}", r#"{"workers":2}"#, r#"{"max_steps":20}"#] {
            assert_matrix_matches_scores(&held, &rows, budget);
        }
        let statuses: Vec<_> = held
            .score_matrix(4, &rows.concat(), "{}")
            .unwrap()
            .iter()
            .map(|row| row.status)
            .collect();
        assert_eq!(statuses, ["complete", "infeasible", "invalid", "complete"]);
    }

    /// Full records from lockstep lanes must equal the scalar kernel byte for byte
    /// for every batch size from one row up to `rows.len()`, which covers full,
    /// padded and single leftover groups. A `wall_ms` budget is monitored, which
    /// forces the scalar kernel.
    fn assert_lanes_match_scalar(prepared: &Prepared, rows: &[Value], budget: Value) {
        let mut statuses = HashSet::new();
        for count in 1..=rows.len() {
            let candidates: Vec<Value> = rows[..count]
                .iter()
                .enumerate()
                .map(|(i, p)| json!({"id": format!("c{i}"), "parameters": p}))
                .collect();
            let candidates = Value::from(candidates).to_string();
            let mut scalar = budget.clone();
            scalar["wall_ms"] = json!(3_600_000);
            for workers in [1, 3] {
                let mut lockstep = budget.clone();
                lockstep["workers"] = json!(workers);
                scalar["workers"] = json!(workers);
                let lanes = prepared
                    .evaluate_json(&candidates, &lockstep.to_string())
                    .unwrap();
                let reference = prepared
                    .evaluate_json(&candidates, &scalar.to_string())
                    .unwrap();
                assert_eq!(lanes, reference, "{count} rows, {lockstep}");
                let lanes: Value = serde_json::from_str(&lanes).unwrap();
                for row in lanes["records"].as_array().unwrap() {
                    statuses.insert(row["status"].as_str().unwrap().to_owned());
                }
            }
        }
        assert!(statuses.len() >= 2, "{budget}: {statuses:?}");
    }

    #[test]
    fn lockstep_lanes_match_scalar_kernel() {
        let example = fixture();
        let mut model = example["model"].clone();
        model["target_m"] = json!(1.8);
        model["initial_x_m"] = json!(0.0);
        let pd = Prepared::new(&model.to_string()).unwrap();
        let rows = [
            json!([4.0, 2.0]),
            json!([40.0, 0.0]),
            json!([-1.0, 0.0]),
            json!([2.0, 1.0]),
            json!([9.0, 0.5]),
            json!([0.0, 0.0]),
            json!([1e6, 0.0]),
            json!([6.0, 3.0]),
            json!([25.0, 0.1]),
            json!([12.0, 0.2]),
            json!([3.0, 0.0]),
            json!([5.0, 2.5]),
            json!([15.0, 0.3]),
            json!([0.5, 0.5]),
            json!([7.5, 1.0]),
            json!([30.0, 0.05]),
            json!([1.0, 4.0]),
            json!([20.0, 0.0]),
            json!([8.0, 2.0]),
        ];
        assert_lanes_match_scalar(&pd, &rows, json!({}));
        assert_lanes_match_scalar(&pd, &rows, json!({"max_steps": 30}));
        assert_lanes_match_scalar(&pd, &rows, json!({"max_steps": 55}));
        let mut held = example["model"].clone();
        held["control"] = json!({"kind": "held", "switches_s": [0.4, 1.1]});
        let held = Prepared::new(&held.to_string()).unwrap();
        let rows = [
            json!([-1.0, -0.5, 0.0]),
            json!([3.0, 3.0, 3.0]),
            json!([5.0, 0.0, 0.0]),
            json!([0.5, 0.25, -0.25]),
            json!([2.5, 2.9, 0.0]),
            json!([-3.0, 3.0, -3.0]),
            json!([0.0, 0.0]),
            json!([1.0, 2.0, 2.5]),
            json!([-0.5, 0.5, 2.0]),
            json!([2.9, 2.9, 2.9]),
            json!([0.1, 0.2, 0.3]),
            json!([1.5, -1.5, 1.0]),
            json!([2.0, 2.0, 2.0]),
            json!([-2.5, 0.0, 2.5]),
            json!([0.0, 3.0, 0.0]),
            json!([3.0, -3.0, 1.9]),
            json!([0.7, 0.7, 0.7]),
            json!([-1.0, 1.0, -1.0]),
            json!([2.2, 1.1, 0.0]),
        ];
        assert_lanes_match_scalar(&held, &rows, json!({}));
        assert_lanes_match_scalar(&held, &rows, json!({"max_steps": 20}));
        assert_lanes_match_scalar(&held, &rows, json!({"max_steps": 45}));
        let lanes: Value = serde_json::from_str(
            &held
                .evaluate_json(
                    &json!([{"id":"a","parameters":rows[0]},{"id":"b","parameters":rows[1]},
                             {"id":"c","parameters":rows[3]},{"id":"d","parameters":rows[4]}])
                    .to_string(),
                    "{}",
                )
                .unwrap(),
        )
        .unwrap();
        let statuses: Vec<_> = lanes["records"]
            .as_array()
            .unwrap()
            .iter()
            .map(|r| r["status"].as_str().unwrap())
            .collect();
        assert_eq!(
            statuses,
            ["complete", "infeasible", "complete", "infeasible"]
        );
    }

    #[test]
    fn metrics_keys_keep_sorted_json_order() {
        let example = fixture();
        let prepared = Prepared::new(&example["model"].to_string()).unwrap();
        for trace in [false, true] {
            let candidates = json!([{"id":"a","parameters":example["parameters"],"trace":trace}]);
            let text = prepared
                .evaluate_json(&candidates.to_string(), "{}")
                .unwrap();
            let keys = [
                "integrated_error",
                "integrated_force_squared",
                "max_abs_force_n",
                "max_abs_position_m",
                "terminal_position_m",
                "terminal_velocity_m_s",
                "trace",
                "trace_columns",
            ];
            let positions: Vec<_> = keys
                .iter()
                .map(|k| text.find(&format!("\"{k}\":")).unwrap())
                .collect();
            assert!(positions.windows(2).all(|w| w[0] < w[1]), "{text}");
            let value: Value = serde_json::from_str(&text).unwrap();
            let metrics = serde_json::to_string(&value["records"][0]["metrics"]).unwrap();
            assert!(text.contains(&metrics), "{text}");
        }
    }

    #[test]
    fn traced_rollouts_stop_after_the_trace_limit() {
        let example = fixture();
        let mut model = example["model"].clone();
        model["dt_s"] = json!(1e-4);
        let prepared = Prepared::new(&model.to_string()).unwrap();
        let rows = json!([
            {"id": "traced", "parameters": example["parameters"], "trace": true},
            {"id": "untraced", "parameters": example["parameters"]},
        ]);
        let out: Value =
            serde_json::from_str(&prepared.evaluate_json(&rows.to_string(), "{}").unwrap())
                .unwrap();
        let traced = &out["records"][0];
        assert_eq!(traced["status"], "invalid");
        assert_eq!(traced["steps"], 10_001);
        assert_eq!(
            traced["detail"],
            "trace requires at most 10,000 integration steps"
        );
        assert_eq!(out["records"][1]["status"], "complete");
        let short = json!([{"id": "t", "parameters": example["parameters"], "trace": true}]);
        let out: Value = serde_json::from_str(
            &Prepared::new(&example["model"].to_string())
                .unwrap()
                .evaluate_json(&short.to_string(), "{}")
                .unwrap(),
        )
        .unwrap();
        let trace = out["records"][0]["metrics"]["trace"].as_array().unwrap();
        assert_eq!(
            trace.len() as u64,
            out["records"][0]["steps"].as_u64().unwrap()
        );
        assert_eq!(trace.last().unwrap()[0], example["model"]["horizon_s"]);
    }

    /// The fast decoder must give the reference decoder's values bit for bit,
    /// or its exact error.
    fn assert_same_decoding(numbers: &str) {
        let text = format!(r#"[{{"id":"a","parameters":{numbers}}}]"#);
        let reference = serde_json::from_str::<Vec<Candidate>>(&text).map_err(|e| e.to_string());
        let fast = decode_candidates(&text);
        let bits = |r: &Result<Vec<Candidate>, String>| {
            r.as_ref()
                .map(|c| {
                    c[0].parameters
                        .iter()
                        .map(|v| v.to_bits())
                        .collect::<Vec<_>>()
                })
                .map_err(Clone::clone)
        };
        assert_eq!(bits(&fast), bits(&reference), "{numbers}");
        if let Some(values) = parse_numbers(numbers.trim()) {
            assert!(
                serde_json::from_str::<Vec<FastCandidate>>(&text).is_ok(),
                "{numbers}"
            );
            let reference: Vec<f64> = serde_json::from_str(numbers).unwrap();
            let same = values
                .iter()
                .zip(&reference)
                .all(|(a, b)| a.to_bits() == b.to_bits());
            assert!(same && values.len() == reference.len(), "{numbers}");
        }
    }

    #[test]
    fn fast_parameter_decoding_matches_serde_json() {
        for numbers in [
            "[]",
            "[ ]",
            "[0]",
            "[-0]",
            "[-0.0]",
            "[0.0e5]",
            "[-0e-5]",
            "[1]",
            "[1.0]",
            "[4,2]",
            " [ 4 , 2 ] ",
            "[\t1,\n2\r]",
            "[1e5]",
            "[1E5]",
            "[1e+5]",
            "[1.5e-3]",
            "[1e22]",
            "[1e23]",
            "[1e-22]",
            "[1e-23]",
            "[123e20]",
            "[9007199254740992]",
            "[9007199254740993]",
            "[-9007199254740993]",
            "[9007199254740993e-3]",
            "[1234567890123456789]",
            "[12345678901234567890]",
            "[18446744073709551615]",
            "[18446744073709551616]",
            "[-9223372036854775808]",
            "[-9223372036854775809]",
            "[123456789012345678901234567890]",
            "[0.0000000000000000001]",
            "[0.1000000000000000055511151231257827021181583404541015625]",
            "[2.2250738585072011e-308]",
            "[2.2250738585072014e-308]",
            "[4.9e-324]",
            "[2.4703282292062327e-324]",
            "[2.4703282292062328e-324]",
            "[1e-400]",
            "[-1e-400]",
            "[1e400]",
            "[-1e400]",
            "[1.7976931348623157e308]",
            "[1.7976931348623159e308]",
            "[0e99999999999]",
            "[1e-99999999999]",
            "[1e99999999999]",
            "[0.5e99999999999999999999]",
            "[01]",
            "[1.]",
            "[.5]",
            "[+1]",
            "[1e]",
            "[1e+]",
            "[-]",
            "[1,]",
            "[,1]",
            "[[1]]",
            "[\"1\"]",
            "[null]",
            "[true]",
            "1",
            "{}",
            "null",
            "[1 2]",
            "[NaN]",
            "[Infinity]",
            "[-Infinity]",
            "[0.30000000000000004]",
            "[5e-324,1.0,-2.5e+10,7E-3,0.1,0.2,0.3]",
        ] {
            assert_same_decoding(numbers);
        }
        let mut state = 0x9e37_79b9_7f4a_7c15_u64;
        let mut next = || {
            state ^= state << 13;
            state ^= state >> 7;
            state ^= state << 17;
            state
        };
        for _ in 0..20_000 {
            let value = f64::from_bits(next());
            let small = (next() % 2_000_000) as f64 / 64.0 - 15_000.0;
            let integer = next() >> (next() % 64);
            let scale = (next() % 45) as i32 - 22;
            if value.is_finite() {
                let forms = [
                    format!("{value}"),
                    format!("{value:e}"),
                    format!("{value:.20e}"),
                    format!("{value:.3e}"),
                ];
                assert_same_decoding(&format!("[{}]", forms.join(",")));
            }
            assert_same_decoding(&format!(
                "[{small},{small:e},{integer},-{integer},{integer}.5e-7,{}e{scale},-{}.{}E{scale}]",
                integer % 1_000_000,
                integer % 100_000,
                integer % 1000
            ));
        }
        let request = json!({"schema": "robotics-control-craft.request/0.1",
            "model": fixture()["model"], "candidates": [{"id": "a", "parameters": [4, 2]}]});
        let fast = run(&request.to_string()).unwrap();
        assert_eq!(fast["records"][0]["status"], "complete");
        let text = request.to_string().replace("[4,2]", "[4,\"2\"]");
        let error = run(&text).unwrap_err();
        let reference = serde_json::from_str::<Request<Candidate>>(&text)
            .err()
            .unwrap();
        assert_eq!(error, reference.to_string());
    }

    #[test]
    fn batch_writer_matches_serde_json() {
        let example = fixture();
        let parameters = &example["parameters"];
        let check = |prepared: &Prepared, candidates: Value, budget: &str, metrics: bool| {
            let candidates: Vec<Candidate> = serde_json::from_value(candidates).unwrap();
            let budget: Budget = serde_json::from_str(budget).unwrap();
            let batch = prepared.evaluate(&candidates, budget, metrics).unwrap();
            let mut out = Vec::new();
            batch.write_json(&mut out).unwrap();
            assert_eq!(
                String::from_utf8(out).unwrap(),
                serde_json::to_string(&batch).unwrap()
            );
        };
        let pd = Prepared::new(&example["model"].to_string()).unwrap();
        let mixed = json!([
            {"id": "plain", "parameters": parameters},
            {"id": "esc\"aped\\id\n\u{e9}\u{1}", "parameters": parameters, "trace": true},
            {"id": "bad", "parameters": vec![1.0; 63]},
            {"id": "negative", "parameters": [-1.0, 2.0]},
        ]);
        let cancel = std::env::temp_dir().join(format!("writer-cancel-{}", std::process::id()));
        std::fs::write(&cancel, b"").unwrap();
        let cancel_budget = json!({"cancel_file": cancel}).to_string();
        for budget in [
            "{}",
            r#"{"max_steps":5}"#,
            r#"{"workers":2}"#,
            &cancel_budget,
        ] {
            check(&pd, mixed.clone(), budget, true);
            let untraced = json!([mixed[0], mixed[2], mixed[3]]);
            check(&pd, untraced, budget, false);
        }
        std::fs::remove_file(&cancel).unwrap();
        let mut held = example["model"].clone();
        held["control"] = json!({"kind": "held", "switches_s": [0.4, 1.1]});
        let held = Prepared::new(&held.to_string()).unwrap();
        let forces = json!([
            {"id": "ok", "parameters": [-1.0, -0.5, 0.0], "trace": true},
            {"id": "far", "parameters": [3.0, 3.0, 3.0]},
        ]);
        check(&held, forces, "{}", true);
        let detail = Batch {
            model_sha256: "sha",
            records: vec![Record {
                id: "x",
                status: "infeasible",
                steps: 3,
                score: None,
                metrics: Some(RecordMetrics::Json(json!({"b": [1.5, null], "a": "q\"t"}))),
                detail: Some("tab\tand \"quote\"".into()),
            }],
            schema: "robotics-control-craft.result/0.1",
            version: "0.0.0",
        };
        let mut out = Vec::new();
        detail.write_json(&mut out).unwrap();
        assert_eq!(
            String::from_utf8(out).unwrap(),
            serde_json::to_string(&detail).unwrap()
        );
    }

    #[test]
    fn score_matrix_rejects_bad_shapes_and_budgets() {
        let example = fixture();
        let prepared = Prepared::new(&example["model"].to_string()).unwrap();
        assert!(prepared.score_matrix(0, &[], "{}").is_err());
        assert!(prepared.score_matrix(4097, &[], "{}").is_err());
        assert!(prepared.score_matrix(4096, &[], "{}").is_ok());
        assert!(prepared.score_matrix(2, &[1.0, 2.0, 3.0], "{}").is_err());
        assert!(prepared.score_matrix(1, &[1.0; 65], "{}").is_err());
        assert!(prepared.score_matrix(1, &[1.0; 64], "{}").is_ok());
        for budget in [
            r#"{"workers":0}"#,
            r#"{"max_steps":0}"#,
            r#"{"wall_ms":0}"#,
            "{",
            r#"{"x":1}"#,
        ] {
            assert!(
                prepared.score_matrix(1, &[4.0, 2.0], budget).is_err(),
                "{budget}"
            );
        }
    }

    #[test]
    fn model_and_candidate_validation_are_independent() {
        let mut example = fixture();
        example["model"]["unexpected"] = json!(true);
        assert!(Prepared::new(&example["model"].to_string()).is_err());
        let example = fixture();
        let prepared = Prepared::new(&example["model"].to_string()).unwrap();
        let duplicate = json!([
            {"id":"a","parameters":example["parameters"]},
            {"id":"a","parameters":example["parameters"]},
        ]);
        assert!(
            prepared
                .evaluate_json(&duplicate.to_string(), "{}")
                .is_err()
        );
    }
}
