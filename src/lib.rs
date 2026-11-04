mod domain;

use serde::{Deserialize, Serialize};
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
        let candidates: Vec<Candidate> =
            serde_json::from_str(candidates).map_err(|e| e.to_string())?;
        let budget = serde_json::from_str(budget).map_err(|e| e.to_string())?;
        self.evaluate_encoded(candidates, budget, true)
    }

    /// Evaluate untraced candidates with full feasibility checks and null metrics.
    pub fn scores_json(&self, candidates: &str, budget: &str) -> Result<String, String> {
        let candidates: Vec<Candidate> =
            serde_json::from_str(candidates).map_err(|e| e.to_string())?;
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
        serde_json::to_string(&self.evaluate(&candidates, budget, metrics_enabled)?)
            .map_err(|e| e.to_string())
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
    /// domain check are integrated `domain::LANES` at a time in lockstep. Traced
    /// rows, monitored budgets and a single leftover row use the scalar kernel.
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
        let mut group = [0usize; domain::LANES];
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
            if filled == domain::LANES {
                self.lanes(&group, filled, &row, budget, metrics_enabled, &mut emit);
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
            _ => self.lanes(&group, filled, &row, budget, metrics_enabled, &mut emit),
        }
    }

    /// Integrate the first `filled` rows of `group` in lockstep, padding unused lanes.
    fn lanes<'p>(
        &self,
        group: &[usize; domain::LANES],
        filled: usize,
        row: &impl Fn(usize) -> (&'p [f64], bool),
        budget: &Budget,
        metrics_enabled: bool,
        emit: &mut impl FnMut(usize, Evaluation, u64),
    ) {
        let parameters = std::array::from_fn(|lane| row(group[lane.min(filled - 1)]).0);
        let mut work = Work {
            budget,
            deadline: None,
            used: 0,
            metrics_enabled,
            monitored: false,
        };
        let results = domain::evaluate_lanes(&self.model, parameters, &mut work);
        for (lane, (result, steps)) in results.into_iter().enumerate().take(filled) {
            emit(group[lane], finite_objective(result), steps);
        }
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
    /// index range; it runs inline, or on scoped workers that claim ranges of
    /// `domain::LANES` indexes.
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
        let chunk = domain::LANES;
        let next = AtomicUsize::new(0);
        let slots: Vec<_> = (0..count).map(|_| OnceLock::new()).collect();
        std::thread::scope(|scope| {
            for _ in 0..workers.min(count.div_ceil(chunk)) {
                scope.spawn(|| {
                    loop {
                        let start = next.fetch_add(chunk, Ordering::Relaxed);
                        if start >= count {
                            break;
                        }
                        fill(start..(start + chunk).min(count), &mut |index, item| {
                            assert!(slots[index].set(item).is_ok(), "duplicate work index");
                        });
                    }
                });
            }
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
struct Request {
    schema: String,
    model: Box<RawValue>,
    candidates: Vec<Candidate>,
    #[serde(default)]
    budget: Budget,
}

pub fn run(text: &str) -> Result<Value, String> {
    let req: Request = serde_json::from_str(text).map_err(|e| e.to_string())?;
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
