//! Phase timing of prepared calls on the `examples/performance.rs` workloads.
//!
//! Ignored by default. Run with
//! `cargo test --release --locked breakdown -- --ignored --nocapture`.
//! Prints one JSON document with per-phase medians in microseconds per call:
//! decode (candidate and budget JSON), evaluate (validation, dispatch, domain
//! kernel and records), encode (result JSON), plus matrix calls and worker
//! scaling.

use super::*;
use std::hint::black_box;

/// Model, candidates JSON, row-major parameters, rows and workers of a workload.
fn workload(name: &str) -> (Value, String, Vec<f64>, usize, usize) {
    let example: Value = serde_json::from_str(include_str!("../models/example.json")).unwrap();
    let mut model = example["model"].clone();
    let mut parameters = example["parameters"].clone();
    let (count, workers, traced) = match name {
        "one" => (1, 1, false),
        "serial" => (96, 1, false),
        "parallel" => (96, 4, false),
        "trace" => (12, 1, true),
        "invalid" => (1024, 1, false),
        "domain" | "domain2" => {
            model["dt_s"] = json!(0.001);
            model["horizon_s"] = json!(4.0);
            if name == "domain2" {
                model["control"] = json!({"kind":"held","switches_s":[0.91,1.03,1.91,3.25]});
                parameters = json!([-1.0, 1.0, 0.5, -0.25, 0.0]);
            }
            (48, 1, false)
        }
        _ => panic!("unknown workload"),
    };
    let candidates: Vec<Value> = (0..count)
        .map(|i| {
            let mut values = parameters.clone();
            if name == "invalid" {
                values = json!(vec![1.0; 63]);
            } else if let Some(first) = values.as_array_mut().unwrap().first_mut() {
                *first = json!(first.as_f64().unwrap() * (1.0 + (i % 13) as f64 * 0.003));
            }
            json!({"id": format!("candidate-{i:04}"), "parameters": values, "trace": traced})
        })
        .collect();
    let matrix = candidates
        .iter()
        .flat_map(|c| c["parameters"].as_array().unwrap().iter())
        .map(|v| v.as_f64().unwrap())
        .collect();
    (
        model,
        serde_json::to_string(&candidates).unwrap(),
        matrix,
        count,
        workers,
    )
}

fn median(mut values: Vec<f64>) -> f64 {
    values.sort_by(f64::total_cmp);
    values[values.len() / 2]
}

/// Iterations so that one workload measures for roughly a quarter second.
fn iterations(once: f64) -> usize {
    ((0.25 / once.max(1e-7)) as usize).clamp(11, 20_001)
}

fn phases(prepared: &Prepared, candidates: &str, budget: &str, metrics: bool) -> Value {
    let run = || {
        let start = Instant::now();
        let decoded = decode_candidates(candidates).unwrap();
        let parsed: Budget = serde_json::from_str(budget).unwrap();
        let decoded_at = Instant::now();
        let batch = prepared.evaluate(&decoded, parsed, metrics).unwrap();
        let evaluated_at = Instant::now();
        let mut out = Vec::with_capacity(256 + batch.records.len() * 256);
        batch.write_json(&mut out).unwrap();
        black_box(out);
        let encoded_at = Instant::now();
        [
            decoded_at - start,
            evaluated_at - decoded_at,
            encoded_at - evaluated_at,
            encoded_at - start,
        ]
        .map(|d| d.as_secs_f64() * 1e6)
    };
    let first = run();
    let samples: Vec<_> = (0..iterations(first[3] * 1e-6)).map(|_| run()).collect();
    let column = |i: usize| median(samples.iter().map(|s| s[i]).collect());
    json!({"decode_us": column(0), "evaluate_us": column(1), "encode_us": column(2),
           "total_us": column(3), "iterations": samples.len()})
}

fn timed(mut call: impl FnMut()) -> Value {
    let mut once = || {
        let start = Instant::now();
        call();
        start.elapsed().as_secs_f64() * 1e6
    };
    let first = once();
    let samples: Vec<_> = (0..iterations(first * 1e-6)).map(|_| once()).collect();
    json!({"total_us": median(samples.clone()), "iterations": samples.len()})
}

#[test]
#[ignore = "timing harness; run explicitly with --ignored"]
fn breakdown() {
    let mut report = serde_json::Map::new();
    for name in [
        "one", "serial", "parallel", "trace", "invalid", "domain", "domain2",
    ] {
        let (model, candidates, matrix, rows, workers) = workload(name);
        let prepared = Prepared::new(&model.to_string()).unwrap();
        let budget = json!({ "workers": workers }).to_string();
        let mut entry = serde_json::Map::new();
        entry.insert("rows".into(), json!(rows));
        entry.insert("workers".into(), json!(workers));
        entry.insert("full".into(), phases(&prepared, &candidates, &budget, true));
        if name != "trace" {
            entry.insert(
                "lean".into(),
                phases(&prepared, &candidates, &budget, false),
            );
            entry.insert(
                "matrix".into(),
                timed(|| {
                    black_box(prepared.score_matrix(rows, &matrix, &budget).unwrap());
                }),
            );
        }
        report.insert(name.into(), Value::Object(entry));
    }
    let mut scaling = serde_json::Map::new();
    for name in ["serial", "domain"] {
        let (model, candidates, matrix, rows, _) = workload(name);
        let prepared = Prepared::new(&model.to_string()).unwrap();
        let mut entry = serde_json::Map::new();
        for workers in [1, 2, 4] {
            let budget = json!({ "workers": workers }).to_string();
            entry.insert(
                format!("full_workers_{workers}"),
                timed(|| {
                    black_box(prepared.evaluate_json(&candidates, &budget).unwrap());
                }),
            );
            entry.insert(
                format!("matrix_workers_{workers}"),
                timed(|| {
                    black_box(prepared.score_matrix(rows, &matrix, &budget).unwrap());
                }),
            );
        }
        scaling.insert(name.into(), Value::Object(entry));
    }
    report.insert("worker_scaling".into(), Value::Object(scaling));
    let example: Value = serde_json::from_str(include_str!("../models/example.json")).unwrap();
    let prepared = Prepared::new(&example["model"].to_string()).unwrap();
    let kernel = prepared.model.kernel_name();
    report.insert("lane_kernel".into(), json!(kernel));
    report.insert(
        "lane_width".into(),
        json!(domain::lane_width(&prepared.model)),
    );
    println!("{}", serde_json::to_string_pretty(&report).unwrap());
}
