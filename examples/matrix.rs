//! Grid search over PD gains with the typed score-matrix API.
//!
//! `cargo run --release --locked --example matrix -- models/example.json`
use robotics_control_craft::Prepared;
use serde_json::Value;
use std::io;

fn main() -> Result<(), io::Error> {
    let file = std::env::args_os()
        .nth(1)
        .ok_or_else(|| io::Error::other("example.json path required"))?;
    let example: Value = serde_json::from_str(&std::fs::read_to_string(file)?)?;
    let prepared = Prepared::new(&example["model"].to_string()).map_err(io::Error::other)?;
    // Row-major gains: kp in 0..=20, kd in 0..=8 (step 0.5).
    let gains: Vec<[f64; 2]> = (0..=20)
        .flat_map(|p| (0..=16).map(move |d| [f64::from(p), f64::from(d) * 0.5]))
        .collect();
    let parameters: Vec<f64> = gains.iter().flatten().copied().collect();
    let rows = prepared
        .score_matrix(gains.len(), &parameters, r#"{"workers":2}"#)
        .map_err(io::Error::other)?;
    let complete = rows.iter().filter(|r| r.status == "complete").count();
    let (best, row) = rows
        .iter()
        .enumerate()
        .filter_map(|(i, r)| r.score.map(|s| (i, s)))
        .min_by(|a, b| a.1.total_cmp(&b.1))
        .map(|(i, _)| (gains[i], &rows[i]))
        .ok_or_else(|| io::Error::other("no complete row"))?;
    println!(
        "{} rows, {complete} complete; best kp={} kd={} score={} steps={}",
        rows.len(),
        best[0],
        best[1],
        row.score.unwrap(),
        row.steps
    );
    println!("model sha256 {}", prepared.model_sha256());
    Ok(())
}
