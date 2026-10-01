# Generated measurements

`all_runs.json` selects the runs used by the current report. `batch_summary.json`
summarizes that selection. Timestamp directories contain individual run plots;
older directories remain as development history and are not automatically
included in the current report.

To reproduce the final selection, run `analysis/summarize.py` on the completed
batch in `analysis/evidence/`, then run `analysis/build_report.py` from the
repository root. Full telemetry and selected unmodified camera images are
retained in the evidence directory. No official scoring metric is inferred.
