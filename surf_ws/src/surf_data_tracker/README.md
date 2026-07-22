# SURF data tracker

`data_tracker` centralizes sender pipeline, receiver delivery, and emulated-link metrics.
Every tracker start creates a separate UTC-timestamped, collision-safe run directory
under the package source, independent of the shell's working directory:

```text
surf_ws/src/surf_data_tracker/data_collection/
└── 20260722T220531_123456Z_a1b2c3d4/
├── telemetry.sqlite3
├── summary.json
├── report.csv
└── report.md
```

For an installed deployment without a source workspace, it falls back to the package
installation prefix. The `output_directory` ROS parameter can explicitly override the
location when needed.

The database contains raw, queryable observations and the JSON contains report-ready
totals and reductions. `report.csv` and `report.md` provide a row-per-process/topic
table with message sizes, latency, throughput, and defensible percentages. Files are
updated every two seconds. They include the run identifier and
UTC start time. An optional human-readable label is prepended to the identifier:

```bash
ros2 run surf_data_tracker data_tracker --ros-args -p run_label:=no_static_filter
```

The database deliberately preserves raw measurements. Static filtering and temporal
delta percentages are based on voxel counts; compression and total transfer reduction
are based on exact serialized byte counts. For causal claims, run controlled ablations
with only one preprocessing feature changed and compare the resulting databases.
Unsupported counterfactuals are written as `N/A`; the report never substitutes an
estimated byte saving for a measured count reduction.

The report calls node-local execution cost **compute time** or **processing overhead**,
and reserves **latency** for message delivery. Compute rows cover transform lookup,
point filtering/voxelization, occupancy selection, clearing, compression, and the full
sender pipeline. Pose/transform resolution covers either direct odometry composition or
TF lookup, according to the sender configuration. A measured stage duration is not presented as guaranteed time saved:
that counterfactual remains `N/A` until a paired feature-disabled ablation is run.

Useful queries:

```sql
SELECT link_name, avg(throughput_mbps), avg(latency_ms),
       max(dropped_loss), max(dropped_stale) FROM link GROUP BY link_name;

SELECT operating_mode, sum(raw_bytes), sum(wire_bytes), avg(processing_ms)
FROM pipeline WHERE traffic_class = 1 GROUP BY operating_mode;
```
