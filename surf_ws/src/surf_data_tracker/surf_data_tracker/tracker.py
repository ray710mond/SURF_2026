import csv
import json
import sqlite3
from datetime import datetime, timezone
from pathlib import Path
from uuid import uuid4

import rclpy
from ament_index_python.packages import get_package_prefix
from rclpy.node import Node
from surf_multirobot_msgs.msg import DeliveryMetrics, LinkMetrics, PipelineMetrics


class DataTracker(Node):
    """Writes raw observations to SQLite and a report-ready aggregate to JSON."""

    def __init__(self):
        super().__init__('data_tracker')
        output_override = self.declare_parameter('output_directory', '').value.strip()
        output = Path(output_override).expanduser() if output_override else self._default_output()
        output.mkdir(parents=True, exist_ok=True)
        run_label = self.declare_parameter('run_label', '').value.strip()
        safe_label = ''.join(
            character if character.isalnum() or character in '-_' else '_'
            for character in run_label).strip('_')
        self.started_at = datetime.now(timezone.utc)
        timestamp = self.started_at.strftime('%Y%m%dT%H%M%S_%fZ')
        self.run_id = f'{timestamp}_{uuid4().hex[:8]}'
        if safe_label:
            self.run_id = f'{safe_label}_{self.run_id}'
        self.run_directory = output / self.run_id
        self.run_directory.mkdir(parents=False, exist_ok=False)
        self.db_path = self.run_directory / 'telemetry.sqlite3'
        self.summary_path = self.run_directory / 'summary.json'
        self.report_csv_path = self.run_directory / 'report.csv'
        self.report_markdown_path = self.run_directory / 'report.md'
        self.db = sqlite3.connect(self.db_path)
        self.db.execute('PRAGMA journal_mode=WAL')
        self.db.execute('PRAGMA synchronous=NORMAL')
        self._create_schema()
        self.db.execute(
            'INSERT INTO run_metadata VALUES (?,?,?)',
            (self.run_id, self.started_at.isoformat(), run_label))
        self.db.commit()
        pipeline_topic = self.declare_parameter(
            'pipeline_topic', '/drone/comm/pipeline_metrics').value
        delivery_topic = self.declare_parameter(
            'delivery_topic', '/humanoid/comm/delivery_metrics').value
        link_topics = self.declare_parameter('link_topics', [
            '/drone/comm/halow_metrics', '/drone/comm/wifi_metrics']).value
        self.create_subscription(PipelineMetrics, pipeline_topic, self._pipeline, 50)
        self.create_subscription(DeliveryMetrics, delivery_topic, self._delivery, 50)
        self.link_subscriptions = [
            self.create_subscription(LinkMetrics, topic, self._link, 20)
            for topic in link_topics
        ]
        self.create_timer(2.0, self._flush)
        self.get_logger().info(f'Recording experiment telemetry in {self.db_path}')

    @staticmethod
    def _default_output():
        """Resolve <workspace>/src/surf_data_tracker/data_collection independent of cwd."""
        package_prefix = Path(get_package_prefix('surf_data_tracker')).resolve()
        workspace = package_prefix.parent.parent
        source_package = workspace / 'src' / 'surf_data_tracker'
        if source_package.is_dir():
            return source_package / 'data_collection'
        # Installed deployments may not retain a source tree. Keep data beside the
        # package-specific prefix rather than making the location depend on cwd.
        return package_prefix / 'share' / 'surf_data_tracker' / 'data_collection'

    def _create_schema(self):
        self.db.executescript('''
        CREATE TABLE IF NOT EXISTS pipeline (
          time_ns INTEGER, source_id TEXT, operating_mode INTEGER, traffic_class INTEGER,
          input_rate_hz REAL, raw_points INTEGER, valid_points INTEGER,
          unique_voxels INTEGER, static_prior_voxels INTEGER,
          temporal_suppressed_voxels INTEGER, selected_voxels INTEGER,
          raw_bytes INTEGER, encoded_bytes INTEGER, payload_bytes INTEGER, wire_bytes INTEGER,
          codec TEXT, compression_ms REAL, transform_lookup_ms REAL,
          point_preprocessing_ms REAL, occupancy_selection_ms REAL, clearing_ms REAL,
          processing_ms REAL, stale_drops INTEGER);
        CREATE TABLE IF NOT EXISTS delivery (
          time_ns INTEGER, source_id TEXT, map_epoch INTEGER, version INTEGER,
          traffic_class INTEGER, wire_bytes INTEGER, voxel_count INTEGER,
          transport_ms REAL, decode_ms REAL, end_to_end_ms REAL,
          accepted INTEGER, rejection_reason TEXT);
        CREATE TABLE IF NOT EXISTS link (
          time_ns INTEGER, link_name TEXT, bandwidth_mbps REAL, throughput_mbps REAL,
          latency_ms REAL, queue_depth INTEGER, received INTEGER, delivered INTEGER,
          dropped_loss INTEGER, dropped_stale INTEGER, transmitted_bytes INTEGER);
        CREATE TABLE IF NOT EXISTS run_metadata (
          run_id TEXT PRIMARY KEY, started_at_utc TEXT, run_label TEXT);
        ''')

    @staticmethod
    def _stamp_ns(header):
        return header.stamp.sec * 1_000_000_000 + header.stamp.nanosec

    def _pipeline(self, m):
        self.db.execute('INSERT INTO pipeline VALUES (?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?)', (
            self._stamp_ns(m.header), m.source_id, m.operating_mode, m.traffic_class,
            m.input_rate_hz, m.raw_points, m.valid_points, m.unique_voxels,
            m.static_prior_voxels, m.temporal_suppressed_voxels, m.selected_voxels,
            m.raw_serialized_bytes, m.uncompressed_bytes, m.payload_bytes, m.wire_bytes,
            m.codec, m.compression_latency_ms, m.transform_lookup_ms,
            m.point_preprocessing_ms, m.occupancy_selection_ms, m.clearing_ms,
            m.processing_latency_ms, m.stale_input_drops))

    def _delivery(self, m):
        self.db.execute('INSERT INTO delivery VALUES (?,?,?,?,?,?,?,?,?,?,?,?)', (
            self._stamp_ns(m.header), m.source_id, m.map_epoch, m.version, m.traffic_class,
            m.wire_bytes, m.voxel_count, m.transport_latency_ms, m.decode_latency_ms,
            m.end_to_end_latency_ms, int(m.accepted), m.rejection_reason))

    def _link(self, m):
        self.db.execute('INSERT INTO link VALUES (?,?,?,?,?,?,?,?,?,?,?)', (
            self._stamp_ns(m.header), m.link_name, m.configured_bandwidth_mbps,
            m.measured_throughput_mbps, m.mean_delivery_latency_ms, m.queue_depth,
            m.received_packets, m.delivered_packets, m.dropped_loss, m.dropped_stale,
            m.transmitted_bytes))

    def _flush(self):
        self.db.commit()
        row = self.db.execute('''SELECT COUNT(*), SUM(raw_points), SUM(valid_points),
          SUM(unique_voxels), SUM(static_prior_voxels), SUM(temporal_suppressed_voxels),
          SUM(selected_voxels), SUM(raw_bytes), SUM(encoded_bytes), SUM(payload_bytes),
          SUM(wire_bytes), AVG(processing_ms), AVG(compression_ms) FROM pipeline
          WHERE traffic_class=1''').fetchone()
        n, raw_pts, valid, unique, static, temporal, selected, raw_b, encoded, payload, wire, proc, comp = [
            value or 0 for value in row]
        pct = lambda removed, before: 100.0 * removed / before if before else 0.0
        summary = {
            'run': {
                'id': self.run_id,
                'started_at_utc': self.started_at.isoformat(),
                'directory': str(self.run_directory),
            },
            'samples': n,
            'totals': {'raw_bytes': raw_b, 'encoded_bytes': encoded,
                       'payload_bytes': payload, 'wire_bytes': wire},
            'reductions_percent': {
                'invalid_or_range_filter_points': pct(raw_pts - valid, raw_pts),
                'voxelization_elements': pct(valid - unique, valid),
                'static_prior_voxels': pct(static, unique),
                'temporal_delta_voxels': pct(temporal, max(0, unique - static)),
                'compression_bytes': pct(encoded - payload, encoded),
                'end_to_end_wire_vs_raw_bytes': pct(raw_b - wire, raw_b),
            },
            'mean_latency_ms': {'sender_processing': proc, 'compression': comp},
            'notes': [
                'Point/voxel reductions are element-count attribution, not byte attribution.',
                'Compression and end-to-end reductions use measured serialized byte counts.',
                'Compare experimental runs with one preprocessing feature changed for causal claims.'
            ]
        }
        self.summary_path.write_text(json.dumps(summary, indent=2) + '\n')
        self._write_table_report(raw_b, encoded, payload, wire)

    @staticmethod
    def _percent(numerator, denominator):
        return 100.0 * numerator / denominator if denominator else None

    def _write_table_report(self, total_raw, total_encoded, total_payload, total_wire):
        """Produce readable tables using measured values only; unknowns remain N/A."""
        columns = [
            'category', 'process_or_topic', 'unit', 'samples', 'total', 'average',
            'minimum', 'maximum', 'time_type', 'avg_time_ms', 'min_time_ms', 'max_time_ms',
            'avg_rate_mbps', 'improvement_pct', 'contribution_to_total_improvement_pct',
            'change_if_feature_removed_pct', 'measurement', 'formula_or_note']
        rows = []

        def add(category, name, unit='', samples=None, total=None, average=None,
                minimum=None, maximum=None, avg_latency=None, min_latency=None,
                max_latency=None, avg_rate=None, improvement=None, contribution=None,
                removed=None, measurement='measured', note='', time_type='N/A'):
            rows.append(dict(zip(columns, [
                category, name, unit, samples, total, average, minimum, maximum,
                time_type, avg_latency, min_latency, max_latency, avg_rate, improvement,
                contribution, removed, measurement, note])))

        traffic_names = {1: 'HaLow realtime traffic', 2: 'Wi-Fi sync traffic'}
        for traffic_class, name in traffic_names.items():
            values = self.db.execute('''SELECT COUNT(*), SUM(wire_bytes), AVG(wire_bytes),
              MIN(wire_bytes), MAX(wire_bytes), AVG(processing_ms), MIN(processing_ms),
              MAX(processing_ms) FROM pipeline WHERE traffic_class=?''',
              (traffic_class,)).fetchone()
            add('traffic', name, 'wire bytes', *values[:5],
                avg_latency=values[5], min_latency=values[6], max_latency=values[7],
                time_type='sender processing time',
                note='Exact ROS serialized message size; latency is sender processing time.')

        values = self.db.execute('''SELECT COUNT(*), SUM(wire_bytes), AVG(wire_bytes),
          MIN(wire_bytes), MAX(wire_bytes), AVG(processing_ms), MIN(processing_ms),
          MAX(processing_ms) FROM pipeline''').fetchone()
        add('total', 'All transmitted traffic', 'wire bytes', *values[:5],
            avg_latency=values[5], min_latency=values[6], max_latency=values[7],
            time_type='sender processing time',
            note='Total of all realtime and synchronization messages at sender.')

        realtime = self.db.execute('''SELECT COUNT(*), SUM(raw_bytes), AVG(raw_bytes),
          MIN(raw_bytes), MAX(raw_bytes), SUM(raw_points), SUM(valid_points),
          SUM(unique_voxels), SUM(static_prior_voxels),
          SUM(temporal_suppressed_voxels), SUM(selected_voxels),
          AVG(compression_ms), MIN(compression_ms), MAX(compression_ms)
          FROM pipeline WHERE traffic_class=1''').fetchone()
        count, raw_sum, raw_avg, raw_min, raw_max, raw_points, valid_points, unique, static, temporal, selected, comp_avg, comp_min, comp_max = realtime
        if not count:
            raw_sum = raw_avg = raw_min = raw_max = None
            raw_points = valid_points = unique = static = temporal = selected = None
            comp_avg = comp_min = comp_max = None
        add('pipeline', 'Raw point-cloud input', 'serialized bytes', count, raw_sum,
            raw_avg, raw_min, raw_max, note='Exact CDR-serialized PointCloud2 size.')
        add('preprocessing', 'Range/validity filtering', 'points', count,
            valid_points, valid_points / count if count else None, None, None,
            improvement=self._percent(raw_points - valid_points, raw_points) if raw_points is not None else None,
            measurement='measured count',
            note='improvement = 100 × (raw_points - valid_points) / raw_points. No byte claim.')
        add('preprocessing', 'Voxelization', 'voxels', count, unique,
            unique / count if count else None,
            improvement=self._percent(valid_points - unique, valid_points) if valid_points is not None else None,
            measurement='measured count',
            note='improvement = 100 × (valid_points - unique_voxels) / valid_points. Units change from points to voxels; no byte claim.')
        voxel_selection_savings = static + temporal if static is not None else None
        add('preprocessing', 'Static-prior filtering', 'voxels removed', count, static,
            static / count if count else None,
            improvement=self._percent(static, unique),
            contribution=self._percent(static, voxel_selection_savings) if static is not None else None,
            removed=None, measurement='measured count',
            note='contribution = static_removed / (static_removed + temporal_suppressed). Removal counterfactual is N/A without a paired ablation run.')
        after_static = max(0, unique - static) if unique is not None else None
        add('preprocessing', 'Temporal delta suppression', 'voxels suppressed', count,
            temporal, temporal / count if count else None,
            improvement=self._percent(temporal, after_static),
            contribution=self._percent(temporal, voxel_selection_savings) if temporal is not None else None,
            removed=None, measurement='measured count',
            note='contribution = temporal_suppressed / (static_removed + temporal_suppressed). Removal counterfactual is N/A without a paired ablation run.')
        add('preprocessing', 'Selected voxel output', 'voxels', count, selected,
            selected / count if count else None, measurement='measured count')

        stage_names = [
            ('Pose/transform resolution', 'transform_lookup_ms'),
            ('Point filtering, transform, and voxelization', 'point_preprocessing_ms'),
            ('Occupancy classification and transmission selection', 'occupancy_selection_ms'),
            ('Ray clearing and stale-cell cleanup', 'clearing_ms'),
            ('Complete sender pipeline', 'processing_ms'),
        ]
        mean_total_processing = self.db.execute(
            'SELECT AVG(processing_ms) FROM pipeline WHERE traffic_class=1').fetchone()[0]
        for stage_name, column in stage_names:
            timing = self.db.execute(f'''SELECT COUNT(*), AVG({column}), MIN({column}),
              MAX({column}) FROM pipeline WHERE traffic_class=1''').fetchone()
            add('compute overhead', stage_name, 'milliseconds', timing[0], None,
                timing[1], timing[2], timing[3], avg_latency=timing[1],
                min_latency=timing[2], max_latency=timing[3],
                contribution=self._percent(timing[1], mean_total_processing),
                removed=None, time_type='measured compute time',
                note='Contribution = mean stage compute time / mean complete sender time. Time saved if removed is N/A; removing a stage can change downstream work and requires an ablation run.')

        zstd = self.db.execute('''SELECT COUNT(*), SUM(encoded_bytes), SUM(payload_bytes),
          AVG(encoded_bytes), MIN(encoded_bytes), MAX(encoded_bytes), AVG(payload_bytes),
          MIN(payload_bytes), MAX(payload_bytes), AVG(compression_ms),
          MIN(compression_ms), MAX(compression_ms)
          FROM pipeline WHERE traffic_class=1 AND codec='zstd-svd1' ''').fetchone()
        zstd_count = zstd[0]
        zstd_encoded = zstd[1] if zstd_count else None
        zstd_payload = zstd[2] if zstd_count else None
        compression_savings = zstd_encoded - zstd_payload if zstd_count else None
        total_transfer_savings = max(0, total_raw - total_wire)
        encoded_stats = self.db.execute('''SELECT AVG(encoded_bytes), MIN(encoded_bytes),
          MAX(encoded_bytes), AVG(payload_bytes), MIN(payload_bytes), MAX(payload_bytes)
          FROM pipeline WHERE traffic_class=1''').fetchone()
        add('compression', 'SVD1 encoded input', 'payload bytes', count,
            total_encoded if count else None,
            encoded_stats[0], encoded_stats[1], encoded_stats[2],
            note='Exact uncompressed SVD1 payload size before optional zstd.')
        add('compression', 'zstd compression', 'payload bytes', zstd_count, zstd_payload,
            zstd[6], zstd[7], zstd[8],
            avg_latency=zstd[9], min_latency=zstd[10], max_latency=zstd[11],
            time_type='measured compute time',
            improvement=self._percent(compression_savings, zstd_encoded) if zstd_count else None,
            contribution=self._percent(compression_savings, total_transfer_savings) if zstd_count else None,
            removed=self._percent(compression_savings, zstd_payload) if zstd_count else None,
            note='improvement = (encoded-payload)/encoded; contribution = compression_savings/(raw-wire); removed change = (encoded-payload)/payload. Payload layer only; no wire-overhead estimate.')

        link_names = [row[0] for row in self.db.execute(
            'SELECT DISTINCT link_name FROM link ORDER BY link_name')]
        for link_name in link_names:
            link = self.db.execute('''SELECT COUNT(*), AVG(throughput_mbps),
              MIN(throughput_mbps), MAX(throughput_mbps), AVG(latency_ms),
              MIN(latency_ms), MAX(latency_ms), MAX(transmitted_bytes)
              FROM link WHERE link_name=?''', (link_name,)).fetchone()
            add('link', link_name, 'Mbps', link[0], None, link[1], link[2], link[3],
                avg_latency=link[4], min_latency=link[5], max_latency=link[6],
                avg_rate=link[1], time_type='network delivery latency',
                note=f'Throughput is measured per link window; final measured cumulative transmitted bytes = {link[7]}.')
        link_total = self.db.execute('''SELECT COUNT(*), AVG(throughput_mbps),
          MIN(throughput_mbps), MAX(throughput_mbps), AVG(latency_ms),
          MIN(latency_ms), MAX(latency_ms) FROM link''').fetchone()
        add('total', 'All link metric windows', 'Mbps', link_total[0], None,
            link_total[1], link_total[2], link_total[3], avg_latency=link_total[4],
            min_latency=link_total[5], max_latency=link_total[6], avg_rate=link_total[1],
            time_type='network delivery latency',
            note='Unweighted aggregate across configured links and reporting windows.')

        delivery = self.db.execute('''SELECT COUNT(*), SUM(wire_bytes), AVG(wire_bytes),
          MIN(wire_bytes), MAX(wire_bytes), AVG(end_to_end_ms), MIN(end_to_end_ms),
          MAX(end_to_end_ms) FROM delivery WHERE accepted=1''').fetchone()
        add('delivery', 'All accepted receiver traffic', 'wire bytes', *delivery[:5],
            avg_latency=delivery[5], min_latency=delivery[6], max_latency=delivery[7],
            time_type='end-to-end delivery and decode time',
            note='Exact receiver observations; end-to-end latency uses message timestamp plus decode time.')

        def display(value):
            if value is None:
                return 'N/A'
            if isinstance(value, float):
                return f'{value:.6g}'
            return str(value)

        with self.report_csv_path.open('w', newline='') as stream:
            writer = csv.DictWriter(stream, fieldnames=columns)
            writer.writeheader()
            writer.writerows(rows)
        markdown = [
            f'# Telemetry report: {self.run_id}', '',
            'All values are measured unless the measurement column says otherwise. `N/A` means the current instrumentation cannot support that claim.', '',
            '| ' + ' | '.join(columns) + ' |',
            '| ' + ' | '.join(['---'] * len(columns)) + ' |']
        for row in rows:
            markdown.append('| ' + ' | '.join(
                display(row[column]).replace('|', '\\|') for column in columns) + ' |')
        markdown.extend(['', 'Percentages are scoped to the units and formulas shown in each row. They must not be compared across point, voxel, payload-byte, and wire-byte units as if those units were interchangeable.', ''])
        self.report_markdown_path.write_text('\n'.join(markdown))

    def destroy_node(self):
        self._flush()
        self.db.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = DataTracker()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
