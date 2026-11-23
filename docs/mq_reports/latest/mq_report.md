# JAKA Dual Arm Measurement Qualification

Result: **PRE_RUN_READY**

Raw experiment evidence was READ ONLY. No robot motion commanded.

## Gate checks

| Gate | Result |
|---|---|
| run_ids_equal | PASS |
| run_closed_completed | PASS |
| v3_packet_schema | PASS |
| provenance_raw_sha256 | PASS |
| provenance_manifest_sha256 | PASS |
| provenance_urdf_sha256 | PASS |
| analyzer_version | PASS |
| bounded_interpolation_policy | PASS |
| positive_duration | PASS |
| left_packet_quantity | PASS |
| left_joint_field_coverage | PASS |
| left_timestamps_monotonic | PASS |
| left_sequence_monotonic | PASS |
| left_receive_gap_bounded | PASS |
| right_packet_quantity | PASS |
| right_joint_field_coverage | PASS |
| right_timestamps_monotonic | PASS |
| right_sequence_monotonic | PASS |
| right_receive_gap_bounded | PASS |
| phase_match_status | PASS |
| phase_coverage | PASS |
| same_packet_tracking_available | PASS |
| left_tracking_usable | PASS |
| right_tracking_usable | PASS |
| offline_regression_suite | PASS |
| backend_snapshot_errors | PASS |
| direct_port10000_selected | PASS |
| left_live_packet_cadence | PASS |
| left_live_freshness | PASS |
| right_live_packet_cadence | PASS |
| right_live_freshness | PASS |
| left_ros_feedback_fresh | PASS |
| right_ros_feedback_fresh | PASS |
| recorder_idle | PASS |

## Traceable details

```json
{
  "schema": "jaka-dual-arm-mq/v1",
  "overall": "PRE_RUN_READY",
  "historical_reference_run": "exp-20260911T082849318Z-bb3bd4b5",
  "historical_reference_is_formal_exp_result": false,
  "checks": [
    {
      "name": "run_ids_equal",
      "passed": true,
      "details": "exp-20260911T082849318Z-bb3bd4b5"
    },
    {
      "name": "run_closed_completed",
      "passed": true,
      "details": {
        "state": "CLOSED",
        "execution": "COMPLETED"
      }
    },
    {
      "name": "v3_packet_schema",
      "passed": true,
      "details": "port10000-packets/v2"
    },
    {
      "name": "provenance_raw_sha256",
      "passed": true,
      "details": {
        "recorded": "6af415e6b89bd25345ebec2f64db6ca6521fac464a624810f9852e0b13c97e4f",
        "current": "6af415e6b89bd25345ebec2f64db6ca6521fac464a624810f9852e0b13c97e4f"
      }
    },
    {
      "name": "provenance_manifest_sha256",
      "passed": true,
      "details": {
        "recorded": "d2914611fc67b4b8bd2619056b3770f9d0cf838765cab0054aa5a5c5e773c788",
        "current": "d2914611fc67b4b8bd2619056b3770f9d0cf838765cab0054aa5a5c5e773c788"
      }
    },
    {
      "name": "provenance_urdf_sha256",
      "passed": true,
      "details": {
        "recorded": "6883f9e096f7858dc5c53f163583f194de8953d6e2f0f425aff9fd786189c4a3",
        "current": "6883f9e096f7858dc5c53f163583f194de8953d6e2f0f425aff9fd786189c4a3"
      }
    },
    {
      "name": "analyzer_version",
      "passed": true,
      "details": "exp2-same-frame-phase/v3"
    },
    {
      "name": "bounded_interpolation_policy",
      "passed": true,
      "details": 250.0
    },
    {
      "name": "positive_duration",
      "passed": true,
      "details": 28.032
    },
    {
      "name": "left_packet_quantity",
      "passed": true,
      "details": {
        "packets": 286,
        "duration_s": 28.032,
        "min_nominal_hz": 8
      }
    },
    {
      "name": "left_joint_field_coverage",
      "passed": true,
      "details": {
        "valid": 286,
        "packets": 286
      }
    },
    {
      "name": "left_timestamps_monotonic",
      "passed": true,
      "details": {
        "timestamps": 286,
        "max_gap_ms": 166.0
      }
    },
    {
      "name": "left_sequence_monotonic",
      "passed": true,
      "details": {
        "sequences": 286
      }
    },
    {
      "name": "left_receive_gap_bounded",
      "passed": true,
      "details": {
        "max_gap_ms": 166.0,
        "threshold_ms": 250
      }
    },
    {
      "name": "right_packet_quantity",
      "passed": true,
      "details": {
        "packets": 288,
        "duration_s": 28.032,
        "min_nominal_hz": 8
      }
    },
    {
      "name": "right_joint_field_coverage",
      "passed": true,
      "details": {
        "valid": 288,
        "packets": 288
      }
    },
    {
      "name": "right_timestamps_monotonic",
      "passed": true,
      "details": {
        "timestamps": 288,
        "max_gap_ms": 147.0
      }
    },
    {
      "name": "right_sequence_monotonic",
      "passed": true,
      "details": {
        "sequences": 288
      }
    },
    {
      "name": "right_receive_gap_bounded",
      "passed": true,
      "details": {
        "max_gap_ms": 147.0,
        "threshold_ms": 250
      }
    },
    {
      "name": "phase_match_status",
      "passed": true,
      "details": "PARTIAL"
    },
    {
      "name": "phase_coverage",
      "passed": true,
      "details": {
        "fraction": 0.9703703703703703,
        "usable": 524,
        "candidate": 540,
        "minimum_fraction": 0.95
      }
    },
    {
      "name": "same_packet_tracking_available",
      "passed": true,
      "details": "AVAILABLE"
    },
    {
      "name": "left_tracking_usable",
      "passed": true,
      "details": {
        "joint_sample_count": 1716
      }
    },
    {
      "name": "right_tracking_usable",
      "passed": true,
      "details": {
        "joint_sample_count": 1728
      }
    },
    {
      "name": "offline_regression_suite",
      "passed": true,
      "details": {
        "exit_code": 0,
        "output": "....................................................................     [100%]\n68 passed in 0.83s\n"
      }
    },
    {
      "name": "backend_snapshot_errors",
      "passed": true,
      "details": {
        "errors": [],
        "count": 0
      }
    },
    {
      "name": "direct_port10000_selected",
      "passed": true,
      "details": {
        "snapshots": 147,
        "distinct_sources": [
          "jaka_port10000_actual_feedback"
        ]
      }
    },
    {
      "name": "left_live_packet_cadence",
      "passed": true,
      "details": {
        "distinct_receive_timestamps": 123,
        "max_observed_gap_ms": 223,
        "mean_observed_gap_ms": 122.30327868852459,
        "expected_min_samples_per_second": 7
      }
    },
    {
      "name": "left_live_freshness",
      "passed": true,
      "details": {
        "fresh_fraction": 1.0,
        "minimum": 0.98
      }
    },
    {
      "name": "right_live_packet_cadence",
      "passed": true,
      "details": {
        "distinct_receive_timestamps": 128,
        "max_observed_gap_ms": 221,
        "mean_observed_gap_ms": 117.51968503937007,
        "expected_min_samples_per_second": 7
      }
    },
    {
      "name": "right_live_freshness",
      "passed": true,
      "details": {
        "fresh_fraction": 1.0,
        "minimum": 0.98
      }
    },
    {
      "name": "left_ros_feedback_fresh",
      "passed": true,
      "details": {
        "status": "LIVE",
        "joint_age_ms": 56
      }
    },
    {
      "name": "right_ros_feedback_fresh",
      "passed": true,
      "details": {
        "status": "LIVE",
        "joint_age_ms": 44
      }
    },
    {
      "name": "recorder_idle",
      "passed": true,
      "details": {
        "state": "IDLE",
        "run_id": null
      }
    }
  ],
  "limitations": [
    "Host receive timestamps are not controller sample timestamps.",
    "Phase-matching is NOT simultaneous physical-time synchronization.",
    "Encoder+FK is NOT external physical metrology.",
    "Passive API snapshots do NOT prove packet-complete logging.",
    "An EXP1–EXP4 run must pass its own V3 captured-evidence check after Execute.",
    "Robot physical safety, workpiece payload and E-stop checks remain operator responsibilities."
  ]
}
```

Interpretation: PRE_RUN_READY qualifies the offline pipeline and passive receiver, not physical robot safety, simultaneous controller time, or an upcoming run's captured data. Every new recorded run must pass its own saved-run gate before EXP1–EXP4 results are accepted.
