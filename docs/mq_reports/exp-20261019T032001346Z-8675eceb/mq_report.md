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

## Traceable details

```json
{
  "schema": "jaka-dual-arm-mq/v1",
  "overall": "PRE_RUN_READY",
  "historical_reference_run": "exp-20261019T032001346Z-8675eceb",
  "historical_reference_is_formal_exp_result": false,
  "checks": [
    {
      "name": "run_ids_equal",
      "passed": true,
      "details": "exp-20261019T032001346Z-8675eceb"
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
        "recorded": "d9212b3b099408c1fec36919590abe1ce7082511d2087849b06d18f8b4b25d59",
        "current": "d9212b3b099408c1fec36919590abe1ce7082511d2087849b06d18f8b4b25d59"
      }
    },
    {
      "name": "provenance_manifest_sha256",
      "passed": true,
      "details": {
        "recorded": "ee9662eaf9523587450aa4a997d965d0e6909fc64fec1a22c50a0353f7c26f69",
        "current": "ee9662eaf9523587450aa4a997d965d0e6909fc64fec1a22c50a0353f7c26f69"
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
      "details": 24.528000000000002
    },
    {
      "name": "left_packet_quantity",
      "passed": true,
      "details": {
        "packets": 226,
        "duration_s": 24.528000000000002,
        "min_nominal_hz": 8
      }
    },
    {
      "name": "left_joint_field_coverage",
      "passed": true,
      "details": {
        "valid": 226,
        "packets": 226
      }
    },
    {
      "name": "left_timestamps_monotonic",
      "passed": true,
      "details": {
        "timestamps": 226,
        "max_gap_ms": 215.0
      }
    },
    {
      "name": "left_sequence_monotonic",
      "passed": true,
      "details": {
        "sequences": 226
      }
    },
    {
      "name": "left_receive_gap_bounded",
      "passed": true,
      "details": {
        "max_gap_ms": 215.0,
        "threshold_ms": 250
      }
    },
    {
      "name": "right_packet_quantity",
      "passed": true,
      "details": {
        "packets": 231,
        "duration_s": 24.528000000000002,
        "min_nominal_hz": 8
      }
    },
    {
      "name": "right_joint_field_coverage",
      "passed": true,
      "details": {
        "valid": 231,
        "packets": 231
      }
    },
    {
      "name": "right_timestamps_monotonic",
      "passed": true,
      "details": {
        "timestamps": 231,
        "max_gap_ms": 211.0
      }
    },
    {
      "name": "right_sequence_monotonic",
      "passed": true,
      "details": {
        "sequences": 231
      }
    },
    {
      "name": "right_receive_gap_bounded",
      "passed": true,
      "details": {
        "max_gap_ms": 211.0,
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
        "fraction": 0.9796839729119639,
        "usable": 434,
        "candidate": 443,
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
        "joint_sample_count": 1356
      }
    },
    {
      "name": "right_tracking_usable",
      "passed": true,
      "details": {
        "joint_sample_count": 1386
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
