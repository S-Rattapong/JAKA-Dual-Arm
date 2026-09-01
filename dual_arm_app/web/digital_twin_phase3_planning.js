// Phase 3B pure Web contracts for OFFLINE object-waypoint planning and
// integrated joint/object preview. This module never executes robot motion.
import {
  SYNTHETIC_OBJECT_TRAJECTORY_DEFINITION,
} from "./digital_twin_object_trajectory_preview.js";

export const PHASE3_GLOBAL_PLAN_ENDPOINT = "/api/digital-twin/plan-object-global";
export const WEB_GLOBAL_DEFAULT_CANDIDATE_ATTEMPTS_PER_ARM = 3;
export const WEB_GLOBAL_MAX_CANDIDATE_ATTEMPTS_PER_ARM = 13;
export const PHASE3_OPTIMALITY_SCOPE = (
  "Exact optimum over generated layered candidate graph"
);
export const PHASE3_PLANNER_SOURCE = (
  "GLOBAL GRAPH SEARCH — GENERATED GRAPH OPTIMUM"
);
export const PHASE3_PLAN_WARNINGS = Object.freeze([
  "PLAN ONLY",
  "NO ROBOT EXECUTION",
  "NOT PHASE-4 VALIDATED",
  "NOT SAFE-TO-EXECUTE CLAIM",
]);

const fixture = SYNTHETIC_OBJECT_TRAJECTORY_DEFINITION;
export const PHASE3_DEMO_OBJECT_WAYPOINTS = Object.freeze([
  Object.freeze({ identifier: "W0", translation_m: fixture.startTranslationM }),
  Object.freeze({
    identifier: "W1",
    translation_m: Object.freeze(
      fixture.startTranslationM.map((value, axis) => (
        (value + fixture.endTranslationM[axis]) / 2
      )),
    ),
  }),
  Object.freeze({ identifier: "W2", translation_m: fixture.endTranslationM }),
]);
export const PHASE3_DEMO_FIXED_ORIENTATION_RPY_RAD = fixture.startRpyRad;

function finiteNumber(value, label) {
  if (typeof value !== "number" || !Number.isFinite(value)) {
    throw new TypeError(`${label} must be a finite number`);
  }
  return value;
}

export function normalizeWaypointProfilePercent(value, label = "waypoint profile") {
  const checked = value === null || value === undefined
    ? 100.0 : finiteNumber(value, label);
  if (checked <= 0 || checked > 100) {
    throw new RangeError(`${label} must be within (0, 100]`);
  }
  return checked;
}

function integerAtLeast(value, minimum, label) {
  if (!Number.isInteger(value) || value < minimum) {
    throw new RangeError(`${label} must be an integer >= ${minimum}`);
  }
  return value;
}

function integerWithin(value, minimum, maximum, label) {
  integerAtLeast(value, minimum, label);
  if (value > maximum) {
    throw new RangeError(`${label} must be an integer within [${minimum}, ${maximum}]`);
  }
  return value;
}

function vector(values, length, label) {
  if (!Array.isArray(values) || values.length !== length) {
    throw new TypeError(`${label} must contain exactly ${length} finite numbers`);
  }
  return values.map((value, index) => finiteNumber(value, `${label}[${index}]`));
}

function nonEmptyString(value, label) {
  if (typeof value !== "string" || !value.trim()) {
    throw new TypeError(`${label} must be a non-empty string`);
  }
  return value.trim();
}

export function normalizePlanningStartState(values, jointLimitMetadata = null) {
  if (!values || typeof values !== "object" || Array.isArray(values)) {
    throw new TypeError("Planning Start State must contain left and right arrays");
  }
  const normalized = {
    left: vector(values.left, 6, "Planning Start State left"),
    right: vector(values.right, 6, "Planning Start State right"),
  };
  if (jointLimitMetadata !== null) {
    const order = jointLimitMetadata && jointLimitMetadata.joint_order;
    const limits = jointLimitMetadata && jointLimitMetadata.position_limits;
    if (!Array.isArray(order) || order.length !== 12 || !limits) {
      throw new TypeError("canonical joint-limit metadata is invalid");
    }
    const combined = [...normalized.left, ...normalized.right];
    order.forEach((jointName, index) => {
      const limit = limits[jointName];
      if (!limit || !Number.isFinite(limit.min) || !Number.isFinite(limit.max)) {
        throw new TypeError(`canonical limits are missing for ${jointName}`);
      }
      if (combined[index] < limit.min || combined[index] > limit.max) {
        throw new RangeError(
          `${jointName} must be within [${limit.min}, ${limit.max}] rad`,
        );
      }
    });
  }
  return normalized;
}

function identifier(value, label = "waypoint identifier") {
  if (typeof value !== "string" || !value.trim()) {
    throw new TypeError(`${label} must be a non-empty string`);
  }
  const checked = value.trim();
  if (!/^[A-Za-z0-9][A-Za-z0-9._-]*$/u.test(checked)) {
    throw new TypeError(
      `${label} must start with an alphanumeric character and contain only `
      + "alphanumeric, '.', '_', or '-' characters",
    );
  }
  return checked;
}

export function normalizeObjectPlanningWaypoints(waypoints, minimumCount = 0) {
  if (!Array.isArray(waypoints)) {
    throw new TypeError("Object Waypoints must be an array");
  }
  if (waypoints.length < minimumCount) {
    throw new RangeError(`at least ${minimumCount} ordered Object Waypoints are required`);
  }
  const normalized = waypoints.map((waypoint, index) => {
    if (!waypoint || typeof waypoint !== "object" || Array.isArray(waypoint)) {
      throw new TypeError(`waypoints[${index}] must be an object`);
    }
    return {
      identifier: identifier(waypoint.identifier, `waypoints[${index}].identifier`),
      translation_m: vector(
        waypoint.translation_m,
        3,
        `waypoints[${index}].translation_m`,
      ),
      rpy_rad: waypoint.rpy_rad === null || waypoint.rpy_rad === undefined
        ? null
        : vector(waypoint.rpy_rad, 3, `waypoints[${index}].rpy_rad`),
      speed_percent: normalizeWaypointProfilePercent(
        waypoint.speed_percent,
        `waypoints[${index}].speed_percent`,
      ),
      acceleration_percent: normalizeWaypointProfilePercent(
        waypoint.acceleration_percent,
        `waypoints[${index}].acceleration_percent`,
      ),
    };
  });
  const names = normalized.map((waypoint) => waypoint.identifier.toLocaleLowerCase());
  if (new Set(names).size !== names.length) {
    throw new RangeError("Object Waypoint identifiers must be unique");
  }
  return normalized;
}

export function renumberObjectPlanningWaypoints(waypoints) {
  if (!Array.isArray(waypoints)) {
    throw new TypeError("Object Waypoints must be an array");
  }
  return normalizeObjectPlanningWaypoints(waypoints.map((waypoint, index) => ({
    ...waypoint,
    identifier: `W${index}`,
  })));
}

export function duplicateObjectPlanningWaypoint(waypoints, index) {
  const checked = renumberObjectPlanningWaypoints(waypoints);
  integerAtLeast(index, 0, "waypoint index");
  if (index >= checked.length) throw new RangeError("waypoint index is out of range");
  const source = checked[index];
  const duplicate = {
    identifier: `W${index + 1}`,
    translation_m: [...source.translation_m],
    rpy_rad: source.rpy_rad ? [...source.rpy_rad] : null,
    speed_percent: source.speed_percent,
    acceleration_percent: source.acceleration_percent,
  };
  checked.splice(index + 1, 0, duplicate);
  return renumberObjectPlanningWaypoints(checked);
}

export function reorderObjectPlanningWaypoint(waypoints, index, insertionIndex) {
  const checked = renumberObjectPlanningWaypoints(waypoints);
  integerAtLeast(index, 0, "waypoint index");
  integerAtLeast(insertionIndex, 0, "waypoint insertion index");
  if (index >= checked.length || insertionIndex > checked.length) {
    throw new RangeError("waypoint reorder is out of range");
  }
  if (insertionIndex === index || insertionIndex === index + 1) return checked;
  const [moved] = checked.splice(index, 1);
  const adjustedInsertionIndex = insertionIndex > index
    ? insertionIndex - 1 : insertionIndex;
  checked.splice(adjustedInsertionIndex, 0, moved);
  return renumberObjectPlanningWaypoints(checked);
}

export function addObjectPlanningWaypoint(waypoints, waypoint) {
  return normalizeObjectPlanningWaypoints([...waypoints, waypoint]);
}

export function deleteObjectPlanningWaypoint(waypoints, index) {
  const checked = normalizeObjectPlanningWaypoints(waypoints);
  integerAtLeast(index, 0, "waypoint index");
  if (index >= checked.length) throw new RangeError("waypoint index is out of range");
  return checked.filter((_waypoint, waypointIndex) => waypointIndex !== index);
}

export function moveObjectPlanningWaypoint(waypoints, index, offset) {
  const checked = normalizeObjectPlanningWaypoints(waypoints);
  integerAtLeast(index, 0, "waypoint index");
  if (offset !== -1 && offset !== 1) throw new RangeError("offset must be -1 or 1");
  const target = index + offset;
  if (index >= checked.length || target < 0 || target >= checked.length) {
    throw new RangeError("waypoint move is out of range");
  }
  [checked[index], checked[target]] = [checked[target], checked[index]];
  return checked;
}

export function normalizeApproachWaypoints(waypoints) {
  if (waypoints === null || waypoints === undefined) return [];
  if (!Array.isArray(waypoints)) throw new TypeError("approachWaypoints must be an array");
  return waypoints.map((waypoint, index) => {
    if (!waypoint || typeof waypoint !== "object" || Array.isArray(waypoint)) {
      throw new TypeError(`approachWaypoints[${index}] must be an object`);
    }
    const pose = (side) => {
      const value = waypoint[side];
      if (!value || typeof value !== "object" || Array.isArray(value)) {
        throw new TypeError(`approachWaypoints[${index}].${side} must be an object`);
      }
      return {
        translation_m: vector(value.translation_m, 3, `approachWaypoints[${index}].${side}.translation_m`),
        rpy_rad: vector(value.rpy_rad, 3, `approachWaypoints[${index}].${side}.rpy_rad`),
      };
    };
    return {
      identifier: identifier(waypoint.identifier, `approachWaypoints[${index}].identifier`),
      left: pose("left"),
      right: pose("right"),
    };
  });
}

export function buildObjectGlobalPlanRequest({
  name,
  waypoints,
  approachWaypoints = [],
  fixedOrientationRpyRad,
  segmentDurationS,
  samplesPerSegment,
  candidateAttemptsPerArm = WEB_GLOBAL_DEFAULT_CANDIDATE_ATTEMPTS_PER_ARM,
  initialJointStateRad = null,
  planningStartStateSource = null,
  expectedGraspContentRevision,
  expectedLockGeneration,
  expectedLockRevision,
  expectedCalibrationRevision,
  expectedModelCalibrationRevision,
}) {
  const fallbackOrientation = vector(
    fixedOrientationRpyRad, 3, "fixedOrientationRpyRad",
  );
  const request = {
    name: identifier(name, "trajectory name"),
    waypoints: normalizeObjectPlanningWaypoints(waypoints, 2).map((waypoint) => ({
      ...waypoint,
      rpy_rad: waypoint.rpy_rad === null ? [...fallbackOrientation] : [...waypoint.rpy_rad],
    })),
    approach_waypoints: normalizeApproachWaypoints(approachWaypoints),
    fixed_orientation_rpy_rad: fallbackOrientation,
    segment_duration_s: (() => {
      const duration = finiteNumber(segmentDurationS, "segmentDurationS");
      if (duration <= 0) throw new RangeError("segmentDurationS must be greater than zero");
      return duration;
    })(),
    samples_per_segment: integerAtLeast(
      samplesPerSegment,
      2,
      "samplesPerSegment",
    ),
    candidate_attempts_per_arm: integerWithin(
      candidateAttemptsPerArm,
      1,
      WEB_GLOBAL_MAX_CANDIDATE_ATTEMPTS_PER_ARM,
      "candidateAttemptsPerArm",
    ),
    expected_grasp_content_revision: nonEmptyString(
      expectedGraspContentRevision, "expectedGraspContentRevision",
    ),
    expected_lock_generation: integerAtLeast(
      expectedLockGeneration, 1, "expectedLockGeneration",
    ),
    expected_lock_revision: nonEmptyString(
      expectedLockRevision, "expectedLockRevision",
    ),
    expected_calibration_revision: nonEmptyString(
      expectedCalibrationRevision, "expectedCalibrationRevision",
    ),
    expected_model_calibration_revision: nonEmptyString(
      expectedModelCalibrationRevision, "expectedModelCalibrationRevision",
    ),
  };
  if (initialJointStateRad !== null) {
    request.initial_joint_state_rad = normalizePlanningStartState(initialJointStateRad);
    if (typeof planningStartStateSource !== "string" || !planningStartStateSource.trim()) {
      throw new TypeError(
        "planningStartStateSource must accompany an explicit initialJointStateRad",
      );
    }
    request.planning_start_state_source = planningStartStateSource.trim();
  } else if (planningStartStateSource !== null) {
    throw new TypeError(
      "planningStartStateSource cannot be supplied without initialJointStateRad",
    );
  }
  return request;
}

function normalizePlanAuthority(response) {
  const grasp = response.grasp;
  const calibration = response.calibration;
  if (!grasp || grasp.status !== "GRASP_LOCKED") {
    throw new TypeError("successful plan must embed GRASP_LOCKED metadata");
  }
  if (grasp.source !== "LOCKED OPERATOR GRASP") {
    throw new TypeError("successful plan grasp source must be LOCKED OPERATOR GRASP");
  }
  if (!grasp.left || !grasp.right || !grasp.expected_left_T_right) {
    throw new TypeError("successful plan must embed both locked transforms and relative pose");
  }
  const lockGeneration = integerAtLeast(
    grasp.lock_generation, 1, "grasp.lock_generation",
  );
  if (!calibration || calibration.revision_status !== "MATCH") {
    throw new TypeError("successful plan model/calibration revision must MATCH");
  }
  const revision = nonEmptyString(calibration.revision, "calibration.revision");
  const modelRevision = nonEmptyString(
    calibration.model_revision, "calibration.model_revision",
  );
  if (revision !== modelRevision) {
    throw new RangeError("successful plan model revision must equal calibration revision");
  }
  return {
    grasp: JSON.parse(JSON.stringify({
      ...grasp,
      content_revision: nonEmptyString(
        grasp.content_revision, "grasp.content_revision",
      ),
      lock_generation: lockGeneration,
      lock_revision: nonEmptyString(grasp.lock_revision, "grasp.lock_revision"),
    })),
    calibration: JSON.parse(JSON.stringify({
      ...calibration,
      revision,
      model_revision: modelRevision,
    })),
  };
}

function objectSample(sample, index) {
  if (!sample || typeof sample !== "object") {
    throw new TypeError(`object_samples[${index}] must be an object`);
  }
  if (!sample.object_pose || typeof sample.object_pose !== "object") {
    throw new TypeError(`object_samples[${index}].object_pose must be an object`);
  }
  return {
    ...sample,
    sample_index: integerAtLeast(sample.sample_index, 0, `object_samples[${index}].sample_index`),
    time_from_start_s: finiteNumber(
      sample.time_from_start_s,
      `object_samples[${index}].time_from_start_s`,
    ),
    object_pose: {
      ...sample.object_pose,
      translation_m: vector(
        sample.object_pose.translation_m,
        3,
        `object_samples[${index}].object_pose.translation_m`,
      ),
      rpy_rad: vector(
        sample.object_pose.rpy_rad,
        3,
        `object_samples[${index}].object_pose.rpy_rad`,
      ),
    },
  };
}

function jointPathPoint(point, index) {
  if (!point || typeof point !== "object") {
    throw new TypeError(`global_path[${index}] must be an object`);
  }
  const left = vector(point.left, 6, `global_path[${index}].left`);
  const right = vector(point.right, 6, `global_path[${index}].right`);
  const combined = vector(point.combined, 12, `global_path[${index}].combined`);
  if (combined.some((value, jointIndex) => value !== [...left, ...right][jointIndex])) {
    throw new RangeError(`global_path[${index}].combined must equal left + right`);
  }
  return {
    ...point,
    sample_index: integerAtLeast(point.sample_index, 0, `global_path[${index}].sample_index`),
    time_from_start_s: finiteNumber(
      point.time_from_start_s,
      `global_path[${index}].time_from_start_s`,
    ),
    left,
    right,
    combined,
    graph_node_id: String(
      point.graph_node_id === null || point.graph_node_id === undefined
        ? ""
        : point.graph_node_id,
    ),
    graph_node_index: point.graph_node_index,
    edge_cost_rad2: point.edge_cost_rad2,
    cumulative_cost_rad2: point.cumulative_cost_rad2,
  };
}

export function normalizeObjectGlobalPlanResponse(response) {
  if (!response || typeof response !== "object" || Array.isArray(response)) {
    throw new TypeError("global plan response must be an object");
  }
  if (response.ok !== true) {
    return {
      ...response,
      ok: false,
      planner_status: String(response.planner_status || "FAILED"),
      failure: response.failure && typeof response.failure === "object"
        ? { ...response.failure }
        : { reason: "UNKNOWN", message: "Global planning failed" },
      warnings: Array.isArray(response.warnings) ? [...response.warnings] : [],
      planner_source: String(response.planner_source || PHASE3_PLANNER_SOURCE),
      optimality_scope: String(response.optimality_scope || PHASE3_OPTIMALITY_SCOPE),
      graph: response.graph && typeof response.graph === "object"
        ? { ...response.graph }
        : null,
      global: response.global && typeof response.global === "object"
        ? { ...response.global }
        : null,
      greedy: response.greedy && typeof response.greedy === "object"
        ? { ...response.greedy }
        : null,
      comparison: response.comparison && typeof response.comparison === "object"
        ? { ...response.comparison }
        : null,
    };
  }
  if (response.plan_only !== true) {
    throw new RangeError("successful global plan must be explicitly PLAN ONLY");
  }
  if (response.optimality_scope !== PHASE3_OPTIMALITY_SCOPE) {
    throw new RangeError(
      `optimality_scope must be '${PHASE3_OPTIMALITY_SCOPE}'`,
    );
  }
  if (!Array.isArray(response.object_samples) || !Array.isArray(response.global_path)) {
    throw new TypeError("successful plan must contain object_samples and global_path arrays");
  }
  if (response.object_samples.length < 2
      || response.object_samples.length !== response.global_path.length) {
    throw new RangeError("successful plan paths must have equal lengths of at least 2");
  }
  const samples = response.object_samples.map(objectSample);
  const path = response.global_path.map(jointPathPoint);
  for (let index = 0; index < path.length; index += 1) {
    if (samples[index].sample_index !== index || path[index].sample_index !== index) {
      throw new RangeError("plan sample indices must be chronological from zero");
    }
    if (samples[index].time_from_start_s !== path[index].time_from_start_s) {
      throw new RangeError("object and joint plan timestamps must match exactly");
    }
    if (index > 0 && path[index - 1].time_from_start_s >= path[index].time_from_start_s) {
      throw new RangeError("plan timestamps must be strictly increasing");
    }
  }
  const fixedOrientationRpyRad = vector(
    response.fixed_orientation_rpy_rad,
    3,
    "fixed_orientation_rpy_rad",
  );
  const declaredDurationS = finiteNumber(response.duration_s, "duration_s");
  if (declaredDurationS !== path[path.length - 1].time_from_start_s) {
    throw new RangeError("duration_s must equal the final common timestamp");
  }
  if (response.object_sample_count !== samples.length) {
    throw new RangeError("object_sample_count must equal the common path length");
  }
  const baselineSegmentDurationS = response.segment_duration_s === undefined
    || response.segment_duration_s === null
    ? null : finiteNumber(response.segment_duration_s, "segment_duration_s");
  const segmentDurationsS = response.segment_durations_s === undefined
    ? []
    : vector(
      response.segment_durations_s,
      response.segment_durations_s.length,
      "segment_durations_s",
    );
  if (baselineSegmentDurationS !== null) {
    if (baselineSegmentDurationS <= 0) {
      throw new RangeError("segment_duration_s must be greater than zero");
    }
    if (segmentDurationsS.some((duration) => duration < baselineSegmentDurationS)) {
      throw new RangeError("relative waypoint profiles must not shorten a baseline segment");
    }
    if (segmentDurationsS.length > 0
        && segmentDurationsS.reduce((sum, duration) => sum + duration, 0)
          !== declaredDurationS) {
      throw new RangeError("segment_durations_s must sum to duration_s");
    }
  }
  const segmentTiming = Array.isArray(response.segment_timing)
    ? response.segment_timing.map((item) => ({...item})) : [];
  const candidateAttemptsPerArm = integerWithin(
    response.candidate_attempts_per_arm,
    1,
    WEB_GLOBAL_MAX_CANDIDATE_ATTEMPTS_PER_ARM,
    "candidate_attempts_per_arm",
  );
  if (response.candidate_pruning_applied !== false) {
    throw new RangeError("Web Global response must not apply candidate pruning");
  }
  const planningStartState = response.planning_start_state_rad
    ? normalizePlanningStartState(response.planning_start_state_rad)
    : null;
  const firstSelectedState = response.first_selected_joint_state_rad === null
    || response.first_selected_joint_state_rad === undefined
    ? null
    : normalizePlanningStartState(response.first_selected_joint_state_rad);
  const authority = normalizePlanAuthority(response);
  const approachPath = Array.isArray(response.approach_path)
    ? response.approach_path.map(jointPathPoint) : [];
  const combinedPath = Array.isArray(response.combined_path)
    ? response.combined_path.map(jointPathPoint) : path;
  const combinedSamples = Array.isArray(response.combined_object_samples)
    ? response.combined_object_samples.map(objectSample) : samples;
  const combinedDurationS = response.combined_duration_s === undefined
    ? declaredDurationS : finiteNumber(response.combined_duration_s, "combined_duration_s");
  if (combinedPath.length < 2 || combinedSamples.length !== combinedPath.length) {
    throw new RangeError(
      "combined_path and combined_object_samples must have equal lengths of at least 2",
    );
  }
  for (let index = 0; index < combinedPath.length; index += 1) {
    if (combinedPath[index].sample_index !== index
        || combinedSamples[index].sample_index !== index) {
      throw new RangeError("combined plan sample indices must be chronological from zero");
    }
    if (combinedPath[index].time_from_start_s !== combinedSamples[index].time_from_start_s) {
      throw new RangeError("combined object and joint timestamps must match exactly");
    }
    if (index > 0
        && combinedPath[index - 1].time_from_start_s >= combinedPath[index].time_from_start_s) {
      throw new RangeError("combined plan timestamps must be strictly increasing");
    }
  }
  if (combinedPath[combinedPath.length - 1].time_from_start_s !== combinedDurationS) {
    throw new RangeError("combined_duration_s must equal the final combined timestamp");
  }
  return {
    ...response,
    ok: true,
    planner_status: String(response.planner_status || "READY"),
    planner_source: String(response.planner_source || PHASE3_PLANNER_SOURCE),
    optimality_scope: String(response.optimality_scope || PHASE3_OPTIMALITY_SCOPE),
    fixed_orientation_rpy_rad: fixedOrientationRpyRad,
    object_samples: samples,
    global_path: path,
    approach_path: approachPath,
    combined_path: combinedPath,
    combined_object_samples: combinedSamples,
    combined_duration_s: combinedDurationS,
    duration_s: declaredDurationS,
    segment_duration_s: baselineSegmentDurationS,
    segment_durations_s: segmentDurationsS,
    segment_timing: segmentTiming,
    timing_semantic: String(response.timing_semantic || "UNAVAILABLE"),
    candidate_attempts_per_arm: candidateAttemptsPerArm,
    candidate_exploration_profile: String(
      response.candidate_exploration_profile || "UNAVAILABLE",
    ),
    candidate_pruning_applied: false,
    planning_start_state_rad: planningStartState,
    planning_start_state_source: String(
      response.planning_start_state_source || "UNAVAILABLE",
    ),
    first_selected_joint_state_rad: firstSelectedState,
    grasp: authority.grasp,
    calibration: authority.calibration,
    start_to_first_raw_joint_delta_rad: Array.isArray(
      response.start_to_first_raw_joint_delta_rad,
    )
      ? vector(response.start_to_first_raw_joint_delta_rad, 12,
        "start_to_first_raw_joint_delta_rad")
      : null,
    start_to_first_maximum_raw_joint_delta: (
      response.start_to_first_maximum_raw_joint_delta
      && typeof response.start_to_first_maximum_raw_joint_delta === "object"
    ) ? { ...response.start_to_first_maximum_raw_joint_delta } : null,
    graph: response.graph && typeof response.graph === "object"
      ? {
        ...response.graph,
        node_counts_per_layer: Array.isArray(response.graph.node_counts_per_layer)
          ? [...response.graph.node_counts_per_layer]
          : [],
        selected_node_ids: Array.isArray(response.graph.selected_node_ids)
          ? [...response.graph.selected_node_ids]
          : [],
        left_candidate_counts_per_layer: Array.isArray(
          response.graph.left_candidate_counts_per_layer
        ) ? [...response.graph.left_candidate_counts_per_layer] : [],
        right_candidate_counts_per_layer: Array.isArray(
          response.graph.right_candidate_counts_per_layer
        ) ? [...response.graph.right_candidate_counts_per_layer] : [],
        candidate_pair_counts_per_layer: Array.isArray(
          response.graph.candidate_pair_counts_per_layer
        ) ? [...response.graph.candidate_pair_counts_per_layer] : [],
        valid_pair_counts_per_layer: Array.isArray(
          response.graph.valid_pair_counts_per_layer
        ) ? [...response.graph.valid_pair_counts_per_layer] : [],
      }
      : null,
    global: response.global && typeof response.global === "object"
      ? { ...response.global }
      : null,
    greedy: response.greedy && typeof response.greedy === "object"
      ? { ...response.greedy }
      : null,
    comparison: response.comparison && typeof response.comparison === "object"
      ? { ...response.comparison }
      : null,
    warnings: Array.isArray(response.warnings) ? [...response.warnings] : [],
  };
}

function interpolate(left, right, alpha) {
  return left.map((value, index) => value + (right[index] - value) * alpha);
}

function rpyToQuaternionXyzw(rpy) {
  const [roll, pitch, yaw] = rpy;
  const cr = Math.cos(roll * 0.5);
  const sr = Math.sin(roll * 0.5);
  const cp = Math.cos(pitch * 0.5);
  const sp = Math.sin(pitch * 0.5);
  const cy = Math.cos(yaw * 0.5);
  const sy = Math.sin(yaw * 0.5);
  return [
    sr * cp * cy - cr * sp * sy,
    cr * sp * cy + sr * cp * sy,
    cr * cp * sy - sr * sp * cy,
    cr * cp * cy + sr * sp * sy,
  ];
}

function quaternionXyzwToRpy(quaternion) {
  const [x, y, z, w] = quaternion;
  const sinrCosp = 2 * (w * x + y * z);
  const cosrCosp = 1 - 2 * (x * x + y * y);
  const roll = Math.atan2(sinrCosp, cosrCosp);
  const sinp = 2 * (w * y - z * x);
  const pitch = Math.abs(sinp) >= 1
    ? Math.sign(sinp) * Math.PI / 2 : Math.asin(sinp);
  const sinyCosp = 2 * (w * z + x * y);
  const cosyCosp = 1 - 2 * (y * y + z * z);
  const yaw = Math.atan2(sinyCosp, cosyCosp);
  return [roll, pitch, yaw];
}

function slerpQuaternionXyzw(start, end, alpha) {
  let q1 = [...end];
  let dot = start.reduce((sum, value, index) => sum + value * q1[index], 0);
  if (dot < 0) {
    q1 = q1.map((value) => -value);
    dot = -dot;
  }
  dot = Math.max(-1, Math.min(1, dot));
  if (dot > 0.9995) {
    const mixed = start.map((value, index) => value + alpha * (q1[index] - value));
    const norm = Math.hypot(...mixed);
    return mixed.map((value) => value / norm);
  }
  const theta0 = Math.acos(dot);
  const sinTheta0 = Math.sin(theta0);
  const theta = theta0 * alpha;
  const s0 = Math.cos(theta) - dot * Math.sin(theta) / sinTheta0;
  const s1 = Math.sin(theta) / sinTheta0;
  return start.map((value, index) => s0 * value + s1 * q1[index]);
}

function interpolateRpy(left, right, alpha) {
  return quaternionXyzwToRpy(slerpQuaternionXyzw(
    rpyToQuaternionXyzw(left), rpyToQuaternionXyzw(right), alpha,
  ));
}

export function sampleIntegratedObjectGlobalPlan(normalizedPlan, requestedTimeS) {
  const plan = normalizeObjectGlobalPlanResponse(normalizedPlan);
  if (!plan.ok) throw new RangeError("cannot preview an unsuccessful global plan");
  const path = Array.isArray(plan.combined_path) && plan.combined_path.length >= 2
    ? plan.combined_path : plan.global_path;
  const samples = Array.isArray(plan.combined_object_samples)
    && plan.combined_object_samples.length === path.length
    ? plan.combined_object_samples : plan.object_samples;
  const duration = Number.isFinite(Number(plan.combined_duration_s))
    ? Number(plan.combined_duration_s) : plan.duration_s;
  const time = Math.max(0, Math.min(duration, finiteNumber(requestedTimeS, "requestedTimeS")));
  let segmentIndex = path.length - 2;
  for (let index = 0; index < path.length - 1; index += 1) {
    if (time <= path[index + 1].time_from_start_s) {
      segmentIndex = index;
      break;
    }
  }
  const firstPath = path[segmentIndex];
  const secondPath = path[segmentIndex + 1];
  const firstObject = samples[segmentIndex];
  const secondObject = samples[segmentIndex + 1];
  const dt = secondPath.time_from_start_s - firstPath.time_from_start_s;
  const alpha = dt > 0 ? (time - firstPath.time_from_start_s) / dt : 0;
  return {
    time_from_start_s: time,
    segmentIndex,
    alpha,
    left: interpolate(firstPath.left, secondPath.left, alpha),
    right: interpolate(firstPath.right, secondPath.right, alpha),
    objectPose: {
      translationM: interpolate(
        firstObject.object_pose.translation_m,
        secondObject.object_pose.translation_m,
        alpha,
      ),
      rpyRad: interpolateRpy(
        firstObject.object_pose.rpy_rad, secondObject.object_pose.rpy_rad, alpha,
      ),
    },
  };
}

export function globalPlanToPlannedTrajectory(normalizedPlan) {
  const plan = normalizeObjectGlobalPlanResponse(normalizedPlan);
  if (!plan.ok) throw new RangeError("cannot load an unsuccessful global plan");
  const source = Array.isArray(plan.combined_path) && plan.combined_path.length
    ? plan.combined_path : plan.global_path;
  return {
    name: String(plan.trajectory_name || "Object Global Plan"),
    points: source.map((point) => ({
      time_from_start_s: point.time_from_start_s,
      left: [...point.left],
      right: [...point.right],
    })),
  };
}

export async function requestObjectGlobalPlan(payload, fetchImpl = fetch) {
  if (typeof fetchImpl !== "function") throw new TypeError("fetchImpl must be a function");
  const response = await fetchImpl(PHASE3_GLOBAL_PLAN_ENDPOINT, {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify(payload),
  });
  let body;
  try {
    body = await response.json();
  } catch (_error) {
    throw new Error(`Global planning returned non-JSON HTTP ${response.status}`);
  }
  if (!response.ok) {
    const detail = typeof body.detail === "string" ? body.detail : `HTTP ${response.status}`;
    throw new Error(`Global planning request failed: ${detail}`);
  }
  return normalizeObjectGlobalPlanResponse(body);
}
