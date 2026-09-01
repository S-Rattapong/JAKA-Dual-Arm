"use strict";

// Pure browser-side mapping from an integrated Phase-3 plan to the next rigid
// waypoint target. This module has no transport, robot, ROS, or rendering API.

const JOINT_COUNT_PER_ARM = 6;
// Visual-only waypoint indicator tolerance. This never authorizes motion,
// safety, completion, or trajectory state; those remain backend/driver owned.
export const EXECUTION_GHOST_JOINT_TOLERANCE_RAD = 0.015;
export const EXECUTION_GHOST_ACTUAL_FRESHNESS_MS = 250;

function finiteNumber(value, label) {
  if (typeof value !== "number" || !Number.isFinite(value)) {
    throw new TypeError(`${label} must be a finite number`);
  }
  return value;
}

function jointVector(value, label) {
  if (!Array.isArray(value) || value.length !== JOINT_COUNT_PER_ARM) {
    throw new TypeError(`${label} must contain exactly ${JOINT_COUNT_PER_ARM} joints`);
  }
  return value.map((joint, index) => finiteNumber(joint, `${label}[${index}]`));
}

function pathPoint(value, index, label) {
  if (!value || typeof value !== "object" || Array.isArray(value)) {
    throw new TypeError(`${label}[${index}] must be an object`);
  }
  const left = jointVector(value.left, `${label}[${index}].left`);
  const right = jointVector(value.right, `${label}[${index}].right`);
  const timeS = finiteNumber(
    value.time_from_start_s,
    `${label}[${index}].time_from_start_s`,
  );
  if (timeS < 0) throw new RangeError(`${label} timestamps must be nonnegative`);
  return { left, right, timeS };
}

function checkedPath(value, label, minimumLength = 2) {
  if (!Array.isArray(value) || value.length < minimumLength) {
    throw new TypeError(`${label} must contain at least ${minimumLength} samples`);
  }
  const result = value.map((point, index) => pathPoint(point, index, label));
  result.forEach((point, index) => {
    if (index > 0 && result[index - 1].timeS >= point.timeS) {
      throw new RangeError(`${label} timestamps must be strictly increasing`);
    }
  });
  return result;
}

function samePose(left, right) {
  return left.left.every((joint, index) => joint === right.left[index])
    && left.right.every((joint, index) => joint === right.right[index]);
}

function checkedWaypointIdentifiers(waypoints) {
  if (!Array.isArray(waypoints) || waypoints.length < 2) {
    throw new TypeError("plan.waypoints must contain at least two rigid waypoints");
  }
  return waypoints.map((waypoint, index) => {
    const identifier = waypoint && typeof waypoint.identifier === "string"
      ? waypoint.identifier.trim() : "";
    if (!identifier) {
      throw new TypeError(`plan.waypoints[${index}].identifier must be non-empty`);
    }
    return identifier;
  });
}

function validateObjectSamples(plan, globalPath) {
  if (!Array.isArray(plan.object_samples)
      || plan.object_samples.length !== globalPath.length) {
    throw new RangeError("plan.object_samples must match plan.global_path length");
  }
  plan.object_samples.forEach((sample, index) => {
    if (!sample || typeof sample !== "object" || Array.isArray(sample)) {
      throw new TypeError(`plan.object_samples[${index}] must be an object`);
    }
    const timeS = finiteNumber(
      sample.time_from_start_s,
      `plan.object_samples[${index}].time_from_start_s`,
    );
    if (timeS !== globalPath[index].timeS) {
      throw new RangeError("plan.object_samples and plan.global_path timestamps must match");
    }
  });
}

export function deriveRigidWaypointTimeline(plan) {
  if (!plan || typeof plan !== "object" || Array.isArray(plan) || plan.ok !== true) {
    throw new TypeError("a successful integrated global plan is required");
  }
  const identifiers = checkedWaypointIdentifiers(plan.waypoints);
  if (plan.object_waypoint_count !== undefined
      && plan.object_waypoint_count !== identifiers.length) {
    throw new RangeError("plan.object_waypoint_count must match plan.waypoints length");
  }

  const globalPath = checkedPath(plan.global_path, "plan.global_path");
  validateObjectSamples(plan, globalPath);
  if (globalPath[0].timeS !== 0) {
    throw new RangeError("plan.global_path must begin at zero time");
  }

  const segmentCount = identifiers.length - 1;
  const rigidIntervals = globalPath.length - 1;
  if (rigidIntervals % segmentCount !== 0) {
    throw new RangeError(
      "rigid sample count must equal 1 + segments * subdivisions exactly",
    );
  }
  const subdivisions = rigidIntervals / segmentCount;
  if (!Number.isInteger(subdivisions) || subdivisions < 1) {
    throw new RangeError("rigid subdivisions per segment must be a positive integer");
  }

  const rawApproach = Array.isArray(plan.approach_path) ? plan.approach_path : [];
  const hasApproach = rawApproach.length > 0;
  const approachPath = hasApproach
    ? checkedPath(rawApproach, "plan.approach_path") : [];
  if (hasApproach && approachPath[0].timeS !== 0) {
    throw new RangeError("plan.approach_path must begin at zero time");
  }
  const approachDurationS = hasApproach
    ? approachPath[approachPath.length - 1].timeS : 0;
  if (hasApproach && plan.approach && plan.approach.duration_s !== undefined
      && finiteNumber(plan.approach.duration_s, "plan.approach.duration_s")
        !== approachDurationS) {
    throw new RangeError("plan approach duration must match approach_path end time");
  }

  const combinedPath = checkedPath(plan.combined_path, "plan.combined_path");
  const approachPrefixSampleCount = hasApproach ? approachPath.length - 1 : 0;
  const expectedCombinedCount = approachPrefixSampleCount + globalPath.length;
  if (combinedPath.length !== expectedCombinedCount) {
    throw new RangeError("plan.combined_path is inconsistent with approach + rigid paths");
  }

  const targets = identifiers.map((identifier, waypointIndex) => {
    const rigidSampleIndex = waypointIndex * subdivisions;
    const combinedSampleIndex = approachPrefixSampleCount + rigidSampleIndex;
    const rigidPoint = globalPath[rigidSampleIndex];
    const combinedPoint = combinedPath[combinedSampleIndex];
    const timeS = approachDurationS + rigidPoint.timeS;
    if (combinedPoint.timeS !== timeS || !samePose(rigidPoint, combinedPoint)) {
      throw new RangeError(
        `combined rigid waypoint sample ${waypointIndex} does not match global_path`,
      );
    }
    return {
      identifier,
      waypointIndex,
      rigidSampleIndex,
      combinedSampleIndex,
      timeS,
      pose: {
        left: [...rigidPoint.left],
        right: [...rigidPoint.right],
      },
    };
  });

  return {
    sampleCount: globalPath.length,
    waypointCount: identifiers.length,
    segmentCount,
    subdivisions,
    hasApproach,
    approachDurationS,
    targets,
  };
}

function targetSnapshot(timeline, executionIdentity, targetIndex, extra = {}) {
  if (targetIndex >= timeline.targets.length) {
    return {
      executionIdentity,
      targetIndex,
      visible: false,
      reason: "FINAL_RIGID_WAYPOINT_REACHED",
      ...extra,
    };
  }
  return {
    executionIdentity,
    targetIndex,
    visible: true,
    ...timeline.targets[targetIndex],
    ...extra,
  };
}

function checkedExecutionIdentity(value) {
  if (typeof value !== "string" || !value.trim()) {
    throw new TypeError("execution identity must be a non-empty string");
  }
  return value.trim();
}

function checkedActualPose(value) {
  if (!value || typeof value !== "object" || Array.isArray(value)) return null;
  try {
    return {
      left: jointVector(value.left, "actual pose.left"),
      right: jointVector(value.right, "actual pose.right"),
    };
  } catch (_error) {
    return null;
  }
}

function actualPoseIsFresh(receivedAtMs, nowMs, freshnessThresholdMs) {
  return typeof receivedAtMs === "number"
    && Number.isFinite(receivedAtMs)
    && typeof nowMs === "number"
    && Number.isFinite(nowMs)
    && typeof freshnessThresholdMs === "number"
    && Number.isFinite(freshnessThresholdMs)
    && freshnessThresholdMs >= 0
    && Math.max(0, nowMs - receivedAtMs) <= freshnessThresholdMs;
}

function poseWithinTolerance(actual, target, toleranceRad) {
  return ["left", "right"].every((side) => actual[side].every(
    (joint, index) => Math.abs(joint - target[side][index]) <= toleranceRad,
  ));
}

export function createRigidWaypointTargetCursor(plan, executionIdentity) {
  const timeline = deriveRigidWaypointTimeline(plan);
  const identity = checkedExecutionIdentity(executionIdentity);
  const initialTargetIndex = timeline.hasApproach ? 0 : 1;
  return targetSnapshot(timeline, identity, initialTargetIndex, {
    actualFresh: false,
    advanced: false,
  });
}

export function updateRigidWaypointTargetCursor(
  plan,
  cursor,
  {
    executionIdentity,
    actualPose,
    actualReceivedAtMs,
    nowMs,
    freshnessThresholdMs = EXECUTION_GHOST_ACTUAL_FRESHNESS_MS,
    toleranceRad = EXECUTION_GHOST_JOINT_TOLERANCE_RAD,
  },
) {
  const timeline = deriveRigidWaypointTimeline(plan);
  const identity = checkedExecutionIdentity(executionIdentity);
  if (!cursor || cursor.executionIdentity !== identity) {
    return {
      executionIdentity: identity,
      targetIndex: null,
      visible: false,
      actualFresh: false,
      advanced: false,
      reason: "EXECUTION_IDENTITY_MISMATCH",
    };
  }
  const targetIndex = Number(cursor.targetIndex);
  if (!Number.isInteger(targetIndex) || targetIndex < 0) {
    throw new TypeError("target cursor index must be a nonnegative integer");
  }
  if (targetIndex >= timeline.targets.length) {
    return targetSnapshot(timeline, identity, targetIndex, {
      actualFresh: false,
      advanced: false,
    });
  }

  const normalizedActual = checkedActualPose(actualPose);
  const fresh = normalizedActual !== null && actualPoseIsFresh(
    actualReceivedAtMs, nowMs, freshnessThresholdMs,
  );
  if (!fresh) {
    return targetSnapshot(timeline, identity, targetIndex, {
      actualFresh: false,
      advanced: false,
      reason: "ACTUAL_FEEDBACK_UNAVAILABLE_OR_STALE",
    });
  }
  if (!Number.isFinite(toleranceRad) || toleranceRad < 0) {
    throw new RangeError("visual-only joint tolerance must be nonnegative");
  }
  if (!poseWithinTolerance(
    normalizedActual, timeline.targets[targetIndex].pose, toleranceRad,
  )) {
    return targetSnapshot(timeline, identity, targetIndex, {
      actualFresh: true,
      advanced: false,
    });
  }
  return targetSnapshot(timeline, identity, targetIndex + 1, {
    actualFresh: true,
    advanced: true,
  });
}
