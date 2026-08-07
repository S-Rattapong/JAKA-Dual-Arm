// Pure Phase 1D.2 helpers. Time is seconds and every joint value is radians.

const JOINTS_PER_ARM = 6;
const MINIMUM_POINT_COUNT = 2;
const ZERO_TIME_TOLERANCE_S = 1e-9;

function finiteNumber(value, label) {
  if (typeof value !== "number" || !Number.isFinite(value)) {
    throw new TypeError(`${label} must be a finite number`);
  }
  return value;
}

function normalizeJointArray(side, values, pointIndex) {
  if (!Array.isArray(values) || values.length !== JOINTS_PER_ARM) {
    throw new TypeError(
      `points[${pointIndex}].${side} must contain exactly six radians values`,
    );
  }
  values.forEach((value, jointIndex) => {
    finiteNumber(value, `points[${pointIndex}].${side}[${jointIndex}]`);
  });
  return [...values];
}

function normalizedPoint(point, pointIndex) {
  if (!point || typeof point !== "object" || Array.isArray(point)) {
    throw new TypeError(`points[${pointIndex}] must be an object`);
  }
  const time = finiteNumber(
    point.time_from_start_s,
    `points[${pointIndex}].time_from_start_s`,
  );
  if (time < 0) {
    throw new RangeError(`points[${pointIndex}].time_from_start_s must be >= 0`);
  }
  return {
    time_from_start_s: time,
    left: normalizeJointArray("left", point.left, pointIndex),
    right: normalizeJointArray("right", point.right, pointIndex),
  };
}

export function normalizeDualArmTrajectory(trajectory) {
  if (!trajectory || typeof trajectory !== "object" || Array.isArray(trajectory)) {
    throw new TypeError("Trajectory must be an object");
  }
  if (!Array.isArray(trajectory.points)) {
    throw new TypeError("Trajectory points must be an array");
  }
  if (trajectory.points.length < MINIMUM_POINT_COUNT) {
    throw new RangeError("Trajectory requires at least two points");
  }

  const points = trajectory.points.map(normalizedPoint);
  if (Math.abs(points[0].time_from_start_s) > ZERO_TIME_TOLERANCE_S) {
    throw new RangeError("Trajectory first timestamp must be 0 seconds");
  }
  points[0].time_from_start_s = 0;
  for (let index = 1; index < points.length; index += 1) {
    if (points[index].time_from_start_s <= points[index - 1].time_from_start_s) {
      throw new RangeError("Trajectory timestamps must be strictly increasing");
    }
  }

  const name = typeof trajectory.name === "string" && trajectory.name.trim()
    ? trajectory.name.trim()
    : "Unnamed Trajectory";
  return { name, points };
}

export function trajectoryDurationSeconds(trajectory) {
  const normalized = normalizeDualArmTrajectory(trajectory);
  return normalized.points[normalized.points.length - 1].time_from_start_s;
}

export function trajectoryPointAtIndex(trajectory, index) {
  const normalized = normalizeDualArmTrajectory(trajectory);
  if (!Number.isInteger(index)) {
    throw new TypeError("Trajectory point index must be an integer");
  }
  if (index < 0 || index >= normalized.points.length) {
    throw new RangeError("Trajectory point index is out of range");
  }
  const point = normalized.points[index];
  return {
    time_from_start_s: point.time_from_start_s,
    left: [...point.left],
    right: [...point.right],
    pointIndex: index,
  };
}

export function flattenTrajectoryPoint(point) {
  const normalized = normalizedPoint(point, 0);
  return [...normalized.left, ...normalized.right];
}

function sampledPose(point, segmentIndex, alpha) {
  return {
    time_from_start_s: point.time_from_start_s,
    left: [...point.left],
    right: [...point.right],
    segmentIndex,
    alpha,
  };
}

export function sampleTrajectoryAtTime(trajectory, timeSeconds) {
  const normalized = normalizeDualArmTrajectory(trajectory);
  finiteNumber(timeSeconds, "Trajectory preview time");
  const { points } = normalized;
  const lastIndex = points.length - 1;
  const duration = points[lastIndex].time_from_start_s;

  if (timeSeconds <= 0) return sampledPose(points[0], 0, 0);
  if (timeSeconds >= duration) {
    return sampledPose(points[lastIndex], lastIndex - 1, 1);
  }

  for (let index = 0; index < lastIndex; index += 1) {
    const start = points[index];
    const end = points[index + 1];
    if (timeSeconds === start.time_from_start_s) {
      return sampledPose(start, index, 0);
    }
    if (timeSeconds <= end.time_from_start_s) {
      if (timeSeconds === end.time_from_start_s) {
        const nextSegment = Math.min(index + 1, lastIndex - 1);
        return sampledPose(end, nextSegment, nextSegment === index ? 1 : 0);
      }
      const alpha = (
        (timeSeconds - start.time_from_start_s)
        / (end.time_from_start_s - start.time_from_start_s)
      );
      return {
        time_from_start_s: timeSeconds,
        left: start.left.map(
          (value, jointIndex) => value + alpha * (end.left[jointIndex] - value),
        ),
        right: start.right.map(
          (value, jointIndex) => value + alpha * (end.right[jointIndex] - value),
        ),
        segmentIndex: index,
        alpha,
      };
    }
  }

  return sampledPose(points[lastIndex], lastIndex - 1, 1);
}

export function trajectoryMetadata(trajectory) {
  const normalized = normalizeDualArmTrajectory(trajectory);
  return {
    name: normalized.name,
    pointCount: normalized.points.length,
    durationSeconds: normalized.points[normalized.points.length - 1].time_from_start_s,
    jointCountPerPoint: JOINTS_PER_ARM * 2,
  };
}
