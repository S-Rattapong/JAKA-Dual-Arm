// Pure Phase 1D.3A stored-waypoint position-limit validation in radians.

function finiteNumber(value, label) {
  if (typeof value !== "number" || !Number.isFinite(value)) {
    throw new TypeError(`${label} must be a finite number`);
  }
  return value;
}

function checkedMetadata(metadata) {
  if (!metadata || typeof metadata !== "object" || Array.isArray(metadata)) {
    throw new TypeError("Joint-limit metadata must be an object");
  }
  if (!Array.isArray(metadata.joint_order) || metadata.joint_order.length !== 12) {
    throw new TypeError("Joint-limit metadata must define exactly 12 ordered joints");
  }
  if (!metadata.position_limits || typeof metadata.position_limits !== "object") {
    throw new TypeError("Joint-limit metadata must define position limits");
  }
  const jointOrder = [...metadata.joint_order];
  const uniqueJoints = new Set(jointOrder);
  if (uniqueJoints.size !== jointOrder.length) {
    throw new TypeError("Joint-limit metadata contains duplicate joint names");
  }
  const positionLimits = {};
  jointOrder.forEach((jointName) => {
    const limits = metadata.position_limits[jointName];
    if (!limits || typeof limits !== "object") {
      throw new TypeError(`Position limits are missing for ${jointName}`);
    }
    const min = finiteNumber(limits.min, `${jointName} minimum position`);
    const max = finiteNumber(limits.max, `${jointName} maximum position`);
    if (min >= max) throw new RangeError(`${jointName} minimum must be less than maximum`);
    positionLimits[jointName] = { min, max };
  });
  return { jointOrder, positionLimits };
}

function pointValue(point, jointName, pointIndex) {
  if (!point || typeof point !== "object" || Array.isArray(point)) {
    throw new TypeError(`Trajectory point ${pointIndex} must be an object`);
  }
  const match = /^(left|right)_joint_([1-6])$/.exec(jointName);
  if (!match) throw new TypeError(`Unsupported dual-arm joint name: ${jointName}`);
  const side = match[1];
  const jointIndex = Number(match[2]) - 1;
  if (!Array.isArray(point[side]) || point[side].length !== 6) {
    throw new TypeError(`Trajectory point ${pointIndex} ${side} side must have 6 joints`);
  }
  return finiteNumber(
    point[side][jointIndex],
    `Trajectory point ${pointIndex} ${jointName}`,
  );
}

function violationFor(point, pointIndex, jointName, limits) {
  const value = pointValue(point, jointName, pointIndex);
  if (value >= limits.min && value <= limits.max) return null;
  return {
    pointIndex,
    timeFromStartS: finiteNumber(
      point.time_from_start_s,
      `Trajectory point ${pointIndex} timestamp`,
    ),
    jointName,
    valueRad: value,
    minRad: limits.min,
    maxRad: limits.max,
    amountOutsideRad: value < limits.min
      ? limits.min - value
      : value - limits.max,
  };
}

function validationResult(checkedPointCount, checkedJointCount, violations) {
  const copiedViolations = violations.map((violation) => ({ ...violation }));
  return {
    valid: copiedViolations.length === 0,
    checkedPointCount,
    checkedJointCount,
    violations: copiedViolations,
    firstViolation: copiedViolations.length > 0
      ? { ...copiedViolations[0] }
      : null,
  };
}

export function validateTrajectoryPointJointLimits(
  point,
  pointIndex,
  jointLimitMetadata,
) {
  if (!Number.isInteger(pointIndex) || pointIndex < 0) {
    throw new TypeError("Trajectory point index must be a non-negative integer");
  }
  const { jointOrder, positionLimits } = checkedMetadata(jointLimitMetadata);
  const violations = [];
  jointOrder.forEach((jointName) => {
    const violation = violationFor(
      point,
      pointIndex,
      jointName,
      positionLimits[jointName],
    );
    if (violation) violations.push(violation);
  });
  return validationResult(1, jointOrder.length, violations);
}

export function validateTrajectoryJointLimits(trajectory, jointLimitMetadata) {
  if (!trajectory || typeof trajectory !== "object" || Array.isArray(trajectory)) {
    throw new TypeError("Normalized trajectory must be an object");
  }
  if (!Array.isArray(trajectory.points) || trajectory.points.length < 2) {
    throw new TypeError("Normalized trajectory must contain at least two points");
  }
  const { jointOrder, positionLimits } = checkedMetadata(jointLimitMetadata);
  const violations = [];
  trajectory.points.forEach((point, pointIndex) => {
    jointOrder.forEach((jointName) => {
      const violation = violationFor(
        point,
        pointIndex,
        jointName,
        positionLimits[jointName],
      );
      if (violation) violations.push(violation);
    });
  });
  return validationResult(
    trajectory.points.length,
    trajectory.points.length * jointOrder.length,
    violations,
  );
}
