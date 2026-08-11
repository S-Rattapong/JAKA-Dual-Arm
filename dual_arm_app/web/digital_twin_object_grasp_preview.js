// Phase 1E.2 pure transform helpers for the offline object/grasp preview.
// Convention: ^A T_B is frame B expressed in frame A and maps column vectors
// as ^A p = ^A T_B * ^B p. Translation is meters and RPY is radians.

const TRANSLATION_LIMIT_M = 3;
const RPY_LIMIT_RAD = 2 * Math.PI;

function finiteNumber(value, label) {
  if (typeof value !== "number" || !Number.isFinite(value)) {
    throw new TypeError(`${label} must be a finite number`);
  }
  return value;
}

function vector3(values, label) {
  if (!Array.isArray(values) || values.length !== 3) {
    throw new TypeError(`${label} must contain exactly three finite numbers`);
  }
  return values.map((value, index) => finiteNumber(value, `${label}[${index}]`));
}

function freezePose(translationM, rpyRad) {
  return Object.freeze({
    translationM: Object.freeze([...translationM]),
    rpyRad: Object.freeze([...rpyRad]),
  });
}

function freezeDimensions(dimensions) {
  return Object.freeze({ ...dimensions });
}

export const SYNTHETIC_OBJECT_DIMENSIONS_M = freezeDimensions({
  length: 0.50,
  width: 0.15,
  height: 0.10,
});

// Chosen only to make the workpiece visible between the zero-pose arms.
// This is synthetic scene metadata, not physical calibration.
export const INITIAL_SYNTHETIC_OBJECT_POSE = freezePose(
  [-1.32, 0.0, 1.20],
  [0.0, 0.0, 0.0],
);

// Fixed Phase 1E.1 object-relative planning/grasp fixture values.
export const SYNTHETIC_OBJECT_T_LEFT_POSE = freezePose(
  [0.0, 0.25, 0.0],
  [0.0, 0.0, 0.0],
);
export const SYNTHETIC_OBJECT_T_RIGHT_POSE = freezePose(
  [0.0, -0.25, 0.0],
  [0.0, 0.0, 0.0],
);

export function normalizeObjectPreviewPose(pose) {
  if (!pose || typeof pose !== "object" || Array.isArray(pose)) {
    throw new TypeError("object pose must contain translationM and rpyRad");
  }
  const translationM = vector3(pose.translationM, "translationM");
  const rpyRad = vector3(pose.rpyRad, "rpyRad");
  if (translationM.some((value) => Math.abs(value) > TRANSLATION_LIMIT_M)) {
    throw new RangeError(`object translation must be within +/-${TRANSLATION_LIMIT_M} m`);
  }
  if (rpyRad.some((value) => Math.abs(value) > RPY_LIMIT_RAD)) {
    throw new RangeError(`object RPY must be within +/-${RPY_LIMIT_RAD} rad`);
  }
  return { translationM, rpyRad };
}

export function matrix4FromTranslationRpy(pose) {
  const { translationM, rpyRad } = normalizeObjectPreviewPose(pose);
  const [x, y, z] = translationM;
  const [roll, pitch, yaw] = rpyRad;
  const cr = Math.cos(roll);
  const sr = Math.sin(roll);
  const cp = Math.cos(pitch);
  const sp = Math.sin(pitch);
  const cy = Math.cos(yaw);
  const sy = Math.sin(yaw);

  // ROS URDF fixed-axis RPY for column vectors: Rz(yaw) * Ry(pitch) * Rx(roll).
  return [
    [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr, x],
    [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr, y],
    [-sp, cp * sr, cp * cr, z],
    [0, 0, 0, 1],
  ];
}

export function multiplyMatrix4(left, right) {
  for (const [label, matrix] of [["left", left], ["right", right]]) {
    if (
      !Array.isArray(matrix)
      || matrix.length !== 4
      || matrix.some((row) => !Array.isArray(row) || row.length !== 4)
      || matrix.some((row) => row.some(
        (value) => typeof value !== "number" || !Number.isFinite(value),
      ))
    ) {
      throw new TypeError(`${label} must be a finite 4x4 matrix`);
    }
  }
  return Array.from({ length: 4 }, (_unused, row) => (
    Array.from({ length: 4 }, (_unusedColumn, column) => (
      left[row].reduce(
        (total, value, index) => total + value * right[index][column],
        0,
      )
    ))
  ));
}

const SYNTHETIC_OBJECT_T_LEFT = matrix4FromTranslationRpy(
  SYNTHETIC_OBJECT_T_LEFT_POSE,
);
const SYNTHETIC_OBJECT_T_RIGHT = matrix4FromTranslationRpy(
  SYNTHETIC_OBJECT_T_RIGHT_POSE,
);

export function computeWorldGraspFrameMatricesFromWorldTObject(worldTObject) {
  return {
    worldTObject,
    // ^W T_L = ^W T_O * ^O T_L; ^O T_L remains fixed.
    worldTLeft: multiplyMatrix4(worldTObject, SYNTHETIC_OBJECT_T_LEFT),
    // ^W T_R = ^W T_O * ^O T_R; ^O T_R remains fixed.
    worldTRight: multiplyMatrix4(worldTObject, SYNTHETIC_OBJECT_T_RIGHT),
  };
}

export function computeWorldGraspFrameMatrices(objectPose) {
  return computeWorldGraspFrameMatricesFromWorldTObject(
    matrix4FromTranslationRpy(objectPose),
  );
}
