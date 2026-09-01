// Phase 1E.2 pure transform helpers for the offline object/grasp preview.
// Convention: ^A T_B is frame B expressed in frame A and maps column vectors
// as ^A p = ^A T_B * ^B p. Translation is meters and RPY is radians.

const TRANSLATION_LIMIT_M = 3;
const RPY_LIMIT_RAD = 2 * Math.PI;

export const GRASP_FRAME_ALIGNMENT_TRANSLATION_TOLERANCE_M = 1e-9;
export const GRASP_FRAME_ALIGNMENT_ORIENTATION_TOLERANCE_RAD = 1e-9;

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

// Migration/display defaults only. Runtime authority is loaded from the
// backend grasp-state API; these values are not a locked planning grasp.
export const SYNTHETIC_OBJECT_T_LEFT_POSE = freezePose(
  [0.0, 0.25, 0.0],
  [0.0, 0.0, 0.0],
);
export const SYNTHETIC_OBJECT_T_RIGHT_POSE = freezePose(
  [0.0, -0.25, 0.0],
  [0.0, 0.0, 0.0],
);

export const SYNTHETIC_DEFAULT_GRASP_DRAFT = Object.freeze({
  left: SYNTHETIC_OBJECT_T_LEFT_POSE,
  right: SYNTHETIC_OBJECT_T_RIGHT_POSE,
  source: "SYNTHETIC_DEFAULT_DRAFT",
});

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

export function normalizeObjectRelativeGraspPose(pose, label = "grasp pose") {
  if (!pose || typeof pose !== "object" || Array.isArray(pose)) {
    throw new TypeError(`${label} must contain translationM and rpyRad`);
  }
  const translationM = vector3(pose.translationM, `${label}.translationM`);
  const rpyRad = vector3(pose.rpyRad, `${label}.rpyRad`);
  if (translationM.some((value) => Math.abs(value) > TRANSLATION_LIMIT_M)) {
    throw new RangeError(`${label} translation must be within +/-${TRANSLATION_LIMIT_M} m`);
  }
  if (rpyRad.some((value) => Math.abs(value) > RPY_LIMIT_RAD)) {
    throw new RangeError(`${label} RPY must be within +/-${RPY_LIMIT_RAD} rad`);
  }
  return { translationM, rpyRad };
}

export function normalizeRigidGraspDraft(configuration) {
  if (!configuration || typeof configuration !== "object" || Array.isArray(configuration)) {
    throw new TypeError("rigid grasp draft must contain left and right poses");
  }
  return {
    left: normalizeObjectRelativeGraspPose(configuration.left, "left grasp"),
    right: normalizeObjectRelativeGraspPose(configuration.right, "right grasp"),
  };
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
  validateMatrix4(left, "left");
  validateMatrix4(right, "right");
  return Array.from({ length: 4 }, (_unused, row) => (
    Array.from({ length: 4 }, (_unusedColumn, column) => (
      left[row].reduce(
        (total, value, index) => total + value * right[index][column],
        0,
      )
    ))
  ));
}

function validateMatrix4(matrix, label = "matrix") {
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
  return matrix;
}

export function inverseRigidMatrix4(matrix) {
  validateMatrix4(matrix);
  const inverseRotation = [
    [matrix[0][0], matrix[1][0], matrix[2][0]],
    [matrix[0][1], matrix[1][1], matrix[2][1]],
    [matrix[0][2], matrix[1][2], matrix[2][2]],
  ];
  const translation = [matrix[0][3], matrix[1][3], matrix[2][3]];
  const inverseTranslation = inverseRotation.map((row) => (
    -row.reduce((total, value, index) => total + value * translation[index], 0)
  ));
  return [
    [...inverseRotation[0], inverseTranslation[0]],
    [...inverseRotation[1], inverseTranslation[1]],
    [...inverseRotation[2], inverseTranslation[2]],
    [0, 0, 0, 1],
  ];
}

export function poseFromRigidMatrix4(matrix) {
  validateMatrix4(matrix);
  const pitch = Math.asin(Math.max(-1, Math.min(1, -matrix[2][0])));
  const cosinePitch = Math.cos(pitch);
  let roll;
  let yaw;
  if (Math.abs(cosinePitch) > 1e-9) {
    roll = Math.atan2(matrix[2][1], matrix[2][2]);
    yaw = Math.atan2(matrix[1][0], matrix[0][0]);
  } else {
    roll = Math.atan2(-matrix[1][2], matrix[1][1]);
    yaw = 0;
  }
  return {
    translationM: [matrix[0][3], matrix[1][3], matrix[2][3]],
    rpyRad: [roll, pitch, yaw],
  };
}

// Build the editable Center-relative grasp draft from the two model-tip world
// transforms currently displayed by the Digital Twin.  The Center orientation
// is deliberately world-aligned; only its translation is the tip midpoint.
export function deriveGraspFrameResetFromWorldTips(worldTLeft, worldTRight) {
  validateMatrix4(worldTLeft, "worldTLeft");
  validateMatrix4(worldTRight, "worldTRight");
  const centerPose = normalizeObjectPreviewPose({
    translationM: [0, 1, 2].map(
      (axis) => (worldTLeft[axis][3] + worldTRight[axis][3]) / 2,
    ),
    rpyRad: [0, 0, 0],
  });
  const worldTCenter = matrix4FromTranslationRpy(centerPose);
  const centerTWorld = inverseRigidMatrix4(worldTCenter);
  const centerTLeft = multiplyMatrix4(centerTWorld, worldTLeft);
  const centerTRight = multiplyMatrix4(centerTWorld, worldTRight);
  return {
    centerPose,
    worldTCenter,
    centerTLeft,
    centerTRight,
    draft: {
      left: poseFromRigidMatrix4(centerTLeft),
      right: poseFromRigidMatrix4(centerTRight),
    },
  };
}

function rigidTransformResidual(reference, reconstructed, label) {
  validateMatrix4(reference, `${label}.reference`);
  validateMatrix4(reconstructed, `${label}.reconstructed`);
  const translationM = Math.hypot(
    reconstructed[0][3] - reference[0][3],
    reconstructed[1][3] - reference[1][3],
    reconstructed[2][3] - reference[2][3],
  );
  const referenceTRotation = [
    [reference[0][0], reference[1][0], reference[2][0]],
    [reference[0][1], reference[1][1], reference[2][1]],
    [reference[0][2], reference[1][2], reference[2][2]],
  ];
  const relativeRotation = Array.from({ length: 3 }, (_unused, row) => (
    Array.from({ length: 3 }, (_unusedColumn, column) => (
      referenceTRotation[row].reduce(
        (total, value, index) => total + value * reconstructed[index][column],
        0,
      )
    ))
  ));
  const cosine = Math.max(-1, Math.min(1, (
    relativeRotation[0][0]
    + relativeRotation[1][1]
    + relativeRotation[2][2]
    - 1
  ) / 2));
  const sine = Math.min(1, Math.max(0, 0.5 * Math.hypot(
    relativeRotation[2][1] - relativeRotation[1][2],
    relativeRotation[0][2] - relativeRotation[2][0],
    relativeRotation[1][0] - relativeRotation[0][1],
  )));
  return {
    translationM,
    orientationRad: Math.atan2(sine, cosine),
  };
}

// Verify the editable RPY representation by reconstructing both world-tip
// transforms through the same composition used by the renderer. This is a
// model-consistency check only; it does not add a TCP or calibration offset.
export function verifyGraspFrameAlignment({
  centerPose,
  draft,
  worldTLeft,
  worldTRight,
  translationToleranceM = GRASP_FRAME_ALIGNMENT_TRANSLATION_TOLERANCE_M,
  orientationToleranceRad = GRASP_FRAME_ALIGNMENT_ORIENTATION_TOLERANCE_RAD,
}) {
  finiteNumber(translationToleranceM, "translationToleranceM");
  finiteNumber(orientationToleranceRad, "orientationToleranceRad");
  if (translationToleranceM < 0 || orientationToleranceRad < 0) {
    throw new RangeError("alignment tolerances must be non-negative");
  }
  validateMatrix4(worldTLeft, "worldTLeft");
  validateMatrix4(worldTRight, "worldTRight");
  const reconstructed = computeWorldGraspFrameMatrices(centerPose, draft);
  const left = rigidTransformResidual(
    worldTLeft,
    reconstructed.worldTLeft,
    "left",
  );
  const right = rigidTransformResidual(
    worldTRight,
    reconstructed.worldTRight,
    "right",
  );
  const maxTranslationM = Math.max(left.translationM, right.translationM);
  const maxOrientationRad = Math.max(left.orientationRad, right.orientationRad);
  return {
    ok: maxTranslationM <= translationToleranceM
      && maxOrientationRad <= orientationToleranceRad,
    left,
    right,
    maxTranslationM,
    maxOrientationRad,
    translationToleranceM,
    orientationToleranceRad,
    reconstructed: {
      worldTLeft: reconstructed.worldTLeft,
      worldTRight: reconstructed.worldTRight,
    },
  };
}

function normalizedGraspMatrices(configuration = SYNTHETIC_DEFAULT_GRASP_DRAFT) {
  const normalized = normalizeRigidGraspDraft(configuration);
  return {
    normalized,
    objectTLeft: matrix4FromTranslationRpy(normalized.left),
    objectTRight: matrix4FromTranslationRpy(normalized.right),
  };
}

export function computeObjectGraspRelativeState(
  configuration = SYNTHETIC_DEFAULT_GRASP_DRAFT,
) {
  const { normalized, objectTLeft, objectTRight } = normalizedGraspMatrices(
    configuration,
  );
  const leftTRightMatrix = multiplyMatrix4(
    inverseRigidMatrix4(objectTLeft),
    objectTRight,
  );
  return {
    objectTLeft: {
      translationM: [...normalized.left.translationM],
      rpyRad: [...normalized.left.rpyRad],
    },
    objectTRight: {
      translationM: [...normalized.right.translationM],
      rpyRad: [...normalized.right.rpyRad],
    },
    leftTRight: poseFromRigidMatrix4(leftTRightMatrix),
    translationUnit: "meter",
    orientationUnit: "radian",
    frameNotation: {
      objectTLeft: "^O T_L",
      objectTRight: "^O T_R",
      leftTRight: "^L T_R = inverse(^O T_L) * ^O T_R",
    },
    semanticSource: "OPERATOR-CONFIGURABLE RIGID GRASP",
    orientationConvention: "R = Rz(yaw) * Ry(pitch) * Rx(roll)",
  };
}

export function computeWorldGraspFrameMatricesFromWorldTObject(
  worldTObject,
  configuration = SYNTHETIC_DEFAULT_GRASP_DRAFT,
) {
  const { objectTLeft, objectTRight } = normalizedGraspMatrices(configuration);
  return {
    worldTObject,
    // ^W T_L = ^W T_O * ^O T_L; ^O T_L remains fixed.
    worldTLeft: multiplyMatrix4(worldTObject, objectTLeft),
    // ^W T_R = ^W T_O * ^O T_R; ^O T_R remains fixed.
    worldTRight: multiplyMatrix4(worldTObject, objectTRight),
  };
}

export function computeWorldGraspFrameMatrices(
  objectPose,
  configuration = SYNTHETIC_DEFAULT_GRASP_DRAFT,
) {
  return computeWorldGraspFrameMatricesFromWorldTObject(
    matrix4FromTranslationRpy(objectPose),
    configuration,
  );
}
