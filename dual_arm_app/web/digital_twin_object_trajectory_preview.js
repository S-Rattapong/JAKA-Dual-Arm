// Phase 1F.2 pure, offline object Cartesian trajectory preview logic.
// This reuses Phase 1E.2's ^A T_B column-vector convention and fixed grasp
// transforms. It does not represent joint motion or executable robot motion.
//
// For input sample count N, duration T [s], and endpoint translations p0/p1 [m]:
//   alpha_i = i/(N-1)                 for sample index i = 0..N-1
//   t_i = alpha_i T                   output time from start in seconds
//   p_i = (1-alpha_i)p0 + alpha_i p1 output world translation in meters
// The initial 3x3 object rotation is copied to every output; orientation is constant.
// RPY is never interpolated. Fixed object-relative grasp inputs
// produce Cartesian preview outputs through:
//   ^W T_L(i) = ^W T_O(i) ^O T_L
//   ^W T_R(i) = ^W T_O(i) ^O T_R
// This assumes a rigid grasp and is only a Cartesian visualization. It provides
// no IK, joint-trajectory, collision, feasibility, or execution guarantee.
import {
  computeWorldGraspFrameMatricesFromWorldTObject,
  matrix4FromTranslationRpy,
} from "./digital_twin_object_grasp_preview.js";

export const SYNTHETIC_OBJECT_TRAJECTORY_NOTICE = (
  "OFFLINE SYNTHETIC OBJECT TRAJECTORY — NOT FOR ROBOT EXECUTION"
);

export const SYNTHETIC_OBJECT_TRAJECTORY_DEFINITION = Object.freeze({
  name: SYNTHETIC_OBJECT_TRAJECTORY_NOTICE,
  startTranslationM: Object.freeze([0.0, 0.0, 0.8]),
  startRpyRad: Object.freeze([0.1, -0.2, 0.3]),
  endTranslationM: Object.freeze([0.4, 0.0, 1.0]),
  durationS: 4.0,
  sampleCount: 5,
  translationOnly: true,
});

function freezeMatrix4(matrix) {
  return Object.freeze(matrix.map((row) => Object.freeze([...row])));
}

function freezePose(pose) {
  return Object.freeze({
    translationM: Object.freeze([...pose.translationM]),
    rpyRad: Object.freeze([...pose.rpyRad]),
  });
}

function finiteNumber(value, label) {
  if (typeof value !== "number" || !Number.isFinite(value)) {
    throw new TypeError(`${label} must be a finite number`);
  }
  return value;
}

function finiteVector3(values, label) {
  if (!Array.isArray(values) || values.length !== 3) {
    throw new TypeError(`${label} must contain exactly three finite numbers`);
  }
  return values.map((value, index) => finiteNumber(value, `${label}[${index}]`));
}

export function buildTranslationOnlyObjectTrajectory(definition) {
  if (!definition || typeof definition !== "object" || Array.isArray(definition)) {
    throw new TypeError("trajectory definition must be an object");
  }
  if (typeof definition.name !== "string" || !definition.name.trim()) {
    throw new TypeError("trajectory name must be a non-empty string");
  }
  const startTranslationM = finiteVector3(
    definition.startTranslationM,
    "startTranslationM",
  );
  const startRpyRad = finiteVector3(definition.startRpyRad, "startRpyRad");
  const endTranslationM = finiteVector3(
    definition.endTranslationM,
    "endTranslationM",
  );
  const durationS = finiteNumber(definition.durationS, "durationS");
  if (durationS <= 0) throw new RangeError("durationS must be greater than zero");
  const sampleCount = definition.sampleCount;
  if (!Number.isInteger(sampleCount) || sampleCount < 2) {
    throw new RangeError("sampleCount must be an integer of at least 2");
  }

  const initialWorldTObject = matrix4FromTranslationRpy({
    translationM: startTranslationM,
    rpyRad: startRpyRad,
  });
  const samples = Array.from({ length: sampleCount }, (_unused, index) => {
    // i is 0..N-1. alpha_i is a unitless fraction and t_i is seconds.
    const alpha = index / (sampleCount - 1);
    const timeFromStartS = alpha * durationS;
    // p_i=(1-alpha_i)p0+alpha_i*p1; all translations are world-frame meters.
    const translationM = startTranslationM.map(
      (startValue, axis) => (
        (1 - alpha) * startValue + alpha * endTranslationM[axis]
      ),
    );
    // Copy the initial 3x3 rotation exactly. RPY is never interpolated.
    const worldTObject = initialWorldTObject.map((row, rowIndex) => (
      rowIndex < 3
        ? [row[0], row[1], row[2], translationM[rowIndex]]
        : [0, 0, 0, 1]
    ));
    // ^W T_L(i)=^W T_O(i)^O T_L and ^W T_R(i)=^W T_O(i)^O T_R.
    // These are derived Cartesian preview outputs, never independent paths.
    const frames = computeWorldGraspFrameMatricesFromWorldTObject(worldTObject);
    return Object.freeze({
      index,
      alpha,
      timeFromStartS,
      objectPose: freezePose({ translationM, rpyRad: startRpyRad }),
      worldTObject: freezeMatrix4(frames.worldTObject),
      worldTLeft: freezeMatrix4(frames.worldTLeft),
      worldTRight: freezeMatrix4(frames.worldTRight),
    });
  });

  return Object.freeze({
    name: definition.name.trim(),
    durationS,
    sampleCount,
    translationOnly: true,
    samples: Object.freeze(samples),
  });
}

export const SYNTHETIC_TRANSLATION_ONLY_OBJECT_TRAJECTORY = (
  buildTranslationOnlyObjectTrajectory(SYNTHETIC_OBJECT_TRAJECTORY_DEFINITION)
);

export function objectTrajectorySampleAtIndex(trajectory, index) {
  if (!trajectory || !Array.isArray(trajectory.samples)) {
    throw new TypeError("trajectory must contain samples");
  }
  if (!Number.isInteger(index) || index < 0 || index >= trajectory.samples.length) {
    throw new RangeError("object trajectory sample index is out of range");
  }
  return trajectory.samples[index];
}

export function nearestObjectTrajectorySample(trajectory, requestedTimeS) {
  if (!trajectory || !Array.isArray(trajectory.samples) || !trajectory.samples.length) {
    throw new TypeError("trajectory must contain samples");
  }
  const checkedTimeS = finiteNumber(requestedTimeS, "requestedTimeS");
  const clampedTimeS = Math.max(0, Math.min(trajectory.durationS, checkedTimeS));
  // Discrete policy: nearest sample; exact ties deterministically choose earlier.
  return trajectory.samples.reduce((nearest, candidate) => {
    const nearestDistance = Math.abs(nearest.timeFromStartS - clampedTimeS);
    const candidateDistance = Math.abs(candidate.timeFromStartS - clampedTimeS);
    return candidateDistance < nearestDistance ? candidate : nearest;
  });
}
