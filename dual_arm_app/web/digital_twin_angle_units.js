// Pure presentation helpers for operator-facing rotation values.
// Canonical Digital Twin poses, planning requests, TF, IK, and validation stay radians.

export const ANGLE_UNIT_DEGREES = "degrees";
export const ANGLE_UNIT_RADIANS = "radians";
export const DEFAULT_ANGLE_UNIT = ANGLE_UNIT_DEGREES;

const METADATA = Object.freeze({
  [ANGLE_UNIT_DEGREES]: Object.freeze({
    value: ANGLE_UNIT_DEGREES,
    choiceLabel: "Degrees (°)",
    symbol: "°",
    shortLabel: "deg",
    spokenLabel: "degrees",
    min: -360,
    max: 360,
    step: 0.1,
    fractionDigits: 8,
  }),
  [ANGLE_UNIT_RADIANS]: Object.freeze({
    value: ANGLE_UNIT_RADIANS,
    choiceLabel: "Radians (rad)",
    symbol: "rad",
    shortLabel: "rad",
    spokenLabel: "radians",
    min: -2 * Math.PI,
    max: 2 * Math.PI,
    step: 0.01,
    fractionDigits: 10,
  }),
});

export function normalizeAngleUnit(value) {
  return value === ANGLE_UNIT_RADIANS ? ANGLE_UNIT_RADIANS : ANGLE_UNIT_DEGREES;
}

export function angleUnitMetadata(unit) {
  return METADATA[normalizeAngleUnit(unit)];
}

export function radiansToDegrees(value) {
  return Number(value) * 180 / Math.PI;
}

export function degreesToRadians(value) {
  return Number(value) * Math.PI / 180;
}

export function rotationRadiansToDisplay(valueRad, unit) {
  return normalizeAngleUnit(unit) === ANGLE_UNIT_DEGREES
    ? radiansToDegrees(valueRad) : Number(valueRad);
}

export function rotationDisplayToRadians(value, unit) {
  return normalizeAngleUnit(unit) === ANGLE_UNIT_DEGREES
    ? degreesToRadians(value) : Number(value);
}

export function presentCanonicalRadians(valuesRad, unit) {
  if (!Array.isArray(valuesRad)) {
    throw new TypeError("canonical rotation values must be an array");
  }
  return valuesRad.map((value, index) => {
    if (typeof value !== "number" || !Number.isFinite(value)) {
      throw new TypeError(`canonical rotation values[${index}] must be finite radians`);
    }
    return rotationRadiansToDisplay(value, unit);
  });
}

export function formatRotationRadians(valueRad, unit) {
  const metadata = angleUnitMetadata(unit);
  const displayed = rotationRadiansToDisplay(valueRad, unit);
  const normalized = Math.abs(displayed) < 5e-12 ? 0 : displayed;
  return normalized.toFixed(metadata.fractionDigits);
}
