// Phase 1D.3B validation-only HTTP source. This module never sends robot commands.

export const MOVEIT_VALIDATION_ENDPOINT = "/api/digital-twin/validate-trajectory";
export const DEFAULT_MOVEIT_VALIDATION_TIMEOUT_MS = 30000;

export async function requestMoveItTrajectoryValidation(
  trajectory,
  {
    fetchImpl = globalThis.fetch,
    timeoutMs = DEFAULT_MOVEIT_VALIDATION_TIMEOUT_MS,
  } = {},
) {
  if (typeof fetchImpl !== "function") {
    throw new Error("MoveIt validation transport is unavailable");
  }
  const controller = new AbortController();
  const timeoutId = setTimeout(() => controller.abort(), timeoutMs);
  try {
    const response = await fetchImpl(MOVEIT_VALIDATION_ENDPOINT, {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ trajectory }),
      signal: controller.signal,
    });
    let payload;
    try {
      payload = await response.json();
    } catch (_error) {
      throw new Error("MoveIt validation returned invalid JSON");
    }
    if (!response.ok) {
      const detail = payload && (payload.detail || payload.error);
      throw new Error(detail || `MoveIt validation HTTP ${response.status}`);
    }
    if (!payload || typeof payload !== "object" || !payload.validation) {
      throw new Error("MoveIt validation response is malformed");
    }
    return payload;
  } catch (error) {
    if (error && error.name === "AbortError") {
      const timeoutError = new Error("MoveIt validation request timed out");
      timeoutError.code = "TIMEOUT";
      throw timeoutError;
    }
    throw error;
  } finally {
    clearTimeout(timeoutId);
  }
}
