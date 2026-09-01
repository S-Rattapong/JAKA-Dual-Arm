// Read-only transport for the developer-owned Planning Start State config.
// This source only retrieves offline planning metadata; it never commands a robot.

export const PLANNING_START_STATE_ENDPOINT =
  "/api/digital-twin/planning-start-state";

export async function requestPlanningStartState(
  fetchImpl = globalThis.fetch,
) {
  if (typeof fetchImpl !== "function") {
    throw new Error("Planning Start State transport is unavailable");
  }
  const response = await fetchImpl(PLANNING_START_STATE_ENDPOINT, {
    method: "GET",
    headers: { Accept: "application/json" },
  });
  if (!response.ok) {
    throw new Error(`Planning Start State HTTP ${response.status}`);
  }
  try {
    return await response.json();
  } catch (_error) {
    throw new Error("Planning Start State returned invalid JSON");
  }
}
