"use strict";

const LIST_ENDPOINT = "/api/digital-twin/center-paths";
const SAVE_ENDPOINT = "/api/digital-twin/center-path/save";
const LOAD_ENDPOINT = "/api/digital-twin/center-path/load";
const RENAME_ENDPOINT = "/api/digital-twin/center-path/rename";
const DELETE_ENDPOINT = "/api/digital-twin/center-path/delete";

async function requestJson(url, options, fetchImpl) {
  const response = await fetchImpl(url, options);
  let payload = null;
  try {
    payload = await response.json();
  } catch (_error) {
    throw new Error(`Center Path Library returned non-JSON response (${response.status})`);
  }
  if (!response.ok || !payload || payload.ok !== true) {
    const detail = payload && payload.error ? payload.error : `HTTP ${response.status}`;
    throw new Error(detail);
  }
  return payload;
}

function postOptions(payload) {
  return {
    method: "POST",
    headers: {"Content-Type": "application/json"},
    body: JSON.stringify(payload),
  };
}
export async function listCenterPaths(fetchImpl = fetch) {
  return requestJson(LIST_ENDPOINT, {method: "GET"}, fetchImpl);
}

export async function saveCenterPath(name, path, overwrite = true, fetchImpl = fetch) {
  return requestJson(
    SAVE_ENDPOINT,
    postOptions({name, path, overwrite: Boolean(overwrite)}),
    fetchImpl,
  );
}

export async function loadCenterPath(name, fetchImpl = fetch) {
  return requestJson(LOAD_ENDPOINT, postOptions({name}), fetchImpl);
}

export async function renameCenterPath(name, newName, fetchImpl = fetch) {
  return requestJson(
    RENAME_ENDPOINT,
    postOptions({name, new_name: newName}),
    fetchImpl,
  );
}

export async function deleteCenterPath(name, fetchImpl = fetch) {
  return requestJson(DELETE_ENDPOINT, postOptions({name}), fetchImpl);
}
