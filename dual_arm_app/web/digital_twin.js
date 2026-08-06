// Phase 1B pinned browser dependencies:
// Three.js 0.160.0 and urdf-loader 0.12.5 (resolved by index.html import map).
import * as THREE from "three";
import { OrbitControls } from "three/addons/controls/OrbitControls.js";
import URDFLoader from "urdf-loader";

const MODEL_URL = "/digital-twin/assets/dual_jaka_a12_web.urdf";
const LOAD_TIMEOUT_MS = 20000;
const INITIALIZATION_FLAG = "__dualArmDigitalTwinPhase1BInitialized";
const EXPECTED_JOINTS = [
  "left_joint_1",
  "left_joint_2",
  "left_joint_3",
  "left_joint_4",
  "left_joint_5",
  "left_joint_6",
  "right_joint_1",
  "right_joint_2",
  "right_joint_3",
  "right_joint_4",
  "right_joint_5",
  "right_joint_6",
];

const loadState = {
  status: "INITIALIZING",
  loadedMovableJoints: 0,
  loadedVisuals: 0,
  error: null,
};

let container = null;
let statusElement = null;
let scene = null;
let camera = null;
let renderer = null;
let controls = null;
let grid = null;
let axes = null;
let robot = null;
let homeCameraPosition = null;
let homeCameraTarget = null;

function setStatus(message, kind = "info") {
  if (loadState.status === "ERROR" && kind !== "error") return;
  loadState.status = kind === "error" ? "ERROR" : loadState.status;
  if (statusElement) {
    statusElement.textContent = message;
    statusElement.dataset.state = kind;
  }
}

function fail(message, error = null) {
  const detail = error && error.message ? `${message}: ${error.message}` : message;
  loadState.status = "ERROR";
  loadState.error = detail;
  setStatus(detail, "error");
  console.error(`[DualArmDigitalTwin] ${detail}`, error || "");
}

function render() {
  if (renderer && scene && camera) {
    renderer.render(scene, camera);
  }
}

function fitModel(rememberAsHome = false) {
  if (!robot || !camera || !controls || !container) {
    return false;
  }

  robot.updateMatrixWorld(true);
  const bounds = new THREE.Box3().setFromObject(robot);
  if (bounds.isEmpty()) {
    fail("Cannot fit camera because the model bounds are empty");
    return false;
  }

  const size = bounds.getSize(new THREE.Vector3());
  const center = bounds.getCenter(new THREE.Vector3());
  const maxDimension = Math.max(size.x, size.y, size.z, 0.1);
  const halfFovRadians = THREE.MathUtils.degToRad(camera.fov * 0.5);
  const distance = (maxDimension * 0.65) / Math.tan(halfFovRadians);
  const viewDirection = new THREE.Vector3(1, -1, 0.7).normalize();

  controls.target.copy(center);
  camera.position.copy(center).addScaledVector(viewDirection, distance);
  camera.near = Math.max(distance / 1000, 0.001);
  camera.far = Math.max(distance * 100, 100);
  camera.aspect = container.clientWidth / Math.max(container.clientHeight, 1);
  camera.updateProjectionMatrix();
  controls.update();

  if (rememberAsHome || !homeCameraPosition) {
    homeCameraPosition = camera.position.clone();
    homeCameraTarget = controls.target.clone();
  }
  render();
  return true;
}

function resetCamera() {
  if (!camera || !controls || !homeCameraPosition || !homeCameraTarget) {
    return false;
  }
  camera.position.copy(homeCameraPosition);
  controls.target.copy(homeCameraTarget);
  controls.update();
  render();
  return true;
}

function toggleGrid() {
  if (!grid) return false;
  grid.visible = !grid.visible;
  render();
  return grid.visible;
}

function toggleAxes() {
  if (!axes) return false;
  axes.visible = !axes.visible;
  render();
  return axes.visible;
}

function validateJointArray(side, values) {
  if (!Array.isArray(values) || values.length !== 6) {
    throw new TypeError(`${side} must be an array of exactly six radians values`);
  }
  if (!values.every((value) => typeof value === "number" && Number.isFinite(value))) {
    throw new TypeError(`${side} must contain six finite numbers`);
  }
}

function setJointValues(values) {
  if (!robot) {
    throw new Error("Digital Twin model is not ready");
  }
  if (!values || typeof values !== "object") {
    throw new TypeError("Joint values must contain left and right arrays");
  }

  validateJointArray("left", values.left);
  validateJointArray("right", values.right);
  for (const side of ["left", "right"]) {
    values[side].forEach((value, index) => {
      const jointName = `${side}_joint_${index + 1}`;
      const joint = robot.joints[jointName];
      if (!joint) {
        throw new Error(`Model joint is unavailable: ${jointName}`);
      }
      joint.setJointValue(value);
    });
  }
  robot.updateMatrixWorld(true);
  render();
  return true;
}

function getLoadState() {
  return { ...loadState };
}

const publicApi = {
  resetCamera,
  fitModel: () => fitModel(false),
  toggleGrid,
  toggleAxes,
  setJointValues,
  getLoadState,
};

function handleResize() {
  if (!container || !camera || !renderer) return;
  const width = Math.max(container.clientWidth, 1);
  const height = Math.max(container.clientHeight, 1);
  camera.aspect = width / height;
  camera.updateProjectionMatrix();
  renderer.setSize(width, height, false);
  renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
  render();
}

function bindControls() {
  const bindings = {
    digitalTwinResetCamera: resetCamera,
    digitalTwinFitModel: () => fitModel(false),
    digitalTwinToggleGrid: toggleGrid,
    digitalTwinToggleAxes: toggleAxes,
  };
  Object.entries(bindings).forEach(([id, handler]) => {
    const element = document.getElementById(id);
    if (element) element.addEventListener("click", handler);
  });
}

function loadRobot() {
  const manager = new THREE.LoadingManager();
  const failedAssets = new Set();
  let urdfParsed = false;
  let allAssetsLoaded = false;
  let finalized = false;

  const loadTimeout = window.setTimeout(() => {
    if (!finalized && loadState.status !== "ERROR") {
      finalized = true;
      fail("Timed out waiting for Digital Twin visual meshes");
    }
  }, LOAD_TIMEOUT_MS);

  function countVisualMeshes(model) {
    let count = 0;
    model.traverse((object) => {
      const geometry = object.geometry;
      const positions = geometry && geometry.attributes
        ? geometry.attributes.position
        : null;
      if (object.isMesh && positions && positions.count > 0) count += 1;
    });
    return count;
  }

  function finalizeWhenComplete() {
    if (finalized || !urdfParsed || !allAssetsLoaded || !robot) return;

    if (failedAssets.size > 0) {
      finalized = true;
      window.clearTimeout(loadTimeout);
      fail(`Failed to load asset(s): ${Array.from(failedAssets).join(", ")}`);
      return;
    }

    robot.updateMatrixWorld(true);
    const visualCount = countVisualMeshes(robot);
    const bounds = new THREE.Box3().setFromObject(robot);

    if (visualCount === 0 || bounds.isEmpty()) {
      finalized = true;
      window.clearTimeout(loadTimeout);
      fail(
        "URDF loaded, but no renderable visual meshes produced non-empty bounds"
      );
      return;
    }

    if (!fitModel(true)) {
      window.clearTimeout(loadTimeout);
      return;
    }

    finalized = true;
    window.clearTimeout(loadTimeout);
    loadState.status = "READY";
    loadState.loadedMovableJoints = EXPECTED_JOINTS.length;
    loadState.loadedVisuals = visualCount;
    loadState.error = null;
    setStatus(
      `READY — ${EXPECTED_JOINTS.length} MOVABLE JOINTS LOADED` +
      ` — ${visualCount} VISUALS`,
      "ready",
    );
  }

  manager.onProgress = (_url, loaded = 0, total = 0) => {
    const percent = total > 0 ? Math.round((loaded / total) * 100) : 0;
    setStatus(`LOADING ASSETS ${loaded}/${total} (${percent}%)`);
  };

  manager.onError = (url) => {
    failedAssets.add(String(url || "unknown asset"));
    console.error(`[DualArmDigitalTwin] Asset loading error: ${url}`);
  };

  manager.onLoad = () => {
    allAssetsLoaded = true;
    finalizeWhenComplete();
  };

  const loader = new URDFLoader(manager);
  loader.parseCollision = false;

  loadState.status = "LOADING";
  setStatus("LOADING DUAL JAKA A12 URDF");

  loader.load(
    MODEL_URL,
    (loadedRobot) => {
      const missingJoints = EXPECTED_JOINTS.filter(
        (jointName) => !loadedRobot.joints[jointName],
      );

      if (missingJoints.length > 0) {
        finalized = true;
        window.clearTimeout(loadTimeout);
        fail(`URDF is missing expected joints: ${missingJoints.join(", ")}`);
        return;
      }

      robot = loadedRobot;

      EXPECTED_JOINTS.forEach((jointName) => {
        robot.joints[jointName].setJointValue(0);
      });

      scene.add(robot);
      robot.updateMatrixWorld(true);
      urdfParsed = true;
      setStatus("URDF PARSED — WAITING FOR VISUAL MESHES");
      finalizeWhenComplete();
    },
    (event) => {
      if (event && event.lengthComputable && event.total > 0) {
        const percent = Math.round((event.loaded / event.total) * 100);
        setStatus(`LOADING URDF ${percent}%`);
      }
    },
    (error) => {
      window.clearTimeout(loadTimeout);
      fail("Failed to load Digital Twin URDF", error);
    },
  );
}

function initialize() {
  container = document.getElementById("digitalTwinViewer");
  statusElement = document.getElementById("digitalTwinStatus");
  if (!container) {
    fail("Digital Twin viewer container is missing");
    return;
  }

  THREE.Object3D.DEFAULT_UP.set(0, 0, 1);
  scene = new THREE.Scene();
  scene.background = new THREE.Color(0x0b1120);
  camera = new THREE.PerspectiveCamera(45, 1, 0.001, 1000);

  try {
    renderer = new THREE.WebGLRenderer({ antialias: true, alpha: false });
  } catch (error) {
    fail("WebGL initialization failed", error);
    return;
  }
  renderer.outputColorSpace = THREE.SRGBColorSpace;
  container.replaceChildren(renderer.domElement);

  controls = new OrbitControls(camera, renderer.domElement);
  controls.enableDamping = true;
  controls.dampingFactor = 0.08;
  controls.addEventListener("change", render);

  scene.add(new THREE.AmbientLight(0xffffff, 1.6));
  const directionalLight = new THREE.DirectionalLight(0xffffff, 2.2);
  directionalLight.position.set(3, -4, 6);
  scene.add(directionalLight);

  grid = new THREE.GridHelper(6, 30, 0x475569, 0x273449);
  grid.rotation.x = Math.PI / 2;
  scene.add(grid);

  axes = new THREE.AxesHelper(0.75);
  scene.add(axes);

  bindControls();
  window.addEventListener("resize", handleResize);
  if (typeof ResizeObserver !== "undefined") {
    new ResizeObserver(handleResize).observe(container);
  }
  handleResize();

  const animate = () => {
    requestAnimationFrame(animate);
    if (controls) controls.update();
    render();
  };
  animate();
  loadRobot();
}

if (window[INITIALIZATION_FLAG]) {
  console.warn("[DualArmDigitalTwin] Duplicate module initialization ignored");
} else {
  window[INITIALIZATION_FLAG] = true;
  window.dualArmDigitalTwin = publicApi;
  initialize();
}
