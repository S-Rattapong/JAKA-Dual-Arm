"""Offline tests for PRE-P5.A1+A2 calibration and model consumption."""

from __future__ import annotations

import copy
import json
import math
import subprocess
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

from dual_arm_app.backend.world_frame_calibration import (
    CALIBRATION_STATE_PHYSICAL,
    DEFAULT_CALIBRATION_PATH,
    TCP_CONTRACT_TOOL0_J6_VERIFIED,
    WorldCalibrationError,
    compute_calibration_revision,
    load_world_calibration,
)
from dual_arm_app.tools.build_web_urdf import (
    METADATA_RELATIVE_PATH,
    XACRO_RELATIVE_PATH,
    generate_model_metadata,
    generate_urdf,
)


ROOT = Path(__file__).resolve().parents[2]
XACRO_PATH = ROOT / XACRO_RELATIVE_PATH
WEB_URDF_PATH = ROOT / "dual_arm_app/web/assets/dual_jaka_a12_web.urdf"
WEB_METADATA_PATH = ROOT / METADATA_RELATIVE_PATH
BACKEND_PATH = ROOT / "dual_arm_app/backend/dual_jaka_web_backend.py"
WEB_JS_PATH = ROOT / "dual_arm_app/web/digital_twin.js"
CALIBRATION_JS_PATH = ROOT / "dual_arm_app/web/digital_twin_world_calibration.js"
LAUNCH_DIRECTORY = (
    ROOT / "src/jaka_ros2/src/jaka_a12_moveit_config/launch"
)
EXPECTED_LEFT_XYZ = (-0.7596239842607274, 0.11058694383387384, 0.13651014459880412)
EXPECTED_LEFT_RPY = (1.574439006095868, 0.0015787065831678235, 3.1411559532164106)
EXPECTED_RIGHT_XYZ = (-0.7627012046262431, -0.07178515589567372, 0.13452249878565917)
EXPECTED_RIGHT_RPY = (-1.5711124814852797, 0.007331140961250616, -3.1411551591076234)
TEST_ARTIFACT_DIRECTORY = ROOT / ".a3_test_artifacts"
TEST_ARTIFACT_DIRECTORY.mkdir(parents=True, exist_ok=True)


def _artifact_data() -> dict:
    return json.loads(DEFAULT_CALIBRATION_PATH.read_text(encoding="utf-8"))


def _write_valid_revision(data: dict) -> None:
    data["revision"] = compute_calibration_revision(data)


def _load_temporary(data: dict):
    path = TEST_ARTIFACT_DIRECTORY / "world_calibration_case.json"
    path.write_text(json.dumps(data, allow_nan=True), encoding="utf-8")
    return load_world_calibration(path)


def _fixed_origin(root: ET.Element, name: str) -> tuple[tuple[float, ...], tuple[float, ...]]:
    joint = next(item for item in root.findall("joint") if item.get("name") == name)
    origin = joint.find("origin")
    return (
        tuple(float(value) for value in origin.get("xyz", "").split()),
        tuple(float(value) for value in origin.get("rpy", "").split()),
    )


class PureWorldCalibrationTests(unittest.TestCase):
    def test_physical_artifact_preserves_exact_calibrated_transforms(self):
        calibration = load_world_calibration()
        self.assertEqual(calibration.world_to_left_base.translation_m, EXPECTED_LEFT_XYZ)
        self.assertEqual(calibration.world_to_left_base.rotation_rpy_rad, EXPECTED_LEFT_RPY)
        self.assertEqual(calibration.world_to_right_base.translation_m, EXPECTED_RIGHT_XYZ)
        self.assertEqual(calibration.world_to_right_base.rotation_rpy_rad, EXPECTED_RIGHT_RPY)

    def test_revision_is_deterministic_and_matches_artifact(self):
        data = _artifact_data()
        self.assertEqual(compute_calibration_revision(data), data["revision"])
        reordered = {key: data[key] for key in reversed(tuple(data))}
        self.assertEqual(compute_calibration_revision(reordered), data["revision"])

    def test_transform_frame_and_tool_changes_change_revision(self):
        original = _artifact_data()
        variants = []
        transform = copy.deepcopy(original)
        transform["transforms"]["world_to_left_base"]["translation_m"][0] = 0.001
        variants.append(transform)
        frame = copy.deepcopy(original)
        frame["frames"]["world"] = "workcell_world"
        variants.append(frame)
        tool = copy.deepcopy(original)
        tool["tcp_tool_contract"]["planning_tip_frames"]["left"] = "left_tool0"
        variants.append(tool)
        for variant in variants:
            with self.subTest(variant=variant):
                self.assertNotEqual(
                    compute_calibration_revision(variant), original["revision"]
                )

    def test_invalid_schema_missing_field_and_invalid_units_are_rejected(self):
        cases = []
        schema = _artifact_data()
        schema["schema_version"] = 2
        cases.append(schema)
        missing = _artifact_data()
        del missing["frames"]["left_base"]
        cases.append(missing)
        units = _artifact_data()
        units["units"]["translation"] = "millimeter"
        cases.append(units)
        for data in cases:
            with self.subTest(data=data), self.assertRaises(WorldCalibrationError):
                _load_temporary(data)

    def test_nonfinite_boolean_and_invalid_transform_are_rejected(self):
        cases = []
        for invalid in (math.nan, math.inf, True):
            data = _artifact_data()
            data["transforms"]["world_to_left_base"]["translation_m"][0] = invalid
            cases.append(data)
        bad_shape = _artifact_data()
        bad_shape["transforms"]["world_to_right_base"]["rotation_rpy_rad"] = [0, 1]
        cases.append(bad_shape)
        for data in cases:
            with self.subTest(data=data), self.assertRaises(WorldCalibrationError):
                _load_temporary(data)

    def test_invalid_frame_and_stale_revision_are_rejected(self):
        invalid_frame = _artifact_data()
        invalid_frame["frames"]["world"] = "/invalid frame"
        with self.assertRaises(WorldCalibrationError):
            _load_temporary(invalid_frame)
        stale = _artifact_data()
        stale["transforms"]["world_to_right_base"]["translation_m"][1] = -0.08
        with self.assertRaisesRegex(WorldCalibrationError, "revision does not match"):
            _load_temporary(stale)

    def test_returned_state_is_immutable_and_public_payload_is_defensive_copy(self):
        calibration = load_world_calibration()
        with self.assertRaises(TypeError):
            calibration.xacro_mappings()["left_base_xyz"] = "changed"
        first = calibration.public_payload()
        first["transforms"]["world_to_left_base"]["translation_m"][0] = 99
        second = calibration.public_payload()
        self.assertEqual(second["transforms"]["world_to_left_base"]["translation_m"][0], EXPECTED_LEFT_XYZ[0])

    def test_physical_calibration_and_tcp_contract_are_truthful(self):
        calibration = load_world_calibration()
        self.assertEqual(calibration.calibration_state, CALIBRATION_STATE_PHYSICAL)
        self.assertTrue(calibration.physically_calibrated)
        self.assertEqual(calibration.physical_calibration, "CALIBRATED")
        self.assertEqual(calibration.tcp_tool_contract_status, TCP_CONTRACT_TOOL0_J6_VERIFIED)
        self.assertEqual(calibration.verification_status, "PHYSICALLY_CALIBRATED_SINGLE_PASS_NOT_REPEATABILITY_VERIFIED")


class CalibrationModelConsumptionTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.calibration = load_world_calibration()
        cls.xacro = XACRO_PATH.read_text(encoding="utf-8")
        cls.generated = generate_urdf(XACRO_PATH, cls.calibration)
        cls.generated_root = ET.fromstring(cls.generated)

    def test_xacro_is_parameterized_without_owned_base_constants(self):
        for argument in (
            "left_base_xyz", "left_base_rpy", "right_base_xyz", "right_base_rpy"
        ):
            self.assertIn(f'<xacro:arg name="{argument}" default=""/>', self.xacro)
            self.assertIn(f'$(arg {argument})', self.xacro)
        self.assertNotIn('base_xyz="0.0 0.075 1.2"', self.xacro)
        self.assertNotIn('base_rpy="1.570796326795 0.0 3.14159265359"', self.xacro)

    def test_parameterized_xacro_reproduces_current_fixed_transforms(self):
        for joint_name, expected_xyz, expected_rpy in (
            ("left_fixed", EXPECTED_LEFT_XYZ, EXPECTED_LEFT_RPY),
            ("right_fixed", EXPECTED_RIGHT_XYZ, EXPECTED_RIGHT_RPY),
        ):
            actual_xyz, actual_rpy = _fixed_origin(self.generated_root, joint_name)
            for actual, expected in zip(actual_xyz, expected_xyz):
                self.assertAlmostEqual(actual, expected, places=12)
            for actual, expected in zip(actual_rpy, expected_rpy):
                self.assertAlmostEqual(actual, expected, places=12)

    def test_checked_in_web_urdf_matches_canonical_generation_and_semantics(self):
        checked_in = WEB_URDF_PATH.read_text(encoding="utf-8")
        self.assertEqual(checked_in, self.generated)
        root = ET.fromstring(checked_in)
        self.assertEqual(root.get("name"), "dual_jaka_a12")
        self.assertEqual(len(root.findall("link")), 15)
        self.assertEqual(len(root.findall("joint")), 14)

    def test_model_metadata_binds_revision_without_copying_transforms(self):
        expected = json.loads(generate_model_metadata(self.calibration))
        actual = json.loads(WEB_METADATA_PATH.read_text(encoding="utf-8"))
        self.assertEqual(actual, expected)
        self.assertEqual(actual["calibration_revision"], self.calibration.revision)
        self.assertNotIn("transforms", actual)

    def test_moveit_and_web_paths_use_the_shared_loader(self):
        builder = (ROOT / "dual_arm_app/tools/build_web_urdf.py").read_text(encoding="utf-8")
        helper = (LAUNCH_DIRECTORY / "world_calibration_launch.py").read_text(encoding="utf-8")
        self.assertIn("load_world_calibration", builder)
        self.assertIn("load_world_calibration", helper)
        consumers = sorted(LAUNCH_DIRECTORY.glob("dual_*.launch.py"))
        calibrated_consumers = [
            path for path in consumers
            if "dual_jaka_a12.urdf.xacro" in path.read_text(encoding="utf-8")
        ]
        self.assertTrue(calibrated_consumers)
        for path in calibrated_consumers:
            with self.subTest(path=path.name):
                source = path.read_text(encoding="utf-8")
                self.assertIn("calibrated_robot_description(xacro_file)", source)

    def test_backend_read_only_endpoint_uses_canonical_payload(self):
        source = BACKEND_PATH.read_text(encoding="utf-8")
        self.assertIn('@app.get("/api/digital-twin/world-calibration")', source)
        self.assertIn("world_calibration_public_payload()", source)
        self.assertNotIn('@app.post("/api/digital-twin/world-calibration")', source)
        from dual_arm_app.tests.test_selective_scope_reset import _mocked_backend_import

        backend, cleanup = _mocked_backend_import()
        try:
            route = next(
                route.endpoint
                for route in backend.app.routes
                if route.path == "/api/digital-twin/world-calibration"
            )
            payload = route()
        finally:
            cleanup()
        self.assertTrue(payload["ok"])
        self.assertEqual(
            payload["calibration"]["revision"], self.calibration.revision
        )
        self.assertTrue(payload["calibration"]["physically_calibrated"])

    def test_revision_match_and_mismatch_are_detectable_and_gate_is_fail_closed(self):
        module_path = TEST_ARTIFACT_DIRECTORY / "world_calibration.mjs"
        module_path.write_text(
            CALIBRATION_JS_PATH.read_text(encoding="utf-8"), encoding="utf-8"
        )
        script = f"""
          import {{ normalizeCalibrationRevisionSources }} from {json.dumps(module_path.as_uri())};
          const revision = {json.dumps(self.calibration.revision)};
          const backend = {{ok:true, calibration:{{revision, calibration_state:'PHYSICAL_CALIBRATED', physically_calibrated:true}}}};
          const match = normalizeCalibrationRevisionSources(backend, {{calibration_revision:revision}});
          const mismatch = normalizeCalibrationRevisionSources(backend, {{calibration_revision:'sha256:different'}});
          console.log(JSON.stringify({{match, mismatch}}));
        """
        result = subprocess.run(
            ["node", "--input-type=module", "--eval", script],
            check=True,
            capture_output=True,
            text=True,
        )
        states = json.loads(result.stdout)
        self.assertTrue(states["match"]["planningReady"])
        self.assertFalse(states["mismatch"]["planningReady"])
        self.assertEqual(states["mismatch"]["status"], "MODEL/CALIBRATION REVISION MISMATCH")
        main_js = WEB_JS_PATH.read_text(encoding="utf-8")
        self.assertIn("MODEL_CALIBRATION_REVISION_MISMATCH", main_js)
        self.assertIn("executionReady: false", main_js)

    def test_invalid_artifact_fails_closed_for_consumers(self):
        data = _artifact_data()
        data["units"]["rotation"] = "degree"
        with self.assertRaises(WorldCalibrationError):
            _load_temporary(data)

    def test_new_production_code_contains_no_motion_surface(self):
        source = "\n".join(
            path.read_text(encoding="utf-8")
            for path in (
                ROOT / "dual_arm_app/backend/world_frame_calibration.py",
                ROOT / "dual_arm_app/tools/build_web_urdf.py",
                LAUNCH_DIRECTORY / "world_calibration_launch.py",
                CALIBRATION_JS_PATH,
            )
        )
        for forbidden in (
            "joint_move", "linear_move", "servo", "Jog", "Home",
            "/api/program/run",
        ):
            with self.subTest(forbidden=forbidden):
                self.assertNotIn(forbidden, source)


if __name__ == "__main__":
    unittest.main()
