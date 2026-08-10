"""Offline source tests for the Phase 1B Static Digital Twin."""

from __future__ import annotations

import math
import re
import subprocess
import unittest
import xml.etree.ElementTree as ET
from html.parser import HTMLParser
from pathlib import Path
from urllib.parse import urljoin, urlparse


REPOSITORY_ROOT = Path(__file__).resolve().parents[2]
APP_ROOT = REPOSITORY_ROOT / "dual_arm_app"
URDF_PATH = APP_ROOT / "web/assets/dual_jaka_a12_web.urdf"
HTML_PATH = APP_ROOT / "web/index.html"
JAVASCRIPT_PATH = APP_ROOT / "web/digital_twin.js"
BACKEND_PATH = APP_ROOT / "backend/dual_jaka_web_backend.py"
MESH_DIRECTORY = (
    REPOSITORY_ROOT
    / "src/jaka_ros2/src/jaka_description/meshes/jaka_a12_meshes"
)
URDF_URL = (
    "http://127.0.0.1:8000/"
    "digital-twin/assets/dual_jaka_a12_web.urdf"
)
EXPECTED_JOINTS = {
    f"{side}_joint_{index}"
    for side in ("left", "right")
    for index in range(1, 7)
}
EXPECTED_LINKS = {
    "world",
    "left_base_link",
    "right_base_link",
    *(f"left_J{index}" for index in range(1, 7)),
    *(f"right_J{index}" for index in range(1, 7)),
}


class _IdCollector(HTMLParser):
    def __init__(self) -> None:
        super().__init__()
        self.ids: list[str] = []

    def handle_starttag(self, _tag: str, attrs: list[tuple[str, str | None]]) -> None:
        for name, value in attrs:
            if name == "id" and value is not None:
                self.ids.append(value)


class StaticDigitalTwinTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.urdf_text = URDF_PATH.read_text(encoding="utf-8")
        cls.urdf_root = ET.fromstring(cls.urdf_text)
        cls.html = HTML_PATH.read_text(encoding="utf-8")
        cls.javascript = JAVASCRIPT_PATH.read_text(encoding="utf-8")
        cls.backend = BACKEND_PATH.read_text(encoding="utf-8")

    def test_generated_urdf_is_valid_xml_with_expected_robot_name(self) -> None:
        self.assertEqual(self.urdf_root.tag, "robot")
        self.assertEqual(self.urdf_root.get("name"), "dual_jaka_a12")

    def test_generated_urdf_contains_expected_links_and_joints(self) -> None:
        links = {element.get("name") for element in self.urdf_root.findall("link")}
        joints = {element.get("name") for element in self.urdf_root.findall("joint")}
        self.assertTrue(EXPECTED_LINKS.issubset(links))
        self.assertTrue(EXPECTED_JOINTS.issubset(joints))

    def test_generated_urdf_uses_only_relative_web_mesh_references(self) -> None:
        self.assertNotIn("package://", self.urdf_text)
        mesh_urls = [
            element.get("filename")
            for element in self.urdf_root.findall(".//mesh")
        ]
        self.assertTrue(mesh_urls)
        self.assertTrue(
            all(
                url is not None
                and re.fullmatch(r"\.\./meshes/(?:base_link|J[1-6])\.STL", url)
                for url in mesh_urls
            )
        )

    def test_mesh_urls_resolve_to_the_mesh_static_route(self) -> None:
        mesh_urls = {
            element.get("filename")
            for element in self.urdf_root.findall(".//mesh")
        }
        for mesh_url in mesh_urls:
            with self.subTest(mesh_url=mesh_url):
                resolved = urljoin(URDF_URL, mesh_url)
                parsed = urlparse(resolved)
                self.assertEqual(parsed.scheme, "http")
                self.assertEqual(parsed.netloc, "127.0.0.1:8000")
                self.assertEqual(
                    parsed.path,
                    f"/digital-twin/meshes/{Path(mesh_url).name}",
                )
                self.assertNotIn(
                    "/digital-twin/assets//digital-twin/",
                    resolved,
                )

    def test_every_referenced_stl_exists(self) -> None:
        mesh_urls = {
            element.get("filename")
            for element in self.urdf_root.findall(".//mesh")
        }
        missing = sorted(
            url
            for url in mesh_urls
            if url is None or not (MESH_DIRECTORY / Path(url).name).is_file()
        )
        self.assertEqual(missing, [])

    def test_existing_controls_and_new_panel_remain_in_index(self) -> None:
        preserved_labels = (
            "Live Position / Direct Move",
            "Direct Joint Move",
            "Direct TCP Move",
            "Dual JAKA A12 Manual Jog",
            "STOP BOTH",
            "Home Both",
            "Refresh Status",
            "Waypoint Manager",
            "Program / Sequence",
            "System Status",
        )
        for label in preserved_labels:
            with self.subTest(label=label):
                self.assertIn(label, self.html)

        self.assertIn("Dual-Arm Digital Twin", self.html)
        self.assertIn('id="digitalTwinViewer"', self.html)
        self.assertIn('id="digitalTwinStatus"', self.html)

    def test_digital_twin_panel_has_no_execute_button(self) -> None:
        match = re.search(
            r'<section class="panel" aria-labelledby="digitalTwinTitle">(.*?)</section>',
            self.html,
            flags=re.DOTALL,
        )
        self.assertIsNotNone(match)
        self.assertNotRegex(match.group(1), r"<button[^>]*>\s*Execute\b")

    def test_html_ids_are_not_duplicated(self) -> None:
        parser = _IdCollector()
        parser.feed(self.html)
        duplicates = sorted({item for item in parser.ids if parser.ids.count(item) > 1})
        self.assertEqual(duplicates, [])

    def test_javascript_has_expected_joints_and_no_backend_calls(self) -> None:
        for joint_name in EXPECTED_JOINTS:
            with self.subTest(joint=joint_name):
                self.assertIn(f'"{joint_name}"', self.javascript)
        self.assertNotIn("/api/status", self.javascript)
        self.assertNotIn("/api/", self.javascript)
        self.assertNotIn("fetch(", self.javascript)

    def test_javascript_uses_two_independent_builtin_urdf_loading_flows(self) -> None:
        self.assertEqual(self.javascript.count("loader.load("), 2)
        self.assertEqual(self.javascript.count("loader.parseCollision = false"), 2)
        self.assertIn("state.manager.onLoad", self.javascript)
        self.assertIn("Timed out waiting for ${role} model readiness", self.javascript)
        self.assertNotIn("loadMeshCb", self.javascript)
        self.assertNotIn("STLLoader", self.javascript)
        self.assertNotIn("MAX_FINALIZE_ATTEMPTS", self.javascript)

    def test_namespaced_api_is_exposed(self) -> None:
        self.assertIn("window.dualArmDigitalTwin", self.javascript)
        for function_name in (
            "resetCamera",
            "fitModel",
            "toggleGrid",
            "toggleAxes",
            "setJointValues",
            "getLoadState",
        ):
            with self.subTest(function=function_name):
                self.assertIn(function_name, self.javascript)

    def test_viewer_zoom_is_normalized_bounded_and_has_no_duplicate_wheel_handler(self) -> None:
        initialize = self.javascript.split("function initialize()", 1)[1]
        self.assertIn("new THREE.PerspectiveCamera(45, 1, 0.001, 1000)", initialize)
        self.assertIn("new OrbitControls(camera, renderer.domElement)", initialize)
        self.assertIn("controls.enableZoom = false", initialize)
        self.assertIn(
            'renderer.domElement.addEventListener("wheel", handleViewerWheel, '
            "{ passive: false })",
            initialize,
        )

        zoom_rate = re.search(
            r"const NORMALIZED_WHEEL_ZOOM_RATE = ([0-9.]+);",
            self.javascript,
        )
        minimum_ratio = re.search(
            r"const ORBIT_MIN_DISTANCE_RADIUS_MULTIPLIER = ([0-9.]+);",
            self.javascript,
        )
        maximum_ratio = re.search(
            r"const ORBIT_MAX_DISTANCE_RADIUS_MULTIPLIER = ([0-9.]+);",
            self.javascript,
        )
        self.assertIsNotNone(zoom_rate)
        self.assertIsNotNone(minimum_ratio)
        self.assertIsNotNone(maximum_ratio)
        self.assertGreater(float(zoom_rate.group(1)), 0)
        self.assertLessEqual(float(zoom_rate.group(1)), 0.05)
        self.assertGreater(float(minimum_ratio.group(1)), 0)
        self.assertLess(float(minimum_ratio.group(1)), float(maximum_ratio.group(1)))
        fit_distance_per_max_dimension = 0.65 / math.tan(math.radians(45 * 0.5))
        largest_possible_radius_per_max_dimension = math.sqrt(3) * 0.5
        smallest_possible_radius_per_max_dimension = 0.5
        self.assertLess(
            float(minimum_ratio.group(1)) * largest_possible_radius_per_max_dimension,
            fit_distance_per_max_dimension,
        )
        self.assertLess(
            fit_distance_per_max_dimension,
            float(maximum_ratio.group(1)) * smallest_possible_radius_per_max_dimension,
        )

        fit_model = self.javascript.split("function fitModel", 1)[1].split(
            "function resetCamera", 1
        )[0]
        self.assertIn("bounds.getBoundingSphere(new THREE.Sphere())", fit_model)
        self.assertIn(
            "controls.minDistance = sceneRadius * ORBIT_MIN_DISTANCE_RADIUS_MULTIPLIER",
            fit_model,
        )
        self.assertIn(
            "controls.maxDistance = sceneRadius * ORBIT_MAX_DISTANCE_RADIUS_MULTIPLIER",
            fit_model,
        )
        self.assertEqual(
            len(re.findall(r"addEventListener\(\s*[\"']wheel[\"']", self.javascript)),
            1,
        )
        self.assertIn("function normalizedWheelSteps", self.javascript)
        self.assertIn("Math.max(-1, Math.min(1, deltaY / divisor))", self.javascript)
        self.assertNotIn("onwheel", self.javascript)

        reset_camera = self.javascript.split("function resetCamera", 1)[1].split(
            "function toggleGrid", 1
        )[0]
        self.assertIn("camera.position.copy(homeCameraPosition)", reset_camera)
        self.assertIn("controls.target.copy(homeCameraTarget)", reset_camera)
        self.assertIn("controls.update()", reset_camera)

    def test_static_routes_and_existing_index_route_are_present(self) -> None:
        for route in (
            '"/digital-twin/assets"',
            '"/digital-twin/meshes"',
            '"/web-assets"',
        ):
            with self.subTest(route=route):
                self.assertIn(route, self.backend)
                self.assertEqual(self.backend.count(route), 1)
        self.assertIn("StaticFiles", self.backend)
        self.assertIn('@app.get("/")', self.backend)
        self.assertIn("FileResponse", self.backend)

    def test_user_waypoint_data_has_no_unstaged_change(self) -> None:
        result = subprocess.run(
            [
                "git",
                "diff",
                "--quiet",
                "--",
                "dual_arm_app/tasks/waypoints.json",
            ],
            cwd=REPOSITORY_ROOT,
            check=False,
        )
        self.assertEqual(
            result.returncode,
            0,
            "Phase 1B must not alter waypoint data relative to the existing index",
        )


if __name__ == "__main__":
    unittest.main()
