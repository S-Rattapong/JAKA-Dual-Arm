from __future__ import annotations

import pytest
from fastapi import HTTPException
from pydantic import ValidationError

from dual_arm_app.backend.operator_api import StateRequest, system_control_router
from dual_arm_app.backend.operator_runtime import RuntimeRejected


class FakeNode:
    def __init__(self):
        self.calls = []
        self.failure = None

    def _result(self, name, *args):
        self.calls.append((name, *args))
        if self.failure is not None:
            raise self.failure
        return {"method": name, "args": list(args)}

    def operator_status(self):
        return self._result("status")

    def operator_connect(self):
        return self._result("connect")

    def operator_control(self, side, control, enabled):
        return self._result("control", side, control, enabled)

    def operator_reset_moveit(self):
        return self._result("reset")


def endpoint(router, path, method):
    for route in router.routes:
        if route.path == path and method.upper() in route.methods:
            return route.endpoint
    raise AssertionError(f"missing {method} {path}")


def test_router_exposes_only_fixed_system_control_surface_and_forwards_calls():
    node = FakeNode()
    router = system_control_router(node)
    paths = {(method, route.path) for route in router.routes for method in route.methods}
    expected = {
        ("GET", "/api/system-control/status"),
        ("POST", "/api/system-control/connect-robots"),
        ("POST", "/api/system-control/robot/{side}/power"),
        ("POST", "/api/system-control/robot/{side}/enable"),
        ("POST", "/api/system-control/reset-moveit"),
    }
    assert paths == expected

    assert endpoint(router, "/api/system-control/status", "GET")()["method"] == "status"
    assert endpoint(router, "/api/system-control/connect-robots", "POST")()["method"] == "connect"
    endpoint(router, "/api/system-control/robot/{side}/power", "POST")("left", StateRequest(enabled=False))
    endpoint(router, "/api/system-control/robot/{side}/enable", "POST")("right", StateRequest(enabled=True))
    endpoint(router, "/api/system-control/reset-moveit", "POST")()
    assert node.calls == [
        ("status",),
        ("connect",),
        ("control", "left", "power", False),
        ("control", "right", "enable", True),
        ("reset",),
    ]


def test_state_request_is_strict_boolean_and_forbids_extra_fields():
    assert StateRequest(enabled=True).enabled is True
    with pytest.raises(ValidationError):
        StateRequest(enabled=1)
    with pytest.raises(ValidationError):
        StateRequest(enabled="true")
    with pytest.raises(ValidationError):
        StateRequest(enabled=True, command="rm -rf /")


def test_invalid_side_is_rejected_before_node_control_call():
    node = FakeNode()
    router = system_control_router(node)
    power = endpoint(router, "/api/system-control/robot/{side}/power", "POST")
    with pytest.raises(HTTPException) as caught:
        power("both", StateRequest(enabled=True))
    assert caught.value.status_code == 422
    assert node.calls == []


def test_runtime_rejection_maps_to_conflict_and_oserror_maps_to_unavailable():
    node = FakeNode()
    router = system_control_router(node)
    status = endpoint(router, "/api/system-control/status", "GET")

    node.failure = RuntimeRejected("unsafe transition")
    with pytest.raises(HTTPException) as caught:
        status()
    assert caught.value.status_code == 409
    assert "unsafe transition" in str(caught.value.detail)

    node.failure = OSError("systemd unavailable")
    with pytest.raises(HTTPException) as caught:
        status()
    assert caught.value.status_code == 503
    assert "systemd unavailable" in str(caught.value.detail)


def test_router_contains_no_command_or_service_name_request_fields():
    fields = StateRequest.model_fields
    assert set(fields) == {"enabled"}
