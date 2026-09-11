"""Fixed localhost operator-control HTTP routes; no ROS/session ownership here."""
from fastapi import APIRouter, HTTPException
from pydantic import BaseModel, ConfigDict, StrictBool

from .operator_runtime import RuntimeRejected


class StateRequest(BaseModel):
    model_config = ConfigDict(extra="forbid")
    enabled: StrictBool


def system_control_router(node):
    router = APIRouter(prefix="/api/system-control", tags=["system-control"])

    def guarded(action):
        try:
            return action()
        except RuntimeRejected as error:
            raise HTTPException(status_code=409, detail=str(error)) from error
        except (OSError, TimeoutError) as error:
            raise HTTPException(status_code=503, detail=str(error)) from error

    @router.get("/status")
    def status():
        return guarded(node.operator_status)

    @router.post("/connect-robots")
    def connect_robots():
        return guarded(node.operator_connect)

    @router.post("/robot/{side}/power")
    def robot_power(side: str, request: StateRequest):
        if side not in {"left", "right"}:
            raise HTTPException(status_code=422, detail="side must be left or right")
        return guarded(lambda: node.operator_control(side, "power", request.enabled))

    @router.post("/robot/{side}/enable")
    def robot_enable(side: str, request: StateRequest):
        if side not in {"left", "right"}:
            raise HTTPException(status_code=422, detail="side must be left or right")
        return guarded(lambda: node.operator_control(side, "enable", request.enabled))

    @router.post("/reset-moveit")
    def reset_moveit():
        return guarded(node.operator_reset_moveit)

    return router
