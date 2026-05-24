from __future__ import annotations

from dataclasses import dataclass
from enum import Enum, auto
from typing import Optional


class MissionState(Enum):
    IDLE = auto()
    PRE_FLIGHT = auto()
    TAKEOFF = auto()
    TRANSIT = auto()
    INSPECT = auto()
    REPLAN = auto()
    SAFE_HOVER = auto()
    RETURN_HOME = auto()
    LAND = auto()
    EMERGENCY_LAND = auto()


@dataclass
class MissionContext:
    mission_started: bool = False
    altitude: float = 0.0
    target_altitude: float = 1.5
    distance_to_target: float = 10.0
    distance_to_home: float = 10.0
    inspection_complete: bool = False
    battery_percent: float = 100.0
    obstacle_detected: bool = False
    localization_confidence: float = 1.0
    replanning_succeeded: bool = False
    critical_fault: bool = False


class MissionManager:
    """
    A small mission-level FSM scaffold for a PyBullet quadcopter lab.

    This is intentionally simple:
    - it does not control the drone directly
    - it decides which mission state should be active
    - the simulation loop should map each state to a high-level action
    """

    def __init__(self) -> None:
        self.state = MissionState.IDLE

    def update(self, ctx: MissionContext) -> MissionState:
        if ctx.critical_fault:
            self._transition(MissionState.EMERGENCY_LAND, "critical fault")
            return self.state

        if ctx.battery_percent <= 10.0 and self.state is not MissionState.LAND:
            self._transition(MissionState.EMERGENCY_LAND, "battery critical")
            return self.state

        if self.state == MissionState.IDLE:
            if ctx.mission_started:
                self._transition(MissionState.PRE_FLIGHT, "mission started")

        elif self.state == MissionState.PRE_FLIGHT:
            self._transition(MissionState.TAKEOFF, "pre-flight checks passed")

        elif self.state == MissionState.TAKEOFF:
            if ctx.altitude >= ctx.target_altitude:
                self._transition(MissionState.TRANSIT, "target altitude reached")

        elif self.state == MissionState.TRANSIT:
            if ctx.localization_confidence < 0.4:
                self._transition(MissionState.SAFE_HOVER, "localization confidence low")
            elif ctx.obstacle_detected:
                self._transition(MissionState.REPLAN, "obstacle detected")
            elif ctx.battery_percent < 30.0:
                self._transition(MissionState.RETURN_HOME, "battery low")
            elif ctx.distance_to_target < 0.5:
                self._transition(MissionState.INSPECT, "inspection zone reached")

        elif self.state == MissionState.INSPECT:
            if ctx.localization_confidence < 0.4:
                self._transition(MissionState.SAFE_HOVER, "localization confidence low")
            elif ctx.battery_percent < 25.0:
                self._transition(MissionState.RETURN_HOME, "battery low during inspection")
            elif ctx.inspection_complete:
                self._transition(MissionState.RETURN_HOME, "inspection complete")

        elif self.state == MissionState.REPLAN:
            if ctx.replanning_succeeded:
                self._transition(MissionState.TRANSIT, "new route available")
            elif ctx.battery_percent < 25.0:
                self._transition(MissionState.RETURN_HOME, "battery low while replanning")

        elif self.state == MissionState.SAFE_HOVER:
            if ctx.localization_confidence >= 0.7:
                self._transition(MissionState.RETURN_HOME, "localization recovered, return home")
            elif ctx.battery_percent < 20.0:
                self._transition(MissionState.EMERGENCY_LAND, "hover no longer safe")

        elif self.state == MissionState.RETURN_HOME:
            if ctx.distance_to_home < 0.5:
                self._transition(MissionState.LAND, "home reached")

        return self.state

    def command_hint(self) -> str:
        """
        A simple high-level action suggestion that can be mapped
        to the student's existing PyBullet controller.
        """
        if self.state == MissionState.IDLE:
            return "hold_on_ground"
        if self.state == MissionState.PRE_FLIGHT:
            return "run_preflight_checks"
        if self.state == MissionState.TAKEOFF:
            return "climb_to_target_altitude"
        if self.state == MissionState.TRANSIT:
            return "fly_to_target_waypoint"
        if self.state == MissionState.INSPECT:
            return "hold_position_for_inspection"
        if self.state == MissionState.REPLAN:
            return "compute_new_route"
        if self.state == MissionState.SAFE_HOVER:
            return "hold_current_pose"
        if self.state == MissionState.RETURN_HOME:
            return "fly_home"
        if self.state == MissionState.LAND:
            return "descend_and_land"
        if self.state == MissionState.EMERGENCY_LAND:
            return "emergency_descent"
        return "no_command"

    def _transition(self, new_state: MissionState, reason: str) -> None:
        if new_state != self.state:
            print(f"[MISSION] {self.state.name} -> {new_state.name} | reason: {reason}")
            self.state = new_state


def demo() -> None:
    """
    Small text-only demo so students can understand the state logic
    before wiring it into PyBullet.
    """
    manager = MissionManager()
    ctx = MissionContext()

    steps = [
        {"mission_started": True},
        {},
        {"altitude": 1.6},
        {"distance_to_target": 0.3},
        {"inspection_complete": True},
        {"distance_to_home": 0.2},
    ]

    for i, changes in enumerate(steps, start=1):
        for key, value in changes.items():
            setattr(ctx, key, value)
        state = manager.update(ctx)
        print(f"[STEP {i}] state={state.name} action={manager.command_hint()}")


if __name__ == "__main__":
    demo()
