"""State tracking for a sequential waypoint mission."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Iterable, Optional, Sequence, Tuple


Waypoint = Tuple[float, float, float]


def parse_waypoints(values: Optional[Iterable[float]]) -> Tuple[Waypoint, ...]:
    """Convert a flat x/y/z array into validated waypoints."""
    if values is None:
        raise ValueError("waypoints must contain one or more x, y, z triples")
    flat_values = tuple(float(value) for value in values)
    if not flat_values or len(flat_values) % 3 != 0:
        raise ValueError("waypoints must contain one or more x, y, z triples")
    if not all(math.isfinite(value) for value in flat_values):
        raise ValueError("waypoints must contain only finite values")
    return tuple(
        (flat_values[index], flat_values[index + 1], flat_values[index + 2])
        for index in range(0, len(flat_values), 3)
    )


@dataclass(frozen=True)
class ProgressEvent:
    """Result of observing the robot position once."""

    reached_index: int
    next_index: Optional[int]
    completed: bool
    looped: bool = False


class WaypointSequence:
    """Advance a waypoint queue after a stable odometry-based arrival."""

    def __init__(
        self,
        waypoints: Sequence[Waypoint],
        xy_tolerance: float,
        z_tolerance: float,
        hold_time: float,
        loop: bool,
    ) -> None:
        if not waypoints:
            raise ValueError("at least one waypoint is required")
        if not math.isfinite(xy_tolerance) or xy_tolerance <= 0.0:
            raise ValueError("xy_tolerance must be greater than zero")
        if not math.isfinite(z_tolerance):
            raise ValueError("z_tolerance must be finite")
        if not math.isfinite(hold_time) or hold_time < 0.0:
            raise ValueError("hold_time must be non-negative")

        self.waypoints = tuple(waypoints)
        self.xy_tolerance = xy_tolerance
        self.z_tolerance = z_tolerance
        self.hold_time = hold_time
        self.loop = loop
        self.current_index = 0
        self.completed = False
        self._current_confirmed = False
        self._arrival_started_at: Optional[float] = None

    @property
    def current_waypoint(self) -> Optional[Waypoint]:
        if self.completed:
            return None
        return self.waypoints[self.current_index]

    @property
    def current_confirmed(self) -> bool:
        return self._current_confirmed

    def confirm_current_waypoint(self) -> bool:
        """Mark the current goal as accepted by the global planning pipeline."""
        if self.completed or self._current_confirmed:
            return False
        self._current_confirmed = True
        return True


    def reset_current_confirmation(self) -> bool:
        """Require a fresh global path for the current waypoint."""
        if self.completed:
            return False

        self._current_confirmed = False
        self._arrival_started_at = None
        return True

    def observe(
        self, position: Waypoint, timestamp_seconds: float
    ) -> Optional[ProgressEvent]:
        """Observe a position and return an event only when a waypoint is reached."""
        if self.completed or not self._current_confirmed:
            return None
        if not math.isfinite(timestamp_seconds):
            raise ValueError("timestamp_seconds must be finite")

        target = self.waypoints[self.current_index]
        xy_distance = math.hypot(position[0] - target[0], position[1] - target[1])
        z_reached = (
            self.z_tolerance < 0.0
            or abs(position[2] - target[2]) <= self.z_tolerance
        )
        if xy_distance > self.xy_tolerance or not z_reached:
            self._arrival_started_at = None
            return None

        if self._arrival_started_at is None or timestamp_seconds < self._arrival_started_at:
            self._arrival_started_at = timestamp_seconds
        if timestamp_seconds - self._arrival_started_at < self.hold_time:
            return None

        reached_index = self.current_index
        self._arrival_started_at = None
        self._current_confirmed = False
        if reached_index + 1 < len(self.waypoints):
            self.current_index += 1
            return ProgressEvent(reached_index, self.current_index, completed=False)

        if self.loop:
            self.current_index = 0
            return ProgressEvent(reached_index, 0, completed=False, looped=True)

        self.completed = True
        return ProgressEvent(reached_index, None, completed=True)
