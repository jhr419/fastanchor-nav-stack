import pytest

from navigation_bringup.waypoint_sequence import WaypointSequence, parse_waypoints


def test_parse_waypoints_accepts_xyz_triples():
    assert parse_waypoints([1.0, 2.0, 0.3, 4.0, 5.0, 0.4]) == (
        (1.0, 2.0, 0.3),
        (4.0, 5.0, 0.4),
    )


@pytest.mark.parametrize(
    "values", [None, [], [1.0, 2.0], [1.0, 2.0, float("nan")]]
)
def test_parse_waypoints_rejects_invalid_input(values):
    with pytest.raises(ValueError):
        parse_waypoints(values)


def test_sequence_requires_stable_arrival_before_advancing():
    sequence = WaypointSequence(
        [(1.0, 0.0, 0.3), (2.0, 0.0, 0.3)],
        xy_tolerance=0.5,
        z_tolerance=-1.0,
        hold_time=0.5,
        loop=False,
    )
    sequence.confirm_current_waypoint()

    assert sequence.observe((1.2, 0.0, 2.0), 1.0) is None
    assert sequence.observe((1.8, 0.0, 0.3), 1.2) is None
    assert sequence.observe((1.1, 0.0, 0.3), 1.3) is None
    event = sequence.observe((1.1, 0.0, 0.3), 1.8)

    assert event is not None
    assert event.reached_index == 0
    assert event.next_index == 1
    assert not event.completed
    assert sequence.current_waypoint == (2.0, 0.0, 0.3)


def test_sequence_completes_after_last_waypoint():
    sequence = WaypointSequence(
        [(1.0, 0.0, 0.3)],
        xy_tolerance=0.5,
        z_tolerance=0.2,
        hold_time=0.0,
        loop=False,
    )
    sequence.confirm_current_waypoint()

    assert sequence.observe((1.0, 0.0, 0.8), 1.0) is None
    event = sequence.observe((1.0, 0.0, 0.4), 1.1)

    assert event is not None and event.completed
    assert sequence.current_waypoint is None
    assert sequence.observe((1.0, 0.0, 0.3), 2.0) is None


def test_sequence_can_loop():
    sequence = WaypointSequence(
        [(1.0, 0.0, 0.3)],
        xy_tolerance=0.5,
        z_tolerance=-1.0,
        hold_time=0.0,
        loop=True,
    )
    sequence.confirm_current_waypoint()

    event = sequence.observe((1.0, 0.0, 0.3), 1.0)

    assert event is not None and event.looped
    assert not sequence.completed
    assert sequence.current_index == 0


def test_each_waypoint_requires_a_separate_global_path_confirmation():
    sequence = WaypointSequence(
        [(1.0, 0.0, 0.3), (2.0, 0.0, 0.3)],
        xy_tolerance=0.5,
        z_tolerance=-1.0,
        hold_time=0.0,
        loop=False,
    )

    assert sequence.observe((1.0, 0.0, 0.3), 1.0) is None
    assert sequence.confirm_current_waypoint()
    first_event = sequence.observe((1.0, 0.0, 0.3), 1.1)
    assert first_event is not None and first_event.next_index == 1

    assert not sequence.current_confirmed
    assert sequence.observe((2.0, 0.0, 0.3), 2.0) is None
    assert sequence.confirm_current_waypoint()
    second_event = sequence.observe((2.0, 0.0, 0.3), 2.1)
    assert second_event is not None and second_event.completed


def test_reset_current_confirmation_requires_fresh_path():
    sequence = WaypointSequence(
        waypoints=((1.0, 0.0, 0.0),),
        xy_tolerance=0.5,
        z_tolerance=-1.0,
        hold_time=0.0,
        loop=False,
    )

    assert sequence.confirm_current_waypoint()
    assert sequence.current_confirmed

    assert sequence.reset_current_confirmation()
    assert not sequence.current_confirmed

    # Being physically inside the tolerance is not enough after resume.
    # A fresh global path must confirm the waypoint again.
    event = sequence.observe((1.0, 0.0, 0.0), 1.0)
    assert event is None

    assert sequence.confirm_current_waypoint()

    event = sequence.observe((1.0, 0.0, 0.0), 2.0)
    assert event is not None
    assert event.completed
