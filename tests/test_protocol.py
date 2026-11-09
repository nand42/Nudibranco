import pytest

from ziphius import protocol as P
from ziphius.state import RobotState


def test_move_and_clamp():
    assert P.move(0, 100) == "0100"
    assert P.move(0, 200) == "0160"
    assert P.move(2, 10, force=True) == "F230"
    with pytest.raises(ValueError):
        P.move(7, 10)


def test_packets_respect_count_and_length():
    cmds = [f"1{a}" for a in range(40, 60)]
    pkts = P.packets(cmds)
    assert all(p.startswith("A") and p.endswith("Z") for p in pkts)
    assert all(p.count("X") <= 3 and len(p) <= P.MAX_PACKET_LEN for p in pkts)
    assert sum(p[1:-1].count("X") + 1 for p in pkts) == len(cmds)


def test_validate_rejects_reserved_chars():
    for bad in ("", "0X1", "AbcZ"):
        with pytest.raises(ValueError):
            P.validate(bad)


def test_split_commands():
    assert P.split_commands("AP1lXw500XP1cZ") == ["P1l", "w500", "P1c"]
    assert P.split_commands("hello") == ["hello"]


@pytest.mark.parametrize("line,kind,data", [
    ("READY", "ready", {}),
    ("IDLE", "idle", {}),
    ("ACK 12", "ack", {"queue": 12}),
    ("Motor 1 moved to 50", "moved", {"motor": 1, "angle": 50}),
    ("TOUCH 4", "touch", {"distance": 4}),
    ("PRESENCE 20", "presence", {"distance": 20}),
    ("ERROR Invalid angle.", "error", {"message": "Invalid angle."}),
    ("Laser ON", "laser_on", {}),
    ("Waiting for 500 ms", "info", {}),
])
def test_parse_line(line, kind, data):
    ev = P.parse_line(line)
    assert ev.kind == kind and ev.data == data


def test_positions_and_state():
    ev = P.parse_line("Positions: M0:80 M1:110 M2:50 M3:65 L:1 D:23 R:0 E:1 T:1 G:20000")
    assert ev.data["positions"] == [80, 110, 50, 65]
    s = RobotState()
    assert s.apply(ev)
    assert s.laser and s.distance == 23 and s.guard_ms == 20000
    assert s.apply(P.parse_line("TOUCH 5")) and s.zone == "touch" and not s.laser
    assert not s.apply(P.parse_line("Waiting for 1 ms"))
