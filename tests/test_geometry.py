import math

from ziphius.geometry import WallProjector

WALL = {"K": 3.0, "pan_center": 70, "tilt_center": 90, "pan_gain": 1.0, "tilt_gain": -1.0,
        "arm_pose": 50, "step_deg": 2.0, "max_active_servos": 1}


def test_round_trip():
    w = WallProjector(WALL)
    for u, v in [(0, 0), (0.5, 0.2), (-0.7, -0.4), (1.0, 0.8)]:
        pan, tilt = w.wall_to_angles(u, v)
        uu, vv = w.angles_to_wall(pan, tilt)
        assert math.isclose(u, uu, abs_tol=1e-9) and math.isclose(v, vv, abs_tol=1e-9)


def test_center_and_resolution():
    w = WallProjector(WALL)
    assert w.wall_to_angles(0, 0) == (70, 90)
    assert math.isclose(w.resolution(), 3 * math.tan(math.radians(1)))


def test_plan_one_motor_at_a_time_and_laser():
    w = WallProjector(WALL, {"0": (60, 120), "1": (30, 110)})
    cmds, path = w.plan_strokes([[(0, 0), (0.5, 0.3)], [(-0.3, 0), (-0.3, 0.3)]])
    assert cmds[0] == "L0" and cmds[-1] == "L0"
    assert cmds.count("L1") == 2
    # ogni passo cambia un solo motore e al massimo di step_deg (arrotondato)
    for (p0, t0, on0), (p1, t1, on1) in zip(path, path[1:]):
        if on0 and on1:
            assert (p0 == p1) or (t0 == t1)
            assert abs(p1 - p0) <= 3 and abs(t1 - t0) <= 3
    # tra le due spezzate il laser e' spento durante lo spostamento
    i = cmds.index("L1", cmds.index("L1") + 1)
    assert cmds[i - 3] == "L0"
    assert w.estimate_time(path) > 0
