import asyncio


from ziphius.config import Config
from ziphius.controller import Controller
from tests.fake_arduino import FakeArduino


def make(fake=None, **kw):
    fake = fake or FakeArduino(**kw)
    c = Controller(Config(None, {"pin": "1234"}), transport_factory=lambda: fake)
    return c, fake


async def until(cond, timeout=5.0):
    end = asyncio.get_running_loop().time() + timeout
    while not cond():
        if asyncio.get_running_loop().time() > end:
            raise AssertionError("condizione non raggiunta")
        await asyncio.sleep(0.01)



async def test_connect_ready_and_init():
    c, fake = make()
    await c.start()
    await until(lambda: c.state.ready and c.state.positions[0] == 80)
    assert fake.received[0] == "delta1Xread"
    assert c.state.delta_ang == 1
    await c.close()



async def test_jogs_are_coalesced():
    c, fake = make(move_time=0.2)
    await c.start()
    await until(lambda: c.state.ready and not c.state.busy and c.state.positions[1] == 110)
    c.jog(1, 1)                       # parte subito
    await until(lambda: len(fake.received) == 2)
    for _ in range(10):               # mentre si muove: 10 jog -> un solo movimento
        c.jog(1, 1)
    await until(lambda: c.state.positions[1] == 121 and not c.state.busy)
    assert fake.received[1:] == ["1111", "1121"]
    await c.close()



async def test_job_packets_flow_control_and_pause():
    c, fake = make()
    await c.start()
    await until(lambda: c.state.ready)
    job = c.submit("t", [f"1{a}" for a in range(40, 100, 10)] + ["L1", "L0"])
    assert len(job.packets) == 2
    await until(lambda: job.status == "done")
    assert fake.executed[-2:] == ["L1", "L0"]
    job2 = c.submit("t2", ["0100"] * 3 + ["0110"] * 3)
    c.pause(job2.id)
    await asyncio.sleep(0.1)
    assert job2.sent == 0
    c.resume(job2.id)
    await until(lambda: job2.status == "done")
    await c.close()



async def test_stop_cancels_everything():
    c, fake = make(move_time=0.1)
    await c.start()
    await until(lambda: c.state.ready)
    job = c.submit("lungo", [f"1{a}" for a in range(30, 110, 10)] * 3)
    await until(lambda: job.sent >= 1 or job.status == "running")
    c.laser(True)
    c.stop()
    await until(lambda: "stop" in fake.received)
    await asyncio.sleep(0.3)
    assert job.status == "cancelled" and not c.state.laser
    await until(lambda: fake.received[-1] == "read")
    await c.close()



async def test_reconnect_after_disconnect():
    fakes = [FakeArduino(), FakeArduino()]
    it = iter(fakes)
    c = Controller(Config(None, {"pin": "1"}), transport_factory=lambda: next(it))
    c.link.reconnect_delay = 0.05
    await c.start()
    await until(lambda: c.state.ready)
    fakes[0].closed = True
    await until(lambda: not c.state.connected)
    await until(lambda: c.state.ready, timeout=3)
    assert fakes[1].received[0] == "delta1Xread"
    await c.close()



async def test_poses(tmp_path):
    fake = FakeArduino()
    c = Controller(Config(str(tmp_path / "c.json"), {"pin": "1"}), transport_factory=lambda: fake)
    await c.start()
    await until(lambda: c.state.ready and c.state.positions[3] == 65)
    c.save_pose("prova", [100, 60, 90, 40])
    assert (tmp_path / "c.json").exists()
    c.go_pose("prova")
    await until(lambda: c.state.positions == [100, 60, 90, 40])
    await c.close()
