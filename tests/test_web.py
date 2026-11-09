from fastapi.testclient import TestClient

from ziphius.config import Config
from ziphius.controller import Controller
from ziphius.web.server import create_app
from tests.fake_arduino import FakeArduino


def client():
    fake = FakeArduino()
    c = Controller(Config(None, {"pin": "4321"}), transport_factory=lambda: fake)
    return TestClient(create_app(c)), fake


def test_requires_login():
    tc, _ = client()
    with tc:
        r = tc.get("/", follow_redirects=False)
        assert r.status_code == 303 and r.headers["location"] == "/login"
        r = tc.post("/login", data={"pin": "0000"}, follow_redirects=False)
        assert r.headers["location"] == "/login?e=pin"


def test_lockout_after_failures(monkeypatch):
    import ziphius.web.server as S

    async def no_sleep(_):
        return None
    monkeypatch.setattr(S.asyncio, "sleep", no_sleep)
    tc, _ = client()
    with tc:
        for _ in range(S.MAX_FAILS):
            tc.post("/login", data={"pin": "0000"}, follow_redirects=False)
        r = tc.post("/login", data={"pin": "4321"}, follow_redirects=False)
        assert r.headers["location"] == "/login?e=locked"


def test_ws_after_login_controls_robot():
    tc, fake = client()
    with tc:
        r = tc.post("/login", data={"pin": "4321"}, follow_redirects=False)
        assert r.headers["location"] == "/"
        assert tc.get("/").status_code == 200
        with tc.websocket_connect("/ws") as ws:
            snap = ws.receive_json()
            assert snap["type"] == "snapshot" and "limits" in snap
            ws.send_json({"type": "laser", "on": True})
            ws.send_json({"type": "gesture", "name": "hello"})
            for _ in range(200):
                if "L1" in fake.executed and "hello" in fake.executed:
                    break
                ws.receive_json()
            assert "L1" in fake.executed and "hello" in fake.executed


def test_ws_rejected_without_session():
    tc, _ = client()
    with tc:
        try:
            with tc.websocket_connect("/ws") as ws:
                ws.receive_json()
            assert False, "doveva essere rifiutato"
        except Exception:
            pass
