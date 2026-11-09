"""Server web di Ziphius: pagina di controllo + WebSocket, protetti da PIN.

Accesso: POST /login con il PIN -> cookie di sessione (HttpOnly, SameSite=Strict).
Dopo 5 PIN errati da uno stesso indirizzo, blocco di 60 s.
Nota: in LAN la connessione e' HTTP in chiaro; il PIN protegge da accessi casuali,
non da chi intercetta il traffico sulla stessa rete.
"""

import asyncio
import contextlib
import hmac
import json
import logging
import os
import secrets
import time

from fastapi import FastAPI, Form, Request, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse, RedirectResponse
from fastapi.staticfiles import StaticFiles

log = logging.getLogger("ziphius.web")

STATIC = os.path.join(os.path.dirname(__file__), "static")
COOKIE = "ziphius_session"
MAX_FAILS, LOCKOUT_S = 5, 60


class Auth:
    def __init__(self, pin):
        self.pin = str(pin)
        self.sessions = set()
        self.fails = {}  # ip -> (conteggio, istante del blocco)

    def locked(self, ip):
        n, since = self.fails.get(ip, (0, 0))
        if n >= MAX_FAILS and time.monotonic() - since < LOCKOUT_S:
            return True
        if n >= MAX_FAILS:
            self.fails.pop(ip, None)
        return False

    def login(self, ip, pin):
        if self.locked(ip):
            return None
        if hmac.compare_digest(str(pin).strip(), self.pin):
            self.fails.pop(ip, None)
            token = secrets.token_urlsafe(32)
            self.sessions.add(token)
            return token
        n, _ = self.fails.get(ip, (0, 0))
        self.fails[ip] = (n + 1, time.monotonic())
        return None

    def valid(self, token):
        return token is not None and token in self.sessions


def create_app(controller, manage_lifecycle=True):
    auth = Auth(controller.config["pin"])

    @contextlib.asynccontextmanager
    async def lifespan(app):
        if manage_lifecycle:
            await controller.start()
        yield
        if manage_lifecycle:
            await controller.close()

    app = FastAPI(title="Ziphius", docs_url=None, redoc_url=None, openapi_url=None, lifespan=lifespan)
    app.state.controller, app.state.auth = controller, auth

    app.mount("/static", StaticFiles(directory=STATIC), name="static")

    def authed(request):
        return auth.valid(request.cookies.get(COOKIE))

    @app.get("/")
    async def index(request: Request):
        if not authed(request):
            return RedirectResponse("/login", status_code=303)
        return FileResponse(os.path.join(STATIC, "index.html"))

    @app.get("/login")
    async def login_page():
        return FileResponse(os.path.join(STATIC, "login.html"))

    @app.post("/login")
    async def login(request: Request, pin: str = Form(...)):
        ip = request.client.host if request.client else "?"
        if auth.locked(ip):
            return RedirectResponse("/login?e=locked", status_code=303)
        token = auth.login(ip, pin)
        if token is None:
            await asyncio.sleep(1)  # rallenta i tentativi
            return RedirectResponse("/login?e=pin", status_code=303)
        resp = RedirectResponse("/", status_code=303)
        resp.set_cookie(COOKIE, token, httponly=True, samesite="strict", max_age=30 * 24 * 3600)
        return resp

    @app.post("/logout")
    async def logout(request: Request):
        auth.sessions.discard(request.cookies.get(COOKIE))
        resp = RedirectResponse("/login", status_code=303)
        resp.delete_cookie(COOKIE)
        return resp

    @app.websocket("/ws")
    async def ws(websocket: WebSocket):
        if not auth.valid(websocket.cookies.get(COOKIE)):
            await websocket.close(code=4401)
            return
        await websocket.accept()
        outbox = asyncio.Queue()
        listener = outbox.put_nowait
        controller.listeners.add(listener)
        await websocket.send_text(json.dumps(controller.snapshot()))

        async def pump():
            while True:
                await websocket.send_text(json.dumps(await outbox.get()))

        sender = asyncio.create_task(pump())
        try:
            while True:
                msg = json.loads(await websocket.receive_text())
                try:
                    handle(controller, msg)
                except (ValueError, KeyError) as e:
                    outbox.put_nowait({"type": "log", "kind": "error", "line": f"richiesta non valida: {e}"})
        except WebSocketDisconnect:
            pass
        finally:
            sender.cancel()
            controller.listeners.discard(listener)

    return app


def handle(c, msg):
    """Azioni inviate dalla pagina."""
    from .. import protocol as P
    t = msg.get("type")
    if t == "stop":
        c.stop()
    elif t == "jog":
        c.jog(_motor(msg), int(msg["delta"]))
    elif t == "set":
        c.set_angle(_motor(msg), int(msg["angle"]))
    elif t == "laser":
        c.laser(bool(msg["on"]))
    elif t == "gesture":
        c.gesture(msg["name"])
    elif t == "cmd":
        # comandi brevi di configurazione (rand0, guard0, read, ...)
        c.command(*P.split_commands(msg["text"]))
    elif t == "raw":
        c.submit("console", P.split_commands(msg["text"]))
    elif t == "pose_go":
        c.go_pose(msg["name"])
    elif t == "pose_save":
        c.save_pose(msg["name"])
    elif t == "pose_delete":
        c.delete_pose(msg["name"])
    elif t == "job_pause":
        c.pause(int(msg["id"]))
    elif t == "job_resume":
        c.resume(int(msg["id"]))
    elif t == "job_cancel":
        c.cancel(int(msg["id"]))
    else:
        raise ValueError(f"tipo sconosciuto: {t}")


def _motor(msg):
    m = int(msg["motor"])
    if m not in range(4):
        raise ValueError(f"motore {m}")
    return m
