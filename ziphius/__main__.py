"""Riga di comando di Ziphius.

  python -m ziphius serve [--host 0.0.0.0] [--port 8042] [--serial auto]
  python -m ziphius send "P1lXw500XP1cXhello"
  python -m ziphius ports
  python -m ziphius pin [nuovo_pin]
"""

import argparse
import asyncio
import logging

from . import __version__
from . import protocol as P
from .config import Config
from .link import find_ports


def cmd_serve(cfg, args):
    import uvicorn
    from .controller import Controller
    from .web.server import create_app

    controller = Controller(cfg)
    app = create_app(controller)
    print(f"Ziphius {__version__}: http://{cfg['host']}:{cfg['port']}  (PIN in {cfg.path})")
    uvicorn.run(app, host=cfg["host"], port=cfg["port"], log_level="warning")


def cmd_send(cfg, args):
    from .controller import Controller

    async def run():
        c = Controller(cfg)
        done = asyncio.Event()

        def on_msg(m):
            if m["type"] == "log":
                print(m["line"])
            if m["type"] == "jobs" and all(j["status"] not in ("queued", "running") for j in m["jobs"]):
                done.set()

        c.listeners.add(on_msg)
        await c.start()
        c.submit("cli", P.split_commands(args.commands))
        try:
            await asyncio.wait_for(done.wait(), timeout=args.timeout)
        except asyncio.TimeoutError:
            print("Timeout.")
        await c.close()

    asyncio.run(run())


def main():
    ap = argparse.ArgumentParser(prog="ziphius", description="Controllo di Claudio")
    ap.add_argument("--config", help="file di configurazione (default ~/.config/ziphius/config.json)")
    ap.add_argument("-v", "--verbose", action="store_true")
    sub = ap.add_subparsers(dest="cmd", required=True)
    s = sub.add_parser("serve", help="avvia la pagina web")
    s.add_argument("--host")
    s.add_argument("--port", type=int)
    s.add_argument("--serial", help="porta seriale o 'auto'")
    s = sub.add_parser("send", help="invia comandi e attende la fine")
    s.add_argument("commands")
    s.add_argument("--serial")
    s.add_argument("--timeout", type=float, default=300)
    sub.add_parser("ports", help="elenca le porte seriali")
    s = sub.add_parser("pin", help="mostra o cambia il PIN della pagina web")
    s.add_argument("new_pin", nargs="?")
    args = ap.parse_args()

    logging.basicConfig(level=logging.DEBUG if args.verbose else logging.INFO,
                        format="%(asctime)s %(name)s %(levelname)s %(message)s")
    cfg = Config.load(args.config)
    for key in ("host", "port", "serial"):
        val = getattr(args, key, None)
        if val is not None:
            cfg["serial_port" if key == "serial" else key] = val

    if args.cmd == "serve":
        cmd_serve(cfg, args)
    elif args.cmd == "send":
        cmd_send(cfg, args)
    elif args.cmd == "ports":
        print("\n".join(find_ports()) or "nessuna porta trovata")
    elif args.cmd == "pin":
        if args.new_pin:
            if not (args.new_pin.isdigit() and len(args.new_pin) >= 4):
                raise SystemExit("Il PIN deve avere almeno 4 cifre.")
            cfg["pin"] = args.new_pin
            cfg.save()
        print(f"PIN: {cfg['pin']}")


if __name__ == "__main__":
    main()
