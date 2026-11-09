"use strict";
// Pagina di controllo di Ziphius: stato via WebSocket, comandi come messaggi JSON.

const NAMES = ["M0 · Laser (verticale)", "M1 · Corpo (orizzontale)", "M2 · Braccio", "M3 · Pinza"];
const ZONES = { far: "nessuno vicino", presence: "presenza", touch: "TOCCO" };
let ws, state = {}, limits = {}, dragging = new Set();
const $ = (s) => document.querySelector(s);

function send(msg) {
  if (ws && ws.readyState === WebSocket.OPEN) ws.send(JSON.stringify(msg));
}

function connect() {
  const proto = location.protocol === "https:" ? "wss" : "ws";
  ws = new WebSocket(`${proto}://${location.host}/ws`);
  ws.onmessage = (e) => onMessage(JSON.parse(e.data));
  ws.onclose = (e) => {
    if (e.code === 4401) { location.href = "/login"; return; }
    setPill($("#conn"), "server non raggiungibile", "off");
    setTimeout(connect, 2000);
  };
}

function onMessage(m) {
  switch (m.type) {
    case "snapshot":
      limits = m.limits;
      buildMotors();
      buildGestures(m.gestures);
      renderPoses(m.poses);
      renderJobs(m.jobs);
      $("#log").textContent = "";
      m.log.forEach(addLog);
      renderState(m.state);
      break;
    case "state": renderState(m.state); break;
    case "log": addLog(m); break;
    case "jobs": renderJobs(m.jobs); break;
    case "poses": renderPoses(m.poses); break;
  }
}

// ------------------------------------------------------------------ motori
function buildMotors() {
  const box = $("#motors");
  box.innerHTML = "";
  for (let i = 0; i < 4; i++) {
    const [lo, hi] = limits[i];
    const div = document.createElement("div");
    div.className = "motor";
    div.innerHTML = `
      <span class="name">${NAMES[i]}</span><span class="val" id="val${i}">—</span>
      <input type="range" id="rng${i}" min="${lo}" max="${hi}" step="1">
      <div class="jog">
        <button data-m="${i}" data-d="-5">−5</button><button data-m="${i}" data-d="-1">−1</button>
        <button data-m="${i}" data-d="1">+1</button><button data-m="${i}" data-d="5">+5</button>
      </div>`;
    box.appendChild(div);
    const rng = div.querySelector("input");
    rng.addEventListener("input", () => { dragging.add(i); $(`#val${i}`).textContent = `→ ${rng.value}°`; });
    // invio al rilascio: un solo movimento invece di decine
    rng.addEventListener("change", () => { dragging.delete(i); send({ type: "set", motor: i, angle: +rng.value }); });
  }
  box.querySelectorAll(".jog button").forEach((b) =>
    b.addEventListener("click", () => send({ type: "jog", motor: +b.dataset.m, delta: +b.dataset.d })));
}

function renderState(s) {
  state = s;
  if (!s.connected) setPill($("#conn"), "Claudio scollegato", "off");
  else if (!s.ready) setPill($("#conn"), `avvio… ${s.port}`, "warn");
  else setPill($("#conn"), `collegato ${s.port}`, "on");
  setPill($("#busy"), s.busy ? "in movimento" : "fermo", s.busy ? "warn" : "");
  const z = s.zone === "touch" ? "off" : s.zone === "presence" ? "warn" : "";
  setPill($("#zone"), `${ZONES[s.zone]}${s.distance > 0 ? ` · ${s.distance} cm` : ""}`, z);
  for (let i = 0; i < 4; i++) {
    const a = s.positions[i];
    if (a === null || dragging.has(i)) continue;
    $(`#val${i}`).textContent = `${a}°`;
    $(`#rng${i}`).value = a;
  }
  const lb = $("#laser");
  lb.classList.toggle("on", s.laser);
  lb.textContent = s.laser ? "Laser ON" : "Laser OFF";
  $("#flags").textContent =
    `Random: ${s.periodic ? "attivo" : s.periodic_enabled ? "dopo 60 s" : "off"} · ` +
    `ritorno in posizione: ${s.guard_ms ? s.guard_ms / 1000 + " s" : "off"} · ` +
    `tatto: ${s.touch_enabled ? "on" : "off"} · soglia: ${s.delta_ang}°` +
    (s.last_error ? ` · ultimo errore: ${s.last_error}` : "");
}

function setPill(el, text, cls) {
  el.textContent = text;
  el.className = "pill" + (cls ? " " + cls : "");
}

// ------------------------------------------------------------------ gesti, pose, lavori
function buildGestures(list) {
  const box = $("#gestures");
  box.innerHTML = "";
  list.forEach((g) => {
    const b = document.createElement("button");
    b.textContent = g;
    b.onclick = () => send({ type: "gesture", name: g });
    box.appendChild(b);
  });
}

function renderPoses(poses) {
  const ul = $("#poses");
  ul.innerHTML = "";
  Object.entries(poses).forEach(([name, a]) => {
    const li = document.createElement("li");
    li.innerHTML = `<span></span><button>Vai</button><button title="Elimina">✕</button>`;
    li.querySelector("span").textContent = `${name} (${a.join(", ")})`;
    const [go, del] = li.querySelectorAll("button");
    go.onclick = () => send({ type: "pose_go", name });
    del.onclick = () => { if (confirm(`Eliminare la posa "${name}"?`)) send({ type: "pose_delete", name }); };
    ul.appendChild(li);
  });
}

function renderJobs(jobs) {
  const ul = $("#jobs");
  ul.innerHTML = "";
  jobs.filter((j) => ["queued", "running", "paused"].includes(j.status) || j.status === "error").slice(-5)
    .forEach((j) => {
      const li = document.createElement("li");
      const span = document.createElement("span");
      span.textContent = `#${j.id} ${j.name}: ${j.status} ${j.sent}/${j.total}${j.error ? " · " + j.error : ""}`;
      li.appendChild(span);
      const act = (label, type) => {
        const b = document.createElement("button");
        b.textContent = label;
        b.onclick = () => send({ type, id: j.id });
        li.appendChild(b);
      };
      if (j.status === "paused") act("Riprendi", "job_resume");
      else if (j.status !== "error") act("Pausa", "job_pause");
      if (j.status !== "error") act("Annulla", "job_cancel");
      ul.appendChild(li);
    });
}

function addLog(e) {
  const log = $("#log");
  const atBottom = log.scrollTop + log.clientHeight >= log.scrollHeight - 4;
  const line = document.createElement("div");
  line.className = e.kind === "tx" ? "tx" : e.kind === "error" ? "error" : e.kind === "sys" ? "sys" : "";
  line.textContent = e.line;
  log.appendChild(line);
  while (log.childElementCount > 300) log.firstChild.remove();
  if (atBottom) log.scrollTop = log.scrollHeight;
}

// ------------------------------------------------------------------ azioni
$("#stop").onclick = () => send({ type: "stop" });
$("#laser").onclick = () => send({ type: "laser", on: !state.laser });
document.querySelectorAll("[data-cmd]").forEach((b) => (b.onclick = () => send({ type: "cmd", text: b.dataset.cmd })));
$("#pose-form").onsubmit = (e) => {
  e.preventDefault();
  send({ type: "pose_save", name: $("#pose-name").value });
  $("#pose-name").value = "";
};
$("#raw-form").onsubmit = (e) => {
  e.preventDefault();
  const t = $("#raw").value.trim();
  if (t) send({ type: "raw", text: t });
  $("#raw").value = "";
};

// Tastiera: tenendo premuto un tasto i jog si accumulano e il server li accorpa
const KEYS = {
  ArrowLeft: [1, 1], ArrowRight: [1, -1], ArrowUp: [0, 1], ArrowDown: [0, -1],
  w: [2, 1], s: [2, -1], q: [3, -1], e: [3, 1],
};
document.addEventListener("keydown", (e) => {
  if (e.target.tagName === "INPUT") return;
  if (e.key === "Escape") { send({ type: "stop" }); return; }
  if (e.key === " ") { e.preventDefault(); if (!e.repeat) send({ type: "laser", on: !state.laser }); return; }
  const k = KEYS[e.key.length === 1 ? e.key.toLowerCase() : e.key];
  if (!k) return;
  e.preventDefault();
  send({ type: "jog", motor: k[0], delta: k[1] * (e.shiftKey ? 5 : 1) });
});

connect();
