/* Shared core of the 2-D and 3-D replay viewers (inlined by replay.py).

   It owns everything except the map: colour tokens and theme switching, the
   plain-language description of what each drone is doing (describe()), the
   header chips, the side panels (follow card, fleet list, stations, event log,
   run summary), the orders timeline, playback and keyboard shortcuts.

   A view plugs in the map:
     view.init(api)         once, before the first render
     view.draw(i, f, t)     draw frame i (+ fraction f towards frame i + 1) at tick t
     view.theme()           colour tokens changed (api.T)
     view.selected(k)       the followed drone changed (-1 = none)

   Continuous-flight replays (DATA.track) also carry every drone's position
   every few seconds; api.trackAt(k, seconds) interpolates it, and
   api.losAt / api.windAt give the losses of separation and the wind.
*/
function startReplay(DATA, view) {
  "use strict";
  const W = DATA.world.w, H = DATA.world.h, L = DATA.world.layers || 1;
  const frames = DATA.frames, LAST = frames.length - 1;
  const N = frames[0].d.length;
  const STATES = DATA.states;
  const LAYER_M = (DATA.meta && DATA.meta.layer_m) || 30;
  // cells are encoded as (z * H + y) * W + x; sites (pads, customers, buildings) have z = 0
  const enc = (x, y, z = 0) => (z * H + y) * W + x;
  const decode = e => [e % W, Math.floor(e / W) % H, Math.floor(e / (W * H))];
  const FLAG = { NONE: 0, WIND: 1, GIVE_WAY: 2, HOLDING: 3, LOWERING: 4 };
  const TRIP = { NONE: 0, LOW: 1, BEFORE_JOB: 2, EMERGENCY: 3, AFTER: 4 };
  const GROUP = { idle: "idle", returning: "idle", to_pickup: "pickup", loading: "pickup",
                  to_customer: "parcel", to_station: "energy", queued: "energy", swapping: "energy", dead: "dead" };
  const altOf = d => d[13] || 0;                  // flight layer, 0 = on the ground

  // ------------------------------------------------------------ continuous track
  const TRACK = DATA.track || null, CONT = !!TRACK;
  const TICK_S = (DATA.meta && DATA.meta.tick_s) || 10;
  const CELL_M = (TRACK && TRACK.cell_m) || 100, TRACK_LAYER_M = (TRACK && TRACK.layer_m) || LAYER_M;
  const TD = TRACK ? TRACK.dt : 1, NS = TRACK ? TRACK.n : 0;
  // positions come as whole metres, first sample absolute then differences: rebuild absolute arrays
  const track = TRACK ? TRACK.pos.map(arr => {
    const out = new Float32Array(arr.length);
    let x = 0, y = 0, z = 0;
    for (let j = 0; j < arr.length; j += 3) { x += arr[j]; y += arr[j + 1]; z += arr[j + 2]; out[j] = x; out[j + 1] = y; out[j + 2] = z; }
    return out;
  }) : null;
  /* Drone k at time `sec`: x, y in cell units (cell centres at integer + .5, like the grid views),
     z in flight layers, zm in metres, velocity in m/s, and whether it is off the ground. */
  function trackAt(k, sec) {
    const a = track[k];
    let s = Math.max(0, Math.min(NS - 1, sec / TD));
    const j = Math.max(0, Math.min(NS - 2, Math.floor(s))), f = NS > 1 ? s - j : 0, i0 = 3 * j, i1 = NS > 1 ? 3 * j + 3 : 3 * j;
    const xm = a[i0] + (a[i1] - a[i0]) * f, ym = a[i0 + 1] + (a[i1 + 1] - a[i0 + 1]) * f, zm = a[i0 + 2] + (a[i1 + 2] - a[i0 + 2]) * f;
    return { x: xm / CELL_M - .5, y: ym / CELL_M - .5, z: zm / TRACK_LAYER_M, zm,
             vx: (a[i1] - a[i0]) / TD, vy: (a[i1 + 1] - a[i0 + 1]) / TD, vz: (a[i1 + 2] - a[i0 + 2]) / TD, air: zm > 0.3 };
  }
  /* Losses of separation going on at `sec` (kept on screen `hold` seconds longer): [{i, j, dmin}] */
  function losAt(sec, hold = 2) {
    const out = [];
    if (!TRACK) return out;
    for (const [i, j, t0, t1, dmin] of TRACK.los) if (t0 <= sec && sec <= t1 + hold) out.push({ i, j, dmin, live: sec <= t1 });
    return out;
  }
  /* Where drone k flew during the last `span` seconds, oldest first, ending at `sec`:
     [{x, y, z, air}] in the same units as trackAt. */
  function trailAt(k, sec, span = 30) {
    const out = [];
    if (!TRACK) return out;
    const a = track[k], j1 = Math.min(NS - 1, Math.floor(sec / TD));
    for (let j = Math.max(0, Math.floor((sec - span) / TD)); j <= j1; j++) {
      const zm = a[3 * j + 2];
      out.push({ x: a[3 * j] / CELL_M - .5, y: a[3 * j + 1] / CELL_M - .5, z: zm / TRACK_LAYER_M, air: zm > .3 });
    }
    const p = trackAt(k, sec);
    out.push({ x: p.x, y: p.y, z: p.z, air: p.air });
    return out;
  }
  /* Drones that ORCA is steering around another drone at `sec` (a Set of ids). */
  const orcaByDrone = TRACK ? Array.from({ length: N }, () => []) : null;
  if (TRACK) for (const [k, t0, t1] of TRACK.orca) if (k < N) orcaByDrone[k].push([t0, t1]);
  function orcaAt(sec) {
    const out = new Set();
    if (!TRACK) return out;
    orcaByDrone.forEach((iv, k) => { if (iv.some(([t0, t1]) => t0 <= sec && sec <= t1 + 1)) out.add(k); });
    return out;
  }
  /* Wind at the city centre, [wx, wy] in m/s (grid axes). */
  function windAt(sec) {
    if (!TRACK || !TRACK.wind.length) return [0, 0];
    const w = TRACK.wind, n = w.length / 2, s = Math.max(0, Math.min(n - 1, sec / TD)), j = Math.min(n - 2, Math.floor(s)), f = n > 1 ? s - j : 0;
    if (n < 2) return [w[0] / 10, w[1] / 10];
    return [(w[2 * j] + (w[2 * j + 2] - w[2 * j]) * f) / 10, (w[2 * j + 1] + (w[2 * j + 3] - w[2 * j + 1]) * f) / 10];
  }

  // ------------------------------------------------------------ tokens
  const TOKENS = ["surface", "ink", "ink2", "muted", "grid", "base", "building", "c-pickup", "c-parcel",
                  "c-energy", "c-idle", "good", "warning", "critical", "bg", "accent", "raised"].concat(view.tokens || []);
  const T = {};
  function readTokens() {
    const cs = getComputedStyle(document.documentElement);
    for (const k of TOKENS) T[k] = cs.getPropertyValue("--" + k).trim();
  }
  const groupColor = g => g === "dead" ? T.critical : T["c-" + g];
  const colorOf = d => groupColor(GROUP[STATES[d[3]]]);
  const socColor = s => s >= 50 ? T.good : s >= 25 ? T.warning : T.critical;
  function textOn(hex) {
    const m = /^#?([0-9a-f]{6})$/i.exec(hex || "");
    if (!m) return "#fff";
    const n = parseInt(m[1], 16);
    const lum = [n >> 16 & 255, n >> 8 & 255, n & 255].map(v => { v /= 255; return v <= .03928 ? v / 12.92 : Math.pow((v + .055) / 1.055, 2.4); });
    return .2126 * lum[0] + .7152 * lum[1] + .0722 * lum[2] > .3 ? "#0b0b0b" : "#ffffff";
  }
  const esc = s => String(s).replace(/[<>&"]/g, c => ({ "<": "&lt;", ">": "&gt;", "&": "&amp;", '"': "&quot;" }[c]));

  // ------------------------------------------------------------ places
  const hubs = DATA.world.hubs, stations = DATA.world.stations;
  const placeByEnc = new Map();
  hubs.forEach(([x, y], i) => placeByEnc.set(enc(x, y), `Hub ${i}`));
  stations.forEach(([x, y], i) => placeByEnc.set(enc(x, y), `Station ${i}`));
  const placeName = e => placeByEnc.get(e) || "the customer";
  const heights = new Map(DATA.world.blocked.map((b, j) => [b, DATA.world.heights ? DATA.world.heights[j] : L]));
  // customers in buildings (optional): height in layers, 0 = a house; parcels go onto the roof
  const CUST_H = DATA.world.customer_heights ? new Map(DATA.world.customers.map(([x, y], j) => [x + "," + y, DATA.world.customer_heights[j]])) : null;
  const custHeight = (x, y) => CUST_H ? (CUST_H.get(x + "," + y) || 0) : -1;
  /* Where an order's parcel goes, in words: "the customer's roof (30 m up)", "the customer's garden" or "the customer"
     (with a preposition, for "Lowering order #3 onto the customer's roof"). */
  function dropPlace(o, prep = false) {
    const h = o ? custHeight(o.x, o.y) : -1;
    if (h < 0) return (prep ? "to " : "") + "the customer";
    return h === 0 ? (prep ? "into " : "") + "the customer's garden" : (prep ? "onto " : "") + `the customer's roof (${h * LAYER_M} m up)`;
  }

  // ------------------------------------------------------------ orders
  const orders = DATA.orders.map(o => ({ oid: o[0], hub: o[1], x: o[2], y: o[3], created: o[4], deadline: o[5],
                                         delivered: o[6], express: !!o[7], drone: o[8], status: o[9], weight: o[10] }));
  const orderById = new Map(orders.map(o => [o.oid, o]));

  // ------------------------------------------------------------ events
  const flashes = [];
  const byDrone = Array.from({ length: N }, () => []);
  frames.forEach((f, i) => f.e.forEach(e => {
    const m = /COLLISION.*at \((\d+), (\d+)(?:, (\d+))?\)(?: on layer (\d+))?/.exec(e);
    if (m) flashes.push({ i, x: +m[1], y: +m[2], z: +(m[4] || m[3] || 1) });
    const seen = new Set();
    for (const mm of e.matchAll(/Drone (\d+)(?!\d)/g)) seen.add(+mm[1]);
    const cm = /drones ([\d and]+) (are|fly)/.exec(e);
    if (cm) for (const x of cm[1].split(" and ")) seen.add(+x);
    for (const k of seen) if (k < N) byDrone[k].push([i, f.t, e]);
  }));

  // ------------------------------------------------------------ language
  function describe(d) {
    const st = STATES[d[3]], oid = d[5], air = !!d[2], flag = d[9], reason = d[10];
    const tgt = d[8] >= 0 ? placeName(d[8]) : null;
    const here = placeByEnc.get(enc(d[0], d[1]));
    const o = orderById.get(oid);
    const ex = o && o.express ? " (express)" : "";
    let text = "", tag = "";
    switch (st) {
      case "idle":
        text = air ? "Hovering with nothing to do" : `Parked at ${here || "a pad"}, ready for the next order`;
        tag = air ? "free" : ""; break;
      case "to_pickup":
        text = air ? `Flying to ${tgt} to collect order #${oid}${ex}` : `Taking off for ${tgt} to collect order #${oid}`;
        tag = `→ ${tgt} for #${oid}`; break;
      case "loading":
        text = `Loading order #${oid}${ex} at ${tgt}`; tag = `loading #${oid}`; break;
      case "to_customer":
        if (flag === FLAG.LOWERING) { text = `Lowering order #${oid} ${dropPlace(o, true)} on a winch`; tag = `lowering #${oid}`; }
        else if (!air) { text = `Taking off with order #${oid}${ex}`; tag = ""; }
        else { text = `Carrying order #${oid}${ex} to the customer`; tag = `#${oid} → customer`; }
        break;
      case "returning":
        text = `Flying back to ${tgt} to wait for the next order`; tag = `→ ${tgt}`; break;
      case "to_station":
        if (reason === TRIP.BEFORE_JOB) { text = `Flying to ${tgt} for a fresh battery before collecting order #${oid}`; tag = `→ ${tgt} battery`; }
        else if (reason === TRIP.EMERGENCY) { text = `Battery emergency: diverting to ${tgt}` + (d[6] ? ` with order #${oid} on board` : ""); tag = `EMERGENCY → ${tgt}`; }
        else if (reason === TRIP.AFTER) { text = `Parcel delivered; flying to ${tgt} to swap its battery (${d[4]}% left)`; tag = `→ ${tgt} battery`; }
        else { text = `Battery low (${d[4]}%): flying to ${tgt} for a swap`; tag = `→ ${tgt} battery`; }
        if (!air) text = text.replace("Flying", "Taking off").replace("flying", "taking off");
        break;
      case "queued":
        text = `Waiting in line at ${tgt} for a battery swap` + (oid >= 0 ? `, then collects order #${oid}` : ""); break;
      case "swapping":
        text = `Getting a fresh battery at ${tgt}` + (oid >= 0 ? `, then collects order #${oid}` : ""); break;
      case "dead":
        text = "Ran out of battery in flight and was lost"; tag = "LOST"; break;
    }
    const why = flag === FLAG.WIND ? "Held back one tick by a wind gust"
              : flag === FLAG.GIVE_WAY ? (CONT ? "Steering around another drone (collision avoidance)" : "Paused to give way to another drone")
              : flag === FLAG.HOLDING ? "Hovering in place until its route is clear" : "";
    const whyTag = flag === FLAG.WIND ? "wind" : flag === FLAG.GIVE_WAY ? (CONT ? "avoiding" : "giving way") : flag === FLAG.HOLDING ? "waiting" : "";
    // the map label: what + (in stacked airspace) which layer + why it paused
    const alt = L > 1 && air && st !== "dead" ? `L${altOf(d)}` : "";
    const label = tag ? [tag, alt, whyTag].filter(Boolean).join(" · ") : "";
    return { text, why, tag: tag && whyTag ? `${tag} · ${whyTag}` : tag, label, alt };
  }
  function altitudeText(d, k = -1) {
    if (CONT && k >= 0) {                 // the recorded height, even mid take-off or landing
      const p = trackAt(k, pos * TICK_S);
      if (!p.air) return "on the ground";
      return `${Math.round(p.zm)} m up` + (d[2] ? ` (layer ${altOf(d)} of ${L})` : p.vz < 0 ? " (landing)" : " (taking off)");
    }
    if (!d[2]) return "on the ground";
    const z = altOf(d);
    return `layer ${z} of ${L} (about ${z * LAYER_M} m up)`;
  }

  // ------------------------------------------------------------- header
  const meta = DATA.meta;
  const ALLOC = { cnp: "Parcels assigned by auction", nearest: "Parcels go to the nearest free drone", round_robin: "Parcels assigned in turn" };
  const COORD = { cooperative: "Routes booked in space and time", reactive: "No route booking: drones give way on sight", none: "No collision avoidance" };
  const BATT = { predictive: "Energy-aware battery swaps", naive: "Naive battery rule (swap below 30 %)" };
  const layersChip = L > 1 ? `${L} flight layers` + (DATA.world.layer_rule === "heading" ? " · east/west odd, north/south even" : "") : "One flight layer";
  const windChip = CONT ? `Wind ${meta.wind[0].toFixed(0)} m/s, gusts ${meta.wind[2].toFixed(1)} m/s` : `Wind gusts ${(meta.gust_prob * 100).toFixed(0)} %`;
  const motionChip = CONT ? "Continuous flight" + (meta.tactical === "orca" ? " · ORCA collision avoidance" : " · no tactical avoidance") : "";
  document.getElementById("chips").innerHTML = [`${meta.n_drones} drones`, layersChip, motionChip, ALLOC[meta.allocation], COORD[meta.coordination],
    BATT[meta.battery_policy], windChip, `Seed ${meta.seed}`]
    .filter(Boolean).map(c => `<span class="chip">${esc(c)}</span>`).join("");
  document.querySelectorAll("[data-layers]").forEach(el => { el.hidden = L === 1; });
  document.querySelectorAll("[data-continuous]").forEach(el => { el.hidden = !CONT; });
  document.querySelectorAll("[data-cust]").forEach(el => { el.hidden = !CUST_H; });
  if (CONT) {
    // make it obvious this is not the grid model: title, and the continuous views switched on
    document.querySelectorAll("[data-cont-default]").forEach(el => { el.checked = true; });
    const h1 = document.querySelector("header h1");
    if (h1) h1.textContent += " · continuous flight";
    document.title += " · continuous flight";
  }
  document.querySelectorAll("[data-grid]").forEach(el => { el.hidden = CONT; });

  const themeBtn = document.getElementById("theme");
  const THEMES = ["auto", "light", "dark"];
  let themeIdx = 0;
  try { const saved = localStorage.getItem("dronefleet-theme"); if (saved) themeIdx = Math.max(0, THEMES.indexOf(saved)); } catch (e) {}
  function applyTheme() {
    const th = THEMES[themeIdx];
    if (th === "auto") document.documentElement.removeAttribute("data-theme");
    else document.documentElement.setAttribute("data-theme", th);
    themeBtn.textContent = "Theme: " + th;
    try { localStorage.setItem("dronefleet-theme", th); } catch (e) {}
    themeChanged();
  }
  function themeChanged() { readTokens(); view.theme(); lastUi = -1; render(); }
  themeBtn.onclick = () => { themeIdx = (themeIdx + 1) % 3; applyTheme(); };
  matchMedia("(prefers-color-scheme: dark)").addEventListener("change", themeChanged);
  const guide = document.getElementById("guide");
  try { if (localStorage.getItem("dronefleet-guide") === "closed") guide.open = false; } catch (e) {}
  guide.addEventListener("toggle", ev => {
    try { localStorage.setItem("dronefleet-guide", ev.target.open ? "open" : "closed"); } catch (e) {}
  });

  // --------------------------------------------------------- side panels
  const nowEl = document.getElementById("now"), kpis = document.getElementById("kpis"), fleet = document.getElementById("fleet"),
        stEl = document.getElementById("stations"), logEl = document.getElementById("log"), followEl = document.getElementById("follow"),
        logFilterWrap = document.getElementById("logFilterWrap"), logFilter = document.getElementById("logFilter"),
        logFilterLabel = document.getElementById("logFilterLabel");
  let selected = -1;

  function summary(fr) {
    const c = { pickup: 0, parcel: 0, energyAir: 0, returning: 0, parked: 0, atStation: 0, lost: 0, loading: 0 };
    const perLayer = new Array(L + 1).fill(0);
    for (const d of fr.d) {
      const st = STATES[d[3]];
      if (d[2] && st !== "dead") perLayer[altOf(d)]++;
      if (st === "dead") c.lost++;
      else if (st === "to_pickup" && d[2]) c.pickup++;
      else if (st === "to_customer" && d[2]) c.parcel++;
      else if (st === "to_station" && d[2]) c.energyAir++;
      else if (st === "returning" || (st === "idle" && d[2])) c.returning++;
      else if (st === "queued" || st === "swapping" || st === "to_station") c.atStation++;
      else if (st === "loading" || st === "to_pickup" || st === "to_customer") c.loading++;
      else c.parked++;
    }
    const flying = c.pickup + c.parcel + c.energyAir + c.returning;
    const parts = [];
    if (c.parcel) parts.push(`<b>${c.parcel}</b> carrying parcels`);
    if (c.pickup) parts.push(`<b>${c.pickup}</b> going to collect one`);
    if (c.energyAir) parts.push(`<b>${c.energyAir}</b> heading for a battery swap`);
    if (c.returning) parts.push(`<b>${c.returning}</b> returning to a hub`);
    const ground = [];
    if (c.loading) ground.push(`${c.loading} loading at a hub`);
    if (c.atStation) ground.push(`${c.atStation} at a swap station`);
    if (c.parked) ground.push(`${c.parked} parked and free`);
    if (c.lost) ground.push(`<b style="color:var(--critical)">${c.lost} lost</b>`);
    const waiting = fr.o[0];
    let s = `<b>t = ${fr.t}</b> · ${flying ? `${flying} of ${N} drones flying: ${parts.join(", ")}.` : "No drones in the air."}`;
    if (flying && L > 1) s += ` By layer: ${perLayer.slice(1).map((n, z) => `L${z + 1} ${n}`).join(", ")}.`;
    if (ground.length) s += ` On the ground: ${ground.join(", ")}.`;
    s += waiting ? ` <b>${waiting}</b> order${waiting > 1 ? "s" : ""} waiting for a drone.` : " No order is waiting for a drone.";
    return s;
  }

  function renderUi(i) {
    const fr = frames[i];
    nowEl.innerHTML = summary(fr);
    const created = orders.filter(o => o.created <= fr.t).length;
    const tiles = [
      ["Delivered", `${fr.o[2]} <small>of ${created}</small>`],
      ["Being delivered", fr.o[1]],
      ["Waiting for a drone", fr.o[0]],
      ["Collisions", fr.c],
    ];
    if (CONT && view.windTile) {
      // wind at the city centre, the arrow drawn in map orientation (x right, y down)
      const [wx, wy] = windAt(fr.t * TICK_S), sp = Math.hypot(wx, wy), ang = Math.atan2(wx, -wy) * 180 / Math.PI;
      tiles.push(["Wind", `<svg class="windk" viewBox="-8 -8 16 16" style="transform:rotate(${ang.toFixed(0)}deg)" aria-hidden="true">` +
                          `<path d="M0 6 L0 -6 M0 -6 L-3.5 -2 M0 -6 L3.5 -2"/></svg>${sp.toFixed(1)} <small>m/s</small>`]);
    }
    kpis.innerHTML = tiles.map(([l, v]) => `<div class="kpi"><div class="label">${l}</div><div class="value">${v}</div></div>`).join("");

    fleet.innerHTML = fr.d.map((d, k) => {
      const col = colorOf(d), desc = describe(d);
      const sub = [desc.alt ? `Layer ${altOf(d)}` : "", desc.why].filter(Boolean).join(" · ");
      return `<button class="frow${k === selected ? " sel" : ""}" data-k="${k}" aria-pressed="${k === selected}">` +
        `<span class="fid" style="background:${col};color:${textOn(col)}">${k}</span>` +
        `<span class="ftext">${esc(desc.text)}${sub ? `<small>${esc(sub)}</small>` : ""}</span>` +
        `<span class="fbat" title="battery ${d[4]}%">${d[4]}%<span class="bar"><i style="width:${d[4]}%;background:${socColor(d[4])}"></i></span></span></button>`;
    }).join("");

    stEl.innerHTML = fr.s.map((s, j) => {
      const [q, act, charged, total] = s;
      const packs = Array.from({ length: total }, (_, p) => `<i class="${p < charged ? "full" : ""}"></i>`).join("");
      const here = enc(stations[j][0], stations[j][1]);
      const swapping = [], queued = [];
      fr.d.forEach((d, k) => { if (d[8] === here) { if (STATES[d[3]] === "swapping") swapping.push(k); else if (STATES[d[3]] === "queued") queued.push(k); } });
      const line = [swapping.length ? `swapping drone ${swapping.join(", ")}` : "",
                    queued.length ? `drone ${queued.join(", ")} waiting` : ""].filter(Boolean).join(" · ") || "free";
      return `<div class="station"><div><b>Station ${j}</b></div><div class="meta">${line}</div>` +
             `<div class="packs" title="${charged} of ${total} spare packs fully charged">${packs}<span>${charged} of ${total} spare packs charged</span></div></div>`;
    }).join("");

    renderFollow(i);
    renderLog(i);
    renderMsgs(i);
  }

  /* The followed drone's height: the last two minutes (solid) and its booked route (dashed). */
  function altitudeProfile(k, i, col) {
    const t = frames[i].t, PAST = 12, AHEAD = 18, past = [], ahead = [];
    if (CONT) {
      for (let s = Math.max(0, (t - PAST) * TICK_S); s <= t * TICK_S + 1e-9; s += TD) past.push([s / TICK_S, trackAt(k, s).zm]);
    } else {
      for (let j = Math.max(0, i - PAST); j <= i; j++) { const d = frames[j].d[k]; past.push([frames[j].t, d[2] ? altOf(d) * LAYER_M : 0]); }
    }
    if (routeAhead(k, t, frames[i].d[k])) {         // on its booked route: the planned layers, tick by tick
      const p = planAt(k, t);
      ahead.push(past[past.length - 1]);
      for (let j = t - p[0] + 1; j < p[1].length && p[0] + j <= t + AHEAD; j++) ahead.push([p[0] + j, decode(p[1][j])[2] * LAYER_M]);
    }
    const Wd = 300, Hd = 84, l = 34, r = 6, top = 6, bot = 18, zmax = (L + .6) * LAYER_M;
    const X = tt => l + (tt - (t - PAST)) / (PAST + AHEAD) * (Wd - l - r), Y = z => top + (1 - z / zmax) * (Hd - top - bot);
    const poly = pts => pts.map(([tt, z], j) => `${j ? "L" : "M"}${X(tt).toFixed(1)},${Y(z).toFixed(1)}`).join(" ");
    let g = "";
    for (let z = 0; z <= L; z++) {
      g += `<line x1="${l}" x2="${Wd - r}" y1="${Y(z * LAYER_M)}" y2="${Y(z * LAYER_M)}" stroke="var(--grid)" stroke-width="1"/>` +
           `<text x="${l - 5}" y="${Y(z * LAYER_M) + 3.5}" text-anchor="end" font-size="10" fill="var(--muted)">${z ? z * LAYER_M + " m" : "0"}</text>`;
    }
    g += `<line x1="${X(t)}" x2="${X(t)}" y1="${top}" y2="${Hd - bot}" stroke="var(--muted)" stroke-dasharray="2 2"/>` +
         `<text x="${X(t)}" y="${Hd - 5}" text-anchor="middle" font-size="10" fill="var(--muted)">now</text>` +
         `<text x="${l}" y="${Hd - 5}" font-size="10" fill="var(--muted)">2 min ago</text>` +
         `<text x="${Wd - r}" y="${Hd - 5}" text-anchor="end" font-size="10" fill="var(--muted)">booked route</text>`;
    if (past.length > 1) g += `<path d="${poly(past)}" fill="none" stroke="${col}" stroke-width="2.4" stroke-linejoin="round"/>`;
    if (ahead.length > 1) g += `<path d="${poly(ahead)}" fill="none" stroke="${col}" stroke-width="2" stroke-dasharray="4 3" opacity=".8"/>`;
    const nowZ = past.length ? past[past.length - 1][1] : 0;
    g += `<circle cx="${X(t)}" cy="${Y(nowZ)}" r="4" fill="${col}" stroke="var(--surface)" stroke-width="1.5"/>`;
    return `<svg class="altprof" viewBox="0 0 ${Wd} ${Hd}" role="img" aria-label="Height over the last two minutes and along the booked route">${g}</svg>`;
  }

  const followMini = document.getElementById("followMini");
  function renderFollow(i) {
    const fr = frames[i];
    followMini.hidden = selected < 0;
    if (selected >= 0) {
      const d = fr.d[selected], col = colorOf(d), desc = describe(d);
      const eta = d[12] >= 0 && d[12] > fr.t ? ` · arrives t=${d[12]}` : "";
      followMini.innerHTML = `<span class="badge" style="background:${col};color:${textOn(col)}">${selected}</span>` +
        `<span class="txt"><b>Drone ${selected}</b>: ${esc(desc.text)}${eta} · battery ${d[4]}%` +
        `${desc.alt ? ` · <span class="alt">${desc.alt}</span>` : ""}${desc.why ? ` · ${esc(desc.why.toLowerCase())}` : ""}</span>` +
        `<button id="unfollowMini">Stop</button>`;
      document.getElementById("unfollowMini").onclick = () => select(-1);
    }
    if (selected < 0) {
      followEl.innerHTML = `<h2>Follow a drone</h2><p class="hint">${view.followHint || "Click a drone on the map or in the fleet list."} You will see what it is doing, why, where it is going and when it gets there.</p>`;
      return;
    }
    const d = fr.d[selected], col = colorOf(d), desc = describe(d);
    const o = orderById.get(d[5]);
    const facts = [];
    facts.push(["Battery", `<span class="bar"><i style="width:${d[4]}%;background:${socColor(d[4])}"></i></span>${d[4]}%`]);
    if ((L > 1 || CONT) && STATES[d[3]] !== "dead") facts.push(["Altitude", `<span class="alt">${desc.alt && !CONT ? desc.alt + " · " : ""}${altitudeText(d, selected)}</span>`]);
    if (CONT && STATES[d[3]] !== "dead") {
      const p = trackAt(selected, pos * TICK_S);
      if (p.air) facts.push(["Speed", `${Math.hypot(p.vx, p.vy).toFixed(1)} m/s` + (Math.abs(p.vz) > .3 ? `, ${p.vz > 0 ? "climbing" : "descending"} ${Math.abs(p.vz).toFixed(1)} m/s` : "")]);
    }
    if (o) facts.push(["Order", `#${o.oid} · ${o.weight != null ? o.weight.toFixed(1) + " kg" : ""}${o.express ? " · express" : ""} · due t=${o.deadline}` +
                               (d[6] ? " · on board" : " · not collected yet")]);
    const st = STATES[d[3]];
    if (d[8] >= 0 && st !== "queued" && st !== "swapping" && st !== "loading")
      facts.push(["Going to", placeByEnc.has(d[8]) ? placeName(d[8]) : `${dropPlace(o)} (ring #${d[5]} on the map)`]);
    if (d[12] >= 0 && d[12] > fr.t) facts.push(["Arrives", `t=${d[12]} (in ${d[12] - fr.t} tick${d[12] - fr.t === 1 ? "" : "s"})`]);
    if (d[11] >= 0) facts.push(["After that", `lands at ${placeName(d[11])}`]);
    const recent = byDrone[selected].filter(([fi]) => fi <= i).slice(-6).reverse();
    followEl.innerHTML =
      `<div class="fhead"><span class="badge" style="background:${col};color:${textOn(col)}">${selected}</span><b>Drone ${selected}</b>` +
      `<button id="unfollow">Stop following</button></div>` +
      `<p class="fnow">${esc(desc.text)}</p>${desc.why ? `<p class="fwhy">${esc(desc.why)}</p>` : ""}` +
      `<dl class="facts">${facts.map(([k, v]) => `<dt>${k}</dt><dd>${v}</dd>`).join("")}</dl>` +
      (STATES[d[3]] !== "dead" && L > 1 ? `<h3>Height</h3>${altitudeProfile(selected, i, col)}` : "") +
      `<h3>Recent activity</h3><ul class="mini-log">${recent.length ? recent.map(([, t, e]) => `<li><span class="t">t=${t}</span>${esc(e.replace(/^!! /, ""))}</li>`).join("") : "<li>Nothing yet.</li>"}</ul>`;
    document.getElementById("unfollow").onclick = () => select(-1);
  }

  function renderLog(i) {
    const only = selected >= 0 && logFilter.checked;
    logFilterWrap.hidden = selected < 0;
    logFilterLabel.textContent = `only Drone ${selected}`;
    const lines = [];
    if (only) {
      const arr = byDrone[selected];
      for (let j = arr.length - 1; j >= 0 && lines.length < 80; j--) if (arr[j][0] <= i) lines.push([arr[j][1], arr[j][2]]);
    } else {
      for (let j = i; j >= 0 && lines.length < 80; j--) {
        const f = frames[j];
        for (let e = f.e.length - 1; e >= 0 && lines.length < 80; e--) lines.push([f.t, f.e[e]]);
      }
    }
    logEl.innerHTML = lines.map(([t, e]) => {
      const cls = /!!|LOST|EMERGENCY|COLLISION|LATE|Loss of separation/.test(e) ? "alert" : /delivers order/.test(e) ? "good" : "";
      return `<li class="${cls}"><span class="t">t=${t}</span>${esc(e.replace(/^!! /, ""))}</li>`;
    }).join("") || "<li>Nothing has happened yet.</li>";
  }
  logFilter.onchange = () => { lastUi = -1; render(); };

  // ------------------------------------------------------------- messages
  const MSGS = DATA.messages || null;     // [t, from, to, act, text], in order
  const ACT = { cfp: "CALL FOR BIDS", propose: "BID", refuse: "REFUSE", accept: "ACCEPT", reject: "REJECT", request: "REQUEST",
                agree: "AGREE", inform: "INFORM", failure: "FAILURE", cancel: "CANCEL" };
  const msgPanel = document.getElementById("msgPanel"), msgList = document.getElementById("msgList"),
        msgFilterWrap = document.getElementById("msgFilterWrap"), msgFilter = document.getElementById("msgFilter"),
        msgFilterLabel = document.getElementById("msgFilterLabel");
  if (msgPanel) msgPanel.hidden = !MSGS;
  function renderMsgs(i) {
    if (!MSGS || !msgList) return;
    const t = frames[i].t, only = selected >= 0 && msgFilter.checked, me = `Drone ${selected}`;
    msgFilterWrap.hidden = selected < 0;
    msgFilterLabel.textContent = `only Drone ${selected}`;
    let lo = 0, hi = MSGS.length - 1, end = -1;           // last message sent at or before tick t
    while (lo <= hi) { const mid = (lo + hi) >> 1; if (MSGS[mid][0] <= t) { end = mid; lo = mid + 1; } else hi = mid - 1; }
    const rows = [];
    for (let j = end; j >= 0 && rows.length < 60; j--) {
      const m = MSGS[j];
      if (only && m[1] !== me && m[2] !== me) continue;
      rows.push(m);
    }
    msgList.innerHTML = rows.map(([mt, from, to, act, text]) =>
      `<li><span class="t">t=${mt}</span><span class="who">${esc(from)}</span> → ${esc(to)}<span class="act">${ACT[act] || esc(act)}</span>` +
      `<span class="txt">${esc(text)}</span></li>`).join("") || "<li>No messages yet.</li>";
  }
  if (msgFilter) msgFilter.onchange = () => { lastUi = -1; render(); };

  fleet.addEventListener("click", ev => {
    const b = ev.target.closest(".frow");
    if (b) select(Number(b.dataset.k) === selected ? -1 : Number(b.dataset.k));
  });
  function select(k) { selected = k; view.selected(k); lastUi = -1; render(); }

  // ------------------------------------------------------------ tooltips
  const tip = document.getElementById("tip");
  function showTip(html, x, y, width = 290) {
    tip.innerHTML = html; tip.hidden = false;
    tip.style.left = Math.max(4, Math.min(x + 14, innerWidth - width)) + "px"; tip.style.top = (y + 14) + "px";
  }
  function hideTip() { tip.hidden = true; }
  function droneTip(k) {
    const d = frames[Math.min(LAST, Math.floor(pos))].d[k], desc = describe(d);
    return `<b>Drone ${k}</b> · battery ${d[4]}%${desc.alt ? ` · ${desc.alt}` : ""}<br>${esc(desc.text)}` +
           `${desc.why ? `<br><span class="why">${esc(desc.why)}</span>` : ""}<br><span class="why">${view.clickHint || "Click to follow"}</span>`;
  }

  // -------------------------------------------------------------- metrics
  const M = DATA.metrics || {};
  const fmt1 = v => v != null ? v.toFixed(1) : null;
  document.getElementById("metrics").innerHTML = [
    ["Orders delivered", M.delivered != null ? `${M.delivered} of ${M.orders}` : null],
    ["Average delivery time", M.avg_delivery_time != null ? `${fmt1(M.avg_delivery_time)} ticks` : null],
    ["Slowest 5 % took at least", M.p95_delivery_time != null ? `${M.p95_delivery_time} ticks` : null],
    ["Delivered on time", M.on_time_rate != null ? `${(M.on_time_rate * 100).toFixed(1)} %` : null],
    ["Collisions", M.collisions], ["Drones lost", M.dead_drones],
    ["Losses of separation", CONT && M.separation_losses != null ? `${M.separation_losses} (${fmt1(M.separation_loss_s)} pair-seconds)` : null],
    ["Closest two drones came", CONT && M.min_separation_m != null ? `${fmt1(M.min_separation_m)} m` : null],
    ["Distance from the planned track", CONT && M.tracking_error_mean_m != null ? `${fmt1(M.tracking_error_mean_m)} m on average, 95 % within ${M.tracking_error_p95_m} m` : null],
    ["Avoidance manoeuvres (ORCA)", CONT && M.orca_interventions != null ? `${M.orca_interventions} (${fmt1(M.orca_per_drone_hour)} per flight hour)` : null],
    ["Battery swaps", M.swaps], ["Battery emergencies", M.emergencies],
    ["Energy per delivery", fmt1(M.energy_per_delivery)],
    ["Time drones spent busy", M.utilisation != null ? `${(M.utilisation * 100).toFixed(1)} %` : null],
    ["Routes planned / shifted after wind", M.replans != null ? `${M.replans} / ${M.plan_repairs}` : null],
    ["Times a drone gave way", CONT ? null : M.forced_holds], ["Auctions held", M.auction_rounds],
    ["Climbs / descents", L > 1 && M.climbs != null ? `${M.climbs} / ${M.descents}` : null],
    ["Flight time above layer 1", L > 1 && M.upper_layer_share != null ? `${(M.upper_layer_share * 100).toFixed(1)} %` : null],
  ].filter(r => r[1] != null).map(([k, v]) => `<tr><td>${k}</td><td>${v}</td></tr>`).join("");

  // ------------------------------------------------------------- timeline
  const FONT = getComputedStyle(document.body).fontFamily;
  const tl = document.getElementById("timeline"), tctx = tl.getContext("2d");
  let tlW = 600, dpr = 1;
  const tlH = 130;
  const PAD = { l: 34, r: 10, t: 8, b: 20 };
  const maxY = Math.max(4, ...frames.map(f => Math.max(f.o[0], f.o[1])));
  const yStep = Math.max(1, Math.ceil(maxY / 4)), yMax = yStep * 4;
  function sizeTimeline() {
    dpr = window.devicePixelRatio || 1;
    tlW = tl.clientWidth || 600;
    tl.width = Math.round(tlW * dpr); tl.height = Math.round(tlH * dpr);
    tctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  }
  const tx = i => PAD.l + (tlW - PAD.l - PAD.r) * i / Math.max(1, LAST);
  const ty = v => tlH - PAD.b - (tlH - PAD.t - PAD.b) * v / yMax;
  function drawTimeline(p) {
    const c = tctx;
    c.clearRect(0, 0, tlW, tlH);
    c.font = `11px ${FONT}`; c.fillStyle = T.muted; c.textAlign = "right"; c.textBaseline = "middle";
    c.strokeStyle = T.grid; c.lineWidth = 1;
    for (let v = 0; v <= yMax; v += yStep) {
      const y = Math.round(ty(v)) + .5;
      c.beginPath(); c.moveTo(PAD.l, y); c.lineTo(tlW - PAD.r, y); c.stroke(); c.fillText(String(v), PAD.l - 6, y);
    }
    c.textAlign = "center"; c.textBaseline = "top";
    const tStep = Math.max(50, Math.ceil(frames[LAST].t / 8 / 50) * 50);
    for (let t = 0; t <= frames[LAST].t; t += tStep) c.fillText(String(t), tx(t), tlH - PAD.b + 5);
    const series = [[0, T["c-pickup"]], [1, T["c-parcel"]]];
    for (const [k, col] of series) {
      c.beginPath(); c.lineWidth = 2; c.lineJoin = "round"; c.strokeStyle = col;
      frames.forEach((f, i) => { const x = tx(i), y = ty(f.o[k]); i ? c.lineTo(x, y) : c.moveTo(x, y); });
      c.stroke();
    }
    const x = tx(p);
    c.strokeStyle = T.ink2; c.lineWidth = 1; c.beginPath(); c.moveTo(x + .5, PAD.t); c.lineTo(x + .5, tlH - PAD.b); c.stroke();
    const f = frames[Math.round(p)];
    for (const [k, col] of series) { c.beginPath(); c.arc(x, ty(f.o[k]), 4, 0, 7); c.fillStyle = col; c.fill(); c.lineWidth = 2; c.strokeStyle = T.surface; c.stroke(); }
  }
  function frameFromEvent(ev) {
    const rect = tl.getBoundingClientRect();
    return Math.max(0, Math.min(LAST, Math.round((ev.clientX - rect.left - PAD.l) / (tlW - PAD.l - PAD.r) * LAST)));
  }
  tl.addEventListener("mousemove", ev => {
    const f = frames[frameFromEvent(ev)];
    showTip(`<b>t = ${f.t}</b><br>waiting for a drone: ${f.o[0]}<br>being delivered: ${f.o[1]}<br>delivered so far: ${f.o[2]}`,
            ev.clientX, ev.clientY - 24, 200);
  });
  tl.addEventListener("mouseleave", hideTip);
  tl.addEventListener("click", ev => seek(frameFromEvent(ev)));
  new ResizeObserver(() => { sizeTimeline(); render(); }).observe(tl);

  // ------------------------------------------------------------- playback
  const slider = document.getElementById("slider"), tlabel = document.getElementById("tlabel"),
        playBtn = document.getElementById("play"), speedSel = document.getElementById("speed");
  slider.max = LAST;
  const opt = id => { const el = document.getElementById(id); return !!(el && el.checked); };
  let pos = 0, playing = false, lastTs = null, lastUi = -1, ready = false;

  function render() {
    if (!ready || !T.surface) return;
    const i = Math.min(LAST, Math.floor(pos)), f = pos - i, t = frames[i].t;
    view.draw(i, f, t);
    drawTimeline(pos);
    if (i !== lastUi) { renderUi(i); lastUi = i; slider.value = i; tlabel.textContent = `t = ${t}`; }
  }
  function seek(i) { pos = Math.max(0, Math.min(LAST, i)); lastUi = -1; render(); }
  function setPlaying(p) {
    playing = p; playBtn.textContent = p ? "❚❚ Pause" : "▶ Play"; playBtn.setAttribute("aria-label", p ? "Pause" : "Play");
    if (p && pos >= LAST) pos = 0;
    lastTs = null; if (p) requestAnimationFrame(loop);
  }
  function loop(ts) {
    if (!playing) return;
    if (lastTs !== null) {
      pos += (ts - lastTs) / 1000 * Number(speedSel.value);
      if (pos >= LAST) { pos = LAST; render(); setPlaying(false); return; }
    }
    lastTs = ts; render(); requestAnimationFrame(loop);
  }
  playBtn.onclick = () => setPlaying(!playing);
  document.getElementById("back").onclick = () => { setPlaying(false); seek(Math.ceil(pos) - 1); };
  document.getElementById("fwd").onclick = () => { setPlaying(false); seek(Math.floor(pos) + 1); };
  slider.oninput = () => { setPlaying(false); seek(Number(slider.value)); };
  document.querySelectorAll("input[data-opt]").forEach(el => { el.onchange = render; });
  addEventListener("keydown", ev => {
    if (ev.target.tagName === "SELECT") return;
    if (ev.target.tagName === "INPUT" && ev.target.type !== "checkbox" && ev.target.type !== "range") return;
    if (ev.code === "Space") { ev.preventDefault(); setPlaying(!playing); }
    else if (ev.key === "ArrowRight") { ev.preventDefault(); setPlaying(false); seek(Math.floor(pos) + (ev.shiftKey ? 10 : 1)); }
    else if (ev.key === "ArrowLeft") { ev.preventDefault(); setPlaying(false); seek(Math.ceil(pos) - (ev.shiftKey ? 10 : 1)); }
    else if (ev.key === "Escape") select(-1);
  });

  // ---------------------------------------------------------- routes
  function planAt(did, t) {
    const arr = DATA.plans[did];
    if (!arr || !arr.length) return null;
    let lo = 0, hi = arr.length - 1, best = -1;
    while (lo <= hi) { const mid = (lo + hi) >> 1; if (arr[mid][0] <= t) { best = mid; lo = mid + 1; } else hi = mid - 1; }
    return best < 0 ? null : arr[best];
  }
  /* The part of drone k's booked route still ahead at tick t, as [[x, y, z], ...]
     (starting with its current cell), or null if it is not following a route. */
  function routeAhead(k, t, d) {
    const p = planAt(k, t);
    if (!p) return null;
    const idx = t - p[0], cells = p[1];
    if (idx < 0 || idx >= cells.length || cells[idx] !== enc(d[0], d[1], altOf(d))) return null;
    const out = [];
    for (let j = idx; j < cells.length; j++) {
      const c = decode(cells[j]);
      const last = out[out.length - 1];
      if (!last || last[0] !== c[0] || last[1] !== c[1] || last[2] !== c[2]) out.push(c);
    }
    return out;
  }

  const api = {
    W, H, L, frames, LAST, N, STATES, FLAG, TRIP, T, FONT, DATA, enc, decode, altOf, heights, hubs, stations,
    CONT, TICK_S, CELL_M, TRACK, trackAt, trailAt, losAt, orcaAt, windAt, CUST_H, custHeight,
    describe, colorOf, socColor, textOn, esc, placeName, placeByEnc, orders, orderById, flashes, byDrone,
    routeAhead, opt, select, showTip, hideTip, droneTip, render,
    get selected() { return selected; }, get pos() { return pos; }, get playing() { return playing; },
  };

  // open on a busy moment in the first half rather than an empty t = 0
  const airborne = f => f.d.filter(d => d[2]).length;
  let busiest = 0;
  for (let i = 0; i < LAST * .5; i++) if (airborne(frames[i]) > airborne(frames[busiest])) busiest = i;
  pos = busiest;
  readTokens();
  view.init(api);
  ready = true;
  sizeTimeline();
  applyTheme();
  return api;
}
