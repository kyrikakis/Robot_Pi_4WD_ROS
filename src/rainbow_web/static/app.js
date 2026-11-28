/* Rainbow's phone remote. Talks to rainbow_web's WebSocket (/ws); see web_node.py. */
'use strict';

const $ = (sel) => document.querySelector(sel);
const $$ = (sel) => Array.from(document.querySelectorAll(sel));

const FOOTPRINT = { length: 0.19, width: 0.14 };    // m (dimensions.xacro)
const BATTERY = { empty: 6.9, full: 8.0 };         // V (the owner's pack)
const DRIVE_HZ = 20;
const HEAD_HZ = 15;

const store = {
  get(key, fallback) {
    try { const v = localStorage.getItem('rainbow.' + key); return v === null ? fallback : JSON.parse(v); }
    catch (e) { return fallback; }
  },
  set(key, value) { try { localStorage.setItem('rainbow.' + key, JSON.stringify(value)); } catch (e) { /* private mode */ } },
};

const state = {
  ws: null,
  connected: false,
  mainView: store.get('mainView', 'camera'),   // 'camera' | 'map'
  pip: store.get('pip', true),
  quality: store.get('quality', 'low'),
  speed: store.get('speed', 60),
  layers: Object.assign({ scan: true, plan: true, places: true, global_costmap: false,
    local_costmap: false }, store.get('layers', {})),
  status: null,
  tracker: null,
  flags: {},
  places: [],
  placesMap: '',
  nav: null,
  callId: 0,
  pending: {},
};

/* ---------------------------------------------------------------- connection */

function connect() {
  const ws = new WebSocket(`ws://${location.host}/ws`);
  ws.binaryType = 'blob';
  state.ws = ws;
  ws.onopen = () => {
    state.connected = true;
    $('#conn').classList.add('ok');
    $('#offline').classList.add('hidden');
    sendWant(true);
  };
  ws.onclose = () => {
    state.connected = false;
    $('#conn').classList.remove('ok');
    $('#chip-rtt').classList.add('hidden');
    $('#offline').classList.remove('hidden');
    sticks.forEach((s) => s.release());
    lastWant = '';
    setTimeout(connect, 1000);
  };
  ws.onerror = () => ws.close();
  ws.onmessage = (ev) => {
    if (typeof ev.data === 'string') onMessage(JSON.parse(ev.data));
    else camera.frame(ev.data);
  };
}

function send(msg) {
  if (state.ws && state.ws.readyState === WebSocket.OPEN) {
    state.ws.send(JSON.stringify(msg));
    return true;
  }
  return false;
}

function call(op, args = {}, quiet = false) {
  const id = ++state.callId;
  return new Promise((resolve) => {
    if (!send({ t: 'call', id, op, args })) {
      toast('Not connected', true);
      resolve({ ok: false });
      return;
    }
    state.pending[id] = (res) => {
      if (!quiet || !res.ok) toast(res.message || (res.ok ? 'done' : 'failed'), !res.ok);
      resolve(res);
    };
  });
}

let lastWant = '';
function sendWant(force = false) {
  const visible = document.visibilityState === 'visible';
  const shown = (v) => visible && (state.mainView === v || state.pip);
  const mapShown = shown('map');
  const want = {
    t: 'want',
    camera: shown('camera'),
    map: mapShown,
    global_costmap: mapShown && state.layers.global_costmap,
    local_costmap: mapShown && state.layers.local_costmap,
    quality: state.quality,
  };
  const key = JSON.stringify(want);
  if (force || key !== lastWant) {
    if (send(want)) lastWant = key;
  }
}

function onMessage(m) {
  switch (m.t) {
    case 'status': state.status = m; renderStatus(); break;
    case 'flag': state.flags[m.name] = m.value; renderStatus(); break;
    case 'tracker': state.tracker = m; renderStatus(); break;
    case 'map': mapView.setMap(m); break;
    case 'costmap': mapView.setCostmap(m); break;
    case 'pose': {
      const first = !mapView.pose;
      mapView.pose = m;
      if (first) mapView.fit();
      mapView.dirty();
      break;
    }
    case 'scan': mapView.scan = m; mapView.scanAt = performance.now(); mapView.dirty(); break;
    case 'plan': mapView.plan = m.points; mapView.dirty(); break;
    case 'places': state.places = m.places; state.placesMap = m.map; renderPlaces(); mapView.dirty(); break;
    case 'nav': onNav(m); break;
    case 'log': addLog(m); break;
    case 'logs': m.entries.forEach(addLog); break;
    case 'pong': {
      const rtt = Math.round(performance.now() - m.time);
      chip('#chip-rtt', `${rtt} ms`, rtt < 150 ? 'ok' : rtt < 400 ? 'warn' : 'bad');
      $('#chip-rtt').classList.remove('hidden');
      break;
    }
    case 'result': {
      const cb = state.pending[m.id];
      delete state.pending[m.id];
      if (cb) cb(m);
      break;
    }
    default: break;
  }
}

/* ---------------------------------------------------------------- toast */

let toastTimer = 0;
function toast(text, bad = false) {
  const el = $('#toast');
  el.textContent = text;
  el.classList.toggle('bad', bad);
  el.classList.remove('hidden');
  clearTimeout(toastTimer);
  toastTimer = setTimeout(() => el.classList.add('hidden'), bad ? 4000 : 2200);
}

/* ---------------------------------------------------------------- views */

function layoutViews() {
  // sizes come from the CSS (.in-pip is the small view); this only says which is which
  $('#pip').classList.toggle('hidden', !state.pip);
  const camMain = state.mainView === 'camera';
  const place = (el, note, inPip) => {
    const visible = !inPip || state.pip;
    for (const e of [el, note]) e.classList.toggle('in-pip', inPip);
    el.classList.toggle('off', !visible);
    note.dataset.slotVisible = visible ? '1' : '';
  };
  place($('#camera'), $('#camera-empty'), !camMain);
  place($('#map'), $('#map-empty'), camMain);
  $('#maptools').classList.toggle('hidden', camMain);
  if (camMain) mapView.setTool(null);
  mapView.resize();
  camera.updateNote();
  mapView.updateNote();
  sendWant();
}

function swapViews() {
  state.mainView = state.mainView === 'camera' ? 'map' : 'camera';
  store.set('mainView', state.mainView);
  layoutViews();
}

/* ---------------------------------------------------------------- camera */

const camera = {
  el: $('#camera'),
  url: null,
  lastFrame: 0,
  frame(blob) {
    const url = URL.createObjectURL(blob);
    const old = this.url;
    this.url = url;
    this.el.onload = () => { if (old) URL.revokeObjectURL(old); };
    this.el.src = url;
    this.lastFrame = performance.now();
    this.updateNote();
  },
  updateNote() {
    const note = $('#camera-empty');
    const fresh = performance.now() - this.lastFrame < 3000;
    note.classList.toggle('hidden', fresh || !note.dataset.slotVisible);
    note.textContent = state.connected ? 'No camera picture' : '';
  },
};
setInterval(() => camera.updateNote(), 1000);
setInterval(() => send({ t: 'ping', time: performance.now() }), 2000);

/* ---------------------------------------------------------------- map */

const mapView = {
  canvas: $('#map'),
  ctx: $('#map').getContext('2d'),
  dpr: 1,
  w: 0,
  h: 0,
  map: null,          // {width, height, resolution, origin, img}
  costmaps: {},
  pose: null,
  scan: null,
  scanAt: 0,
  plan: [],
  center: null,       // world point at the canvas centre
  scale: 60,          // px per metre
  follow: true,
  headingUp: false,
  fitted: true,       // zoom chosen automatically (until the user zooms)
  tool: null,         // 'goal' | 'pose' | 'place'
  draft: null,        // {x, y, yaw} while placing an arrow
  goal: null,
  needsDraw: false,

  setMap(m) {
    const img = new Image();
    img.onload = () => {
      this.map = Object.assign({}, m, { img });
      delete this.map.png;
      this.fit();
      this.updateNote();
      this.dirty();
    };
    img.src = 'data:image/png;base64,' + m.png;
  },
  setCostmap(m) {
    const img = new Image();
    img.onload = () => {
      this.costmaps[m.name] = Object.assign({}, m, { img });
      delete this.costmaps[m.name].png;
      this.dirty();
    };
    img.src = 'data:image/png;base64,' + m.png;
  },
  // whole map in view, or with a pose ~3 m around the robot
  fit() {
    if (!this.fitted || !this.w || !this.h) return;
    const m = this.map;
    if (this.pose && this.follow) {
      this.scale = Math.min(this.w, this.h) / 3;
    } else if (m) {
      if (!this.follow || !this.pose) {
        this.center = { x: m.origin[0] + m.width * m.resolution / 2,
          y: m.origin[1] + m.height * m.resolution / 2 };
      }
      this.scale = Math.min(this.w / (m.width * m.resolution),
        this.h / (m.height * m.resolution)) * 0.92;
    }
    this.dirty();
  },
  updateNote() {
    const note = $('#map-empty');
    note.classList.toggle('hidden', !!this.map || !note.dataset.slotVisible);
  },
  resize() {
    const r = this.canvas.getBoundingClientRect();
    this.dpr = window.devicePixelRatio || 1;
    this.w = r.width;
    this.h = r.height;
    this.canvas.width = Math.round(r.width * this.dpr);
    this.canvas.height = Math.round(r.height * this.dpr);
    this.fit();
    this.dirty();
  },
  dirty() {
    if (!this.needsDraw) {
      this.needsDraw = true;
      requestAnimationFrame(() => this.draw());
    }
  },
  rotation() {
    if (this.headingUp && this.pose) return this.pose.yaw - Math.PI / 2;
    return 0;
  },
  // world (x, y) -> canvas px
  toScreen(x, y) {
    const c = this.center || { x: 0, y: 0 };
    const rot = this.rotation();
    const dx = x - c.x;
    const dy = y - c.y;
    const cos = Math.cos(-rot);
    const sin = Math.sin(-rot);
    const rx = dx * cos - dy * sin;
    const ry = dx * sin + dy * cos;
    return [this.w / 2 + rx * this.scale, this.h / 2 - ry * this.scale];
  },
  toWorld(px, py) {
    const c = this.center || { x: 0, y: 0 };
    const rot = this.rotation();
    const rx = (px - this.w / 2) / this.scale;
    const ry = -(py - this.h / 2) / this.scale;
    const cos = Math.cos(rot);
    const sin = Math.sin(rot);
    return { x: c.x + rx * cos - ry * sin, y: c.y + rx * sin + ry * cos };
  },
  screenAngle(yaw) { return -(yaw - this.rotation()); },
  setTool(tool) {
    this.tool = tool;
    this.draft = null;
    $$('#maptools .mode').forEach((b) => b.classList.toggle('on', b.dataset.tool === tool));
    const hints = {
      goal: 'Tap where to go, drag to set the direction',
      pose: 'Tap where the robot is, drag the way it faces',
      place: 'Tap the place, drag the direction, then name it',
    };
    $('#hint').textContent = hints[tool] || '';
    $('#hint').classList.toggle('hidden', !tool);
    this.dirty();
  },

  draw() {
    this.needsDraw = false;
    const ctx = this.ctx;
    ctx.setTransform(this.dpr, 0, 0, this.dpr, 0, 0);
    ctx.clearRect(0, 0, this.w, this.h);
    if (this.follow && this.pose) this.center = { x: this.pose.x, y: this.pose.y };
    if (!this.center) return;
    const rot = this.rotation();

    // world transform: metres, y up
    ctx.save();
    ctx.translate(this.w / 2, this.h / 2);
    ctx.scale(this.scale, -this.scale);
    ctx.rotate(-rot);
    ctx.translate(-this.center.x, -this.center.y);
    ctx.imageSmoothingEnabled = this.scale < 40;
    const grid = (g) => {
      ctx.save();
      ctx.translate(g.origin[0], g.origin[1]);
      ctx.rotate(g.origin[2]);
      ctx.scale(g.resolution, -g.resolution);
      ctx.drawImage(g.img, 0, -g.height);
      ctx.restore();
    };
    if (this.map) grid(this.map);
    for (const name of ['global_costmap', 'local_costmap']) {
      if (state.layers[name] && this.costmaps[name]) grid(this.costmaps[name]);
    }
    const px = 1 / this.scale;    // one screen pixel in metres

    if (state.layers.plan && this.plan && this.plan.length > 1) {
      ctx.beginPath();
      ctx.moveTo(this.plan[0][0], this.plan[0][1]);
      for (const p of this.plan) ctx.lineTo(p[0], p[1]);
      ctx.strokeStyle = '#3ddc84';
      ctx.lineWidth = 3 * px;
      ctx.lineJoin = 'round';
      ctx.stroke();
    }

    if (state.layers.scan && this.scan) {
      const age = (performance.now() - this.scanAt) / 1000;
      if (age < 3) {
        const s = this.scan;
        const [lx, ly, lyaw] = s.pose;
        ctx.fillStyle = `rgba(255, 64, 64, ${age < 1 ? 0.95 : 0.4})`;
        const d = Math.max(2.5 * px, 0.012);
        for (let i = 0; i < s.ranges.length; i++) {
          const r = s.ranges[i];
          if (!r) continue;
          const a = lyaw + s.angle_min + i * s.angle_inc;
          ctx.fillRect(lx + Math.cos(a) * r / 100 - d / 2, ly + Math.sin(a) * r / 100 - d / 2, d, d);
        }
      }
    }

    if (this.pose) {
      ctx.save();
      ctx.translate(this.pose.x, this.pose.y);
      ctx.rotate(this.pose.yaw);
      const L = FOOTPRINT.length;
      const W = FOOTPRINT.width;
      ctx.fillStyle = 'rgba(74, 163, 255, 0.85)';
      ctx.strokeStyle = '#fff';
      ctx.lineWidth = 1.5 * px;
      ctx.beginPath();
      ctx.rect(-L / 2, -W / 2, L, W);
      ctx.fill();
      ctx.stroke();
      ctx.beginPath();
      ctx.moveTo(L / 2 + 0.05, 0);
      ctx.lineTo(L / 2 - 0.03, W / 2 - 0.02);
      ctx.lineTo(L / 2 - 0.03, -W / 2 + 0.02);
      ctx.closePath();
      ctx.fillStyle = '#fff';
      ctx.fill();
      ctx.restore();
    }
    ctx.restore();

    // screen-space overlays: places, goal, draft arrow
    if (state.layers.places) {
      ctx.font = '600 12px system-ui, sans-serif';
      ctx.textAlign = 'center';
      const boxes = [];
      for (const p of state.places) {
        const [sx, sy] = this.toScreen(p.x, p.y);
        if (sx < -50 || sy < -50 || sx > this.w + 50 || sy > this.h + 50) continue;
        this.arrow(sx, sy, this.screenAngle(p.yaw), p.confirmed ? '#f5b942' : '#b39ddb', 9);
        const tw = ctx.measureText(p.name).width + 6;
        const box = [sx - tw / 2, sy - 28, sx + tw / 2, sy - 12];
        if (boxes.some((b) => box[0] < b[2] && b[0] < box[2] && box[1] < b[3] && b[1] < box[3])) continue;
        boxes.push(box);
        ctx.lineWidth = 3;
        ctx.strokeStyle = 'rgba(0,0,0,0.8)';
        ctx.strokeText(p.name, sx, sy - 14);
        ctx.fillStyle = '#fff';
        ctx.fillText(p.name, sx, sy - 14);
      }
    }
    if (this.goal) {
      const [sx, sy] = this.toScreen(this.goal.x, this.goal.y);
      this.arrow(sx, sy, this.screenAngle(this.goal.yaw), '#3ddc84', 11);
    }
    if (this.draft) {
      const colours = { goal: '#3ddc84', pose: '#4aa3ff', place: '#f5b942' };
      const [sx, sy] = this.toScreen(this.draft.x, this.draft.y);
      this.arrow(sx, sy, this.screenAngle(this.draft.yaw), colours[this.tool] || '#fff', 14, true);
    }
    // scale bar
    const metres = [0.1, 0.2, 0.5, 1, 2, 5].find((m) => m * this.scale > 50) || 5;
    const small = this.w < 400;
    const bx = small ? 8 : 14;
    const by = small ? this.h - 8 : 72;
    ctx.fillStyle = 'rgba(255,255,255,0.8)';
    ctx.fillRect(bx, by, metres * this.scale, 3);
    ctx.font = '11px system-ui, sans-serif';
    ctx.textAlign = 'left';
    ctx.fillText(metres >= 1 ? `${metres} m` : `${metres * 100} cm`, bx, by - 5);
  },
  arrow(x, y, angle, colour, size, long = false) {
    const ctx = this.ctx;
    ctx.save();
    ctx.translate(x, y);
    ctx.rotate(angle);
    ctx.beginPath();
    ctx.arc(0, 0, size * 0.55, 0, Math.PI * 2);
    ctx.fillStyle = colour;
    ctx.fill();
    ctx.strokeStyle = '#000';
    ctx.lineWidth = 1.5;
    ctx.stroke();
    const tip = long ? size * 3 : size * 1.6;
    ctx.beginPath();
    ctx.moveTo(tip, 0);
    ctx.lineTo(size * 0.3, size * 0.6);
    ctx.lineTo(size * 0.3, -size * 0.6);
    ctx.closePath();
    ctx.fill();
    ctx.stroke();
    ctx.restore();
  },
};

/* map gestures: drag to pan, pinch to zoom, tap a place, tools place an arrow */
(function mapGestures() {
  const el = mapView.canvas;
  const pts = new Map();
  let pinch = null;
  let moved = false;
  let downAt = null;
  const local = (e) => {
    const r = el.getBoundingClientRect();
    return [e.clientX - r.left, e.clientY - r.top];
  };
  const isMain = () => state.mainView === 'map';

  el.addEventListener('pointerdown', (e) => {
    if (!isMain()) return;
    el.setPointerCapture(e.pointerId);
    const p = local(e);
    pts.set(e.pointerId, p);
    moved = false;
    downAt = p;
    if (mapView.tool && pts.size === 1) {
      const w = mapView.toWorld(p[0], p[1]);
      mapView.draft = { x: w.x, y: w.y, yaw: mapView.pose ? mapView.pose.yaw : 0 };
      mapView.dirty();
    }
    if (pts.size === 2) {
      const [a, b] = Array.from(pts.values());
      pinch = { dist: Math.hypot(a[0] - b[0], a[1] - b[1]), scale: mapView.scale };
      mapView.draft = null;
    }
  });
  el.addEventListener('pointermove', (e) => {
    if (!pts.has(e.pointerId)) return;
    const prev = pts.get(e.pointerId);
    const p = local(e);
    pts.set(e.pointerId, p);
    if (downAt && Math.hypot(p[0] - downAt[0], p[1] - downAt[1]) > 8) moved = true;
    if (pts.size === 2 && pinch) {
      const [a, b] = Array.from(pts.values());
      const d = Math.hypot(a[0] - b[0], a[1] - b[1]);
      mapView.scale = Math.min(800, Math.max(4, pinch.scale * d / pinch.dist));
      mapView.fitted = false;
      mapView.dirty();
      return;
    }
    if (pts.size !== 1) return;
    if (mapView.tool && mapView.draft) {
      const w = mapView.toWorld(p[0], p[1]);
      if (Math.hypot(w.x - mapView.draft.x, w.y - mapView.draft.y) > 0.03) {
        mapView.draft.yaw = Math.atan2(w.y - mapView.draft.y, w.x - mapView.draft.x);
      }
      mapView.dirty();
      return;
    }
    if (moved) {
      const a = mapView.toWorld(prev[0], prev[1]);
      const b = mapView.toWorld(p[0], p[1]);
      if (!mapView.center) return;
      mapView.center = { x: mapView.center.x - (b.x - a.x), y: mapView.center.y - (b.y - a.y) };
      setFollow(false);
      mapView.fitted = false;
      mapView.dirty();
    }
  });
  const end = (e) => {
    if (!pts.has(e.pointerId)) return;
    const p = local(e);
    pts.delete(e.pointerId);
    if (pts.size < 2) pinch = null;
    if (pts.size > 0) return;
    if (mapView.tool && mapView.draft) {
      finishTool(mapView.tool, mapView.draft);
      return;
    }
    if (!moved) tapMap(p);
  };
  el.addEventListener('pointerup', end);
  el.addEventListener('pointercancel', (e) => { pts.delete(e.pointerId); pinch = null; mapView.draft = null; mapView.dirty(); });
  el.addEventListener('wheel', (e) => {
    mapView.scale = Math.min(800, Math.max(4, mapView.scale * (e.deltaY < 0 ? 1.15 : 0.87)));
    mapView.fitted = false;
    mapView.dirty();
    e.preventDefault();
  }, { passive: false });
})();

function setFollow(on) {
  mapView.follow = on;
  $('[data-tool=follow]').classList.toggle('on', on);
  mapView.dirty();
}

function tapMap(p) {
  if (!state.layers.places) return;
  let best = null;
  let bestD = 26;
  for (const place of state.places) {
    const [sx, sy] = mapView.toScreen(place.x, place.y);
    const d = Math.hypot(sx - p[0], sy - p[1]);
    if (d < bestD) { best = place; bestD = d; }
  }
  if (best) openPlaceSheet(best);
  else closeSheet();
}

function finishTool(tool, draft) {
  const args = { x: draft.x, y: draft.y, yaw: draft.yaw };
  mapView.setTool(null);
  if (tool === 'goal') {
    mapView.goal = args;
    call('goal', args);
  } else if (tool === 'pose') {
    call('initialpose', args);
  } else if (tool === 'place') {
    mapView.draft = Object.assign({}, args);
    mapView.tool = 'place';
    mapView.dirty();
    openSheet('New place', `at ${args.x.toFixed(2)}, ${args.y.toFixed(2)}`, [], (box) => {
      const input = document.createElement('input');
      input.placeholder = 'Name, e.g. kitchen';
      box.before(input);
      setTimeout(() => input.focus(), 50);
      const save = button('Save', async () => {
        const name = input.value.trim();
        if (!name) { input.focus(); return; }
        closeSheet();
        await call('place_save', Object.assign({ name }, args));
      });
      save.classList.add('on');
      box.append(save, button('Cancel', closeSheet));
      input.addEventListener('keydown', (e) => { if (e.key === 'Enter') save.click(); });
    }, () => { mapView.draft = null; mapView.tool = null; mapView.dirty(); });
  }
}

/* ---------------------------------------------------------------- sheet */

let sheetOnClose = null;
function button(label, fn) {
  const b = document.createElement('button');
  b.textContent = label;
  b.addEventListener('click', fn);
  return b;
}
function openSheet(title, sub, buttons, build, onClose) {
  closeSheet();
  $('#sheet-title').textContent = title;
  $('#sheet-sub').textContent = sub || '';
  const box = $('#sheet-buttons');
  box.replaceChildren(...buttons);
  $$('#sheet input').forEach((i) => i.remove());
  if (build) build(box);
  sheetOnClose = onClose || null;
  $('#sheet').classList.remove('hidden');
}
function closeSheet() {
  const sheet = $('#sheet');
  if (sheet.classList.contains('hidden')) return;
  sheet.classList.add('hidden');
  $$('#sheet input').forEach((i) => i.remove());
  const cb = sheetOnClose;
  sheetOnClose = null;
  if (cb) cb();
}
function openPlaceSheet(p) {
  const go = button('Go here', () => { closeSheet(); goToPlace(p.name); });
  go.classList.add('on');
  const del = button('Delete', () => {
    openSheet(`Delete ${p.name}?`, 'It is removed from this map\'s places.', [
      button('Delete', async () => { closeSheet(); await call('place_delete', { name: p.name }); }),
      button('Keep', closeSheet),
    ]);
  });
  del.classList.add('warn');
  const sub = [p.room, p.description, p.confirmed ? '' : 'robot\'s guess'].filter(Boolean).join(' · ');
  openSheet(p.name, sub, [go, del, button('Close', closeSheet)]);
}

/* ---------------------------------------------------------------- navigation */

function goToPlace(name) {
  const p = state.places.find((q) => q.name === name);
  if (p) mapView.goal = { x: p.x, y: p.y, yaw: p.yaw };
  mapView.dirty();
  call('place_go', { place: name });
}

function onNav(m) {
  state.nav = m.active ? m : null;
  const banner = $('#nav-banner');
  if (m.active) {
    let text = `Going to ${m.place}`;
    if (m.remaining !== undefined) text += ` · ${m.remaining.toFixed(1)} m`;
    $('#nav-text').textContent = text;
    banner.classList.remove('hidden');
  } else {
    banner.classList.add('hidden');
    mapView.goal = null;
    mapView.dirty();
    if (m.message) toast(m.message, !m.ok);
  }
}

/* ---------------------------------------------------------------- joysticks */

class Stick {
  constructor(zone, hz, onMove, onStart) {
    this.zone = zone;
    this.base = zone.querySelector('.stick');
    this.knob = zone.querySelector('.knob');
    this.hz = hz;
    this.onMove = onMove;
    this.onStart = onStart;
    this.id = null;
    this.value = [0, 0];
    this.timer = 0;
    this.lastTap = 0;
    zone.addEventListener('pointerdown', (e) => this.down(e));
    zone.addEventListener('pointermove', (e) => this.move(e));
    zone.addEventListener('pointerup', (e) => this.up(e));
    zone.addEventListener('pointercancel', (e) => this.up(e));
    zone.addEventListener('lostpointercapture', (e) => this.up(e));
  }
  radius() { return this.base.getBoundingClientRect().width / 2; }
  down(e) {
    if (this.id !== null || e.target.closest('button')) return;
    e.preventDefault();
    this.id = e.pointerId;
    this.zone.setPointerCapture(e.pointerId);
    const zr = this.zone.getBoundingClientRect();
    const r = this.radius();
    // the stick appears under the finger (kept inside the zone)
    const cx = Math.min(Math.max(e.clientX - zr.left, r + 4), zr.width - r - 4);
    const cy = Math.min(Math.max(e.clientY - zr.top, r + 4), zr.height - r - 4);
    this.base.style.left = cx + 'px';
    this.base.style.top = cy + 'px';
    this.base.classList.add('active');
    this.origin = [zr.left + cx, zr.top + cy];
    const now = performance.now();
    this.doubleTap = now - this.lastTap < 300;
    this.lastTap = now;
    if (navigator.vibrate) navigator.vibrate(8);
    if (this.onStart) this.onStart(this.doubleTap);
    this.update(e);
    this.timer = setInterval(() => this.onMove(this.value[0], this.value[1]), 1000 / this.hz);
  }
  move(e) {
    if (e.pointerId !== this.id) return;
    this.update(e);
  }
  update(e) {
    const r = this.radius();
    let dx = e.clientX - this.origin[0];
    let dy = e.clientY - this.origin[1];
    const d = Math.hypot(dx, dy);
    if (d > r) { dx *= r / d; dy *= r / d; }
    this.knob.style.transform = `translate(${dx}px, ${dy}px)`;
    const shape = (v) => {
      const a = Math.abs(v);
      if (a < 0.08) return 0;
      const t = (a - 0.08) / 0.92;
      return Math.sign(v) * t * (0.4 + 0.6 * t);   // fine control near the centre
    };
    this.value = [shape(dx / r), shape(-dy / r)];
  }
  up(e) {
    if (e.pointerId !== this.id) return;
    this.release();
  }
  release() {
    if (this.id === null) return;
    this.id = null;
    clearInterval(this.timer);
    this.value = [0, 0];
    this.knob.style.transform = '';
    this.base.style.left = '';
    this.base.style.top = '';
    this.base.classList.remove('active');
    // a few zeros, in case one is lost
    this.onMove(0, 0);
    setTimeout(() => this.onMove(0, 0), 60);
    setTimeout(() => this.onMove(0, 0), 150);
  }
}

const driveStick = new Stick($('#zone-drive'), DRIVE_HZ, (x, y) => {
  send({ t: 'drive', x: y, z: -x, speed: state.speed / 100 });
}, () => {
  // Taking the stick cancels what the robot is doing on its own (a trip, a turn)
  const s = state.status;
  if (state.nav || (s && s.activity && s.activity !== 'idle')) call('stop', {}, true);
});
const headStick = new Stick($('#zone-head'), HEAD_HZ, (x, y) => {
  send({ t: 'head', x, y });
}, (doubleTap) => {
  if (state.tracker && state.tracker.active) call('track_stop', {}, true);
  if (doubleTap) call('head_center', {}, true);
});
const sticks = [driveStick, headStick];

/* ---------------------------------------------------------------- status */

function chip(id, text, cls) {
  const el = $(id);
  el.textContent = text;
  el.classList.remove('ok', 'warn', 'bad');
  if (cls) el.classList.add(cls);
}

function renderStatus() {
  const s = state.status;
  const f = state.flags;
  if (s) {
    const v = s.battery;
    if (v > 0) {
      const pct = Math.round(100 * Math.min(1, Math.max(0, (v - BATTERY.empty) / (BATTERY.full - BATTERY.empty))));
      chip('#chip-battery', `🔋 ${pct}% ${v.toFixed(1)}V`, pct > 40 ? 'ok' : pct > 15 ? 'warn' : 'bad');
    } else chip('#chip-battery', '🔋 --', 'bad');
    const loc = { sure: 'ok', roughly: 'warn' }[s.certainty] || 'bad';
    chip('#chip-loc', `📍 ${s.certainty || '?'}`, loc);
    chip('#chip-nav', 'Nav', s.nav2 ? 'ok' : 'bad');
    const espOk = s.battery_age >= 0 && s.battery_age < 3;
    chip('#chip-esp', 'ESP', espOk ? 'ok' : 'bad');
    let act = s.activity && s.activity !== 'idle' ? s.activity : '';
    if (state.tracker && state.tracker.active) act = (act ? act + ' · ' : '') + `following ${state.tracker.target}`;
    $('#activity').textContent = act;
  }
  if (f.temperature !== undefined) {
    const t = f.temperature;
    chip('#chip-temp', `${Math.round(t)}°C`, t < 70 ? 'ok' : t < 80 ? 'warn' : 'bad');
  }
  $('#chip-lifted').classList.toggle('hidden', !f.lifted);
  renderStatusTab();
}

function renderStatusTab() {
  if (!$('#tab-status').classList.contains('on') || !$('#drawer').classList.contains('open')) return;
  const s = state.status || {};
  const f = state.flags;
  const t = state.tracker || {};
  const deg = (r) => (r * 180 / Math.PI).toFixed(0) + '°';
  const age = (a) => (a < 0 ? 'none yet' : `${a.toFixed(1)} s ago`);
  const rows = [
    ['Activity', s.activity],
    ['Mode', s.mode],
    ['Map', s.map],
    ['Localization', `${s.certainty || '?'}${s.position_std >= 0 ? ` (±${(s.position_std * 100).toFixed(0)} cm, ±${deg(s.yaw_std)})` : ''}`],
    ['Pose', s.x !== undefined ? `${s.x.toFixed(2)}, ${s.y.toFixed(2)}, ${deg(s.yaw)}` : ''],
    ['Battery', s.battery ? `${s.battery.toFixed(2)} V (${age(s.battery_age)})` : 'no reading'],
    ['Lidar', s.scan_age !== undefined ? age(s.scan_age) + ' (off at rest)' : ''],
    ['Wheel odometry', s.odom_age !== undefined ? age(s.odom_age) : ''],
    ['Nav2', s.nav2 ? 'ready' : 'down'],
    ['Speech', s.speech ? 'ready' : 'down'],
    ['Faces', s.faces ? 'ready' : 'down'],
    ['Places', s.places],
    ['Head', f.head_moving ? 'moving' : 'still'],
    ['Tracker', t.active ? `${t.target}${t.seen ? ' (seen)' : t.lost ? ' (lost)' : ''}` : 'off'],
    ['Lifted', f.lifted ? 'yes' : 'no'],
    ['Pi temperature', f.temperature !== undefined ? `${f.temperature} °C` : ''],
    ['ESP firmware', f.firmware],
    ['Last event', s.last_event],
    ['Last command', s.last_command],
    ['Brain uptime', s.uptime !== undefined ? `${Math.floor(s.uptime / 60)} min` : ''],
  ];
  if (s.agent) {
    for (const [k, v] of Object.entries(s.agent)) rows.push([`Agent ${k.replace(/_/g, ' ')}`, String(v)]);
  }
  const dl = $('#status');
  dl.replaceChildren();
  for (const [k, v] of rows) {
    if (v === undefined || v === '') continue;
    const dt = document.createElement('dt');
    dt.textContent = k;
    const dd = document.createElement('dd');
    dd.textContent = v;
    dl.append(dt, dd);
  }
}

/* ---------------------------------------------------------------- places tab */

function renderPlaces() {
  $('#places-map').textContent = state.placesMap ? `Map: ${state.placesMap}` : '';
  const ul = $('#places');
  ul.replaceChildren();
  if (!state.places.length) {
    const li = document.createElement('li');
    li.className = 'muted';
    li.textContent = 'No places on this map yet.';
    ul.append(li);
    return;
  }
  for (const p of state.places) {
    const li = document.createElement('li');
    const name = document.createElement('div');
    name.className = 'name';
    name.textContent = p.name;
    const sub = [p.room, p.description, p.confirmed ? '' : 'robot\'s guess'].filter(Boolean).join(' · ');
    if (sub) {
      const small = document.createElement('small');
      small.textContent = sub;
      name.append(small);
    }
    const go = button('Go', () => { closeDrawer(); goToPlace(p.name); });
    go.classList.add('on');
    const del = button('✕', () => {
      if (confirm(`Delete the place "${p.name}"?`)) call('place_delete', { name: p.name });
    });
    li.append(name, go, del);
    ul.append(li);
  }
}

/* ---------------------------------------------------------------- log */

function addLog(m) {
  const ul = $('#log');
  const li = document.createElement('li');
  li.className = m.level;
  const time = new Date(m.time * 1000).toLocaleTimeString();
  const src = document.createElement('span');
  src.className = 'src';
  src.textContent = `${time} ${m.source === 'agent' ? 'agent' : m.name} `;
  li.append(src, document.createTextNode(m.msg));
  ul.prepend(li);
  while (ul.children.length > 100) ul.lastChild.remove();
}

/* ---------------------------------------------------------------- drawer & controls */

function openDrawer() { $('#drawer').classList.add('open'); renderStatusTab(); }
function closeDrawer() { $('#drawer').classList.remove('open'); }

function setSpeed(v) {
  state.speed = v;
  store.set('speed', v);
  $('#speed').textContent = `${v}%`;
  $('#speed-range').value = v;
  $('#speed-val').textContent = `${v}%`;
}

function setup() {
  $('#stop').addEventListener('click', () => {
    sticks.forEach((s) => s.release());
    send({ t: 'drive', x: 0, z: 0, speed: 0 });
    call('stop');
  });
  $('#btn-swap').addEventListener('click', swapViews);
  $('#pip').addEventListener('click', swapViews);
  // the view shown small sits on top of #pip: a tap on it swaps too
  $('#camera').addEventListener('click', () => { if (state.mainView !== 'camera') swapViews(); });
  mapView.canvas.addEventListener('click', () => { if (state.mainView !== 'map') swapViews(); });
  $('#btn-menu').addEventListener('click', openDrawer);
  $('#drawer-close').addEventListener('click', closeDrawer);
  $('#btn-full').addEventListener('click', toggleFullscreen);
  $('#nav-cancel').addEventListener('click', () => call('stop'));

  $$('.tabs [data-tab]').forEach((b) => b.addEventListener('click', () => {
    $$('.tabs [data-tab]').forEach((x) => x.classList.toggle('on', x === b));
    $$('.tab').forEach((t) => t.classList.toggle('on', t.id === `tab-${b.dataset.tab}`));
    if (b.dataset.tab === 'status') renderStatusTab();
    if (b.dataset.tab === 'places') call('places', {}, true);
  }));

  $$('[data-call]').forEach((b) => b.addEventListener('click', () => {
    call(b.dataset.call, b.dataset.args ? JSON.parse(b.dataset.args) : {});
  }));

  $('#save-here').addEventListener('submit', async (e) => {
    e.preventDefault();
    const name = $('#save-name').value.trim();
    if (!name) return;
    const res = await call('place_save', { name });
    if (res.ok) $('#save-name').value = '';
  });
  $('#say').addEventListener('submit', (e) => {
    e.preventDefault();
    const text = $('#say-text').value.trim();
    if (!text) return;
    call('say', { text });
    $('#say-text').value = '';
    $('#say-text').blur();
  });

  // speed: the slider, or tap the label by the drive stick to cycle
  setSpeed(state.speed);
  $('#speed-range').addEventListener('input', (e) => setSpeed(Number(e.target.value)));
  $('#speed').addEventListener('click', () => {
    const steps = [30, 60, 100];
    setSpeed(steps[(steps.indexOf(state.speed) + 1) % steps.length] || 60);
  });

  $$('#quality button').forEach((b) => {
    b.classList.toggle('on', b.dataset.q === state.quality);
    b.addEventListener('click', () => {
      state.quality = b.dataset.q;
      store.set('quality', state.quality);
      $$('#quality button').forEach((x) => x.classList.toggle('on', x === b));
      sendWant();
    });
  });
  $('#pip-on').checked = state.pip;
  $('#pip-on').addEventListener('change', (e) => {
    state.pip = e.target.checked;
    store.set('pip', state.pip);
    layoutViews();
  });
  $$('[data-layer]').forEach((c) => {
    c.checked = !!state.layers[c.dataset.layer];
    c.addEventListener('change', () => {
      state.layers[c.dataset.layer] = c.checked;
      store.set('layers', state.layers);
      if (!c.checked) delete mapView.costmaps[c.dataset.layer];
      sendWant();
      mapView.dirty();
    });
  });

  $$('#maptools .tool').forEach((b) => b.addEventListener('click', () => {
    const tool = b.dataset.tool;
    if (tool === 'follow') {
      setFollow(!mapView.follow);
      if (mapView.follow) { mapView.fitted = true; mapView.fit(); }
    } else if (tool === 'rotate') {
      mapView.headingUp = !mapView.headingUp;
      b.classList.toggle('on', mapView.headingUp);
      if (mapView.headingUp) setFollow(true);
      mapView.dirty();
    } else {
      closeSheet();
      mapView.setTool(mapView.tool === tool ? null : tool);
    }
  }));

  window.addEventListener('resize', () => setTimeout(layoutViews, 50));
  if (window.ResizeObserver) new ResizeObserver(() => mapView.resize()).observe(mapView.canvas);
  window.addEventListener('orientationchange', () => setTimeout(layoutViews, 200));
  document.addEventListener('visibilitychange', () => {
    if (document.visibilityState !== 'visible') sticks.forEach((s) => s.release());
    sendWant();
  });
  window.addEventListener('blur', () => sticks.forEach((s) => s.release()));
  // no page zoom or context menus on long presses
  document.addEventListener('gesturestart', (e) => e.preventDefault());
  document.addEventListener('contextmenu', (e) => e.preventDefault());

  layoutViews();
  connect();
}

function toggleFullscreen() {
  const d = document;
  if (d.fullscreenElement || d.webkitFullscreenElement) {
    (d.exitFullscreen || d.webkitExitFullscreen).call(d);
    return;
  }
  const el = d.documentElement;
  const req = el.requestFullscreen || el.webkitRequestFullscreen;
  if (!req) {
    toast('On iPhone: Share → Add to Home Screen for full screen');
    return;
  }
  Promise.resolve(req.call(el)).then(() => {
    if (screen.orientation && screen.orientation.lock) screen.orientation.lock('landscape').catch(() => {});
  }).catch(() => {});
}

setup();
