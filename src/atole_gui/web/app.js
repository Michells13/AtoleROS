// Web GUI de AtoleROS: comandos y estado por rosbridge (:9090); datos pesados en Lichtblick (foxglove_bridge :8765).
'use strict';

const HOST = location.hostname || 'localhost';
const ros = new Ros(`ws://${HOST}:9090`);
const $ = id => document.getElementById(id);
const CLIENT = 'gui';
const LEVEL = ['OK', 'AVISO', 'ERROR', '?'];
const TYPES = {
  health: 'atole_interfaces/msg/SystemHealth', cams: 'atole_interfaces/msg/CameraStatus',
  arm: 'atole_interfaces/msg/ArmState', grip: 'atole_interfaces/msg/GripperState',
  mission: 'atole_interfaces/msg/MissionStatus', pods: 'atole_interfaces/msg/PodArray',
  config: 'atole_interfaces/msg/ConfigSnapshot', jpeg: 'sensor_msgs/msg/CompressedImage',
  trigger: 'std_srvs/srv/Trigger', setbool: 'std_srvs/srv/SetBool',
};
const state = { config: {}, cams: null, arm: null, grip: null, mission: null, simListed: false, thumbs: new Map() };

// ───────────────────────── utilidades ─────────────────────────
function log(text, cls = '') {
  const line = document.createElement('div');
  line.className = cls;
  line.textContent = `${new Date().toLocaleTimeString()}  ${text}`;
  $('log').prepend(line);
  while ($('log').childElementCount > 60) $('log').lastChild.remove();
}

function el(tag, props = {}, ...children) {
  const e = Object.assign(document.createElement(tag), props);
  children.forEach(c => e.append(c));
  return e;
}

// Ejecuta una acción de botón: lo desactiva mientras dura y registra el resultado.
async function run(button, label, fn) {
  if (button) button.disabled = true;
  try {
    const msg = await fn();
    if (msg) log(`${label}: ${msg}`, 'okm');
  } catch (e) {
    log(`${label}: ${e.message}`, 'err');
  } finally {
    if (button) button.disabled = false;
  }
}

// Servicios que responden {ok|success, message}: error si no fue bien.
async function callOk(service, type, args = {}, timeoutMs) {
  const r = await ros.call(service, type, args, timeoutMs);
  if (r.ok === false || r.success === false) throw new Error(r.message || 'falló');
  return r.message || 'ok';
}

async function actionOk(name, type, args, onFeedback) {
  const { result } = await ros.action(name, type, args, onFeedback).done;
  if (result.ok === false) throw new Error(result.message || 'falló');
  return result.message || 'ok';
}

const deg = r => r * 180 / Math.PI;
const rad = d => d * Math.PI / 180;
const fmt = (v, n = 1) => Number(v).toFixed(n);

// ───────────────────────── cabecera y pestañas ─────────────────────────
ros.onStatus(ok => {
  $('conn').textContent = ok ? 'conectado' : 'sin conexión';
  $('conn').className = `pill ${ok ? 'ok' : 'bad'}`;
  if (ok) log('conectado a rosbridge', 'okm');
});

document.querySelectorAll('.tabs button').forEach(b => b.addEventListener('click', () => {
  document.querySelectorAll('.tabs button').forEach(x => x.classList.toggle('active', x === b));
  document.querySelectorAll('.tab').forEach(t => { t.hidden = t.id !== `tab-${b.dataset.tab}`; });
}));

$('stop').addEventListener('click', () => run(null, 'PARAR', async () => {
  const parts = [];
  if (state.mission && state.mission.state !== 'IDLE' && state.mission.state !== 'ERROR') {
    parts.push(await callOk('/atole/mission/abort', TYPES.trigger).catch(e => e.message));
  }
  parts.push(await callOk('/atole/arm/stop', TYPES.trigger));
  return parts.join(' · ');
}));

const lichtblick = `/lichtblick/?ds=foxglove-websocket&ds.url=${encodeURIComponent(`ws://${HOST}:8765`)}` +
                   `&layoutUrl=${encodeURIComponent(`${location.origin}/lichtblick_layout.json`)}`;
$('lichtblick').src = lichtblick;
$('lb-link').href = lichtblick;

// ───────────────────────── configuración ─────────────────────────
ros.subscribe('/atole/config', TYPES.config, m => {
  state.config = Object.fromEntries(m.entries.map(kv => [kv.key, kv.value]));
  renderPoses();
});
const cfg = (key, def = '') => state.config[key] ?? def;

// ───────────────────────── sistema ─────────────────────────
ros.subscribe('/atole/system/health', TYPES.health, m => {
  const cls = m.ready ? 'ok' : (m.phase === 'FAULT' ? 'bad' : 'warn');
  $('phase').textContent = m.phase;
  $('phase').className = `pill ${cls}`;
  $('h-items').replaceChildren(...m.items.map(i => el('tr', {},
    el('td', { textContent: i.name }), el('td', { textContent: LEVEL[i.level], className: `lvl-${i.level}` }),
    el('td', { textContent: i.message }))));
});

// ───────────────────────── misión ─────────────────────────
ros.subscribe('/atole/mission/status', TYPES.mission, m => {
  state.mission = m;
  $('m-state').textContent = m.state;
  $('mission-state').textContent = m.state;
  $('mission-state').className = `pill ${m.state === 'ERROR' ? 'bad' : m.state === 'IDLE' ? 'muted' : 'warn'}`;
  $('m-message').textContent = m.message || '—';
  $('m-pod').textContent = m.current_pod_id >= 0 ? m.current_pod_id : '—';
  $('m-done').textContent = m.pods_done;
  $('m-remaining').textContent = m.pods_remaining;
  $('m-failed').textContent = m.pods_failed;
  $('m-confirm').hidden = !m.awaiting_confirmation;
  $('m-prompt').textContent = m.confirmation_prompt;
  const busy = !['IDLE', 'ERROR'].includes(m.state);
  $('m-all').disabled = $('m-next').disabled = busy;
});

function harvest(button, mode) {
  run(button, mode === 0 ? 'Cosechar todos' : 'Siguiente pod', () => actionOk('/atole/mission/harvest',
    'atole_interfaces/action/Harvest', {
      mode, step_mode: $('m-step').checked, skip_gripper: $('m-skip').checked, eih_dry_run: false,
      eih_strategy: $('m-eih').value, selection_strategy: $('m-sel').value,
    }));
}
$('m-all').addEventListener('click', e => harvest(e.target, 0));
$('m-next').addEventListener('click', e => harvest(e.target, 1));
$('m-abort').addEventListener('click', e => run(e.target, 'Abortar', () => callOk('/atole/mission/abort', TYPES.trigger)));
$('m-confirm-btn').addEventListener('click', e => run(e.target, 'Confirmar', () => callOk('/atole/mission/confirm_step', TYPES.trigger)));
$('m-step').addEventListener('change', e => {
  if (state.mission && !['IDLE', 'ERROR'].includes(state.mission.state)) {
    run(null, 'Paso a paso', () => callOk('/atole/mission/set_step_mode', TYPES.setbool, { data: e.target.checked }));
  }
});

// ───────────────────────── robot ─────────────────────────
const jointRows = [];
for (let i = 0; i < 6; i++) {
  const value = el('td', { textContent: '—' });
  const jog = sign => el('button', { textContent: `${sign > 0 ? '+' : '−'}`, title: `J${i + 1} ${sign > 0 ? '+' : '−'}paso`,
    onclick: ev => run(ev.target, `J${i + 1}`, () => callOk('/atole/arm/jog_joint', 'atole_interfaces/srv/JogJoint',
      { client_id: CLIENT, joint_index: i, delta_deg: sign * Number($('r-step').value), speed: 0.3 })) });
  jointRows.push(value);
  $('r-joints').append(el('tr', {}, el('th', { textContent: `J${i + 1}` }), value, el('td', {}, jog(-1)), el('td', {}, jog(1))));
}

ros.subscribe('/atole/arm/state', TYPES.arm, m => {
  state.arm = m;
  $('r-conn').textContent = m.connected ? 'conectado' : 'desconectado';
  $('r-mode').textContent = m.robot_mode || '—';
  $('r-safety').textContent = m.safety_mode || 'Normal';
  $('r-owner').textContent = m.control_owner || 'libre';
  m.joints.forEach((j, i) => { jointRows[i].textContent = fmt(deg(j)); });
  const p = m.tcp_pose.position;
  $('r-tcp').textContent = `${fmt(p.x, 3)}, ${fmt(p.y, 3)}, ${fmt(p.z, 3)}`;
  if (document.activeElement !== $('r-freedrive')) $('r-freedrive').checked = m.freedrive;
  if (document.activeElement !== $('r-j6')) $('r-j6').checked = m.j6_locked;
}, { throttle: 200 });

$('r-freedrive').addEventListener('change', e => run(null, 'Freedrive', () => callOk('/atole/arm/set_freedrive',
  'atole_interfaces/srv/SetFreedrive', { client_id: CLIENT, enable: e.target.checked, damping: 0.5 })));
$('r-j6').addEventListener('change', e => run(null, 'Bloqueo J6', () => callOk('/atole/arm/set_j6_lock', TYPES.setbool, { data: e.target.checked })));
$('r-unlock').addEventListener('click', e => run(e.target, 'Desbloquear', () => callOk('/atole/arm/unlock_pstop', TYPES.trigger)));
$('r-reconnect').addEventListener('click', e => run(e.target, 'Reconectar', () => callOk('/atole/arm/reconnect', TYPES.trigger)));

function renderPoses() {
  $('r-poses').replaceChildren(...['Home', 'Home2', 'Release'].map(name => {
    const joints = [1, 2, 3, 4, 5, 6].map(i => Number(cfg(`Poses/${name}/J${i}`, NaN)));
    const valid = joints.every(Number.isFinite);
    const go = el('button', { textContent: 'Ir', disabled: !valid,
      onclick: ev => run(ev.target, `Ir a ${name}`, () => actionOk('/atole/arm/move_joints', 'atole_interfaces/action/MoveJoints',
        { client_id: CLIENT, joints: joints.map(rad), speed: 0.3, accel: 0.3, timeout_s: 90.0, bypass_j6_lock: false })) });
    const save = el('button', { textContent: 'Guardar actual',
      onclick: ev => { if (confirm(`¿Guardar la posición actual del robot como ${name}?`)) run(ev.target, `Guardar ${name}`,
        () => callOk('/atole/config/save_pose', 'atole_interfaces/srv/SavePose', { name: name.toLowerCase(), from_current: true, joints_deg: [0, 0, 0, 0, 0, 0] })); } });
    return el('tr', {}, el('th', { textContent: name }), el('td', { textContent: valid ? joints.map(v => fmt(v)).join(' ') : 'sin guardar' }),
      el('td', {}, go), el('td', {}, save));
  }));
}

// ───────────────────────── gripper ─────────────────────────
ros.subscribe('/atole/gripper/state', TYPES.grip, m => {
  state.grip = m;
  $('g-conn').textContent = m.connected ? m.port.split('/').pop() : 'desconectado';
  $('g-init').textContent = m.init_state_name || '—';
  $('g-open').textContent = m.last_opening;
  $('g-angle').textContent = `${m.last_angle_deg}°`;
  $('g-state').textContent = m.busy ? 'moviendo' : (m.gripper_state_name || '—');
});
const gripper = (label, args) => ev => run(ev.target, label, () => actionOk('/atole/gripper/command',
  'atole_interfaces/action/GripperCommand', Object.assign({ opening: 0, angle_deg: 0, force: 0, speed: 0, timeout_s: 15.0 }, args)));
$('g-openbtn').addEventListener('click', gripper('Abrir', { command: 0 }));
$('g-close').addEventListener('click', gripper('Cerrar', { command: 1 }));
$('g-slider').addEventListener('input', e => { $('g-slider-v').textContent = e.target.value; });
$('g-move').addEventListener('click', ev => gripper('Mover', { command: 2, opening: Number($('g-slider').value) })(ev));
$('g-rotate').addEventListener('click', ev => gripper('Girar', { command: 3, angle_deg: Math.round(Number($('g-rot').value)) })(ev));
$('g-reinit').addEventListener('click', e => { if (confirm('La inicialización mueve el gripper (apertura y rotación completas). ¿Continuar?'))
  run(e.target, 'Inicializar', () => callOk('/atole/gripper/reinit', TYPES.trigger, {}, 30000)); });

// ───────────────────────── cámaras y SIM ─────────────────────────
ros.subscribe('/atole/cameras/status', TYPES.cams, m => {
  state.cams = m;
  $('source').textContent = m.source === 'sim' ? 'SIM' : 'cámaras reales';
  $('source').className = `pill ${m.source === 'sim' ? 'warn' : 'muted'}`;
  $('c-eth').replaceChildren(...['cam0', 'cam1', 'camC'].map(cam => el('button', {
    textContent: cam, className: cam === m.active_eth ? 'primary' : '', disabled: m.swap_in_progress || m.source === 'sim',
    onclick: ev => run(ev.target, `EtH → ${cam}`, () => callOk('/atole/cameras/select_eth', 'atole_interfaces/srv/SelectEth', { camera: cam }, 120000)) })));
  $('c-table').replaceChildren(...m.cameras.map(c => {
    const check = (what, value) => el('input', { type: 'checkbox', checked: value,
      onchange: e => run(null, `${what === 'image' ? 'Imagen' : 'Nube'} de ${c.id}`, () => callOk('/atole/cameras/set_display',
        'atole_interfaces/srv/SetDisplay', { camera: c.id, enable: e.target.checked, what })) });
    const st = c.streaming ? 'transmitiendo' : c.active ? 'arrancando' : c.connected ? 'conectada' : '—';
    return el('tr', {}, el('td', { textContent: `${c.id} (${c.position})` }), el('td', { textContent: c.serial || 'sin leer' }),
      el('td', { textContent: st }), el('td', {}, check('image', c.display)), el('td', {}, check('cloud', c.cloud)));
  }));
  $('sim-box').hidden = m.source !== 'sim';
  if (m.source === 'sim' && !state.simListed) listSim();
  syncThumbs(m);
});

function syncThumbs(m) {
  const want = new Set(m.cameras.filter(c => c.display && c.streaming).map(c => c.id));
  for (const [cam, t] of state.thumbs) if (!want.has(cam)) { t.unsubscribe(); t.fig.remove(); state.thumbs.delete(cam); }
  for (const cam of want) {
    if (state.thumbs.has(cam)) continue;
    const img = el('img', { alt: cam });
    const fig = el('figure', {}, img, el('figcaption', { textContent: cam }));
    $('thumbs').append(fig);
    const unsubscribe = ros.subscribe(`/atole/gui/${cam}/image/compressed`, TYPES.jpeg,
      msg => { img.src = `data:image/jpeg;base64,${msg.data}`; }, { throttle: 400 });
    state.thumbs.set(cam, { fig, unsubscribe });
  }
}

async function listSim() {
  state.simListed = true;
  try {
    const eth = await ros.call('/atole/sim/list', 'atole_interfaces/srv/ListDatasets');
    $('s-eth').replaceChildren(...eth.folders.map(f => el('option', { value: f, textContent: f })));
    const eih = await ros.call('/atole/sim/list_eih', 'atole_interfaces/srv/ListDatasets');
    $('s-eih').replaceChildren(...eih.folders.map(f => el('option', { value: f, textContent: f })));
  } catch (e) { state.simListed = false; log(`listas SIM: ${e.message}`, 'err'); }
}
$('s-load').addEventListener('click', e => run(e.target, 'Cargar dataset', () => callOk('/atole/sim/load',
  'atole_interfaces/srv/LoadDataset', { folder: $('s-eth').value, rate_hz: 0.0 })));
$('s-stop').addEventListener('click', e => run(e.target, 'Detener SIM', () => callOk('/atole/sim/stop', TYPES.trigger)));
$('s-load-eih').addEventListener('click', e => run(e.target, 'Cargar vista EiH', () => callOk('/atole/sim/load_eih',
  'atole_interfaces/srv/LoadDataset', { folder: $('s-eih').value, rate_hz: 0.0 })));

// ───────────────────────── percepción ─────────────────────────
ros.subscribe('/atole/gui/detections/compressed', TYPES.jpeg, m => { $('p-overlay').src = `data:image/jpeg;base64,${m.data}`; });
ros.subscribe('/atole/perception/pods', TYPES.pods, m => {
  $('p-pods').replaceChildren(...m.pods.map(p => el('tr', {},
    el('td', { textContent: p.id }), el('td', { textContent: fmt(p.score, 2) }), el('td', { textContent: p.status }),
    el('td', { textContent: p.status === 'ok' ? `${fmt(p.grasp_bottom.x, 3)}, ${fmt(p.grasp_bottom.y, 3)}, ${fmt(p.grasp_bottom.z, 3)}` : '—' }),
    el('td', { textContent: p.status === 'ok' ? `${fmt(p.length_m * 1000, 0)}×${fmt(p.width_m * 1000, 0)}` : '—' }),
    el('td', { textContent: p.status === 'ok' ? fmt(p.pose_confidence, 2) : '—' }))));
});
$('p-detect').addEventListener('click', e => run(e.target, 'Mask R-CNN', () => callOk('/atole/perception/detect_once',
  'atole_interfaces/srv/DetectOnce', { camera: $('p-cam').value, include_masks: true }, 90000)));
$('p-stream').addEventListener('change', e => run(null, 'Detección continua', () => callOk('/atole/perception/set_stream',
  TYPES.setbool, { data: e.target.checked })));
$('p-estimate').addEventListener('click', e => run(e.target, 'Estimar pods', () => actionOk('/atole/perception/estimate_pods',
  'atole_interfaces/action/EstimatePods', { camera: $('p-cam').value, eih: $('p-cam').value === 'cam2' },
  f => { $('m-message').textContent = `estimate_pods: ${f.stage}`; })));
