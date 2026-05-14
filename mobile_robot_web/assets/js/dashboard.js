// dashboard.js — Shell controller. Owns universal UI: status badges,
// connection, view tabs (with fragment loader), click-mode buttons,
// teleop joystick, pose readout, navigation readout. Delegates ROS work
// to RosApp and view-specific rendering to View2D / View3D.
(function () {
  const $ = (id) => document.getElementById(id);
  const fmt = (n, p = 2) => Number.isFinite(n) ? n.toFixed(p) : '--';

  function pathLength(arr) {
    if (!arr) return 0;
    let s = 0;
    for (let i = 1; i < arr.length; i++) {
      s += Math.hypot(arr[i].x - arr[i - 1].x, arr[i].y - arr[i - 1].y);
    }
    return s;
  }

  // ---- Status badges ----
  const statusEl    = $('status');
  const navStatusEl = $('navStatus');
  RosApp.bus.addEventListener('status', (e) => {
    statusEl.textContent = e.detail.text;
    statusEl.className = 'badge ' + e.detail.cls;
  });
  RosApp.bus.addEventListener('navStatus', () => {
    const ns = RosApp.state.navStatus;
    if (!ns) return;
    navStatusEl.textContent = 'Nav: ' + ns.label;
    navStatusEl.className = 'badge ' + ns.cls;
  });

  // ---- Connection ----
  const hostEl = $('host');
  hostEl.value = location.hostname || 'localhost';
  $('connect').onclick = () => RosApp.connect(hostEl.value);

  // ---- View tabs / fragment loader ----
  let currentView = null;        // { destroy } returned by View*.init()
  let currentViewName = null;

  async function loadView(name) {
    if (currentViewName === name) return;
    if (currentView) {
      try { currentView.destroy?.(); } catch (e) { console.warn(e); }
      currentView = null;
    }
    $('view-container').innerHTML = '';
    $('view-sidebar').innerHTML = '';
    currentViewName = name;

    if (!name) return;

    let html;
    try {
      html = await (await fetch(`${name}.html`)).text();
    } catch (e) {
      console.error(`Failed to load ${name}.html`, e);
      return;
    }
    const tmp = document.createElement('div');
    tmp.innerHTML = html;

    const main = tmp.querySelector('[data-slot="view-main"]');
    const side = tmp.querySelector('[data-slot="view-sidebar"]');
    if (main) $('view-container').appendChild(main);
    if (side) $('view-sidebar').appendChild(side);

    // Instantiate the view's controller once the DOM is in place.
    if (name === '2d' && window.View2D) currentView = window.View2D.init();
    else if (name === '3d' && window.View3D) currentView = window.View3D.init();

    document.querySelectorAll('.tabs button').forEach(b => {
      b.classList.toggle('active', b.dataset.view === name);
    });
  }

  document.querySelectorAll('.tabs button').forEach(b => {
    b.onclick = () => { if (!b.disabled) loadView(b.dataset.view); };
  });

  // ---- Click mode (exposed to views via window.getClickMode) ----
  let mode = 'pan';
  const modePan  = $('mode-pan');
  const modeInit = $('mode-init');
  const modeGoal = $('mode-goal');
  function setMode(m) {
    mode = m;
    modePan.classList.toggle('active',  m === 'pan');
    modeInit.classList.toggle('active', m === 'init');
    modeGoal.classList.toggle('active', m === 'goal');
  }
  modePan.onclick  = () => setMode('pan');
  modeInit.onclick = () => setMode('init');
  modeGoal.onclick = () => setMode('goal');
  window.getClickMode = () => mode;

  $('cancel-goal').onclick = () => RosApp.cancelGoal();

  // ---- Pose readout ----
  function updatePoseReadout() {
    const p = RosApp.state.pose;
    if (p) {
      $('px').textContent   = fmt(p.x, 3) + ' m';
      $('py').textContent   = fmt(p.y, 3) + ' m';
      $('pyaw').textContent = fmt(p.yaw, 3) + ' rad';
      $('poseSrc').textContent = RosApp.state.poseSrc || '--';
    }
    const o = RosApp.state.odom;
    if (o) {
      $('ov').textContent = fmt(o.twist.twist.linear.x, 3) + ' m/s';
      $('ow').textContent = fmt(o.twist.twist.angular.z, 3) + ' rad/s';
    }
  }
  RosApp.bus.addEventListener('pose', updatePoseReadout);
  RosApp.bus.addEventListener('odom', updatePoseReadout);

  // ---- Navigation readout ----
  function updateNavReadout() {
    const s = RosApp.state;
    $('goalXY').textContent = s.goal
      ? '(' + fmt(s.goal.x, 2) + ', ' + fmt(s.goal.y, 2) + ')'
      : '--';

    $('distGoal').textContent = (s.pose && s.goal)
      ? fmt(Math.hypot(s.goal.x - s.pose.x, s.goal.y - s.pose.y), 2) + ' m'
      : '--';

    if (s.plan) {
      const L = pathLength(s.plan);
      $('planLen').textContent = fmt(L, 2) + ' m';
      $('planN').textContent   = String(s.plan.length);

      const v = s.odom?.twist?.twist?.linear?.x;
      if (Number.isFinite(v) && Math.abs(v) > 0.02) {
        const t = L / Math.abs(v);
        const m = Math.floor(t / 60);
        const sec = Math.round(t % 60);
        $('eta').textContent = (m ? m + 'm ' : '') + sec + 's';
      } else {
        $('eta').textContent = '--';
      }
    } else {
      $('planLen').textContent = '--';
      $('planN').textContent   = '--';
      $('eta').textContent     = '--';
    }

    $('trailLen').textContent = fmt(s.trailDist, 2) + ' m';
  }
  ['pose', 'goal', 'plan', 'trail', 'odom'].forEach(ev =>
    RosApp.bus.addEventListener(ev, updateNavReadout)
  );

  // ---- Teleop joystick ----
  const joy   = $('joy');
  const knob  = $('joy-knob');
  const vmaxEl = $('vmax');
  const wmaxEl = $('wmax');
  $('vmaxLbl').textContent = vmaxEl.value;
  $('wmaxLbl').textContent = wmaxEl.value;
  vmaxEl.oninput = () => $('vmaxLbl').textContent = vmaxEl.value;
  wmaxEl.oninput = () => $('wmaxLbl').textContent = wmaxEl.value;

  let dragging = false;
  let joyVec = { x: 0, y: 0 };
  let sendFinalZero = false;
  const radius = 70;

  joy.addEventListener('pointerdown', (e) => {
    joy.setPointerCapture(e.pointerId); dragging = true; moveKnob(e);
  });
  joy.addEventListener('pointermove', (e) => { if (dragging) moveKnob(e); });
  joy.addEventListener('pointerup',     () => { dragging = false; resetKnob(); });
  joy.addEventListener('pointercancel', () => { dragging = false; resetKnob(); });

  function moveKnob(e) {
    const rect = joy.getBoundingClientRect();
    const cx = rect.left + rect.width / 2;
    const cy = rect.top + rect.height / 2;
    let dx = e.clientX - cx;
    let dy = e.clientY - cy;
    const d = Math.hypot(dx, dy);
    if (d > radius) { dx *= radius / d; dy *= radius / d; }
    knob.style.left = (rect.width  / 2 + dx - 30) + 'px';
    knob.style.top  = (rect.height / 2 + dy - 30) + 'px';
    joyVec.x = dx / radius;
    joyVec.y = dy / radius;
  }
  function resetKnob() {
    knob.style.left = (joy.clientWidth  / 2 - 30) + 'px';
    knob.style.top  = (joy.clientHeight / 2 - 30) + 'px';
    joyVec = { x: 0, y: 0 };
    sendFinalZero = true;
  }

  // Publish ONLY while the knob is off-centre (plus one final zero on release).
  // An idle page must not spam zeros onto /cmd_vel — that fights nav2 /
  // collision_monitor / docking_server.
  setInterval(() => {
    const vmax = parseFloat(vmaxEl.value);
    const wmax = parseFloat(wmaxEl.value);
    const v = -joyVec.y * vmax;
    const w = -joyVec.x * wmax;
    $('vlin').textContent = v.toFixed(2) + ' m/s';
    $('vang').textContent = w.toFixed(2) + ' rad/s';

    const moving = Math.abs(joyVec.x) > 1e-3 || Math.abs(joyVec.y) > 1e-3;
    if (!moving && !sendFinalZero) return;
    RosApp.publishTwist(v, w);
    if (!moving) sendFinalZero = false;
  }, 100);

  // ---- Bootstrap ----
  loadView('2d');
  RosApp.connect(hostEl.value);
})();
