// view2d.js — 2D map view module. Owns the ROS2D Viewer, the
// OccupancyGridClient (since both need a CreateJS scene), and the on-map
// overlays (scan / plan / local plan / trail / robot / goal).
//
// Listens to RosApp.bus to redraw; publishes via RosApp.sendNavGoal /
// RosApp.publishInit. Returns a handle from init() so the shell can tear
// down event listeners when switching views.
(function () {
  let viewer = null;
  let gridClient = null;
  let mapInfo = null;

  let trailMarker, planMarker, localPlanMarker, scanMarker, robotMarker, goalMarker;
  const teardown = [];

  // 2D-only layer visibility flags, driven by the Layers toggle buttons.
  const layers = { scan: true, plan: true, local: true, trail: true };

  const $ = (id) => document.getElementById(id);
  const fmt = (n, p = 2) => Number.isFinite(n) ? n.toFixed(p) : '--';

  function ensureMarkers() {
    // Build order = z-order from bottom up.
    if (!trailMarker)     { trailMarker     = new createjs.Shape(); viewer.scene.addChild(trailMarker); }
    if (!planMarker)      { planMarker      = new createjs.Shape(); viewer.scene.addChild(planMarker); }
    if (!localPlanMarker) { localPlanMarker = new createjs.Shape(); viewer.scene.addChild(localPlanMarker); }
    if (!scanMarker)      { scanMarker      = new createjs.Shape(); viewer.scene.addChild(scanMarker); }
    if (!robotMarker)     { robotMarker     = new createjs.Shape(); viewer.scene.addChild(robotMarker); }
    if (!goalMarker)      { goalMarker      = new createjs.Shape(); viewer.scene.addChild(goalMarker); }
  }

  function strokePolyline(g, pts, color, width) {
    if (!pts || pts.length < 2) return;
    g.beginStroke(color).setStrokeStyle(width, 'round', 'round');
    g.moveTo(pts[0].x, -pts[0].y);
    for (let i = 1; i < pts.length; i++) g.lineTo(pts[i].x, -pts[i].y);
    g.endStroke();
  }

  function draw() {
    if (!mapInfo) return;
    ensureMarkers();
    const s = RosApp.state;

    // Traversed trail — soft green polyline.
    trailMarker.graphics.clear();
    if (layers.trail) strokePolyline(trailMarker.graphics, s.trail, 'rgba(140,255,140,0.85)', 0.05);

    // Global plan — blue.
    planMarker.graphics.clear();
    if (layers.plan && s.plan)
      strokePolyline(planMarker.graphics, s.plan, 'rgba(120,180,255,0.95)', 0.08);

    // Local plan — orange, drawn over global plan.
    localPlanMarker.graphics.clear();
    if (layers.local && s.localPlan)
      strokePolyline(localPlanMarker.graphics, s.localPlan, 'rgba(255,180,90,0.95)', 0.06);

    // Laser scan dots — assume lidar ≈ chassis (small offset).
    scanMarker.graphics.clear();
    if (layers.scan && s.pose && s.scan) {
      const r = 0.04;
      let ang = s.scan.angle_min;
      const inc = s.scan.angle_increment;
      const rmin = s.scan.range_min, rmax = s.scan.range_max;
      const px = s.pose.x, py = s.pose.y, pth = s.pose.yaw;
      scanMarker.graphics.beginFill('rgba(255,80,80,0.9)');
      for (let i = 0; i < s.scan.ranges.length; i++) {
        const d = s.scan.ranges[i];
        if (Number.isFinite(d) && d >= rmin && d <= rmax) {
          const lx = d * Math.cos(ang), ly = d * Math.sin(ang);
          const wx = px + lx * Math.cos(pth) - ly * Math.sin(pth);
          const wy = py + lx * Math.sin(pth) + ly * Math.cos(pth);
          scanMarker.graphics.drawCircle(wx, -wy, r);
        }
        ang += inc;
      }
    }

    // Robot arrow.
    robotMarker.graphics.clear();
    if (s.pose) {
      const L = 0.35, W = 0.15;
      const cx = s.pose.x, cy = s.pose.y, th = s.pose.yaw;
      const tip   = [cx + L * Math.cos(th),       cy + L * Math.sin(th)];
      const left  = [cx + W * Math.cos(th + 2.2), cy + W * Math.sin(th + 2.2)];
      const right = [cx + W * Math.cos(th - 2.2), cy + W * Math.sin(th - 2.2)];
      robotMarker.graphics
        .beginFill('rgba(70,180,230,0.9)')
        .beginStroke('#fff').setStrokeStyle(0.02)
        .moveTo(tip[0],  -tip[1])
        .lineTo(left[0], -left[1])
        .lineTo(right[0],-right[1])
        .closePath();
      robotMarker.graphics.beginFill('#fff').drawCircle(cx, -cy, 0.05);
    }

    // Goal marker.
    goalMarker.graphics.clear();
    if (s.goal) {
      goalMarker.graphics
        .beginFill('rgba(255,200,70,0.9)')
        .drawCircle(s.goal.x, -s.goal.y, 0.10);
      const L = 0.45;
      goalMarker.graphics
        .beginStroke('#fc4').setStrokeStyle(0.05)
        .moveTo(s.goal.x, -s.goal.y)
        .lineTo(s.goal.x + L * Math.cos(s.goal.yaw),
                -(s.goal.y + L * Math.sin(s.goal.yaw)));
    }

    // In-canvas HUD.
    if (s.pose) {
      $('hudPose').textContent =
        `x: ${fmt(s.pose.x, 2)}  y: ${fmt(s.pose.y, 2)}  yaw: ${fmt(s.pose.yaw, 2)}`;
    }
    if (s.odom) {
      $('hudVel').textContent =
        `v: ${fmt(s.odom.twist.twist.linear.x, 2)} m/s  ω: ${fmt(s.odom.twist.twist.angular.z, 2)} rad/s`;
    }
  }

  function canvasToWorld(canvas, ev) {
    const rect = canvas.getBoundingClientRect();
    const cx = ev.clientX - rect.left;
    const cy = ev.clientY - rect.top;
    return {
      x: (cx - viewer.scene.x) / viewer.scene.scaleX,
      y: (cy - viewer.scene.y) / viewer.scene.scaleY,
    };
  }

  function attachClickHandler() {
    const canvas = viewer.scene.canvas;
    let downPoint = null;
    const onDown = (e) => {
      const m = window.getClickMode ? window.getClickMode() : 'pan';
      if (m === 'pan') return;
      downPoint = canvasToWorld(canvas, e);
    };
    const onUp = (e) => {
      const m = window.getClickMode ? window.getClickMode() : 'pan';
      if (m === 'pan' || !downPoint) return;
      const upPoint = canvasToWorld(canvas, e);
      const yaw = Math.atan2(upPoint.y - downPoint.y, upPoint.x - downPoint.x);
      if (m === 'goal') RosApp.sendNavGoal(downPoint.x, downPoint.y, yaw);
      else              RosApp.publishInit(downPoint.x, downPoint.y, yaw);
      downPoint = null;
    };
    canvas.addEventListener('pointerdown', onDown);
    canvas.addEventListener('pointerup',   onUp);
    teardown.push(() => {
      canvas.removeEventListener('pointerdown', onDown);
      canvas.removeEventListener('pointerup',   onUp);
    });
  }

  function bindLayerToggle(id, key) {
    const btn = $(id);
    if (!btn) return;
    btn.onclick = () => {
      layers[key] = !layers[key];
      btn.classList.toggle('active', layers[key]);
      draw();
    };
  }

  function setupMapClient() {
    gridClient = new ROS2D.OccupancyGridClient({
      ros: RosApp.ros,
      rootObject: viewer.scene,
      continuous: true,
    });
    gridClient.on('change', () => {
      mapInfo = gridClient.currentGrid;
      viewer.scaleToDimensions(mapInfo.width, mapInfo.height);
      viewer.shift(mapInfo.pose.position.x, mapInfo.pose.position.y);
      $('mapRes').textContent  = fmt(mapInfo.info?.resolution, 3) + ' m/cell';
      $('mapDim').textContent  = mapInfo.width + ' × ' + mapInfo.height;
      $('mapOrig').textContent =
        '(' + fmt(mapInfo.pose.position.x, 2) + ', ' + fmt(mapInfo.pose.position.y, 2) + ')';
      draw();
    });
  }

  function init() {
    viewer = new ROS2D.Viewer({
      divID: 'map-canvas',
      width:  $('map-wrap').clientWidth,
      height: $('map-wrap').clientHeight,
      background: '#111',
    });
    const onResize = () => {
      viewer.scene.canvas.width  = $('map-wrap').clientWidth;
      viewer.scene.canvas.height = $('map-wrap').clientHeight;
    };
    window.addEventListener('resize', onResize);
    teardown.push(() => window.removeEventListener('resize', onResize));

    attachClickHandler();

    bindLayerToggle('layer-scan',  'scan');
    bindLayerToggle('layer-plan',  'plan');
    bindLayerToggle('layer-local', 'local');
    bindLayerToggle('layer-trail', 'trail');
    $('clear-trail').onclick = () => RosApp.clearTrail();

    // Map subscription needs an open ROS connection.
    if (RosApp.state.connected) {
      setupMapClient();
    } else {
      const onReady = () => {
        setupMapClient();
        RosApp.bus.removeEventListener('rosReady', onReady);
      };
      RosApp.bus.addEventListener('rosReady', onReady);
      teardown.push(() => RosApp.bus.removeEventListener('rosReady', onReady));
    }

    const events = ['pose', 'scan', 'plan', 'localPlan', 'goal', 'trail', 'odom'];
    const handler = () => draw();
    events.forEach(ev => RosApp.bus.addEventListener(ev, handler));
    teardown.push(() => events.forEach(ev => RosApp.bus.removeEventListener(ev, handler)));

    return {
      destroy() {
        teardown.forEach(fn => { try { fn(); } catch (e) {} });
        teardown.length = 0;
        const wrap = $('map-canvas');
        if (wrap) wrap.innerHTML = '';
        viewer = null;
        gridClient = null;
        mapInfo = null;
        trailMarker = planMarker = localPlanMarker = null;
        scanMarker = robotMarker = goalMarker = null;
      },
    };
  }

  window.View2D = { init };
})();
