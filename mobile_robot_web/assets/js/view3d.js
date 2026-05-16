// view3d.js — 3D scene view. Lazily pulls in three.js + STLLoader +
// OrbitControls (via the importmap in dashboard.html), parses the URDF
// published on /robot_description into a link/joint tree, loads each
// link's STL mesh, and drives:
//
//   - chassis pose      ← RosApp.state.pose (map frame if AMCL, else odom)
//   - joint angles      ← /joint_states
//   - laser scan points ← /scan          (attached to lidar_link)
//   - occupancy grid    ← /map           (textured plane on the ground)
//   - goal marker       ← RosApp.state.goal
//
// URDF mesh URIs are file:// paths that browsers cannot fetch — we
// rewrite them to http://<host>:<mesh_port>/meshes/<...> at parse time.
// The mesh server is launched alongside the dashboard HTTP server in
// web_bringup.launch.py (default mesh_port = 8001).
(function () {
  // Lazy-loaded module globals (three.js + addons).
  let THREE = null;
  let STLLoader = null;
  let OrbitControls = null;

  let scene, camera, renderer, controls, animId, hostEl, resizeObs;
  let gridHelper, axesHelper;
  let mapMesh = null;
  let scanPoints = null;
  let goalArrow = null;

  let chassisGroup = null;       // root link of the URDF; pose lives on this
  const linkGroups = {};         // linkName -> THREE.Object3D (rendered link)
  const jointPivots = {};        // jointName -> { axisGroup, axis: Vec3, type, origin }
  let urdfLoaded = false;

  // Layer visibility flags — drive scene.visible toggles.
  const layers = { scan: true, map: true, grid: true, axes: true };
  let camFollow = false;

  const teardown = [];
  const $ = (id) => document.getElementById(id);
  const fmt = (n, p = 2) => Number.isFinite(n) ? n.toFixed(p) : '--';

  // ---------- module bootstrap ----------
  async function loadDeps() {
    if (THREE) return;
    THREE = await import('three');
    ({ STLLoader } = await import('three/addons/loaders/STLLoader.js'));
    ({ OrbitControls } = await import('three/addons/controls/OrbitControls.js'));
  }

  // ---------- math helpers ----------
  // ROS RPY is roll-pitch-yaw in fixed axes (extrinsic XYZ). Three.js
  // Euler is intrinsic; intrinsic ZYX equals extrinsic XYZ, so we use
  // order 'ZYX' to apply URDF rpy values directly.
  function applyRPYXYZ(obj, xyz, rpy) {
    obj.position.set(xyz[0], xyz[1], xyz[2]);
    obj.rotation.set(rpy[0], rpy[1], rpy[2], 'ZYX');
  }

  function parseTriplet(str, def = [0, 0, 0]) {
    if (!str) return def.slice();
    const parts = str.trim().split(/\s+/).map(Number);
    return [parts[0] || 0, parts[1] || 0, parts[2] || 0];
  }

  function parseRGBA(str, def = [0.7, 0.7, 0.75, 1]) {
    if (!str) return def.slice();
    const parts = str.trim().split(/\s+/).map(Number);
    return [parts[0] ?? def[0], parts[1] ?? def[1], parts[2] ?? def[2], parts[3] ?? 1];
  }

  // ---------- URDF parsing ----------
  function rewriteMeshUrls(urdfText) {
    // file:///.../share/mobile_robot_description/meshes/foo.STL
    // → /robot_assets/meshes/foo.STL  (same-origin; the launch's HTTP
    // server routes /robot_assets/* to the description package share).
    return urdfText.replace(
      /file:\/\/[^"']*?share\/mobile_robot_description\//g,
      '/robot_assets/',
    );
  }

  function parseURDF(urdfText) {
    const xml = new DOMParser().parseFromString(urdfText, 'text/xml');
    const robot = xml.querySelector('robot');
    if (!robot) throw new Error('No <robot> element in URDF');

    // Index links and joints.
    const links = {};
    const joints = [];
    for (const lk of xml.getElementsByTagName('link')) {
      const name = lk.getAttribute('name');
      const visuals = [];
      for (const v of lk.getElementsByTagName('visual')) {
        const origin = v.getElementsByTagName('origin')[0];
        const mesh   = v.getElementsByTagName('mesh')[0];
        const mat    = v.getElementsByTagName('material')[0];
        const color  = mat?.getElementsByTagName('color')[0];
        visuals.push({
          xyz: parseTriplet(origin?.getAttribute('xyz')),
          rpy: parseTriplet(origin?.getAttribute('rpy')),
          mesh: mesh?.getAttribute('filename') || null,
          rgba: color ? parseRGBA(color.getAttribute('rgba')) : null,
        });
      }
      links[name] = { name, visuals };
    }
    for (const jt of xml.getElementsByTagName('joint')) {
      const name   = jt.getAttribute('name');
      const type   = jt.getAttribute('type');
      const parent = jt.getElementsByTagName('parent')[0]?.getAttribute('link');
      const child  = jt.getElementsByTagName('child')[0]?.getAttribute('link');
      const origin = jt.getElementsByTagName('origin')[0];
      const axis   = jt.getElementsByTagName('axis')[0];
      joints.push({
        name, type, parent, child,
        xyz: parseTriplet(origin?.getAttribute('xyz')),
        rpy: parseTriplet(origin?.getAttribute('rpy')),
        axis: parseTriplet(axis?.getAttribute('xyz'), [1, 0, 0]),
      });
    }

    // Root = link that is never a child of any joint.
    const childSet = new Set(joints.map(j => j.child));
    const rootName = Object.keys(links).find(n => !childSet.has(n));
    if (!rootName) throw new Error('Could not determine URDF root link');
    return { links, joints, rootName };
  }

  function buildLinkObject(linkDef, loader) {
    const g = new THREE.Group();
    g.name = linkDef.name;
    for (const v of linkDef.visuals) {
      if (!v.mesh) continue;
      const visGroup = new THREE.Group();
      applyRPYXYZ(visGroup, v.xyz, v.rpy);
      g.add(visGroup);

      const color = v.rgba
        ? new THREE.Color(v.rgba[0], v.rgba[1], v.rgba[2])
        : new THREE.Color(0xb0b0c0);
      const mat = new THREE.MeshPhongMaterial({
        color,
        opacity: v.rgba && v.rgba[3] < 1 ? v.rgba[3] : 1,
        transparent: v.rgba && v.rgba[3] < 1,
        shininess: 30,
      });
      const placeholder = new THREE.Mesh(new THREE.BufferGeometry(), mat);
      visGroup.add(placeholder);
      loader.load(
        v.mesh,
        (geom) => { placeholder.geometry = geom; geom.computeVertexNormals(); },
        undefined,
        (err) => console.warn('[view3d] STL load failed', v.mesh, err),
      );
    }
    return g;
  }

  function buildURDFScene(urdfText) {
    const rewritten = rewriteMeshUrls(urdfText);
    const { links, joints, rootName } = parseURDF(rewritten);
    const loader = new STLLoader();

    // Build link groups (geometry only; no transforms yet).
    for (const name of Object.keys(links)) {
      linkGroups[name] = buildLinkObject(links[name], loader);
    }

    // Wire joints: each joint = pivot (origin transform) → axis (rotates
    // around the joint axis) → child link. This keeps the origin and the
    // joint-state rotation independent so /joint_states can spin a wheel
    // without disturbing where it's mounted.
    for (const j of joints) {
      const pivot = new THREE.Group();
      pivot.name = j.name + '__pivot';
      applyRPYXYZ(pivot, j.xyz, j.rpy);

      const axisGroup = new THREE.Group();
      axisGroup.name = j.name + '__axis';
      pivot.add(axisGroup);

      const childLink = linkGroups[j.child];
      if (!childLink) {
        console.warn('[view3d] joint refers to unknown child link', j);
        continue;
      }
      axisGroup.add(childLink);

      const parentLink = linkGroups[j.parent];
      if (!parentLink) {
        console.warn('[view3d] joint refers to unknown parent link', j);
        continue;
      }
      parentLink.add(pivot);

      jointPivots[j.name] = {
        axisGroup,
        axis: new THREE.Vector3(j.axis[0], j.axis[1], j.axis[2]).normalize(),
        type: j.type,
      };
    }

    chassisGroup = linkGroups[rootName];
    chassisGroup.name = rootName + '__root';
    scene.add(chassisGroup);
    urdfLoaded = true;
    const hud = $('hud3dURDF');
    if (hud) hud.textContent = `URDF: ${rootName} + ${joints.length} joints`;
  }

  // ---------- ROS topic wiring ----------
  function setupTopics() {
    if (!RosApp.ros) {
      const onReady = () => {
        RosApp.bus.removeEventListener('rosReady', onReady);
        setupTopics();
      };
      RosApp.bus.addEventListener('rosReady', onReady);
      teardown.push(() => RosApp.bus.removeEventListener('rosReady', onReady));
      return;
    }

    // /robot_description — std_msgs/String, latched (transient_local). One-shot.
    const urdfTopic = new ROSLIB.Topic({
      ros: RosApp.ros,
      name: '/robot_description',
      messageType: 'std_msgs/msg/String',
    });
    urdfTopic.subscribe((msg) => {
      urdfTopic.unsubscribe();
      try { buildURDFScene(msg.data); }
      catch (e) { console.error('[view3d] URDF parse failed', e); }
    });
    teardown.push(() => urdfTopic.unsubscribe());

    // /joint_states — drive movable joints.
    const jsTopic = new ROSLIB.Topic({
      ros: RosApp.ros,
      name: '/joint_states',
      messageType: 'sensor_msgs/msg/JointState',
    });
    jsTopic.subscribe(onJointStates);
    teardown.push(() => jsTopic.unsubscribe());

    // /scan — attach points to the lidar link if/when it exists.
    const scanTopic = new ROSLIB.Topic({
      ros: RosApp.ros,
      name: '/scan',
      messageType: 'sensor_msgs/msg/LaserScan',
      throttle_rate: 100,
    });
    scanTopic.subscribe(onScan);
    teardown.push(() => scanTopic.unsubscribe());

    // /map — render once (latched). Resubscribe on demand if it republishes.
    const mapTopic = new ROSLIB.Topic({
      ros: RosApp.ros,
      name: '/map',
      messageType: 'nav_msgs/msg/OccupancyGrid',
    });
    mapTopic.subscribe(onMap);
    teardown.push(() => mapTopic.unsubscribe());
  }

  function onJointStates(msg) {
    if (!msg || !msg.name) return;
    for (let i = 0; i < msg.name.length; i++) {
      const pivot = jointPivots[msg.name[i]];
      if (!pivot || pivot.type === 'fixed') continue;
      pivot.axisGroup.quaternion.setFromAxisAngle(pivot.axis, msg.position[i] || 0);
    }
  }

  function onScan(msg) {
    if (!scanPoints) {
      const geom = new THREE.BufferGeometry();
      geom.setAttribute('position', new THREE.BufferAttribute(new Float32Array(0), 3));
      const mat = new THREE.PointsMaterial({ color: 0xff5555, size: 0.04 });
      scanPoints = new THREE.Points(geom, mat);
    }
    // Build XY points in the scan's local frame. URDF anchors the lidar
    // via a fixed joint, so attaching to the lidar_link group positions
    // the points correctly relative to the robot.
    const N = msg.ranges.length;
    const buf = new Float32Array(N * 3);
    let n = 0;
    for (let i = 0; i < N; i++) {
      const r = msg.ranges[i];
      if (!isFinite(r) || r < msg.range_min || r > msg.range_max) continue;
      const a = msg.angle_min + i * msg.angle_increment;
      buf[n * 3 + 0] = r * Math.cos(a);
      buf[n * 3 + 1] = r * Math.sin(a);
      buf[n * 3 + 2] = 0;
      n++;
    }
    const positions = buf.subarray(0, n * 3);
    scanPoints.geometry.setAttribute('position',
      new THREE.BufferAttribute(positions, 3));
    scanPoints.geometry.computeBoundingSphere();
    scanPoints.visible = layers.scan;

    // Reparent on first scan so we hang off the right frame.
    if (!scanPoints.parent) {
      const parent = linkGroups['lidar_link'] || chassisGroup || scene;
      parent.add(scanPoints);
    }
  }

  function onMap(msg) {
    const { width, height, resolution, origin } = msg.info;
    // Pack into an RGBA texture: free=white, occupied=black, unknown=gray.
    const tex = new Uint8Array(width * height * 4);
    for (let i = 0; i < msg.data.length; i++) {
      const v = msg.data[i];
      let g;
      if (v < 0)        g = 128;  // unknown
      else if (v >= 50) g = 30;   // occupied
      else              g = 235;  // free
      tex[i * 4 + 0] = g;
      tex[i * 4 + 1] = g;
      tex[i * 4 + 2] = g;
      tex[i * 4 + 3] = 220;
    }
    const dataTex = new THREE.DataTexture(tex, width, height, THREE.RGBAFormat);
    dataTex.flipY = true;       // ROS rows are bottom-up; three.js textures default top-down
    dataTex.magFilter = THREE.NearestFilter;
    dataTex.minFilter = THREE.NearestFilter;
    dataTex.needsUpdate = true;

    if (mapMesh) { scene.remove(mapMesh); mapMesh.geometry.dispose(); mapMesh.material.dispose(); }
    const geom = new THREE.PlaneGeometry(width * resolution, height * resolution);
    const mat  = new THREE.MeshBasicMaterial({ map: dataTex, transparent: true, depthWrite: false });
    mapMesh = new THREE.Mesh(geom, mat);
    // Plane is centred at origin; shift to map origin + half-extents.
    mapMesh.position.set(
      origin.position.x + (width * resolution) / 2,
      origin.position.y + (height * resolution) / 2,
      0.001,
    );
    mapMesh.visible = layers.map;
    scene.add(mapMesh);
  }

  // ---------- pose / goal drivers ----------
  function updateRobotPose() {
    const p = RosApp.state.pose;
    if (!p || !chassisGroup) return;
    chassisGroup.position.set(p.x, p.y, 0);
    chassisGroup.rotation.set(0, 0, p.yaw, 'ZYX');

    const hud = $('hud3dPose');
    if (hud) hud.textContent =
      `x: ${fmt(p.x, 2)}  y: ${fmt(p.y, 2)}  yaw: ${fmt(p.yaw, 2)}`;

    if (camFollow) {
      controls.target.set(p.x, p.y, 0.2);
    }
  }

  function updateGoal() {
    const g = RosApp.state.goal;
    if (!g) {
      if (goalArrow) goalArrow.visible = false;
      return;
    }
    if (!goalArrow) {
      goalArrow = new THREE.ArrowHelper(
        new THREE.Vector3(1, 0, 0),
        new THREE.Vector3(0, 0, 0),
        0.6, 0xffc44d, 0.15, 0.1,
      );
      scene.add(goalArrow);
    }
    goalArrow.visible = true;
    goalArrow.position.set(g.x, g.y, 0.02);
    goalArrow.setDirection(new THREE.Vector3(Math.cos(g.yaw), Math.sin(g.yaw), 0));
  }

  // ---------- sidebar hooks ----------
  function wireSidebar() {
    const bind = (id, key) => {
      const el = $(id);
      if (!el) return;
      el.onclick = () => {
        layers[key] = !layers[key];
        el.classList.toggle('active', layers[key]);
        applyLayerVisibility();
      };
    };
    bind('layer3d-scan', 'scan');
    bind('layer3d-map',  'map');
    bind('layer3d-grid', 'grid');
    bind('layer3d-axes', 'axes');

    const reset = $('cam-reset');
    if (reset) reset.onclick = resetCamera;
    const follow = $('cam-follow');
    if (follow) follow.onchange = () => { camFollow = follow.checked; };
  }

  function applyLayerVisibility() {
    if (scanPoints) scanPoints.visible = layers.scan;
    if (mapMesh)    mapMesh.visible    = layers.map;
    if (gridHelper) gridHelper.visible = layers.grid;
    if (axesHelper) axesHelper.visible = layers.axes;
  }

  function resetCamera() {
    camera.position.set(2.5, -2.5, 2);
    controls.target.set(0, 0, 0.2);
    controls.update();
  }

  // ---------- lifecycle ----------
  async function init() {
    hostEl = $('three-canvas');
    if (!hostEl) return null;

    try { await loadDeps(); }
    catch (e) { console.error('[view3d] failed to load three.js modules', e); return null; }

    scene = new THREE.Scene();
    scene.background = new THREE.Color(0x16181d);

    const w = hostEl.clientWidth, h = hostEl.clientHeight;
    camera = new THREE.PerspectiveCamera(55, w / h, 0.05, 200);
    camera.up.set(0, 0, 1);                  // REP-103 Z-up
    camera.position.set(2.5, -2.5, 2);

    renderer = new THREE.WebGLRenderer({ antialias: true });
    renderer.setSize(w, h);
    renderer.setPixelRatio(window.devicePixelRatio);
    hostEl.appendChild(renderer.domElement);

    controls = new OrbitControls(camera, renderer.domElement);
    controls.target.set(0, 0, 0.2);
    controls.update();

    scene.add(new THREE.AmbientLight(0xffffff, 0.55));
    const dl = new THREE.DirectionalLight(0xffffff, 0.75);
    dl.position.set(3, 3, 6);
    scene.add(dl);

    // Grid (XY plane in Z-up world) — three.js GridHelper is on the XZ plane.
    gridHelper = new THREE.GridHelper(20, 40, 0x444444, 0x222222);
    gridHelper.rotation.x = Math.PI / 2;
    scene.add(gridHelper);

    axesHelper = new THREE.AxesHelper(0.5);
    scene.add(axesHelper);

    wireSidebar();
    setupTopics();

    // Pose / goal listeners drive transforms each tick.
    const onPose = () => updateRobotPose();
    const onGoal = () => updateGoal();
    RosApp.bus.addEventListener('pose', onPose);
    RosApp.bus.addEventListener('goal', onGoal);
    teardown.push(() => RosApp.bus.removeEventListener('pose', onPose));
    teardown.push(() => RosApp.bus.removeEventListener('goal', onGoal));

    // Resize: ResizeObserver on the host catches sidebar collapses too.
    resizeObs = new ResizeObserver(() => {
      const w = hostEl.clientWidth, h = hostEl.clientHeight;
      if (w === 0 || h === 0) return;
      camera.aspect = w / h;
      camera.updateProjectionMatrix();
      renderer.setSize(w, h);
    });
    resizeObs.observe(hostEl);

    const animate = () => {
      animId = requestAnimationFrame(animate);
      controls.update();
      renderer.render(scene, camera);
    };
    animate();

    return { destroy };
  }

  function destroy() {
    if (animId) cancelAnimationFrame(animId);
    animId = null;
    teardown.forEach(fn => { try { fn(); } catch (e) {} });
    teardown.length = 0;
    if (resizeObs) { resizeObs.disconnect(); resizeObs = null; }
    if (renderer) {
      renderer.dispose();
      renderer.domElement.remove();
      renderer = null;
    }
    // Drop refs so a re-init starts from a clean state.
    scene = camera = controls = chassisGroup = null;
    Object.keys(linkGroups).forEach(k => delete linkGroups[k]);
    Object.keys(jointPivots).forEach(k => delete jointPivots[k]);
    mapMesh = scanPoints = goalArrow = gridHelper = axesHelper = null;
    urdfLoaded = false;
  }

  // Exposed for console debugging — `View3D.debug()` returns a summary of
  // the current scene state (link names, joint wiring, mesh load status).
  function debug() {
    const linkSummary = {};
    for (const [name, g] of Object.entries(linkGroups)) {
      const meshes = [];
      g.traverse((o) => {
        if (o.isMesh) {
          const n = o.geometry?.attributes?.position?.count || 0;
          meshes.push({ verts: n, visible: o.visible });
        }
      });
      // Walk parent chain to root (or scene).
      const chain = [];
      let p = g;
      while (p && p.parent) { chain.push(p.name || '(unnamed)'); p = p.parent; }
      linkSummary[name] = {
        meshes,
        worldPos: g.getWorldPosition(new THREE.Vector3()).toArray().map(x => +x.toFixed(3)),
        parentChain: chain,
      };
    }
    return {
      links: linkSummary,
      joints: Object.fromEntries(
        Object.entries(jointPivots).map(([n, j]) => [n, { type: j.type, axis: j.axis.toArray() }])),
      sceneChildren: scene?.children.length,
      cameraPos: camera?.position.toArray().map(x => +x.toFixed(2)),
    };
  }

  window.View3D = { init, debug };
})();
