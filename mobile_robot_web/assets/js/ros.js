// ros.js — Backend module: owns the rosbridge connection, ROS topic
// subscriptions, publishers, and shared application state. Views and the
// dashboard shell consume state via the RosApp.bus event emitter and act on
// the robot via RosApp methods (sendNavGoal / publishInit / publishTwist /
// cancelGoal / clearTrail).
//
// Map rendering itself is NOT done here — ROS2D's OccupancyGridClient needs
// a viewer/scene, so the 2D view owns its own gridClient. ros.js exposes the
// underlying RosApp.ros handle for that.
(function () {
  const bus = new EventTarget();
  const emit = (type, detail) => bus.dispatchEvent(new CustomEvent(type, { detail }));

  const state = {
    pose: null,        // {x, y, yaw} — map frame if AMCL active, else odom frame
    poseSrc: null,     // 'amcl' | 'odom'
    odom: null,        // raw nav_msgs/Odometry
    scan: null,        // raw sensor_msgs/LaserScan
    plan: null,        // [{x, y}, ...] from /plan (global)
    localPlan: null,   // [{x, y}, ...] from /local_plan
    goal: null,        // {x, y, yaw}
    trail: [],         // [{x, y}, ...] history (3 cm thinned, 5000-pt cap)
    trailDist: 0,      // total traversed distance in metres
    navStatus: null,   // { label, cls, code }
    connected: false,
  };

  const pubs = { goal: null, init: null, cmdVel: null };

  let ros = null;
  let navActionClient = null;
  let currentGoalHandle = null;

  function quatToYaw(q) {
    const siny = 2 * (q.w * q.z + q.x * q.y);
    const cosy = 1 - 2 * (q.y * q.y + q.z * q.z);
    return Math.atan2(siny, cosy);
  }
  function yawToQuat(yaw) {
    return { x: 0, y: 0, z: Math.sin(yaw / 2), w: Math.cos(yaw / 2) };
  }

  function pushTrail(x, y) {
    const t = state.trail;
    if (t.length === 0) { t.push({ x, y }); emit('trail'); return; }
    const last = t[t.length - 1];
    const d = Math.hypot(x - last.x, y - last.y);
    if (d < 0.03) return;             // 3 cm thinning
    if (d > 5.0) {                     // teleport / pose reset: drop history
      state.trail = [{ x, y }];
      state.trailDist = 0;
      emit('trail');
      return;
    }
    t.push({ x, y });
    state.trailDist += d;
    if (t.length > 5000) t.shift();
    emit('trail');
  }

  function onPoseUpdate(x, y, yaw, src) {
    state.pose = { x, y, yaw };
    state.poseSrc = src;
    pushTrail(x, y);
    emit('pose');
  }

  function connect(host) {
    if (ros) { try { ros.close(); } catch (e) {} }
    ros = new ROSLIB.Ros({ url: 'ws://' + host + ':9090' });
    emit('status', { text: 'connecting…', cls: 'warn' });

    ros.on('connection', () => {
      state.connected = true;
      emit('status', { text: 'connected', cls: 'ok' });
      setupTopics();
      emit('rosReady');
    });
    ros.on('close', () => {
      state.connected = false;
      emit('status', { text: 'disconnected', cls: 'bad' });
    });
    ros.on('error', (e) => {
      console.error('ROS error', e);
      emit('status', { text: 'error', cls: 'bad' });
    });
  }

  function setupTopics() {
    new ROSLIB.Topic({
      ros, name: '/amcl_pose',
      messageType: 'geometry_msgs/msg/PoseWithCovarianceStamped',
    }).subscribe((msg) => {
      onPoseUpdate(
        msg.pose.pose.position.x,
        msg.pose.pose.position.y,
        quatToYaw(msg.pose.pose.orientation),
        'amcl'
      );
    });

    new ROSLIB.Topic({
      ros, name: '/odom',
      messageType: 'nav_msgs/msg/Odometry',
    }).subscribe((msg) => {
      state.odom = msg;
      // Only use /odom as the pose source until AMCL starts publishing.
      if (state.poseSrc !== 'amcl') {
        onPoseUpdate(
          msg.pose.pose.position.x,
          msg.pose.pose.position.y,
          quatToYaw(msg.pose.pose.orientation),
          'odom'
        );
      } else {
        emit('odom');
      }
    });

    new ROSLIB.Topic({
      ros, name: '/scan',
      messageType: 'sensor_msgs/msg/LaserScan',
      throttle_rate: 100,
    }).subscribe((msg) => { state.scan = msg; emit('scan'); });

    new ROSLIB.Topic({
      ros, name: '/plan',
      messageType: 'nav_msgs/msg/Path',
      throttle_rate: 200,
    }).subscribe((msg) => {
      state.plan = msg.poses.map(p => ({ x: p.pose.position.x, y: p.pose.position.y }));
      emit('plan');
    });

    new ROSLIB.Topic({
      ros, name: '/local_plan',
      messageType: 'nav_msgs/msg/Path',
      throttle_rate: 200,
    }).subscribe((msg) => {
      state.localPlan = msg.poses.map(p => ({ x: p.pose.position.x, y: p.pose.position.y }));
      emit('localPlan');
    });

    new ROSLIB.Topic({
      ros, name: '/goal_pose',
      messageType: 'geometry_msgs/msg/PoseStamped',
    }).subscribe((msg) => {
      state.goal = {
        x: msg.pose.position.x,
        y: msg.pose.position.y,
        yaw: quatToYaw(msg.pose.orientation),
      };
      emit('goal');
    });

    // action_msgs/GoalStatusArray statuses:
    //   1=accepted, 2=executing, 3=cancelling, 4=succeeded, 5=canceled, 6=aborted
    new ROSLIB.Topic({
      ros, name: '/navigate_to_pose/_action/status',
      messageType: 'action_msgs/msg/GoalStatusArray',
    }).subscribe((msg) => {
      if (!msg.status_list || msg.status_list.length === 0) return;
      const s = msg.status_list[msg.status_list.length - 1].status;
      const label = { 1: 'accepted', 2: 'executing', 3: 'cancelling',
                      4: 'succeeded', 5: 'canceled', 6: 'aborted' }[s] || 'unknown';
      const cls = (s === 2) ? 'active' : (s === 4) ? 'ok' : (s === 6) ? 'bad' : 'warn';
      state.navStatus = { label, cls, code: s };
      emit('navStatus');
      // Terminal states: drop stale plan visuals.
      if (s === 4 || s === 5 || s === 6) {
        state.plan = null;
        state.localPlan = null;
        emit('plan');
        emit('localPlan');
      }
    });

    pubs.goal   = new ROSLIB.Topic({ ros, name: '/goal_pose',
                                     messageType: 'geometry_msgs/msg/PoseStamped' });
    pubs.init   = new ROSLIB.Topic({ ros, name: '/initialpose',
                                     messageType: 'geometry_msgs/msg/PoseWithCovarianceStamped' });
    pubs.cmdVel = new ROSLIB.Topic({ ros, name: '/cmd_vel',
                                     messageType: 'geometry_msgs/msg/Twist' });

    try {
      navActionClient = new ROSLIB.ActionClient({
        ros,
        serverName: '/navigate_to_pose',
        actionName: 'nav2_msgs/action/NavigateToPose',
      });
    } catch (e) {
      console.warn('NavigateToPose ActionClient unavailable:', e);
    }
  }

  function sendNavGoal(x, y, yaw) {
    const q = yawToQuat(yaw);
    const poseStamped = {
      header: { stamp: { sec: 0, nanosec: 0 }, frame_id: 'map' },
      pose: { position: { x, y, z: 0 }, orientation: q },
    };
    if (navActionClient) {
      try {
        currentGoalHandle = new ROSLIB.Goal({
          actionClient: navActionClient,
          goalMessage: { pose: poseStamped, behavior_tree: '' },
        });
        currentGoalHandle.send();
      } catch (e) { console.warn('Action send failed, falling back to /goal_pose', e); }
    }
    // Always publish to /goal_pose too so any subscriber sees it and our own
    // goal marker / readouts update consistently.
    if (pubs.goal) pubs.goal.publish(new ROSLIB.Message(poseStamped));
  }

  function publishInit(x, y, yaw) {
    if (!pubs.init) return;
    const q = yawToQuat(yaw);
    pubs.init.publish(new ROSLIB.Message({
      header: { stamp: { sec: 0, nanosec: 0 }, frame_id: 'map' },
      pose: {
        pose: { position: { x, y, z: 0 }, orientation: q },
        covariance: Array(36).fill(0).map((_, i) => (i % 7 === 0 ? 0.25 : 0)),
      },
    }));
  }

  function publishTwist(v, w) {
    if (!pubs.cmdVel) return;
    pubs.cmdVel.publish(new ROSLIB.Message({
      linear:  { x: v, y: 0, z: 0 },
      angular: { x: 0, y: 0, z: w },
    }));
  }

  function cancelGoal() {
    if (currentGoalHandle) { try { currentGoalHandle.cancel(); } catch (e) {} }
    if (!ros) return;
    // Belt-and-suspenders: also call the action's cancel service directly.
    try {
      const cancelSrv = new ROSLIB.Service({
        ros, name: '/navigate_to_pose/_action/cancel_goal',
        serviceType: 'action_msgs/srv/CancelGoal',
      });
      cancelSrv.callService(new ROSLIB.ServiceRequest({
        goal_info: { goal_id: { uuid: Array(16).fill(0) }, stamp: { sec: 0, nanosec: 0 } },
      }), () => {}, () => {});
    } catch (e) {}
  }

  function clearTrail() {
    state.trail = [];
    state.trailDist = 0;
    emit('trail');
  }

  window.RosApp = {
    bus, state, pubs,
    get ros() { return ros; },
    connect, sendNavGoal, publishInit, publishTwist, cancelGoal, clearTrail,
    quatToYaw, yawToQuat,
  };
})();
