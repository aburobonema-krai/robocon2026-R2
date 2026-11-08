/**
 * KRAI Operator HMI Web Interactive Replica & Simulator
 * Adapted from krai_operator_gui.py (PyQt5) for Portfolio Demo
 * POLINEMA Robotics Team - Robocon 2026 R2
 */

(function () {
  'use strict';

  // ==========================================================================
  // Mission Catalog (Matches index.yaml & match/*.yaml)
  // ==========================================================================
  const MISSION_CATALOG = {};

  // Register CP1 and CP2 Box missions 01-12
  for (let cp = 1; cp <= 2; cp++) {
    for (let b = 1; b <= 12; b++) {
      const bStr = String(b).padStart(2, '0');
      const id = `cp${cp}_box_${bStr}`;
      MISSION_CATALOG[id] = {
        name: `CP${cp} Match Box ${bStr}`,
        category: 'match',
        file: `match/${id}.yaml`,
        description: `Autonomous navigation from CP${cp} to Box ${bStr} target zone and return.`
      };
    }
    for (let w = 1; w <= 4; w++) {
      const wStr = String(w).padStart(2, '0');
      const id = `cp${cp}_weapon_${wStr}`;
      MISSION_CATALOG[id] = {
        name: `CP${cp} Weapon ${wStr}`,
        category: 'match',
        file: `match/${id}.yaml`,
        description: `Autonomous maneuver from CP${cp} to deploy Weapon ${wStr}.`
      };
    }
  }

  // Debug missions
  MISSION_CATALOG['debug_square'] = {
    name: 'Debug Square',
    category: 'debug',
    file: 'debug/debug_square.yaml',
    description: 'Fake-mode square-style sequence using relative move and turn primitives.'
  };
  MISSION_CATALOG['debug_forward_turn'] = {
    name: 'Debug Forward Turn',
    category: 'debug',
    file: 'debug/debug_forward_turn.yaml',
    description: 'Reset, move 50 cm, turn 45 deg, then stop. Good smoke test.'
  };

  // ==========================================================================
  // Simulation & HMI State
  // ==========================================================================
  const state = {
    selected_team_color: 'merah', // 'merah' or 'biru'
    selected_checkpoint: 1,        // 1 or 2
    selected_box_indices: [],      // Array of ints (1-12)
    selected_weapon_index: null,   // int (1-4) or null
    selected_path: null,           // int (1-8) or null
    selected_mission_id: '',
    
    // Safety & Run State
    is_estop: false,
    is_running: false,
    mission_time: 0,
    current_step_index: 0,
    
    // Robot Telemetry
    rx_count: 1420,
    crc_error: 0,
    cmd_age: 12,
    pose: { x: 0.0, y: 0.0, yaw: 0.0, v: 0.0, wz: 0.0 },
    base: { count_left: 0, count_right: 0, rpm_l: 0, rpm_r: 0, pwm_l: 0, pwm_r: 0 },
    primitive: { id: 0, type: 'idle', state: 'IDLE', progress: 0.0, remaining: 0.0, heading_error: 0.0 },
    mission_state: { name: 'none', step_id: 0, step_type: 'idle', state: 0, detail: 'mission manager ready' },
    last_feedback: '',
    last_result: '-',
    
    // History
    event_logs: [],
    trajectory: []
  };

  // Trajectory plan for simulated runs
  const SIMULATED_STEPS = [
    { type: 'ALIGN_HEADING', duration: 1.5, targetYaw: 25.0, v: 0.0, wz: 0.45 },
    { type: 'MOVE_STRAIGHT', duration: 3.0, targetDist: 1.8, v: 0.65, wz: 0.0 },
    { type: 'TURN_TO_BOX',   duration: 1.2, targetYaw: 85.0, v: 0.1, wz: 0.8 },
    { type: 'APPROACH_SLOW', duration: 2.0, targetDist: 0.6, v: 0.25, wz: 0.0 },
    { type: 'ACTUATE_SLAVE', duration: 1.8, targetDist: 0.0, v: 0.0, wz: 0.0, detail: 'Slave B mechanism engage' },
    { type: 'RETURN_REVERSE',duration: 3.5, targetDist: -2.2, v: -0.6, wz: -0.2 }
  ];

  // ==========================================================================
  // DOM References
  // ==========================================================================
  const elements = {
    // Navigation
    navControl: document.getElementById('nav-btn-control'),
    navIndex: document.getElementById('nav-btn-index'),
    pageControl: document.getElementById('page-control'),
    pageIndex: document.getElementById('page-index'),

    // Control Buttons
    btnPing: document.getElementById('btn-ping'),
    btnResetOdom: document.getElementById('btn-reset-odom'),
    btnResetHeading: document.getElementById('btn-reset-heading'),
    btnResetAll: document.getElementById('btn-reset-all'),
    btnStart: document.getElementById('btn-start'),
    btnStop: document.getElementById('btn-stop'),
    btnEstopOn: document.getElementById('btn-estop-on'),
    btnEstopClear: document.getElementById('btn-estop-clear'),

    // Labels & Textareas
    infoSelected: document.getElementById('info-selected'),
    infoMissionState: document.getElementById('info-mission-state'),
    infoMissionFb: document.getElementById('info-mission-fb'),
    infoHealth: document.getElementById('info-health'),
    infoOdom: document.getElementById('info-odom'),
    infoBase: document.getElementById('info-base'),
    infoPrimitive: document.getElementById('info-primitive'),
    debugBox: document.getElementById('debug-box'),
    logBox: document.getElementById('log-box'),
    canvasCoords: document.getElementById('canvas-coords'),
    fieldCanvas: document.getElementById('field-canvas'),

    // Index page elements
    btnCp1: document.getElementById('btn-cp1'),
    btnCp2: document.getElementById('btn-cp2'),
    pathGrid: document.getElementById('path-grid'),
    boxGrid: document.getElementById('box-grid'),
    weaponGrid: document.getElementById('weapon-grid'),
    btnTeamRed: document.getElementById('btn-team-red'),
    btnTeamBlue: document.getElementById('btn-team-blue'),

    // Modal
    modalOverlay: document.getElementById('modal-overlay'),
    modalTitle: document.getElementById('modal-title'),
    modalBody: document.getElementById('modal-body'),
    modalClose: document.getElementById('modal-close'),

    // Extras
    btnFullscreen: document.getElementById('btn-toggle-fullscreen'),
    simStatusPill: document.getElementById('sim-status-pill')
  };

  // ==========================================================================
  // Logging Helper
  // ==========================================================================
  function appendLog(msg) {
    const now = new Date();
    const timeStr = `${String(now.getHours()).padStart(2, '0')}:${String(now.getMinutes()).padStart(2, '0')}:${String(now.getSeconds()).padStart(2, '0')}.${String(Math.floor(now.getMilliseconds() / 100))}`;
    const entry = `[${timeStr}] ${msg}`;
    state.event_logs.push(entry);
    if (state.event_logs.length > 150) state.event_logs.shift();

    elements.logBox.value = state.event_logs.join('\n');
    elements.logBox.scrollTop = elements.logBox.scrollHeight;
  }

  function showModal(title, message) {
    elements.modalTitle.textContent = title;
    elements.modalBody.textContent = message;
    elements.modalOverlay.classList.add('show');
  }

  function closeModal() {
    elements.modalOverlay.classList.remove('show');
  }

  // ==========================================================================
  // Page Navigation & Team Styles
  // ==========================================================================
  function setPage(pageIndex) {
    if (pageIndex === 0) {
      elements.navControl.classList.add('active');
      elements.navIndex.classList.remove('active');
      elements.pageControl.classList.add('active');
      elements.pageIndex.classList.remove('active');
    } else {
      elements.navControl.classList.remove('active');
      elements.navIndex.classList.add('active');
      elements.pageControl.classList.remove('active');
      elements.pageIndex.classList.add('active');
    }
    refreshSelectionStyles();
  }

  function setTeam(color) {
    state.selected_team_color = color;
    appendLog(`Team set to ${color.toUpperCase()}`);
    
    // Update CSS variables for active accents
    const root = document.documentElement;
    if (color === 'merah') {
      root.style.setProperty('--btn-theme-active-bg', '#991b1b');
      root.style.setProperty('--btn-theme-active-border', '#ef4444');
    } else {
      root.style.setProperty('--btn-theme-active-bg', '#1e40af');
      root.style.setProperty('--btn-theme-active-border', '#3b82f6');
    }

    refreshSelectionStyles();
    refreshSelectionLabel();
  }

  // ==========================================================================
  // Selection Handlers (Matches PyQt5 logic)
  // ==========================================================================
  function selectCheckpoint(cp) {
    state.selected_checkpoint = cp;
    appendLog(`Selected CP${cp}`);
    refreshSelectionStyles();
    resolveSelectedMission();
  }

  function selectPath(pathNumber) {
    if (state.selected_path === pathNumber) {
      state.selected_path = null;
    } else {
      state.selected_path = pathNumber;
    }
    appendLog(`Path set to: ${state.selected_path || 'NONE'}`);
    refreshSelectionStyles();
    resolveSelectedMission();
  }

  function selectTarget(type, index) {
    if (state.selected_checkpoint === null) {
      appendLog('Pilih CP1 atau CP2 dulu sebelum memilih target.');
      showModal('Peringatan Checkpoint', 'Pilih CP1 atau CP2 terlebih dahulu sebelum memilih target box atau senjata!');
      refreshSelectionStyles();
      return;
    }

    if (type === 'box') {
      const idx = state.selected_box_indices.indexOf(index);
      if (idx > -1) {
        state.selected_box_indices.splice(idx, 1);
      } else {
        // Operator GUI allows single/multi box selection
        state.selected_box_indices = [index];
      }
    } else if (type === 'weapon') {
      if (state.selected_weapon_index === index) {
        state.selected_weapon_index = null;
      } else {
        state.selected_weapon_index = index;
      }
    }

    refreshSelectionStyles();
    resolveSelectedMission();
  }

  function resolveSelectedMission() {
    if (state.selected_checkpoint === null) {
      state.selected_mission_id = '';
      refreshSelectionLabel();
      return;
    }

    const parts = [`cp${state.selected_checkpoint}`];
    if (state.selected_box_indices.length > 0) {
      const sortedBoxes = [...state.selected_box_indices].sort((a, b) => a - b);
      for (const b of sortedBoxes) {
        parts.push(`box_${String(b).padStart(2, '0')}`);
      }
    }
    if (state.selected_weapon_index !== null) {
      parts.push(`weapon_${String(state.selected_weapon_index).padStart(2, '0')}`);
    }

    if (parts.length > 1) {
      state.selected_mission_id = parts.join('_');
    } else {
      state.selected_mission_id = '';
    }

    refreshSelectionLabel();
  }

  function refreshSelectionStyles() {
    const isRed = state.selected_team_color === 'merah';

    // Team buttons
    elements.btnTeamRed.classList.toggle('selected-red', isRed);
    elements.btnTeamBlue.classList.toggle('selected-blue', !isRed);

    // CP buttons
    elements.btnCp1.classList.toggle('checked', state.selected_checkpoint === 1);
    elements.btnCp2.classList.toggle('checked', state.selected_checkpoint === 2);

    // Path buttons
    const pathButtons = elements.pathGrid.querySelectorAll('.btn-path');
    pathButtons.forEach((btn, idx) => {
      btn.classList.toggle('checked', state.selected_path === (idx + 1));
    });

    // Box buttons
    const boxButtons = elements.boxGrid.querySelectorAll('.btn-box');
    boxButtons.forEach((btn, idx) => {
      btn.classList.toggle('checked', state.selected_box_indices.includes(idx + 1));
    });

    // Weapon buttons
    const weaponButtons = elements.weaponGrid.querySelectorAll('.btn-weapon');
    weaponButtons.forEach((btn, idx) => {
      btn.classList.toggle('checked', state.selected_weapon_index === (idx + 1));
    });
  }

  function refreshSelectionLabel() {
    const missionId = state.selected_mission_id;
    const exists = Boolean(MISSION_CATALOG[missionId]);
    const entry = MISSION_CATALOG[missionId] || null;

    const targetArr = [];
    if (state.selected_box_indices.length > 0) {
      const sortedBoxes = [...state.selected_box_indices].sort((a, b) => a - b);
      targetArr.push(`box ${sortedBoxes.join(', ')}`);
    }
    if (state.selected_weapon_index !== null) {
      targetArr.push(`weapon ${state.selected_weapon_index}`);
    }

    elements.infoSelected.textContent = [
      `Team: ${state.selected_team_color.toUpperCase()}`,
      `CP: ${state.selected_checkpoint || '-'}`,
      `Path: ${state.selected_path || '-'}`,
      `Target: ${targetArr.join(', ') || '-'}`,
      `Mission ID: ${missionId || '-'}`,
      `Catalog: ${exists ? 'OK' : 'not found / not loaded'}`,
      `Name: ${entry ? entry.name : '-'}`,
      `Desc: ${entry ? entry.description : '-'}`
    ].join('\n');
  }

  // ==========================================================================
  // Robot Control Actions (Start, Stop, Reset, Estop, Ping)
  // ==========================================================================
  function onStart() {
    resolveSelectedMission();
    if (!state.selected_mission_id) {
      showModal('Mission Belum Dipilih', 'Pilih Checkpoint (CP) dan target Box / Senjata terlebih dahulu di tab 2 INDEX.');
      appendLog('START blocked: no mission selected');
      return;
    }

    if (!MISSION_CATALOG[state.selected_mission_id]) {
      showModal('Mission ID Belum Terdaftar', `${state.selected_mission_id} belum ada di index catalog missions.`);
      appendLog(`START blocked, mission missing: ${state.selected_mission_id}`);
      return;
    }

    if (state.is_estop) {
      showModal('Emergency Stop Aktif', 'Lepas ESTOP CLEAR terlebih dahulu sebelum menjalankan misi.');
      appendLog('START blocked: ESTOP is active');
      return;
    }

    state.is_running = true;
    state.mission_time = 0;
    state.current_step_index = 0;
    state.last_result = '-';
    state.trajectory = [{ x: state.pose.x, y: state.pose.y }];

    appendLog(`START ${state.selected_mission_id}`);
    appendLog(`RUN_MISSION: sending goal ${state.selected_mission_id}`);
    setTimeout(() => {
      if (state.is_running) {
        appendLog('RUN_MISSION: goal accepted by action server');
      }
    }, 200);
  }

  function onStop() {
    state.is_running = false;
    state.pose.v = 0.0;
    state.pose.wz = 0.0;
    state.base.rpm_l = 0;
    state.base.rpm_r = 0;
    state.base.pwm_l = 0;
    state.base.pwm_r = 0;
    state.mission_state = {
      name: state.selected_mission_id || 'none',
      step_id: state.current_step_index,
      step_type: 'stopped',
      state: 2,
      detail: 'motion interrupted by /cmd_stop'
    };
    appendLog('STOP requested. Selection stays, next START restarts mission from beginning.');
  }

  function onResetAll() {
    onStop();
    state.pose = { x: 0.0, y: 0.0, yaw: 0.0, v: 0.0, wz: 0.0 };
    state.trajectory = [];
    state.selected_checkpoint = 1;
    state.selected_box_indices = [];
    state.selected_weapon_index = null;
    state.selected_path = null;
    state.selected_mission_id = '';
    state.last_feedback = '';
    state.last_result = '-';
    state.mission_state = {
      name: 'none',
      step_id: 0,
      step_type: 'idle',
      state: 0,
      detail: 'mission manager ready'
    };

    appendLog('RESET GUI + ROBOT requested: odom, heading & selections reset');
    refreshSelectionStyles();
    refreshSelectionLabel();
    drawField();
  }

  function onResetOdom() {
    state.pose.x = 0.0;
    state.pose.y = 0.0;
    state.trajectory = [{ x: 0, y: 0 }];
    appendLog('RESET ODOM: /reset_odom service OK, position set to (0.00, 0.00)');
    drawField();
  }

  function onResetHeading() {
    state.pose.yaw = 0.0;
    appendLog('RESET HEADING: /reset_heading service OK, yaw set to 0.0 deg');
    drawField();
  }

  function onEstop(enable) {
    state.is_estop = enable;
    if (enable) {
      state.is_running = false;
      state.pose.v = 0.0;
      state.pose.wz = 0.0;
      state.mission_state.detail = 'EMERGENCY STOP TRIGGERED';
      appendLog('ESTOP ON: Hardware brake engaged, emergency stop active!');
    } else {
      appendLog('ESTOP CLEAR: Safety interlock cleared, system ready.');
    }
  }

  function onPing() {
    const pingMs = (2.4 + Math.random() * 1.8).toFixed(1);
    appendLog(`PING sent to ESP32-S3 -> PONG (${pingMs} ms, RSSI=-41 dBm, free_heap=182 KB)`);
  }

  // ==========================================================================
  // Real-Time Simulation Engine (Runs every 100ms - 10Hz)
  // ==========================================================================
  function runSimulationTick() {
    state.rx_count += 1;
    state.cmd_age = Math.floor(Math.random() * 8) + 6;

    // Simulation if mission is actively running
    if (state.is_running && !state.is_estop) {
      state.mission_time += 0.1;
      const step = SIMULATED_STEPS[state.current_step_index];

      if (step) {
        state.primitive = {
          id: state.current_step_index + 1,
          type: step.type,
          state: 'EXECUTING',
          progress: Math.min(100, (state.mission_time / step.duration) * 100),
          remaining: Math.max(0, step.duration - state.mission_time),
          heading_error: (Math.sin(state.mission_time * 3) * 1.5)
        };

        state.mission_state = {
          name: state.selected_mission_id,
          step_id: state.current_step_index + 1,
          step_type: step.type.toLowerCase(),
          state: 1,
          detail: step.detail || `Executing trajectory waypoint ${state.current_step_index + 1}`
        };

        // Kinematics integration
        state.pose.v = step.v;
        state.pose.wz = step.wz;
        const dt = 0.1;
        state.pose.yaw += (step.wz * dt * (180 / Math.PI));
        const rad = (state.pose.yaw * Math.PI) / 180;
        state.pose.x += step.v * Math.cos(rad) * dt;
        state.pose.y += step.v * Math.sin(rad) * dt;

        // Base wheel telemetry
        state.base.rpm_l = Math.round(step.v * 2800 + (step.wz * 400));
        state.base.rpm_r = Math.round(step.v * 2800 - (step.wz * 400));
        state.base.pwm_l = Math.min(255, Math.abs(Math.round(step.v * 200)));
        state.base.pwm_r = Math.min(255, Math.abs(Math.round(step.v * 200)));
        state.base.count_left += Math.round(state.base.rpm_l * 0.05);
        state.base.count_right += Math.round(state.base.rpm_r * 0.05);

        // Record trajectory
        state.trajectory.push({ x: state.pose.x, y: state.pose.y });
        if (state.trajectory.length > 200) state.trajectory.shift();

        // Check step advance
        if (state.mission_time >= step.duration) {
          appendLog(`Step ${state.current_step_index + 1} (${step.type}) completed`);
          state.current_step_index++;
          state.mission_time = 0;

          if (state.current_step_index >= SIMULATED_STEPS.length) {
            // Mission complete
            state.is_running = false;
            state.pose.v = 0.0;
            state.pose.wz = 0.0;
            state.base.rpm_l = 0;
            state.base.rpm_r = 0;
            state.last_result = 'true: Mission completed successfully';
            state.mission_state.detail = 'Mission execution finished. Target reached.';
            appendLog(`RUN_MISSION result: true Mission ${state.selected_mission_id} finished successfully!`);
          }
        }
      }
    }

    // Refresh UI text boxes
    renderStatusLabels();
    renderDebugStream();
    drawField();
  }

  function renderStatusLabels() {
    // HEALTH
    const faultHex = state.is_estop ? '0x0001' : '0x0000';
    const mode = state.is_estop ? 'FAULT' : (state.is_running ? 'RUN' : 'READY');
    elements.infoHealth.textContent = [
      `HEALTH mode=${mode} ready=1 base=1`,
      `fault=${faultHex} rx=${state.rx_count} crc=${state.crc_error} cmd_age=${state.cmd_age}`
    ].join('\n');

    // ODOM
    elements.infoOdom.textContent = [
      `ODOM x=${state.pose.x.toFixed(2)}m y=${state.pose.y.toFixed(2)}m yaw=${state.pose.yaw.toFixed(1)}deg`,
      `v=${state.pose.v.toFixed(2)}m/s wz=${state.pose.wz.toFixed(2)}rad/s`
    ].join('\n');

    // BASE
    const slaveMode = state.is_running ? 'ACTIVE' : 'IDLE';
    elements.infoBase.textContent = [
      `BASE cnt=${state.base.count_left}/${state.base.count_right} d=42/41`,
      `rpm10=${state.base.rpm_l}/${state.base.rpm_r} pwm=${state.base.pwm_l}/${state.base.pwm_r} mode=${slaveMode} age=10`
    ].join('\n');

    // PRIMITIVE
    if (state.is_running) {
      elements.infoPrimitive.textContent = [
        `PRIM id=${state.primitive.id} type=${state.primitive.type} state=${state.primitive.state}`,
        `prog=${state.primitive.progress.toFixed(1)}% rem=${state.primitive.remaining.toFixed(1)}s herr=${state.primitive.heading_error.toFixed(1)}deg`
      ].join('\n');
    } else {
      elements.infoPrimitive.textContent = 'PRIMITIVE\nNo active trajectory';
    }

    // MISSION STATE
    elements.infoMissionState.textContent = [
      'MISSION STATE',
      state.mission_state.name,
      `step=${state.mission_state.step_id} type=${state.mission_state.step_type} state=${state.mission_state.state}`,
      state.mission_state.detail
    ].join('\n');

    // FEEDBACK
    elements.infoMissionFb.textContent = `RUN FEEDBACK\nResult: ${state.last_result}`;

    // Coords on canvas
    elements.canvasCoords.textContent = `X: ${state.pose.x.toFixed(2)}m | Y: ${state.pose.y.toFixed(2)}m | ${state.pose.yaw.toFixed(0)}°`;
  }

  function renderDebugStream() {
    const hexTime = (state.rx_count % 256).toString(16).toUpperCase().padStart(2, '0');
    const lines = [
      `[S3_TX] 55 AA 08 01 ${hexTime} 00 24 10 E2 ... CRC=OK`,
      `[S3_RX] 55 AA 14 02 01 00 00 00 ... V_BATT=24.2V`,
      `[ODOM_FUSION] gyro_z=${(state.pose.wz * 57.3).toFixed(2)} deg/s enc_left=${state.base.count_left}`,
      `[NUC_BRIDGE] heartbeat OK | stream 10Hz active`
    ];
    elements.debugBox.value = lines.join('\n');
  }

  // ==========================================================================
  // 2D Field Pose Visualizer (Canvas)
  // ==========================================================================
  function drawField() {
    const canvas = elements.fieldCanvas;
    if (!canvas) return;
    const ctx = canvas.getContext('2d');
    const w = canvas.width;
    const h = canvas.height;

    // Field bounds: 0 to 3.0m in X, -1.0 to 1.0m in Y
    const originX = 35;
    const originY = h / 2;
    const scale = (w - 70) / 3.0; // pixels per meter

    // Clear background
    ctx.fillStyle = '#061324';
    ctx.fillRect(0, 0, w, h);

    // Grid lines
    ctx.strokeStyle = '#12253e';
    ctx.lineWidth = 1;
    for (let x = 0; x <= 3; x += 0.5) {
      const px = originX + x * scale;
      ctx.beginPath();
      ctx.moveTo(px, 0);
      ctx.lineTo(px, h);
      ctx.stroke();
    }
    for (let y = -1; y <= 1; y += 0.5) {
      const py = originY - y * scale;
      ctx.beginPath();
      ctx.moveTo(0, py);
      ctx.lineTo(w, py);
      ctx.stroke();
    }

    // Field Border
    ctx.strokeStyle = '#1e3a5f';
    ctx.strokeRect(originX, originY - 1 * scale, 3 * scale, 2 * scale);

    // Checkpoint markers
    ctx.fillStyle = '#38bdf8';
    ctx.font = '9px JetBrains Mono';
    ctx.fillText('CP1 (0,0)', originX - 5, originY + 18);
    ctx.beginPath();
    ctx.arc(originX, originY, 4, 0, Math.PI * 2);
    ctx.fill();

    ctx.fillStyle = '#f59e0b';
    ctx.fillText('TARGET ZONE', originX + 2.1 * scale, originY - 0.5 * scale);
    ctx.strokeRect(originX + 2.0 * scale, originY - 0.7 * scale, 0.8 * scale, 1.4 * scale);

    // Trajectory Path
    if (state.trajectory.length > 1) {
      ctx.strokeStyle = state.selected_team_color === 'merah' ? 'rgba(239, 68, 68, 0.7)' : 'rgba(59, 130, 246, 0.7)';
      ctx.lineWidth = 2;
      ctx.beginPath();
      state.trajectory.forEach((pt, i) => {
        const px = originX + pt.x * scale;
        const py = originY - pt.y * scale;
        if (i === 0) ctx.moveTo(px, py);
        else ctx.lineTo(px, py);
      });
      ctx.stroke();
    }

    // Robot Footprint
    const rx = originX + state.pose.x * scale;
    const ry = originY - state.pose.y * scale;
    const yawRad = (-state.pose.yaw * Math.PI) / 180;

    ctx.save();
    ctx.translate(rx, ry);
    ctx.rotate(yawRad);

    // Robot body
    ctx.fillStyle = state.selected_team_color === 'merah' ? '#ef4444' : '#3b82f6';
    ctx.fillRect(-10, -8, 20, 16);
    ctx.strokeStyle = '#ffffff';
    ctx.lineWidth = 1.5;
    ctx.strokeRect(-10, -8, 20, 16);

    // Front pointer
    ctx.fillStyle = '#facc15';
    ctx.beginPath();
    ctx.moveTo(10, 0);
    ctx.lineTo(4, -4);
    ctx.lineTo(4, 4);
    ctx.closePath();
    ctx.fill();

    ctx.restore();
  }

  // ==========================================================================
  // Initialization & Event Binding
  // ==========================================================================
  function initGrids() {
    // 1. Path Grid (1 to 8)
    elements.pathGrid.innerHTML = '';
    for (let i = 1; i <= 8; i++) {
      const btn = document.createElement('button');
      btn.className = 'btn btn-neutral btn-path';
      btn.textContent = String(i);
      btn.addEventListener('click', () => selectPath(i));
      elements.pathGrid.appendChild(btn);
    }

    // 2. Box Grid (1 to 12)
    elements.boxGrid.innerHTML = '';
    for (let i = 1; i <= 12; i++) {
      const btn = document.createElement('button');
      btn.className = 'btn btn-neutral btn-box';
      btn.textContent = String(i);
      btn.addEventListener('click', () => selectTarget('box', i));
      elements.boxGrid.appendChild(btn);
    }

    // 3. Weapon Grid (1 to 4)
    elements.weaponGrid.innerHTML = '';
    for (let i = 1; i <= 4; i++) {
      const btn = document.createElement('button');
      btn.className = 'btn btn-neutral btn-weapon';
      btn.textContent = String(i);
      btn.addEventListener('click', () => selectTarget('weapon', i));
      elements.weaponGrid.appendChild(btn);
    }
  }

  function bindEvents() {
    // Navigation
    elements.navControl.addEventListener('click', () => setPage(0));
    elements.navIndex.addEventListener('click', () => setPage(1));

    // Team color buttons
    elements.btnTeamRed.addEventListener('click', () => setTeam('merah'));
    elements.btnTeamBlue.addEventListener('click', () => setTeam('biru'));

    // Checkpoint buttons
    elements.btnCp1.addEventListener('click', () => selectCheckpoint(1));
    elements.btnCp2.addEventListener('click', () => selectCheckpoint(2));

    // Control buttons
    elements.btnPing.addEventListener('click', onPing);
    elements.btnResetOdom.addEventListener('click', onResetOdom);
    elements.btnResetHeading.addEventListener('click', onResetHeading);
    elements.btnResetAll.addEventListener('click', onResetAll);
    elements.btnStart.addEventListener('click', onStart);
    elements.btnStop.addEventListener('click', onStop);
    elements.btnEstopOn.addEventListener('click', () => onEstop(true));
    elements.btnEstopClear.addEventListener('click', () => onEstop(false));

    // Modal
    elements.modalClose.addEventListener('click', closeModal);
    elements.modalOverlay.addEventListener('click', (e) => {
      if (e.target === elements.modalOverlay) closeModal();
    });

    // Keyboard Shortcuts (Matches PyQt5 Esc and F11)
    window.addEventListener('keydown', (e) => {
      if (e.key === 'F11') {
        e.preventDefault();
        toggleFullscreen();
      } else if (e.key === 'Escape') {
        if (document.body.classList.contains('fullscreen')) {
          toggleFullscreen();
        }
        closeModal();
      }
    });

    // Fullscreen banner button
    elements.btnFullscreen.addEventListener('click', toggleFullscreen);
  }

  function toggleFullscreen() {
    document.body.classList.toggle('fullscreen');
    const isFull = document.body.classList.contains('fullscreen');
    elements.btnFullscreen.textContent = isFull ? '⛶ Normal View' : '⛶ Fullscreen';
  }

  // ==========================================================================
  // Bootstrap Application
  // ==========================================================================
  function init() {
    initGrids();
    bindEvents();
    setTeam('merah');
    selectCheckpoint(1);
    appendLog('KRAI operator GUI web demo ready.');
    appendLog('Pilih Box/Weapon di tab 2 INDEX, lalu klik START di tab 1 CONTROL.');

    // 10Hz UI refresh & simulation tick
    setInterval(runSimulationTick, 100);
  }

  // Start when DOM ready
  if (document.readyState === 'loading') {
    document.addEventListener('DOMContentLoaded', init);
  } else {
    init();
  }
})();
