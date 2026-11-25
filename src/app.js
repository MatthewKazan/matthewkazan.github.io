import { MapViewer } from './viewer.js?v=2';
import { createCameraRecorder } from './camera.js?v=2';
import { drawLoopOverview } from './loop-overview.js?v=2';

const byId = (id) => document.getElementById(id);
const modes = [...document.querySelectorAll('[data-mode]')];
const frame = byId('frame');
const play = byId('play');
const before = byId('loop-before');
const after = byId('loop-after');
const reducedMotion = matchMedia('(prefers-reduced-motion: reduce)');
const number = new Intl.NumberFormat();

let viewer;
let camera;
let mode = 'replay';
let recording;
let replayError = '';
let corrected;
let maps;
let replayIndex = 1;
let timer = null;
let closureDelay = null;
let transition = null;
let correction = 0;

function setState(message, solved = false) {
  const label = byId('state-text');
  if (label.textContent !== message) label.textContent = message;
  byId('map-state').dataset.solved = String(solved);
}

function describePoints(count) {
  const label = byId('point-count');
  const value = number.format(count) + ' points';
  if (label.textContent !== value) label.textContent = value;
}

function updateReplay() {
  if (!recording || mode !== 'replay') return;
  const length = recording.frames.length;
  const finished = replayIndex === length;
  const comparing = finished && !!corrected;
  const solved = comparing && correction === 1;
  const adjustment = comparing ? { poses: corrected, amount: correction } : null;
  describePoints(viewer.setVisible(replayIndex, recording.initial, adjustment));
  frame.value = String(replayIndex);
  frame.style.setProperty('--progress', (100 * (replayIndex - 1) / (length - 1)) + '%');
  byId('frame-count').value = replayIndex + ' / ' + length;
  byId('loop-comparison').hidden = byId('loop-overview').hidden = !comparing;
  if (comparing) {
    const choosingAfter = solved || transition !== null;
    before.setAttribute('aria-pressed', String(!choosingAfter));
    after.setAttribute('aria-pressed', String(choosingAfter));
    drawLoopOverview(byId('loop-path'), recording.initial, corrected, correction);
    const poses = solved ? corrected : recording.initial;
    const end = poses.at(-1).translation;
    const distance = Math.hypot(...poses[0].translation.map((value, axis) => value - end[axis]));
    const gap = byId('trajectory-gap');
    const message = transition !== null ? 'Aligning the start and return scans…'
      : 'Start–end gap ' + (distance < 0.01 ? '< 1 cm' : distance.toFixed(2) + ' m');
    if (gap.textContent !== message) gap.textContent = message;
    gap.dataset.closed = String(solved);
  }
  if (solved) setState('Loop closed', true);
  else if (finished && replayError) setState(replayError);
  else if (finished && !corrected) setState('Solving pose graph…');
  else if (finished) setState(transition !== null ? 'Applying loop closure' : 'Before loop closure');
  else setState(timer ? 'Building map' : 'Replay paused');

  play.textContent = timer ? 'Pause' : finished ? 'Replay' : 'Play';
  play.setAttribute('aria-label', timer ? 'Pause replay' : finished ? 'Restart replay' : 'Play replay');
}

function cancelClosureAnimation() {
  clearTimeout(closureDelay);
  cancelAnimationFrame(transition);
  closureDelay = transition = null;
}

function showCorrection(applyCorrection) {
  cancelClosureAnimation();
  if (!applyCorrection || reducedMotion.matches || correction === 1) {
    correction = applyCorrection ? 1 : 0;
    updateReplay();
    return;
  }
  const from = correction;
  const start = performance.now();
  function animate(now) {
    const fraction = Math.min(1, (now - start) / 1000);
    const eased = fraction * fraction * (3 - 2 * fraction);
    correction = fraction === 1 ? 1 : from + (1 - from) * eased;
    transition = fraction < 1 ? requestAnimationFrame(animate) : null;
    updateReplay();
  }
  transition = requestAnimationFrame(animate);
  updateReplay();
}

function finishReplay() {
  setPlaying(false);
  if (!corrected || mode !== 'replay') return;
  showCorrection(reducedMotion.matches);
  if (!reducedMotion.matches) {
    // Show the drifted return view before moving any samples into alignment.
    closureDelay = setTimeout(() => showCorrection(true), 1800);
  }
}

function setPlaying(playing) {
  clearInterval(timer);
  timer = null;
  if (playing && recording && mode === 'replay') {
    cancelClosureAnimation();
    correction = 0;
    if (replayIndex === recording.frames.length) replayIndex = 1;
    timer = setInterval(() => {
      replayIndex++;
      if (replayIndex >= recording.frames.length) finishReplay();
      else updateReplay();
    }, 290);
  }
  updateReplay();
}

function loadReplay() {
  if (!recording) return;
  viewer.clear();
  for (const sample of recording.frames) viewer.addFrame(sample);
  if (maps) viewer.setReplayMaps(maps);
  frame.max = String(recording.frames.length);
  frame.disabled = false;
  play.disabled = false;
  updateReplay();
}

function startReplayWorker() {
  const worker = new Worker(new URL('./replay-worker.js?v=2', import.meta.url), { type: 'module' });
  worker.onmessage = ({ data }) => {
    if (data.type === 'ready') {
      recording = data;
      replayIndex = reducedMotion.matches ? recording.frames.length : 1;
      if (mode === 'replay') {
        loadReplay();
        if (!reducedMotion.matches) setPlaying(true);
      }
    } else if (data.type === 'solved') {
      if (!data.maps?.length) {
        replayError = 'Reload to update the replay';
        updateReplay();
        worker.terminate();
        return;
      }
      corrected = data.corrected;
      maps = data.maps;
      if (mode === 'replay') viewer.setReplayMaps(maps);
      if (recording && replayIndex === recording.frames.length && mode === 'replay') finishReplay();
      else updateReplay();
      worker.terminate();
    } else if (data.type === 'error') {
      replayError = 'Loop closure unavailable';
      console.error(data.message);
      updateReplay();
      worker.terminate();
    }
  };
  worker.onerror = (error) => {
    console.error(error.message);
    replayError = 'Could not load replay';
    if (mode === 'replay') setState(replayError);
    worker.terminate();
  };
}

function changeMode(next) {
  if (mode === next) return;
  if (transition !== null || closureDelay !== null) correction = 1;
  cancelClosureAnimation();
  setPlaying(false);
  if (mode === 'camera') camera.close();
  mode = next;
  for (const button of modes) button.setAttribute('aria-pressed', String(button.dataset.mode === mode));
  byId('replay-controls').hidden = mode !== 'replay';
  byId('camera-controls').hidden = mode !== 'camera';
  byId('loop-comparison').hidden = byId('loop-overview').hidden = true;
  if (mode === 'camera') {
    byId('demo-description').textContent = 'On-device RGB depth';
    camera.open();
  } else {
    byId('demo-description').textContent = 'Simulated RGB-D · GTSAM in WebAssembly';
    setState(replayError || 'Loading example');
    loadReplay();
  }
}

try {
  viewer = new MapViewer(byId('map'));
  camera = createCameraRecorder({ viewer, setState, describePoints });
  startReplayWorker();
  modes.forEach((button) => button.addEventListener('click', () => changeMode(button.dataset.mode)));
  play.addEventListener('click', () => setPlaying(!timer));
  frame.addEventListener('input', () => {
    replayIndex = frame.valueAsNumber;
    cancelClosureAnimation();
    correction = 0;
    if (replayIndex === recording.frames.length) finishReplay();
    else setPlaying(false);
  });
  before.addEventListener('click', () => showCorrection(false));
  after.addEventListener('click', () => showCorrection(true));
  byId('reset-view').addEventListener('click', () => viewer.reset());
  window.addEventListener('pagehide', () => {
    cancelClosureAnimation();
    setPlaying(false);
    if (mode === 'camera') camera.stop();
  });
} catch (error) {
  console.error(error);
  byId('map-fallback').hidden = false;
  modes.forEach((button) => { button.disabled = true; });
  byId('reset-view').disabled = true;
  setState('Map unavailable');
}
