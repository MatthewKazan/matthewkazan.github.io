import { MapViewer } from './viewer.js?v=6';
import { createCameraRecorder } from './camera.js?v=13';
import { drawLoopOverview } from './loop-overview.js?v=6';

const byId = (id) => document.getElementById(id);
const modes = [...document.querySelectorAll('[data-mode]')];
const frame = byId('frame');
const play = byId('play');
const before = byId('loop-before');
const after = byId('loop-after');
const reducedMotion = matchMedia('(prefers-reduced-motion: reduce)');
const number = new Intl.NumberFormat();
const replayDescription = 'Simulated RGB-D · ICP · Scan Context · GTSAM';
const sensorPreview = byId('sensor-preview');
const sensorImage = byId('sensor-image');
const sensorContext = sensorImage.getContext('2d');

let viewer;
let camera;
let mode = 'replay';
let recording;
let replayWorker;
let replayError = '';
let corrected;
let maps;
let replayIndex = 1;
let timer = null;
let closureDelay = null;
let transition = null;
let correction = 0;
let followProgress = reducedMotion.matches;
let previewFrame = 0;

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
  if (!recording?.frames.length || mode !== 'replay') return;
  const length = recording.frames.length;
  const finished = recording.complete && replayIndex === length;
  const comparing = finished && !!recording.loop;
  const solved = comparing && correction === 1;
  const adjustment = finished && corrected ? { poses: corrected, amount: comparing ? correction : 1 } : null;
  describePoints(viewer.setVisible(replayIndex, recording.initial, adjustment));
  frame.value = String(replayIndex);
  frame.style.setProperty('--progress', (100 * (replayIndex - 1) / Math.max(1, length - 1)) + '%');
  byId('frame-count').value = replayIndex + ' / ' + recording.total;
  byId('loop-comparison').hidden = byId('loop-overview').hidden = !comparing;
  sensorPreview.hidden = comparing;
  if (!comparing && replayIndex !== previewFrame) {
    const pixels = recording.frames[replayIndex - 1].preview;
    sensorContext?.putImageData(new ImageData(pixels, recording.width, recording.height), 0, 0);
    previewFrame = replayIndex;
  }
  if (comparing) {
    const choosingAfter = solved || transition !== null;
    before.setAttribute('aria-pressed', String(!choosingAfter));
    after.setAttribute('aria-pressed', String(choosingAfter));
    const loop = recording.loop;
    drawLoopOverview(byId('loop-path'), recording.initial, corrected, loop, correction);
    const evidence = byId('loop-evidence');
    const message = 'Scan Context ' + loop.similarity.toFixed(4) + ' · ICP ' + Math.round(loop.fitness * 100) + '%';
    if (evidence.textContent !== message) evidence.textContent = message;
    evidence.title = 'Matched keyframes ' + (loop.from + 1) + ' and ' + (loop.to + 1) + ' at scan ' + loop.detectedAt;
  }
  if (solved) setState('Loop closed', true);
  else if (replayError) setState(replayError);
  else if (finished && !comparing) setState('Replay complete · No loop verified');
  else if (finished) setState(transition !== null ? 'Applying loop closure' : 'Loop verified');
  else if (replayIndex === length && !recording.complete && (timer || followProgress)) setState('Registering scans…');
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
  if (!recording?.loop || mode !== 'replay') return;
  showCorrection(reducedMotion.matches);
  if (!reducedMotion.matches) {
    // Show the drifted return view before moving any samples into alignment.
    closureDelay = setTimeout(() => showCorrection(true), 1800);
  }
}

function setPlaying(playing) {
  clearInterval(timer);
  timer = null;
  if (playing && recording?.frames.length && mode === 'replay') {
    cancelClosureAnimation();
    correction = 0;
    if (recording.complete && replayIndex === recording.frames.length) replayIndex = 1;
    timer = setInterval(() => {
      if (replayIndex < recording.frames.length) replayIndex++;
      if (replayIndex === recording.frames.length && recording.complete) finishReplay();
      else updateReplay();
    }, 62);
  }
  updateReplay();
}

function loadReplay() {
  if (!recording?.frames.length) return;
  viewer.clear();
  for (const sample of recording.frames) viewer.addFrame(sample);
  if (maps) viewer.setReplayMaps(maps);
  frame.max = String(recording.frames.length);
  frame.disabled = false;
  play.disabled = false;
  updateReplay();
}

function startReplayWorker() {
  if (replayWorker || recording?.complete) return;
  recording = undefined;
  replayIndex = 1;
  previewFrame = 0;
  correction = 0;
  replayError = '';
  frame.disabled = play.disabled = true;
  byId('frame-count').value = '—';
  const worker = new Worker(new URL('./replay-worker.js?v=8', import.meta.url), { type: 'module' });
  replayWorker = worker;
  worker.onmessage = ({ data }) => {
    if (replayWorker !== worker) return;
    if (data.type === 'start') {
      recording = {
        frames: [], initial: [], total: data.total, width: data.width, height: data.height, complete: false,
      };
      sensorImage.width = data.width;
      sensorImage.height = data.height;
      byId('frame-count').value = '0 / ' + data.total;
    } else if (data.type === 'batch') {
      const first = recording.frames.length === 0;
      recording.frames.push(...data.frames);
      recording.initial.push(...data.poses);
      if (followProgress) replayIndex = recording.frames.length;
      if (mode === 'replay') {
        if (first) {
          loadReplay();
          if (!followProgress) setPlaying(true);
        } else {
          for (const sample of data.frames) viewer.addFrame(sample);
          frame.max = String(recording.frames.length);
          updateReplay();
        }
      }
    } else if (data.type === 'complete') {
      recording.complete = true;
      recording.initial = data.initial;
      recording.loop = data.loop;
      corrected = data.corrected;
      maps = data.maps;
      if (data.loop) {
        const shift = corrected.reduce((largest, { translation }, i) => {
          const previous = data.initial[i].translation;
          return Math.max(largest, Math.hypot(...translation.map((coordinate, axis) => coordinate - previous[axis])));
        }, 0);
        byId('loop-shift').textContent = 'Largest pose correction ' + Math.round(shift * 1000) + ' mm';
      }
      if (mode === 'replay') viewer.setReplayMaps(maps);
      if (replayIndex === recording.frames.length && mode === 'replay') finishReplay();
      else updateReplay();
      stopReplayWorker();
    } else if (data.type === 'error') {
      replayError = 'Replay processing stopped';
      console.error(data.message);
      setPlaying(false);
      if (mode === 'replay') setState(replayError);
      stopReplayWorker();
    }
  };
  worker.onerror = (error) => {
    if (replayWorker !== worker) return;
    console.error(error.message);
    replayError = 'Could not load replay';
    setPlaying(false);
    if (mode === 'replay') setState(replayError);
    stopReplayWorker();
  };
}

function stopReplayWorker() {
  replayWorker?.terminate();
  replayWorker = null;
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
  sensorPreview.hidden = true;
  if (mode === 'camera') {
    stopReplayWorker();
    if (!recording?.complete) recording = undefined;
    byId('demo-description').textContent = 'Camera RGB-D · ICP · Scan Context · GTSAM';
    camera.open();
  } else {
    byId('demo-description').textContent = replayDescription;
    setState('Loading example');
    if (recording?.complete) loadReplay();
    else {
      viewer.clear();
      describePoints(0);
      startReplayWorker();
    }
  }
}

try {
  viewer = new MapViewer(byId('map'));
  camera = createCameraRecorder({ viewer, setState, describePoints });
  startReplayWorker();
  modes.forEach((button) => button.addEventListener('click', () => changeMode(button.dataset.mode)));
  play.addEventListener('click', () => {
    followProgress = false;
    setPlaying(!timer);
  });
  frame.addEventListener('input', () => {
    followProgress = false;
    replayIndex = frame.valueAsNumber;
    cancelClosureAnimation();
    correction = 0;
    if (recording.complete && replayIndex === recording.frames.length) finishReplay();
    else setPlaying(false);
  });
  // In camera mode the recorder handles these buttons.
  before.addEventListener('click', () => mode === 'replay' && showCorrection(false));
  after.addEventListener('click', () => mode === 'replay' && showCorrection(true));
  byId('reset-view').addEventListener('click', () => viewer.reset());
  window.addEventListener('pagehide', () => {
    cancelClosureAnimation();
    setPlaying(false);
    stopReplayWorker();
    camera.stop();
  });
  window.addEventListener('pageshow', () => {
    if (mode === 'replay') startReplayWorker();
  });
} catch (error) {
  console.error(error);
  byId('map-fallback').hidden = false;
  modes.forEach((button) => { button.disabled = true; });
  byId('reset-view').disabled = true;
  setState('Map unavailable');
}
