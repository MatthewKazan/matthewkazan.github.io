import { MapViewer } from './viewer.js';
import { depthInputSize } from './depth.js';

const byId = (id) => document.getElementById(id);
const modes = [...document.querySelectorAll('[data-mode]')];
const canvas = byId('map');
const frame = byId('frame');
const play = byId('play');
const loop = byId('loop-toggle');
const video = byId('camera-preview');
const capture = byId('capture');
const startCameraButton = byId('enable-camera');
const stopCameraButton = byId('stop-camera');
const feedback = byId('camera-feedback');
const pointCount = byId('point-count');
const number = new Intl.NumberFormat();

let viewer;
let mode = 'replay';
let recording;
let replayError = '';
let corrected;
let replayIndex = 1;
let timer = null;
let cameraStream;
let cameraWorker;
let cameraGeneration = 0;
let cameraFrames = 0;
let cameraLoops = 0;
const cameraLimit = 24;

function setState(message, solved = false) {
  byId('state-text').textContent = message;
  byId('map-state').dataset.solved = String(solved);
}

function describePoints(count) {
  pointCount.textContent = number.format(count) + ' points';
}

function updateReplay() {
  if (!recording || mode !== 'replay') return;
  const length = recording.frames.length;
  const finished = replayIndex === length;
  const solved = finished && corrected && loop.getAttribute('aria-pressed') === 'true';
  describePoints(viewer.setVisible(replayIndex, solved ? corrected : recording.initial));
  frame.value = String(replayIndex);
  frame.style.setProperty('--progress', (100 * (replayIndex - 1) / (length - 1)) + '%');
  byId('frame-count').value = replayIndex + ' / ' + length;
  if (solved) setState('Loop closed', true);
  else if (finished && replayError) setState(replayError);
  else if (finished && !corrected) setState('Solving pose graph…');
  else if (finished) setState('Uncorrected odometry');
  else setState(timer ? 'Building map' : 'Replay paused');
}

function setPlaying(playing) {
  clearInterval(timer);
  timer = null;
  if (playing && recording && mode === 'replay') {
    if (replayIndex === recording.frames.length) replayIndex = 1;
    timer = setInterval(() => {
      replayIndex++;
      if (replayIndex >= recording.frames.length) setPlaying(false);
      else updateReplay();
    }, 255);
  }
  play.setAttribute('aria-label', timer ? 'Pause replay' : 'Play replay');
  play.firstElementChild.textContent = timer ? '❚❚' : '▶';
  updateReplay();
}

function loadReplay() {
  if (!recording) return;
  viewer.clear();
  for (const sample of recording.frames) viewer.addFrame(sample);
  frame.max = String(recording.frames.length);
  frame.disabled = false;
  play.disabled = false;
  loop.disabled = !corrected;
  updateReplay();
}

function startReplayWorker() {
  const worker = new Worker(new URL('./replay-worker.js', import.meta.url), { type: 'module' });
  worker.onmessage = ({ data }) => {
    if (data.type === 'ready') {
      recording = data;
      const reduceMotion = matchMedia('(prefers-reduced-motion: reduce)').matches;
      replayIndex = reduceMotion ? recording.frames.length : 7;
      if (mode === 'replay') {
        loadReplay();
        if (!reduceMotion) setPlaying(true);
      }
    } else if (data.type === 'solved') {
      corrected = data.corrected;
      if (mode === 'replay') {
        loop.disabled = false;
        updateReplay();
      }
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
    if (mode === 'replay') setState('Could not load replay');
    worker.terminate();
  };
}

function showFeedback(message) {
  feedback.hidden = !message;
  feedback.textContent = message;
}

function changeMode(next) {
  if (mode === next) return;
  setPlaying(false);
  if (mode === 'camera') stopCamera();
  mode = next;
  for (const button of modes) button.setAttribute('aria-pressed', String(button.dataset.mode === mode));
  byId('replay-controls').hidden = mode !== 'replay';
  byId('camera-controls').hidden = mode !== 'camera';
  byId('camera-prompt').hidden = mode !== 'camera';
  if (mode === 'camera') {
    viewer.clear(true);
    cameraFrames = 0;
    byId('camera-frame-count').textContent = '0 frames';
    describePoints(0);
    byId('demo-description').textContent = 'Experimental · Estimated depth';
    setState('Camera off');
  } else {
    byId('demo-description').textContent = 'Simulated RGB-D · GTSAM in WebAssembly';
    setState('Loading example');
    loadReplay();
  }
}

function stopCamera() {
  cameraGeneration++;
  cameraStream?.getTracks().forEach((track) => track.stop());
  cameraStream = null;
  cameraWorker?.terminate();
  cameraWorker = null;
  video.srcObject = null;
  video.hidden = true;
  startCameraButton.disabled = false;
  capture.disabled = true;
  stopCameraButton.disabled = true;
  showFeedback('');
  if (mode === 'camera') {
    byId('camera-prompt').hidden = false;
    setState('Camera off');
  }
}

async function startCamera() {
  const generation = ++cameraGeneration;
  startCameraButton.disabled = true;
  stopCameraButton.disabled = false;
  setState('Waiting for camera');
  showFeedback('');
  try {
    if (!navigator.mediaDevices?.getUserMedia) throw new Error('Camera permission requires a supported browser over HTTPS.');
    const stream = await navigator.mediaDevices.getUserMedia({
      audio: false,
      video: { facingMode: { ideal: 'environment' }, width: { ideal: 640 }, height: { ideal: 480 } },
    });
    if (generation !== cameraGeneration) {
      stream.getTracks().forEach((track) => track.stop());
      return;
    }
    cameraStream = stream;
    video.srcObject = stream;
    await video.play();
    if (generation !== cameraGeneration) return;
    video.hidden = false;
    byId('camera-prompt').hidden = true;
    viewer.clear(true);
    describePoints(0);
    cameraFrames = 0;
    cameraLoops = 0;
    byId('camera-frame-count').textContent = '0 frames';
    setState('Camera ready');
    showFeedback('Capture a frame, then move a little before the next capture. Keep some of the previous view in sight.');
    capture.disabled = false;
    stream.getVideoTracks().forEach((track) => track.addEventListener('ended', () => {
      if (generation !== cameraGeneration) return;
      stopCamera();
      showFeedback('Camera disconnected. Enable it again to start a new map.');
    }));
  } catch (error) {
    if (generation !== cameraGeneration) return;
    stopCamera();
    if (error.name === 'NotAllowedError') showFeedback('Camera permission was declined. You can still use Replay.');
    else if (error.name === 'NotFoundError') showFeedback('No camera was found. You can still use Replay.');
    else showFeedback('Camera unavailable. ' + error.message);
  }
}

function handleCameraMessage({ data }) {
  if (data.type === 'progress') {
    setState(data.message);
    return;
  }
  if (data.type === 'frame') {
    cameraFrames++;
    viewer.addFrame(data);
    describePoints(viewer.setVisible(cameraFrames, data.poses));
    byId('camera-frame-count').textContent = cameraFrames + (cameraFrames === 1 ? ' frame' : ' frames');
    if (data.loops > cameraLoops) {
      setState('Loop closed', true);
      showFeedback('GTSAM corrected the camera poses after revisiting an earlier view.');
    } else {
      setState('Frame added', true);
      showFeedback('Move a little and capture the next overlapping view.');
    }
    cameraLoops = data.loops;
    if (cameraFrames >= cameraLimit) {
      showFeedback('Map complete. Orbit to inspect, or stop the camera to start a new capture.');
      cameraStream?.getVideoTracks().forEach((track) => track.stop());
      video.srcObject = null;
      video.hidden = true;
      cameraWorker?.terminate();
      cameraWorker = null;
    }
  } else {
    setState(data.type === 'rejected' ? 'Try another view' : 'Frame unavailable');
    showFeedback(data.message);
  }
  capture.disabled = cameraFrames >= cameraLimit;
}

function captureFrame() {
  if (!cameraStream || !video.videoWidth || !video.videoHeight) return;
  const { width, height } = depthInputSize(video.videoWidth, video.videoHeight);
  const image = document.createElement('canvas');
  image.width = width;
  image.height = height;
  const context = image.getContext('2d', { willReadFrequently: true });
  context.drawImage(video, 0, 0, width, height);
  const { data: rgba } = context.getImageData(0, 0, width, height);
  if (!cameraWorker) {
    cameraWorker = new Worker(new URL('./camera-worker.js', import.meta.url), { type: 'module' });
    cameraWorker.onmessage = handleCameraMessage;
    cameraWorker.onerror = () => {
      setState('Frame unavailable');
      showFeedback('The depth worker stopped. Stop the camera and start a new capture to try again.');
      cameraWorker?.terminate();
      cameraWorker = null;
    };
  }
  capture.disabled = true;
  showFeedback('');
  setState('Processing frame…');
  cameraWorker.postMessage({ width, height, rgba }, [rgba.buffer]);
}

try {
  viewer = new MapViewer(canvas);
  startReplayWorker();
  modes.forEach((button) => button.addEventListener('click', () => changeMode(button.dataset.mode)));
  play.addEventListener('click', () => setPlaying(!timer));
  frame.addEventListener('input', () => {
    replayIndex = frame.valueAsNumber;
    setPlaying(false);
  });
  loop.addEventListener('click', () => {
    loop.setAttribute('aria-pressed', String(loop.getAttribute('aria-pressed') !== 'true'));
    updateReplay();
  });
  byId('reset-view').addEventListener('click', () => viewer.reset());
  startCameraButton.addEventListener('click', startCamera);
  stopCameraButton.addEventListener('click', stopCamera);
  capture.addEventListener('click', captureFrame);
  window.addEventListener('pagehide', () => {
    setPlaying(false);
    if (mode === 'camera') stopCamera();
  });
} catch (error) {
  console.error(error);
  byId('map-fallback').hidden = false;
  modes.forEach((button) => { button.disabled = true; });
  byId('reset-view').disabled = true;
  setState('Map unavailable');
}
