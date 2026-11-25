import { depthInputSize } from './depth.js?v=10';
import { getCameraWorker, modelStatus, resetCameraWorker } from './preload-depth.js?v=10';
import { drawLoopOverview } from './loop-overview.js?v=7';

const workerTimeout = 60_000;

const degrees = Math.PI / 180;
const multiply = (a, b) => ({
  w: a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
  x: a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
  y: a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
  z: a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
});
const about = (axis, angle) => {
  const q = { w: Math.cos(angle / 2), x: 0, y: 0, z: 0 };
  q[axis] = Math.sin(angle / 2);
  return q;
};

// Camera-to-world rotation of the rear camera from a deviceorientation event.
// The W3C device frame has x right, y toward the top of the screen and z out of
// the screen, composed as Z(alpha) X(beta) Y(gamma). The scans use the camera
// frame: x right in the image, y down and z forward through the rear lens. The
// video is rotated with the screen, so the screen angle turns the image axes.
export function cameraOrientation({ alpha, beta, gamma }, screenAngle = 0) {
  const device = multiply(multiply(about('z', alpha * degrees), about('x', beta * degrees)), about('y', gamma * degrees));
  return multiply(multiply(device, about('z', -screenAngle * degrees)), about('x', Math.PI));
}

export function createCameraRecorder({ viewer, setState, describePoints }) {
  const video = document.getElementById('camera-preview');
  const prompt = document.getElementById('camera-prompt');
  const depthStatus = document.getElementById('depth-status');
  const start = document.getElementById('record-camera');
  const uploadButton = document.getElementById('upload-camera');
  const upload = document.getElementById('upload-video');
  const stop = document.getElementById('stop-camera');
  const rate = document.getElementById('recording-rate');
  const counter = document.getElementById('camera-frame-count');
  const feedback = document.getElementById('camera-feedback');
  const comparison = document.getElementById('loop-comparison');
  const overview = document.getElementById('loop-overview');
  const before = document.getElementById('loop-before');
  const after = document.getElementById('loop-after');
  const evidence = document.getElementById('loop-evidence');
  const shift = document.getElementById('loop-shift');
  const captureCanvas = document.createElement('canvas');
  const captureContext = captureCanvas.getContext('2d');
  const decodeCanvas = document.createElement('canvas');
  const decodeContext = decodeCanvas.getContext('2d', { willReadFrequently: true });
  let recording;
  let worker;
  let stream;
  let nextId = 0;
  let timer = null;
  let workerDeadline = null;
  let open = false;
  let cameraMessage = '';
  let orientation = null;
  let corrected = true;

  function onOrientation(event) {
    if (event.alpha === null || event.beta === null || event.gamma === null) return;
    const angle = globalThis.screen?.orientation?.angle ?? globalThis.orientation ?? 0;
    orientation = cameraOrientation(event, angle);
  }

  function listenForMotion() {
    orientation = null;
    globalThis.removeEventListener?.('deviceorientation', onOrientation);
    // iOS asks for motion access, and only from the button press.
    const request = globalThis.DeviceOrientationEvent?.requestPermission?.();
    Promise.resolve(request).then((state) => {
      if (!state || state === 'granted') globalThis.addEventListener?.('deviceorientation', onOrientation);
    }, () => {});
  }

  function renderControls() {
    if (!open) return;
    const phase = recording?.phase;
    const captured = recording?.captured ?? 0;
    const queued = captured - (recording?.processed ?? 0);
    const mapped = recording?.samples.length ?? 0;
    // Shows which depth engine is running, e.g. "CPU ×4" when threads are available.
    const engine = recording?.engine ? ' · ' + recording.engine : '';
    const count = captured + ' captured · ' + queued + ' queued · ' + mapped + ' mapped' + engine;
    if (counter.textContent !== count) counter.textContent = count;
    if (depthStatus.textContent !== modelStatus) depthStatus.textContent = modelStatus;
    start.hidden = phase === 'opening' || phase === 'recording' || phase === 'draining';
    start.textContent = phase === 'failed' ? 'Retry mapping' : captured ? 'Record again' : 'Start recording';
    uploadButton.hidden = start.hidden || phase === 'failed';
    stop.hidden = phase !== 'opening' && phase !== 'recording' && phase !== 'failed';
    stop.textContent = phase === 'failed' ? 'Discard recording' : recording?.file ? 'Stop video' : 'Stop recording';
    prompt.hidden = !!phase;

    let status = 'Camera off';
    let message = cameraMessage;
    const stage = recording?.stage && recording.stage + ' · frame ' + (recording.processed + 1);
    const preparation = recording?.restarted ? 'Restarting depth processing · ' + modelStatus : modelStatus;
    if (phase === 'opening') status = recording.file ? 'Opening video' : 'Waiting for camera';
    if (phase === 'recording') {
      status = recording.file ? 'Reading video' : 'Recording';
      message = recording.ready ? stage || recording.feedback : preparation;
    }
    if (phase === 'draining') {
      status = 'Building map';
      message ||= 'Keep this page open while the map builds.';
      if (!recording.ready) message = preparation + ' · ' + message;
      else if (stage) message = stage + ' · ' + message;
    }
    if (phase === 'complete') status = mapped ? 'Recording processed' : 'No frames mapped';
    if (phase === 'failed') status = 'Mapping paused';
    const solved = (phase === 'recording' || phase === 'draining' || phase === 'complete') && !!recording.loops;
    setState(solved ? status + ' · Loop closed' : status, solved);
    if (feedback.textContent !== message) feedback.textContent = message;
    feedback.hidden = !message;
  }

  // Gyro tracking also proposes loops by heading, which can score low on Scan Context.
  function loopEvidence({ similarity, fitness }) {
    const source = similarity > 0.9 ? 'Scan Context ' + similarity.toFixed(3) : 'Heading match';
    return source + ' · ICP ' + Math.round(fitness * 100) + '%';
  }

  // Draws the mapped frames. The same Before closure and After GTSAM switch as the
  // replay compares odometry poses with optimized ones; until a loop is verified
  // the two match.
  function showMap() {
    if (!open) return;
    const scan = recording;
    const count = scan?.samples.length ?? 0;
    const comparing = count > 1 && scan.initial?.length === count;
    const loop = comparing ? scan.loop : null;
    comparison.hidden = overview.hidden = !comparing;
    if (!comparing) {
      describePoints(viewer.setVisible(count, scan?.poses ?? []));
      return;
    }
    const amount = corrected ? 1 : 0;
    describePoints(viewer.setVisible(count, scan.initial, { poses: scan.poses, amount }));
    before.setAttribute('aria-pressed', String(!corrected));
    after.setAttribute('aria-pressed', String(corrected));
    drawLoopOverview(document.getElementById('loop-path'), scan.initial, scan.poses, loop, amount);
    evidence.textContent = loop ? loopEvidence(loop) : 'No loop verified yet';
    evidence.title = loop ? 'Matched keyframes ' + (loop.from + 1) + ' and ' + (loop.to + 1) : '';
    const largest = scan.poses.reduce((most, { translation }, i) => Math.max(most,
      Math.hypot(...translation.map((value, axis) => value - scan.initial[i].translation[axis]))), 0);
    shift.textContent = loop ? 'Largest pose correction ' + Math.round(largest * 1000) + ' mm' : '';
  }

  function resetMap() {
    if (!open) return;
    viewer.clear(true);
    for (const sample of recording?.samples ?? []) viewer.addFrame(sample);
    showMap();
  }

  function cancelTimer() {
    clearTimeout(timer);
    timer = null;
  }

  function releaseCamera() {
    cancelTimer();
    globalThis.removeEventListener?.('deviceorientation', onOrientation);
    stream?.getTracks().forEach((track) => track.stop());
    stream = null;
    video.srcObject = null;
    if (video.src) {
      URL.revokeObjectURL(video.src);
      video.removeAttribute('src');
      video.load?.();
    }
    video.hidden = true;
  }

  function clearWorkerDeadline() {
    clearTimeout(workerDeadline);
    workerDeadline = null;
  }

  function releaseWorker(scan) {
    clearWorkerDeadline();
    if (scan?.id) worker?.postMessage({ type: 'stop', id: scan.id });
  }

  function restartMapping(scan, message) {
    const refresh = !scan.ready;
    resetCameraWorker(worker);
    worker = null;
    if (scan.restarted) {
      failMapping(scan, message + (scan.captured ? ' Retry mapping will fetch the model again.' : ''));
      return;
    }
    scan.restarted = true;
    beginMapping(scan, { forceWasm: true, singleThread: true, refresh });
  }

  function watchWorker(scan) {
    clearWorkerDeadline();
    if (document.hidden) return;
    const id = scan.id;
    workerDeadline = setTimeout(() => {
      if (recording !== scan || scan.id !== id) return;
      const message = scan.ready
        ? (scan.stage || 'Processing') + ' stalled at frame ' + (scan.processed + 1) + '.'
        : 'Depth model startup stalled.';
      restartMapping(scan, message);
    }, workerTimeout);
  }

  function failMapping(scan, message) {
    releaseCamera();
    releaseWorker(scan);
    cameraMessage = message;
    if (scan.captured) {
      scan.phase = 'failed';
      scan.ready = scan.waiting = false;
    } else {
      recording = scan.previous;
    }
    renderControls();
  }

  async function processNext(scan) {
    if (scan.waiting || (scan.phase !== 'recording' && scan.phase !== 'draining')) return;
    if (scan.processed === scan.captured) {
      if (scan.phase === 'draining') {
        releaseWorker(scan);
        scan.phase = 'complete';
        scan.images = [];
        scan.orientations = [];
        cameraMessage = scan.captured ? scan.feedback : 'No camera frames were captured.';
        renderControls();
      }
      return;
    }
    if (!scan.ready) return;
    scan.waiting = true;
    scan.stage = 'Reading saved frame';
    watchWorker(scan);
    const id = scan.id;
    try {
      const image = new Image();
      image.src = scan.images[scan.processed];
      await image.decode();
      if (recording !== scan || scan.id !== id || !scan.ready) return;
      const { naturalWidth: width, naturalHeight: height } = image;
      if (decodeCanvas.width !== width || decodeCanvas.height !== height) {
        decodeCanvas.width = width;
        decodeCanvas.height = height;
      }
      decodeContext.drawImage(image, 0, 0, width, height);
      const { data: rgba } = decodeContext.getImageData(0, 0, width, height);
      worker.postMessage({
        type: 'frame', id, width, height, rgba, orientation: scan.orientations[scan.processed],
      }, [rgba.buffer]);
    } catch (error) {
      if (recording === scan && scan.id === id && scan.ready) {
        failMapping(scan, error.message || 'A queued frame could not be read.');
      }
    }
  }

  function stopCapture(message = '') {
    const scan = recording;
    if (scan?.phase === 'opening') {
      releaseCamera();
      recording = scan.previous;
      cameraMessage = message;
    } else if (scan?.phase === 'recording') {
      scan.phase = 'draining';
      releaseCamera();
      cameraMessage = message;
      processNext(scan);
    }
    renderControls();
  }

  // Downscales the current video frame and queues it as a JPEG with the latest
  // device orientation (null for uploaded videos).
  function captureFrame(scan) {
    const { width, height } = depthInputSize(video.videoWidth, video.videoHeight);
    if (captureCanvas.width !== width || captureCanvas.height !== height) {
      captureCanvas.width = width;
      captureCanvas.height = height;
    }
    captureContext.drawImage(video, 0, 0, width, height);
    const image = captureCanvas.toDataURL('image/jpeg', 0.93);
    if (image === 'data:,') throw new Error('Could not capture a frame.');
    // Keep the sampled images in order, including while inference is busy.
    // JPEG keeps a long backlog small enough for the phone.
    scan.images.push(image);
    scan.orientations.push(scan.file ? null : orientation);
    scan.captured++;
    scan.videoTime = video.currentTime;
    scan.lastCapture = performance.now();
    processNext(scan);
    renderControls();
  }

  // Steps through an uploaded video at the selected rate. It only seeks ahead
  // while the depth queue is short, so long videos do not pile up in memory.
  async function readVideo(scan) {
    const step = 1 / (Number(rate.value) || 4);
    const seek = (time) => new Promise((resolve, reject) => {
      video.addEventListener('seeked', resolve, { once: true });
      video.addEventListener('error', reject, { once: true });
      video.currentTime = time;
    });
    try {
      for (let time = 0; time < video.duration; time += step) {
        while (recording === scan && scan.phase === 'recording' && scan.captured - scan.processed > 2) {
          await new Promise((resolve) => setTimeout(resolve, 100));
        }
        if (recording !== scan || scan.phase !== 'recording') return;
        await seek(time);
        if (recording !== scan || scan.phase !== 'recording') return;
        captureFrame(scan);
      }
      stopCapture();
    } catch {
      if (recording === scan) stopCapture('The video could not be read to the end. Processing the frames read so far.');
    }
  }

  async function startUpload(file) {
    if (!file || (recording && recording.phase !== 'complete')) return;
    const scan = {
      previous: recording, phase: 'opening', file: file.name, images: [], orientations: [], captured: 0,
      processed: 0, samples: [], lastCapture: -Infinity, videoTime: -Infinity,
    };
    recording = scan;
    cameraMessage = '';
    renderControls();
    try {
      video.srcObject = null;
      video.muted = true;
      video.src = URL.createObjectURL(file);
      video.hidden = false;
      await new Promise((resolve, reject) => {
        video.addEventListener('loadeddata', resolve, { once: true });
        video.addEventListener('error', () => reject(new Error('This browser cannot play that video.')), { once: true });
      });
      // Some mobile browsers only draw a video frame after it has played once.
      await video.play().catch(() => {});
      video.pause();
      if (recording !== scan) return;
      scan.phase = 'recording';
      delete scan.previous;
      beginMapping(scan);
      readVideo(scan);
    } catch (error) {
      if (recording === scan) failMapping(scan, error.message);
    }
  }

  function scheduleCapture() {
    const scan = recording;
    if (scan?.phase !== 'recording' || scan.file || document.hidden || timer !== null) return;
    const interval = 1000 / (Number(rate.value) || 4);
    const delay = Math.max(25, interval - (performance.now() - scan.lastCapture));
    timer = setTimeout(() => {
      timer = null;
      if (recording !== scan || scan.phase !== 'recording' || document.hidden) return;
      if (video.readyState >= HTMLMediaElement.HAVE_CURRENT_DATA && video.videoWidth &&
          video.currentTime !== scan.videoTime) {
        try {
          captureFrame(scan);
        } catch {
          stopCapture('Camera capture stopped. Processing the saved frames.');
        }
      }
      scheduleCapture();
    }, delay);
  }

  function handleWorkerMessage(event) {
    if (event.currentTarget !== worker) return;
    const { data } = event;
    const scan = recording;
    const mapping = scan?.phase === 'recording' || scan?.phase === 'draining';
    if (data.type === 'model-status') {
      if (mapping && !scan.ready) watchWorker(scan);
      renderControls();
      return;
    }
    if (!mapping || data.id !== scan.id) return;
    if (data.type === 'error') {
      restartMapping(scan, data.message);
      return;
    }
    if (data.type === 'frame-stage') {
      scan.stage = data.message;
      watchWorker(scan);
      renderControls();
      return;
    }
    clearWorkerDeadline();
    scan.stage = '';
    if (data.type === 'ready') {
      scan.ready = true;
      if (data.provider) scan.engine = data.provider === 'wasm' ? 'CPU ×' + data.threads : 'GPU';
    } else if (data.type === 'frame' || data.type === 'skipped') {
      scan.waiting = false;
      scan.processed++;
      if (data.type === 'frame') {
        scan.samples.push({ points: data.points, colors: data.colors });
        scan.poses = data.poses;
        scan.initial = data.initial;
        if (data.loops > scan.loops) {
          scan.loop = data.match;
          corrected = true;
          scan.loopSummary = loopEvidence(data.match) + ' · GTSAM corrected the map.';
        }
        scan.loops = data.loops;
        scan.feedback = scan.loopSummary;
        if (open) {
          viewer.addFrame(scan.samples.at(-1));
          showMap();
        }
      } else {
        scan.feedback = data.message;
      }
    }
    processNext(scan);
    renderControls();
  }

  function connectWorker(options) {
    const next = getCameraWorker(options);
    if (!next || next === worker) return next;
    worker = next;
    worker.addEventListener('message', handleWorkerMessage);
    const onWorkerError = () => {
      const scan = recording;
      if (worker === next && (scan?.phase === 'recording' || scan?.phase === 'draining')) {
        restartMapping(scan, 'The depth worker stopped.');
      } else {
        renderControls();
      }
    };
    worker.addEventListener('error', onWorkerError);
    worker.addEventListener('messageerror', onWorkerError);
    return worker;
  }

  function beginMapping(scan, options) {
    scan.id = ++nextId;
    scan.ready = scan.waiting = false;
    scan.stage = '';
    scan.processed = scan.loops = 0;
    scan.loopSummary = '';
    scan.feedback = 'Move slowly and keep some of the previous view in sight.';
    scan.samples = [];
    scan.poses = [];
    scan.initial = [];
    scan.loop = null;
    resetMap();
    if (!connectWorker(options)) {
      failMapping(scan, 'This browser could not start the depth worker.');
      return;
    }
    worker.postMessage({ type: 'start', id: scan.id });
    cameraMessage = '';
    watchWorker(scan);
    renderControls();
  }

  async function startRecording() {
    if (recording?.phase === 'failed') {
      resetCameraWorker(worker);
      worker = null;
      recording.phase = 'draining';
      recording.restarted = true;
      beginMapping(recording, { forceWasm: true, singleThread: true, refresh: true });
      return;
    }
    if (recording && recording.phase !== 'complete') return;
    const scan = {
      previous: recording, phase: 'opening', images: [], orientations: [], captured: 0, processed: 0, samples: [],
      lastCapture: -Infinity, videoTime: -Infinity,
    };
    recording = scan;
    cameraMessage = '';
    renderControls();
    stop.focus();
    listenForMotion();
    try {
      if (!navigator.mediaDevices?.getUserMedia) throw new Error('Camera access requires HTTPS and a supported browser.');
      const media = await navigator.mediaDevices.getUserMedia({
        audio: false,
        video: { facingMode: { ideal: 'environment' }, width: { ideal: 640 }, height: { ideal: 480 } },
      });
      if (recording !== scan) {
        media.getTracks().forEach((track) => track.stop());
        return;
      }
      stream = media;
      video.srcObject = stream;
      video.hidden = false;
      await video.play();
      if (recording !== scan) return;
      scan.phase = 'recording';
      delete scan.previous;
      media.getVideoTracks().forEach((track) => track.addEventListener('ended', () => {
        if (recording === scan) stopCapture('Camera disconnected. Processing the saved frames.');
      }));
      beginMapping(scan);
      scheduleCapture();
    } catch (error) {
      if (recording !== scan) return;
      let message = error.message;
      if (error.name === 'NotAllowedError') {
        message = stream ? 'Could not play the camera preview.' : 'Camera access was declined.';
      } else if (error.name === 'NotFoundError') {
        message = 'No camera was found.';
      }
      failMapping(scan, message);
    }
  }

  function discardRecording() {
    releaseCamera();
    releaseWorker(recording);
    recording = null;
    cameraMessage = '';
    resetMap();
    renderControls();
  }

  start.addEventListener('click', startRecording);
  uploadButton.addEventListener('click', () => upload.click());
  upload.addEventListener('change', () => {
    startUpload(upload.files?.[0]);
    upload.value = '';
  });
  for (const [button, value] of [[before, false], [after, true]]) {
    button?.addEventListener('click', () => {
      if (!open || !recording?.samples.length) return;
      corrected = value;
      showMap();
    });
  }
  stop.addEventListener('click', () => {
    if (recording?.phase === 'failed') discardRecording();
    else stopCapture();
  });
  rate.addEventListener('change', () => {
    cancelTimer();
    scheduleCapture();
  });
  document.addEventListener('visibilitychange', () => {
    if (document.hidden) {
      cancelTimer();
      clearWorkerDeadline();
      return;
    }
    scheduleCapture();
    const scan = recording;
    if ((scan?.phase === 'recording' || scan?.phase === 'draining') && (!scan.ready || scan.waiting)) {
      watchWorker(scan);
    }
  });

  return {
    open() {
      open = true;
      resetMap();
      renderControls();
      connectWorker();
    },
    close() {
      open = false;
      stopCapture();
    },
    stop() {
      discardRecording();
      resetCameraWorker();
      worker = null;
    },
  };
}
