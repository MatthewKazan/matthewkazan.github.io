import { depthInputSize } from './depth.js?v=8';
import { getCameraWorker, modelStatus, resetCameraWorker } from './preload-depth.js?v=8';

const workerTimeout = 60_000;

export function createCameraRecorder({ viewer, setState, describePoints }) {
  const video = document.getElementById('camera-preview');
  const prompt = document.getElementById('camera-prompt');
  const depthStatus = document.getElementById('depth-status');
  const start = document.getElementById('record-camera');
  const stop = document.getElementById('stop-camera');
  const rate = document.getElementById('recording-rate');
  const counter = document.getElementById('camera-frame-count');
  const feedback = document.getElementById('camera-feedback');
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

  function renderControls() {
    if (!open) return;
    const phase = recording?.phase;
    const captured = recording?.captured ?? 0;
    const queued = captured - (recording?.processed ?? 0);
    const mapped = recording?.samples.length ?? 0;
    const count = captured + ' captured · ' + queued + ' queued · ' + mapped + ' mapped';
    if (counter.textContent !== count) counter.textContent = count;
    if (depthStatus.textContent !== modelStatus) depthStatus.textContent = modelStatus;
    start.hidden = phase === 'opening' || phase === 'recording' || phase === 'draining';
    start.textContent = phase === 'failed' ? 'Retry mapping' : captured ? 'Record again' : 'Start recording';
    stop.hidden = phase !== 'opening' && phase !== 'recording' && phase !== 'failed';
    stop.textContent = phase === 'failed' ? 'Discard recording' : 'Stop recording';
    prompt.hidden = !!phase;

    let status = 'Camera off';
    let message = cameraMessage;
    const stage = recording?.stage && recording.stage + ' · frame ' + (recording.processed + 1);
    const preparation = recording?.restarted ? 'Restarting depth processing · ' + modelStatus : modelStatus;
    if (phase === 'opening') status = 'Waiting for camera';
    if (phase === 'recording') {
      status = 'Recording';
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

  function resetMap() {
    if (!open) return;
    viewer.clear(true);
    for (const sample of recording?.samples ?? []) viewer.addFrame(sample);
    describePoints(viewer.setVisible(recording?.samples.length ?? 0, recording?.poses ?? []));
  }

  function cancelTimer() {
    clearTimeout(timer);
    timer = null;
  }

  function releaseCamera() {
    cancelTimer();
    stream?.getTracks().forEach((track) => track.stop());
    stream = null;
    video.srcObject = null;
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
    beginMapping(scan, { forceWasm: true, refresh });
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
      worker.postMessage({ type: 'frame', id, width, height, rgba }, [rgba.buffer]);
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

  function scheduleCapture() {
    const scan = recording;
    if (scan?.phase !== 'recording' || document.hidden || timer !== null) return;
    const interval = 1000 / (Number(rate.value) || 4);
    const delay = Math.max(25, interval - (performance.now() - scan.lastCapture));
    timer = setTimeout(() => {
      timer = null;
      if (recording !== scan || scan.phase !== 'recording' || document.hidden) return;
      if (video.readyState >= HTMLMediaElement.HAVE_CURRENT_DATA && video.videoWidth &&
          video.currentTime !== scan.videoTime) {
        try {
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
          scan.captured++;
          scan.videoTime = video.currentTime;
          scan.lastCapture = performance.now();
          processNext(scan);
          renderControls();
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
    } else if (data.type === 'frame' || data.type === 'skipped') {
      scan.waiting = false;
      scan.processed++;
      if (data.type === 'frame') {
        scan.samples.push({ points: data.points, colors: data.colors });
        scan.poses = data.poses;
        if (data.loops > scan.loops) {
          scan.loopSummary = 'Scan Context ' + data.match.similarity.toFixed(3) + ' · ICP ' +
            Math.round(data.match.fitness * 100) + '% · GTSAM corrected the map.';
        }
        scan.loops = data.loops;
        scan.feedback = scan.loopSummary;
        if (open) {
          viewer.addFrame(scan.samples.at(-1));
          describePoints(viewer.setVisible(scan.samples.length, scan.poses));
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
      beginMapping(recording, { forceWasm: true, refresh: true });
      return;
    }
    if (recording && recording.phase !== 'complete') return;
    const scan = {
      previous: recording, phase: 'opening', images: [], captured: 0, processed: 0, samples: [],
      lastCapture: -Infinity, videoTime: -Infinity,
    };
    recording = scan;
    cameraMessage = '';
    renderControls();
    stop.focus();
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
