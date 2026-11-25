import { depthInputSize } from './depth.js?v=6';
import { getCameraWorker, modelStatus } from './preload-depth.js?v=7';

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
    if (phase === 'opening') status = 'Waiting for camera';
    if (phase === 'recording') {
      status = 'Recording';
      message = recording.ready ? recording.feedback : modelStatus;
    }
    if (phase === 'draining') {
      status = 'Building map';
      message ||= 'Keep this page open while the map builds.';
      if (!recording.ready) message = modelStatus + ' · ' + message;
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

  function releaseWorker(scan) {
    if (scan?.id) worker?.postMessage({ type: 'stop', id: scan.id });
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
    if (data.type === 'model-status') {
      renderControls();
      return;
    }
    const scan = recording;
    if (!scan || data.id !== scan.id || (scan.phase !== 'recording' && scan.phase !== 'draining')) return;
    if (data.type === 'error') {
      failMapping(scan, data.message);
      return;
    }
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

  function connectWorker() {
    const next = getCameraWorker();
    if (!next || next === worker) return next;
    worker = next;
    worker.addEventListener('message', handleWorkerMessage);
    worker.addEventListener('error', () => {
      const scan = recording;
      if (worker === next && (scan?.phase === 'recording' || scan?.phase === 'draining')) {
        failMapping(scan, 'The depth worker stopped. Retry mapping to use the saved frames.');
      } else {
        renderControls();
      }
    });
    return worker;
  }

  function beginMapping(scan) {
    scan.id = ++nextId;
    scan.ready = scan.waiting = false;
    scan.processed = scan.loops = 0;
    scan.loopSummary = '';
    scan.feedback = 'Move slowly and keep some of the previous view in sight.';
    scan.samples = [];
    scan.poses = [];
    resetMap();
    if (!connectWorker()) {
      failMapping(scan, 'This browser could not start the depth worker.');
      return;
    }
    worker.postMessage({ type: 'start', id: scan.id });
    cameraMessage = '';
    renderControls();
  }

  async function startRecording() {
    if (recording?.phase === 'failed') {
      recording.phase = 'draining';
      beginMapping(recording);
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
    if (document.hidden) cancelTimer();
    else scheduleCapture();
  });

  return {
    open() {
      open = true;
      resetMap();
      renderControls();
      connectWorker()?.postMessage({ type: 'prepare' });
    },
    close() {
      open = false;
      stopCapture();
    },
    stop: discardRecording,
  };
}
