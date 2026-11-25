import { depthInputSize } from './depth.js?v=6';
import { getCameraWorker, modelStatus } from './preload-depth.js?v=6';

export function createCameraRecorder({ viewer, setState, describePoints }) {
  const video = document.getElementById('camera-preview');
  const prompt = document.getElementById('camera-prompt');
  const depthStatus = document.getElementById('depth-status');
  const start = document.getElementById('record-camera');
  const stop = document.getElementById('stop-camera');
  const rate = document.getElementById('recording-rate');
  const counter = document.getElementById('camera-frame-count');
  const feedback = document.getElementById('camera-feedback');
  const canvas = document.createElement('canvas');
  const context = canvas.getContext('2d', { willReadFrequently: true });
  let worker;
  let stream;
  let scanId = 0;
  let open = false;
  let ready = false;
  let waiting = false;
  let timer = null;
  let nextVideoFrame = null;
  let lastCapture = -Infinity;
  let measuredRate = 0;
  let frames = 0;
  let loops = 0;
  let loopSummary = '';

  function note(message) {
    if (feedback.textContent !== message) feedback.textContent = message;
    feedback.hidden = !message;
  }

  function showCount() {
    const count = frames + (frames === 1 ? ' frame' : ' frames');
    counter.textContent = ready && measuredRate ? count + ' · ' + measuredRate.toFixed(1) + ' fps' : count;
  }

  function cancelScheduledFrame() {
    clearTimeout(timer);
    timer = null;
    if (nextVideoFrame !== null) video.cancelVideoFrameCallback(nextVideoFrame);
    nextVideoFrame = null;
  }

  function capture() {
    nextVideoFrame = null;
    if (!open || !ready || waiting || document.hidden || !stream) return;
    if (video.readyState < HTMLMediaElement.HAVE_CURRENT_DATA || !video.videoWidth) {
      scheduleFrame();
      return;
    }
    const { width, height } = depthInputSize(video.videoWidth, video.videoHeight);
    if (canvas.width !== width || canvas.height !== height) {
      canvas.width = width;
      canvas.height = height;
    }
    context.drawImage(video, 0, 0, width, height);
    const { data: rgba } = context.getImageData(0, 0, width, height);
    const now = performance.now();
    if (Number.isFinite(lastCapture)) {
      const fps = 1000 / (now - lastCapture);
      measuredRate = measuredRate ? 0.6 * measuredRate + 0.4 * fps : fps;
    }
    lastCapture = now;
    waiting = true;
    worker.postMessage({ type: 'frame', id: scanId, width, height, rgba }, [rgba.buffer]);
  }

  function scheduleFrame() {
    if (!open || !ready || waiting || document.hidden || timer !== null || nextVideoFrame !== null) return;
    // Auto takes the next available frame, capped at 4 fps. All rates wait for
    // the previous inference and alignment, so slower devices never queue frames.
    const interval = 1000 / (Number(rate.value) || 4);
    const delay = Math.max(0, interval - (performance.now() - lastCapture));
    timer = setTimeout(() => {
      timer = null;
      if (video.requestVideoFrameCallback) nextVideoFrame = video.requestVideoFrameCallback(capture);
      else capture();
    }, Math.max(16, delay));
  }

  function endScan(message = '') {
    worker?.postMessage({ type: 'stop', id: scanId });
    scanId++;
    ready = waiting = false;
    cancelScheduledFrame();
    stream?.getTracks().forEach((track) => track.stop());
    stream = null;
    video.srcObject = null;
    video.hidden = true;
    start.hidden = false;
    start.textContent = frames ? 'Record again' : 'Start recording';
    stop.hidden = true;
    prompt.hidden = !open || frames > 0;
    if (open) {
      showCount();
      setState(frames ? 'Recording stopped' : 'Camera off');
      note(message);
    }
  }

  function handleWorkerMessage({ data }) {
    if (data.type === 'model-status') {
      depthStatus.textContent = data.message;
      if (open && stream && !ready) setState(data.message);
      return;
    }
    if (data.id !== scanId || !open || !stream) return;
    if (data.type === 'error') {
      endScan(data.message);
      return;
    }
    if (data.type === 'ready') {
      ready = true;
      setState('Recording');
      note('Move slowly and keep some of the previous view in sight.');
    } else {
      waiting = false;
      if (data.type === 'frame') {
        frames++;
        viewer.addFrame(data);
        describePoints(viewer.setVisible(frames, data.poses));
        if (data.loops > loops) {
          loopSummary = 'Scan Context ' + data.match.similarity.toFixed(3) + ' · ICP ' +
            Math.round(data.match.fitness * 100) + '% · GTSAM corrected the map.';
        }
        note(loopSummary || (frames === 1 ? 'Turn slowly and keep some of the previous view in sight.' : ''));
        loops = data.loops;
      } else if (data.type === 'skipped') {
        note(data.message);
      }
      if (data.tracking === 'lost') setState('Tracking lost');
      else setState(loops ? 'Recording · Loop closed' : 'Recording', loops > 0);
    }
    showCount();
    scheduleFrame();
  }

  function connectWorker() {
    const next = getCameraWorker();
    if (!next || next === worker) return next;
    worker = next;
    worker.addEventListener('message', handleWorkerMessage);
    worker.addEventListener('error', () => {
      if (worker === next && open) endScan('The depth worker stopped. Start another recording to retry.');
    });
    return worker;
  }

  async function startRecording() {
    const id = ++scanId;
    start.hidden = true;
    stop.hidden = false;
    stop.focus();
    setState('Waiting for camera');
    note('');
    try {
      if (!navigator.mediaDevices?.getUserMedia) throw new Error('Camera access requires HTTPS and a supported browser.');
      if (!connectWorker()) throw new Error('This browser could not start the depth worker.');
      worker.postMessage({ type: 'prepare' });
      const media = await navigator.mediaDevices.getUserMedia({
        audio: false,
        video: { facingMode: { ideal: 'environment' }, width: { ideal: 640 }, height: { ideal: 480 } },
      });
      if (id !== scanId) {
        media.getTracks().forEach((track) => track.stop());
        return;
      }
      stream = media;
      video.srcObject = stream;
      video.hidden = false;
      await video.play();
      if (id !== scanId) return;
      frames = loops = measuredRate = 0;
      loopSummary = '';
      lastCapture = -Infinity;
      viewer.clear(true);
      describePoints(0);
      showCount();
      prompt.hidden = true;
      setState(modelStatus);
      stream.getVideoTracks().forEach((track) => track.addEventListener('ended', () => {
        if (id === scanId) endScan('Camera disconnected. Start another recording to retry.');
      }));
      worker.postMessage({ type: 'start', id });
    } catch (error) {
      if (id !== scanId) return;
      if (error.name === 'NotAllowedError') endScan('Camera access was declined. You can still use Replay.');
      else if (error.name === 'NotFoundError') endScan('No camera was found. You can still use Replay.');
      else endScan(error.message);
    }
  }

  start.addEventListener('click', startRecording);
  stop.addEventListener('click', () => {
    endScan('Recording stopped. Drag the map to inspect it.');
    start.focus();
  });
  rate.addEventListener('change', () => {
    measuredRate = 0;
    cancelScheduledFrame();
    scheduleFrame();
  });
  document.addEventListener('visibilitychange', () => {
    if (document.hidden) cancelScheduledFrame();
    else scheduleFrame();
  });
  connectWorker();

  return {
    open() {
      open = true;
      frames = 0;
      viewer.clear(true);
      describePoints(0);
      depthStatus.textContent = modelStatus;
      showCount();
      note('');
      start.textContent = 'Start recording';
      prompt.hidden = false;
      setState('Camera off');
      connectWorker()?.postMessage({ type: 'prepare' });
    },
    close() {
      open = false;
      endScan();
      note('');
    },
    stop: endScan,
  };
}
