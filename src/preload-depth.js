export let modelStatus = 'Downloading depth model…';
let cameraWorker;

export function resetCameraWorker(worker = cameraWorker) {
  if (!worker) return;
  worker.terminate();
  if (cameraWorker === worker) {
    cameraWorker = null;
    modelStatus = 'Waiting for depth model…';
  }
}

export function getCameraWorker({ forceWasm = false, refresh = false, singleThread = false } = {}) {
  if (cameraWorker) return cameraWorker;
  try {
    const appleMobile = /iPhone|iPad|iPod/.test(navigator.userAgent) ||
      (navigator.platform === 'MacIntel' && navigator.maxTouchPoints > 1);
    const worker = new Worker(new URL('./camera-worker.js?v=17', import.meta.url), { type: 'module' });
    worker.addEventListener('message', ({ data }) => {
      if (cameraWorker === worker && data.type === 'model-status') modelStatus = data.message;
    });
    worker.addEventListener('error', () => {
      if (cameraWorker === worker) {
        resetCameraWorker(worker);
        modelStatus = 'Depth worker unavailable';
      }
    });
    cameraWorker = worker;
    modelStatus = refresh ? 'Fetching a fresh depth model…' : 'Checking saved depth model…';
    worker.postMessage({ type: 'preload', forceWasm: forceWasm || appleMobile, refresh, singleThread });
    return worker;
  } catch {
    modelStatus = 'Depth worker unavailable';
    return null;
  }
}

getCameraWorker();
