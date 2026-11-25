export let modelStatus = 'Downloading depth model…';
export let cameraWorker;

export function getCameraWorker() {
  if (cameraWorker) return cameraWorker;
  try {
    const worker = new Worker(new URL('./camera-worker.js?v=7', import.meta.url), { type: 'module' });
    worker.addEventListener('message', ({ data }) => {
      if (data.type === 'model-status') modelStatus = data.message;
    });
    worker.addEventListener('error', () => {
      modelStatus = 'Depth worker unavailable';
      worker.terminate();
      if (cameraWorker === worker) cameraWorker = null;
    });
    cameraWorker = worker;
    worker.postMessage({ type: 'preload' });
    return worker;
  } catch {
    modelStatus = 'Depth worker unavailable';
    return null;
  }
}

getCameraWorker();
