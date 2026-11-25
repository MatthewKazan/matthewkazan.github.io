import { downloadDepthModel, estimateCloud, loadDepthModel } from './depth.js?v=6';
import { initGtsam } from './poses.js?v=6';
import { viewingPoints, viewingPose } from './sensor-frame.js?v=6';
import { SlamPipeline } from './slam-pipeline.js?v=6';

let download;
let preparation;
let activeScan;
let processing = Promise.resolve();

function modelStatus(message) {
  self.postMessage({ type: 'model-status', message });
}

function preload() {
  download ??= downloadDepthModel(modelStatus).catch((error) => {
    download = null;
    modelStatus('Depth model download failed. Start recording to retry.');
    throw error;
  });
  return download;
}

function prepare() {
  preparation ??= Promise.all([preload(), initGtsam()])
    .then(async ([bytes, gtsam]) => {
      const depth = await loadDepthModel(bytes, modelStatus);
      download = null;
      return { depth, gtsam };
    })
    .catch((error) => {
      modelStatus('Depth model unavailable. Start recording to retry.');
      preparation = null;
      throw error;
    });
  return preparation;
}

function send(scan, type, payload = {}, transfer = []) {
  if (scan === activeScan) self.postMessage({ type, id: scan.id, ...payload }, transfer);
}

async function processFrame(scan, { width, height, rgba }) {
  const { depth } = await prepare();
  if (activeScan !== scan) return;
  const size = width + 'x' + height;
  const { points, colors, intrinsics } = await estimateCloud(depth, width, height, rgba, scan.calibration.get(size));
  if (activeScan !== scan) return;
  if (points.length < 360) {
    send(scan, 'skipped', { message: 'Try a well-lit view with surfaces closer to the camera.' });
    return;
  }

  const result = scan.slam.addScan(points);
  if (!result.accepted) {
    const message = result.reason === 'lost'
      ? 'Tracking lost. Turn back toward the last mapped view and move slowly.'
      : 'Hold a little overlap as you move to a new view.';
    send(scan, 'skipped', { message, tracking: result.reason });
    return;
  }
  scan.calibration.set(size, intrinsics);
  const positions = viewingPoints(points);
  const poses = scan.slam.poses(true).map(viewingPose);
  send(scan, 'frame', {
    points: positions, colors, poses, loops: scan.slam.loops.length,
    match: result.loops ? scan.slam.latestLoop() : undefined,
  }, [positions.buffer, colors.buffer]);
}

self.onmessage = ({ data }) => {
  if (data.type === 'preload') {
    if (!preparation) preload().catch(() => {});
  } else if (data.type === 'prepare') {
    prepare().catch(() => {});
  } else if (data.type === 'start') {
    const scan = { id: data.id, calibration: new Map() };
    activeScan = scan;
    prepare().then(({ gtsam }) => {
      if (activeScan !== scan) return;
      scan.slam = new SlamPipeline(gtsam, { pointThreshold: 10_000, cameraTracking: true });
      send(scan, 'ready');
    }).catch((error) => {
      send(scan, 'error', { message: error.message });
    });
  } else if (data.id === activeScan?.id) {
    if (data.type === 'stop') {
      activeScan = null;
    } else if (data.type === 'frame') {
      const scan = activeScan;
      processing = processing.then(() => processFrame(scan, data)).catch((error) => {
        console.error('Camera frame could not be processed', error);
        send(scan, 'error', { message: error.message || 'Could not process the camera frame.' });
      });
    }
  }
};
