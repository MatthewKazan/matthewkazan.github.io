import { downloadDepthModel, estimateCloud, loadDepthModel, removeCachedDepthModel } from './depth.js?v=9';
import { initGtsam } from './poses.js?v=6';
import { viewingPoints, viewingPose } from './sensor-frame.js?v=6';
import { SlamPipeline } from './slam-pipeline.js?v=9';

let download;
let preparation;
let activeScan;
let processing = Promise.resolve();
let forceWasm = false;
let refresh = false;
let singleThread = false;

function modelStatus(message) {
  self.postMessage({ type: 'model-status', message });
}

function preload() {
  download ??= downloadDepthModel(modelStatus, refresh).catch((error) => {
    download = null;
    modelStatus('Depth model download failed. Start recording to retry.');
    throw error;
  });
  return download;
}

function prepare() {
  preparation ??= Promise.all([preload(), initGtsam()])
    .then(async ([weights, gtsam]) => {
      download = null;
      let depth;
      try {
        depth = await loadDepthModel(weights, modelStatus, forceWasm, singleThread);
      } catch (error) {
        await removeCachedDepthModel();
        throw error;
      }
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

async function processFrame(scan, { width, height, rgba, orientation }) {
  const { depth } = await prepare();
  if (activeScan !== scan) return;
  const size = width + 'x' + height;
  send(scan, 'frame-stage', { message: 'Estimating depth' });
  const { points, colors, intrinsics } = await estimateCloud(depth, width, height, rgba, scan.calibration.get(size));
  if (activeScan !== scan) return;
  if (points.length < 360) {
    send(scan, 'skipped', { message: 'Try a well-lit view with surfaces closer to the camera.' });
    return;
  }

  send(scan, 'frame-stage', { message: 'Aligning scan' });
  const result = scan.slam.addScan(points, { orientation });
  if (!result.accepted) {
    const message = result.reason === 'lost'
      ? 'ICP skipped a frame with too little overlap or an implausible pose.'
      : 'ICP skipped a frame with little movement.';
    send(scan, 'skipped', { message });
    return;
  }
  scan.calibration.set(size, intrinsics);
  const positions = viewingPoints(points);
  const poses = scan.slam.poses(true).map(viewingPose);
  // Odometry-only poses, for comparing the map before and after loop closure.
  const initial = scan.slam.loops.length ? scan.slam.poses(false).map(viewingPose) : poses;
  send(scan, 'frame', {
    points: positions, colors, poses, initial, loops: scan.slam.loops.length,
    match: result.loops ? scan.slam.latestLoop() : undefined,
  }, [positions.buffer, colors.buffer]);
}

self.onmessage = ({ data }) => {
  if (data.type === 'preload') {
    forceWasm = data.forceWasm;
    refresh = data.refresh;
    singleThread = data.singleThread;
    if (!preparation) preload().catch(() => {});
  } else if (data.type === 'start') {
    const scan = { id: data.id, calibration: new Map() };
    activeScan = scan;
    prepare().then(({ depth, gtsam }) => {
      if (activeScan !== scan) return;
      scan.slam = new SlamPipeline(gtsam, { pointThreshold: 10_000, cameraTracking: true });
      send(scan, 'ready', { provider: depth.provider });
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
