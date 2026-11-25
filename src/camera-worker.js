import { cloudFromDepth, downloadDepthModel, estimateCloud, loadDepthModel, removeCachedDepthModel } from './depth.js?v=11';
import { initGtsam } from './poses.js?v=6';
import { viewingPoints, viewingPose } from './sensor-frame.js?v=6';
import { SlamPipeline } from './slam-pipeline.js?v=11';
import { grayscale, VisualOdometry } from './visual-odometry.js?v=2';

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

// Frames kept until the focal length is calibrated, so the map can be rebuilt
// with the corrected value.
const calibrationFrames = 40;

function resetTracking(scan, gtsam) {
  scan.vo = new VisualOdometry();
  scan.slam = new SlamPipeline(gtsam, { pointThreshold: 10_000, cameraTracking: true, refine: 'none' });
}

// Tracks one frame with visual odometry and adds its scaled cloud to the map.
function mapFrame(scan, { rgba, depth, width, height, orientation }, replay = false) {
  const intrinsics = scan.calibration.get(width + 'x' + height);
  const gray = grayscale(rgba, width, height);
  const result = scan.vo.process({ gray, depth, width, height, intrinsics, orientation });
  const { points, colors } = cloudFromDepth(depth, rgba, width, height, intrinsics, result.scale);
  if (points.length < 360) {
    send(scan, 'skipped', { replay, message: 'Try a well-lit view with surfaces closer to the camera.' });
    return;
  }
  const view = { gray, depth, width, height, intrinsics, scale: result.scale };
  const added = scan.slam.addScan(points, { orientation, pose: result.pose, view });
  if (!added.accepted) {
    send(scan, 'skipped', { replay, message: 'Skipped a frame with little movement.' });
    return;
  }
  const positions = viewingPoints(points);
  const poses = scan.slam.poses(true).map(viewingPose);
  // Odometry-only poses, for comparing the map before and after loop closure.
  const initial = scan.slam.poses(false).map(viewingPose);
  send(scan, 'frame', {
    replay, points: positions, colors, poses, initial, loops: scan.slam.loops.length,
    match: added.loops ? scan.slam.latestLoop() : undefined,
    tracking: result.ok ? '' : 'Lost image features; the pose is a guess until the view overlaps again.',
  }, [positions.buffer, colors.buffer]);
}

async function processFrame(scan, frame) {
  const { depth: model, gtsam } = await prepare();
  if (activeScan !== scan) return;
  const { width, height, rgba } = frame;
  const size = width + 'x' + height;
  send(scan, 'frame-stage', { message: 'Estimating depth' });
  const cloud = await estimateCloud(model, width, height, rgba, scan.calibration.get(size));
  if (activeScan !== scan) return;
  // The network's focal length varies from frame to frame; the first one is
  // kept until visual odometry calibrates it.
  if (!scan.calibration.has(size)) scan.calibration.set(size, cloud.intrinsics);
  const entry = { ...frame, depth: cloud.depth };
  send(scan, 'frame-stage', { message: 'Tracking features' });
  mapFrame(scan, entry);
  if (scan.calibrated) return;
  scan.buffer.push(entry);
  if (scan.vo.samples.length < 16 && scan.buffer.length < calibrationFrames) return;
  scan.calibrated = true;
  const factor = scan.vo.calibrateFocal();
  const buffer = scan.buffer;
  scan.buffer = [];
  if (!factor || Math.abs(factor - 1) < 0.03) return;
  // Rebuild the map from the buffered frames with the calibrated focal length.
  const intrinsics = scan.calibration.get(size);
  scan.calibration.set(size, { ...intrinsics, fx: intrinsics.fx * factor, fy: intrinsics.fy * factor });
  resetTracking(scan, gtsam);
  send(scan, 'rebuild', { message: 'Calibrated the focal length (×' + factor.toFixed(2) + ') and rebuilt the map.' });
  for (const buffered of buffer) mapFrame(scan, buffered, true);
}

self.onmessage = ({ data }) => {
  if (data.type === 'preload') {
    forceWasm = data.forceWasm;
    refresh = data.refresh;
    singleThread = data.singleThread;
    if (!preparation) preload().catch(() => {});
  } else if (data.type === 'start') {
    const scan = { id: data.id, calibration: new Map(), buffer: [], calibrated: false };
    activeScan = scan;
    prepare().then(({ depth, gtsam }) => {
      if (activeScan !== scan) return;
      resetTracking(scan, gtsam);
      send(scan, 'ready', { provider: depth.provider, threads: depth.threads });
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
