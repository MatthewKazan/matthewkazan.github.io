import { downloadDepthModel, estimateCloud, loadDepthModel } from './depth.js?v=2';
import { identity, initGtsam, optimizePoses } from './poses.js?v=2';
import { angleOf, register } from './registration.js?v=2';

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

function proposeClosure(gtsam, scan, points, pose) {
  const { poses, clouds, loops } = scan;
  const last = poses.length - 1;
  if (loops.length && last - loops.at(-1).to < 6) return null;
  let candidates = 0;
  for (let index = 0; index < last - 5; index++) {
    const seed = gtsam.pose3Between(poses[index], pose);
    if (Math.hypot(...seed.translation) > 0.55 || angleOf(seed) > 0.35) continue;
    // Avoid closing a loop until the camera has moved away from the earlier view.
    if (!poses.slice(index + 1, last).some((other) =>
      Math.hypot(...gtsam.pose3Between(poses[index], other).translation) > 0.55)) continue;
    const match = register(gtsam, points, clouds[index], seed);
    if (match && match.overlap >= 0.73 && match.rmse <= 0.095) {
      return { from: index, to: last, transform: match.transform };
    }
    if (++candidates === 3) break;
  }
  return null;
}

async function processFrame(scan, { width, height, rgba }) {
  const { depth, gtsam } = await prepare();
  if (activeScan !== scan) return;
  const { points, colors } = await estimateCloud(depth, width, height, rgba);
  if (activeScan !== scan) return;
  if (points.length < 360) {
    send(scan, 'skipped', { message: 'Try a well-lit view with surfaces closer to the camera.' });
    return;
  }

  let pose = identity;
  if (scan.clouds.length) {
    const match = register(gtsam, points, scan.clouds.at(-1));
    const angle = match ? angleOf(match.transform) : Infinity;
    const distance = match ? Math.hypot(...match.transform.translation) : Infinity;
    if (!match || match.overlap < 0.48 || match.rmse > 0.14 || angle > 0.42 || distance > 0.7) {
      send(scan, 'skipped', { message: 'Move back toward the last mapped view, then move slowly.' });
      return;
    }
    if (angle < 0.045 && distance < 0.045) {
      send(scan, 'skipped', { message: 'Keep moving slowly to add a new viewpoint.' });
      return;
    }
    scan.odometry.push(match.transform);
    pose = gtsam.pose3Compose(scan.poses.at(-1), match.transform);
  }

  scan.clouds.push(points);
  scan.poses.push(pose);
  const closure = proposeClosure(gtsam, scan, points, pose);
  if (closure) {
    try {
      const candidate = [...scan.loops, closure];
      scan.poses = optimizePoses(gtsam, scan.poses, scan.odometry, candidate);
      scan.loops.push(closure);
    } catch (error) {
      console.warn('Camera loop-closure solve failed', error);
    }
  }
  const positions = points.slice();
  send(scan, 'frame', { points: positions, colors, poses: scan.poses, loops: scan.loops.length },
    [positions.buffer, colors.buffer]);
}

self.onmessage = ({ data }) => {
  if (data.type === 'preload') {
    if (!preparation) preload().catch(() => {});
  } else if (data.type === 'prepare') {
    prepare().catch(() => {});
  } else if (data.type === 'start') {
    const scan = { id: data.id, clouds: [], poses: [], odometry: [], loops: [] };
    activeScan = scan;
    prepare().then(() => send(scan, 'ready')).catch((error) => {
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
