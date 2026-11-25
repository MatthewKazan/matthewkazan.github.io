import { estimateCloud, loadDepthModel } from './depth.js';
import { identity, initGtsam, optimizePoses } from './poses.js';
import { angleOf, register } from './registration.js';

let depthModel;
let gtsam;
const clouds = [];
let poses = [];
const odometry = [];
const loops = [];

function status(message) {
  self.postMessage({ type: 'progress', message });
}

function proposeClosure(points, pose) {
  const last = poses.length - 1;
  for (let index = 0; index < last - 5; index++) {
    const seed = gtsam.pose3Between(poses[index], pose);
    if (Math.hypot(...seed.translation) > 0.55 || angleOf(seed) > 0.35) continue;
    // Look for an actual return, with a distinct viewpoint along the intervening path.
    if (!poses.slice(index + 1, last).some((other) =>
      Math.hypot(...gtsam.pose3Between(poses[index], other).translation) > 0.55)) continue;
    const match = register(gtsam, points, clouds[index], seed);
    if (!match || match.overlap < 0.73 || match.rmse > 0.095) continue;
    return { from: index, to: last, transform: match.transform };
  }
  return null;
}

self.onmessage = async ({ data }) => {
  try {
    if (!depthModel) {
      status('Downloading depth model…');
      [depthModel, gtsam] = await Promise.all([loadDepthModel(status), initGtsam()]);
    }
    status('Estimating depth…');
    const { points, colors } = await estimateCloud(depthModel, data.width, data.height, data.rgba);
    if (points.length < 360) {
      self.postMessage({ type: 'rejected', message: 'Try a well-lit view with surfaces closer to the camera.' });
      return;
    }
    let pose = identity;
    if (clouds.length) {
      status('Aligning this frame to the previous view…');
      const alignment = register(gtsam, points, clouds.at(-1));
      if (!alignment || alignment.overlap < 0.48 || alignment.rmse > 0.14 ||
          angleOf(alignment.transform) > 0.42 || Math.hypot(...alignment.transform.translation) > 0.7) {
        self.postMessage({ type: 'rejected', message: 'Frame did not align. Move closer to the last view and try again.' });
        return;
      }
      odometry.push(alignment.transform);
      pose = gtsam.pose3Compose(poses.at(-1), alignment.transform);
    }
    clouds.push(points);
    poses.push(pose);
    const closure = proposeClosure(points, pose);
    if (closure) {
      try {
        status('Optimizing the pose graph…');
        const candidate = [...loops, closure];
        poses = optimizePoses(gtsam, poses, odometry, candidate);
        loops.push(closure);
      } catch (error) {
        // An optional loop solve must not discard an otherwise valid odometry frame.
        console.warn('Camera loop-closure solve failed', error);
      }
    }
    const positions = points.slice();
    self.postMessage({ type: 'frame', points: positions, colors, poses, loops: loops.length }, [positions.buffer, colors.buffer]);
  } catch (error) {
    console.error('Camera frame could not be processed', error);
    self.postMessage({ type: 'error', message: error.message || 'Could not process this camera frame.' });
  }
};
