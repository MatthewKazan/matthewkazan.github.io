import { initGtsam, optimizePoses } from './poses.js';
import { makeReplay } from './replay.js';

const wasm = initGtsam();
const { frames, initial, ideal } = makeReplay();
const buffers = frames.flatMap(({ points, colors }) => [points.buffer, colors.buffer]);
self.postMessage({ type: 'ready', frames, initial }, buffers);

try {
  const gtsam = await wasm;
  const odometry = initial.slice(1).map((pose, index) => gtsam.pose3Between(initial[index], pose));
  const last = initial.length - 1;
  const transform = gtsam.pose3Between(ideal[0], ideal[last]);
  const corrected = optimizePoses(gtsam, initial, odometry, [{ from: 0, to: last, transform }]);
  self.postMessage({ type: 'solved', corrected, version: gtsam.version() });
} catch (error) {
  self.postMessage({ type: 'error', message: String(error) });
}
