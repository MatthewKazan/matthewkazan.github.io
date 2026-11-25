import { initGtsam } from './poses.js?v=2';
import { makeReplay } from './replay.js?v=2';
import { fuseFrames, solveReplay } from './replay-fit.js?v=2';

const wasm = initGtsam();
const { frames, initial, ideal } = makeReplay();
self.postMessage({ type: 'ready', frames, initial });

try {
  const gtsam = await wasm;
  const corrected = solveReplay(gtsam, { initial, ideal });
  const maps = [fuseFrames(frames, initial), fuseFrames(frames, corrected)];
  const buffers = maps.flatMap(({ points, colors }) => [points.buffer, colors.buffer]);
  self.postMessage({ type: 'solved', corrected, maps }, buffers);
} catch (error) {
  self.postMessage({ type: 'error', message: String(error) });
}
