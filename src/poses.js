import { initGtsam, withCleanup } from '../vendor/gtsam/index.js';

export const identity = {
  rotation: { w: 1, x: 0, y: 0, z: 0 },
  translation: [0, 0, 0],
};

const odometryNoise = [0.035, 0.035, 0.035, 0.055, 0.055, 0.055];
const loopNoise = {
  sigmas: [0.012, 0.012, 0.012, 0.025, 0.025, 0.025],
  robust: { type: 'huber', k: 1.345 },
};

export { initGtsam };

export function optimizePoses(gtsam, initial, odometry, loops, {
  sigmas = odometryNoise,
  prior = initial[0],
  priorNoise = 0.0001,
  maxIterations = 65,
} = {}) {
  return withCleanup((track) => {
    const graph = track(new gtsam.NonlinearFactorGraph());
    const values = track(new gtsam.Values());
    graph.addPriorPose3(0, prior, priorNoise);
    initial.forEach((pose, index) => {
      values.insertPose3(index, pose);
      if (index > 0) graph.addBetweenPose3(index - 1, index, odometry[index - 1], sigmas);
    });
    for (const { from, to, transform, noise = loopNoise } of loops) {
      graph.addBetweenPose3(from, to, transform, noise);
    }
    const result = gtsam.optimize(graph, values, { optimizer: 'lm', maxIterations });
    track(result.values);
    return initial.map((_, index) => result.values.atPose3(index));
  });
}
