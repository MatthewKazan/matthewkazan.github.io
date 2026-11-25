import { runIcp, verifyLoop } from './icp.js?v=6';
import { mergeClouds, rejectStatisticalOutliers, transformCloud, voxelDownsample } from './point-cloud.js?v=6';
import { identity, optimizePoses } from './poses.js?v=6';
import { angleOf } from './registration.js?v=6';
import { cosineSimilarity, scanContext } from './scan-context.js?v=6';

const odometrySigmas = [0.3, 0.3, 0.3, 0.15, 0.15, 0.15];
const loopNoise = {
  sigmas: [0.1, 0.1, 0.1, Math.sqrt(0.1), Math.sqrt(0.1), Math.sqrt(0.1)],
  robust: { type: 'huber', k: 1.345 },
};
const priorNoise = { sigmas: [0, 0, 0, 0, 0, 0] };

export class SlamPipeline {
  constructor(gtsam, { pointThreshold = 75_000, similarityThreshold = 0.90, cameraTracking = false } = {}) {
    this.gtsam = gtsam;
    this.voxelSize = 0.02;
    this.pointThreshold = pointThreshold;
    this.similarityThreshold = similarityThreshold;
    this.cameraTracking = cameraTracking;
    this.frames = [];
    this.keyframes = [];
    this.odometry = [];
    this.loops = [];
    this.pending = [];
    this.pendingPoints = 0;
    this.window = [];
    this.lastTransform = identity;
    this.motion = identity;
  }

  trackCamera(source, world) {
    const gtsam = this.gtsam;
    const predicted = gtsam.pose3Compose(this.lastTransform, this.motion);
    // Estimated RGB depth has wider frame-to-frame error than sensor depth.
    const coarse = runIcp(gtsam, voxelDownsample(source, 0.08), voxelDownsample(world, 0.08),
      predicted, 0.32, 35, 0.16);
    const coarseMotion = gtsam.pose3Between(this.lastTransform, coarse.transform);
    const plausibleSeed = coarse.fitness > 0.45 && angleOf(coarseMotion) < 0.55
      && Math.hypot(...coarseMotion.translation) < 0.65;
    return runIcp(gtsam, source, voxelDownsample(world, this.voxelSize),
      plausibleSeed ? coarse.transform : predicted, 0.12, 75);
  }

  addScan(points) {
    if (!points.length) throw new Error('The scan has no depth samples.');
    const size = this.voxelSize;
    let metrics = { transform: identity, fitness: 1, rmse: 0 };
    if (!this.window.length) this.window.push(points);
    else {
      const source = voxelDownsample(points, size);
      const world = mergeClouds(this.window);
      metrics = this.cameraTracking
        ? this.trackCamera(source, world)
        : runIcp(this.gtsam, source, voxelDownsample(world, size), this.lastTransform, size * 2.5, 500);
      if (this.cameraTracking) {
        const relative = this.gtsam.pose3Between(this.lastTransform, metrics.transform);
        const distance = Math.hypot(...relative.translation);
        const turn = angleOf(relative);
        if (metrics.fitness < 0.50 || metrics.rmse > 0.085 || distance > 0.65 || turn > 0.57) {
          this.motion = identity;
          return { accepted: false, reason: 'lost', fitness: metrics.fitness };
        }
        if (distance < 0.020 && turn < 0.018) {
          this.motion = identity;
          return { accepted: false, reason: 'still', fitness: metrics.fitness };
        }
        this.motion = relative;
      }
      this.window.push(transformCloud(source, metrics.transform));
      if (this.window.length > 10) this.window.shift();
    }
    this.lastTransform = metrics.transform;
    this.frames.push({ registered: metrics.transform });
    this.pending.push(transformCloud(points, metrics.transform));
    this.pendingPoints += points.length / 3;
    const loopsBefore = this.loops.length;
    if (this.pendingPoints > this.pointThreshold) {
      const world = mergeClouds(this.pending);
      if (voxelDownsample(world, size).length / 3 > this.pointThreshold) {
        this.addKeyframe(world);
      }
    }
    return { accepted: true, fitness: metrics.fitness, rmse: metrics.rmse, loops: this.loops.length - loopsBefore };
  }

  addKeyframe(world) {
    const gtsam = this.gtsam;
    const start = this.frames.length - this.pending.length;
    const pose = this.frames[start].registered;
    const filtered = rejectStatisticalOutliers(world);
    const index = this.keyframes.length;
    const keyframe = {
      pose,
      start,
      before: pose,
      points: transformCloud(filtered, gtsam.pose3Inverse(pose)),
      descriptor: scanContext(filtered),
    };
    if (index) {
      const previous = this.keyframes[index - 1];
      const relative = gtsam.pose3Between(previous.pose, pose);
      this.odometry.push(relative);
      keyframe.before = gtsam.pose3Compose(previous.before, relative);
    }
    this.keyframes.push(keyframe);
    for (let i = start; i < this.frames.length; i++) {
      this.frames[i].keyframe = keyframe;
      this.frames[i].offset = gtsam.pose3Between(pose, this.frames[i].registered);
    }
    this.pending = [];
    this.pendingPoints = 0;
    if (index > 0 && index % 2 === 0) {
      this.detectLoops(index);
      this.optimize();
    }
  }

  detectLoops(index) {
    const current = this.keyframes[index];
    const gtsam = this.gtsam;
    const candidates = [];
    for (let from = 0; index - from >= 5; from++) {
      const previous = this.keyframes[from];
      const similarity = cosineSimilarity(current.descriptor, previous.descriptor);
      if (similarity > this.similarityThreshold) candidates.push({ from, previous, similarity });
    }
    candidates.sort((a, b) => b.similarity - a.similarity);
    for (const { from, previous, similarity } of candidates.slice(0, 4)) {
      // This multiplication order matches the seed in pose_graph.py.
      const seed = gtsam.pose3Compose(previous.pose, gtsam.pose3Inverse(current.pose));
      const match = verifyLoop(gtsam, previous.points, current.points, seed, this.voxelSize);
      if (match.fitness <= 0.4) continue;
      this.loops.push({
        from, to: index,
        transform: gtsam.pose3Inverse(match.transform), noise: loopNoise,
        fromFrame: previous.start, toFrame: current.start, detectedAt: this.frames.length,
        similarity, fitness: match.fitness, rmse: match.rmse,
      });
    }
  }

  optimize() {
    const poses = optimizePoses(this.gtsam, this.keyframes.map(({ pose }) => pose), this.odometry, this.loops,
      { sigmas: odometrySigmas, prior: this.keyframes[0].before, priorNoise, maxIterations: 100 });
    this.keyframes.forEach((keyframe, index) => { keyframe.pose = poses[index]; });
    // The native rebuild uses the nine newest completed submaps for subsequent ICP.
    this.window = this.keyframes.slice(-9).map(({ points, pose }) => transformCloud(points, pose));
    if (this.cameraTracking) this.lastTransform = this.poseAt(this.frames.length - 1, true);
  }

  poseAt(index, corrected) {
    const entry = this.frames[index];
    const gtsam = this.gtsam;
    if (entry.keyframe) {
      const anchor = corrected ? entry.keyframe.pose : entry.keyframe.before;
      return gtsam.pose3Compose(anchor, entry.offset);
    }
    const previous = this.keyframes.at(-1);
    if (corrected || !previous) return entry.registered;
    const offset = gtsam.pose3Between(previous.pose, entry.registered);
    return gtsam.pose3Compose(previous.before, offset);
  }

  poses(corrected) {
    return this.frames.map((_, index) => this.poseAt(index, corrected));
  }

  latestLoop() {
    const last = this.loops.at(-1);
    // Report the strongest geometric match from the latest solve.
    return this.loops.reduce((best, loop) =>
      loop.detectedAt === last.detectedAt && loop.fitness > best.fitness ? loop : best, last);
  }
}
