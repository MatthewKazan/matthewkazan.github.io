import { runIcp, runMultiscaleIcp, searchTranslation } from './icp.js?v=8';
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

// Rotation of camera b in camera a's frame, from two camera-to-world quaternions.
function relativeRotation(a, b) {
  const w = a.w, x = -a.x, y = -a.y, z = -a.z;
  return {
    w: w * b.w - x * b.x - y * b.y - z * b.z,
    x: w * b.x + x * b.w + y * b.z - z * b.y,
    y: w * b.y - x * b.z + y * b.w + z * b.x,
    z: w * b.z + x * b.y - y * b.x + z * b.w,
  };
}

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
    this.lastOrientation = null;
    this.lostFrames = 0;
    this.gyroTracking = false;
  }

  // `orientation` is an optional camera-to-world rotation from the device's motion
  // sensors. Monocular depth is not consistent enough between frames for ICP to
  // recover rotation (it trades turns for translation), which LiDAR depth was in
  // the original app. When motion sensors are available, the gyro rotation since
  // the last accepted scan sets the rotation and ICP fits only the translation.
  addScan(points, { orientation } = {}) {
    if (!points.length) throw new Error('The scan has no depth samples.');
    const gtsam = this.gtsam;
    const size = this.voxelSize;
    let metrics = { transform: identity, fitness: 1, rmse: 0 };
    if (!this.window.length) this.window.push(points);
    else {
      const source = voxelDownsample(points, size);
      const world = mergeClouds(this.window);
      let seed = this.lastTransform;
      const rotated = !!(orientation && this.lastOrientation);
      if (rotated) {
        const turn = { rotation: relativeRotation(this.lastOrientation, orientation), translation: [0, 0, 0] };
        seed = gtsam.pose3Compose(this.lastTransform, turn);
        this.gyroTracking = true;
        metrics = runIcp(gtsam, source, voxelDownsample(world, size), seed, size * 2.5, 500, 0, true);
      } else if (this.cameraTracking) {
        metrics = runMultiscaleIcp(gtsam, points, world, seed, size, 100);
      } else {
        metrics = runIcp(gtsam, source, voxelDownsample(world, size), seed, size * 2.5, 500);
      }
      if (this.cameraTracking) {
        const relative = gtsam.pose3Between(this.lastTransform, metrics.transform);
        const distance = Math.hypot(...relative.translation);
        const turn = angleOf(relative);
        if (metrics.fitness < 0.50 || distance > 0.65 || (!rotated && turn > 0.57)) {
          // After a second of lost frames, re-anchor at the gyro pose so tracking
          // does not wait for the camera to return to the last accepted view.
          if (!rotated || ++this.lostFrames < 4) {
            return { accepted: false, reason: 'lost', fitness: metrics.fitness };
          }
          metrics = { ...metrics, transform: seed };
        } else if (distance < 0.020 && turn < 0.018) {
          return { accepted: false, reason: 'still', fitness: metrics.fitness };
        }
      }
      this.window.push(transformCloud(source, metrics.transform));
      if (this.window.length > 10) this.window.shift();
    }
    this.lostFrames = 0;
    this.lastOrientation = orientation;
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
    if (!index) return;
    // The native pipeline searches on every second keyframe. Inferred depth yields
    // far fewer keyframes per turn, so gyro tracking searches on each one and
    // solves as soon as a loop is verified.
    const loops = this.loops.length;
    if (this.gyroTracking || index % 2 === 0) this.detectLoops(index);
    if (index % 2 === 0 || this.loops.length > loops) this.optimize();
  }

  detectLoops(index) {
    const current = this.keyframes[index];
    const gtsam = this.gtsam;
    const candidates = [];
    for (let from = 0; index - from >= 5; from++) {
      const previous = this.keyframes[from];
      const similarity = cosineSimilarity(current.descriptor, previous.descriptor);
      // Scan Context, tuned for LiDAR range, separates inferred indoor submaps
      // poorly. Gyro rotations drift little, so with them a keyframe that faced
      // nearly the same way is also a candidate, and the overlap check decides.
      const turn = this.gyroTracking ? angleOf(gtsam.pose3Between(current.pose, previous.pose)) : Infinity;
      if (similarity > this.similarityThreshold || turn < 0.6) candidates.push({ from, previous, similarity, turn });
    }
    candidates.sort((a, b) => this.gyroTracking ? a.turn - b.turn : b.similarity - a.similarity);
    for (const { from, previous, similarity } of candidates.slice(0, 4)) {
      // Without the gyro, keep the multiplication order of the seed in pose_graph.py.
      // That order only approximates the relative pose while keyframe rotations stay
      // small, so gyro tracking, whose rotations are trusted through a full turn,
      // uses the exact relative pose that maps the older submap into the current one.
      const seed = this.gyroTracking
        ? gtsam.pose3Between(current.pose, previous.pose)
        : gtsam.pose3Compose(previous.pose, gtsam.pose3Inverse(current.pose));
      // With gyro rotations, the loop keeps the seed rotation and searches for the
      // drifted translation. Inferred depth also needs a stricter overlap.
      const match = this.gyroTracking
        ? searchTranslation(gtsam, previous.points, current.points, seed)
        : runMultiscaleIcp(gtsam, previous.points, current.points, seed, this.voxelSize, 1000);
      if (match.fitness <= (this.gyroTracking ? 0.5 : 0.4)) continue;
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
