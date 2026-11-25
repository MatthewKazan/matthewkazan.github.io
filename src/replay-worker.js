import { fuseFrames } from './map-fusion.js?v=6';
import { initGtsam } from './poses.js?v=6';
import { replayHeight, replayLength, replayPointThreshold, replayWidth, roomScans } from './replay.js?v=6';
import { viewingPoints, viewingPose } from './sensor-frame.js?v=6';
import { SlamPipeline } from './slam-pipeline.js?v=11';

try {
  const gtsam = await initGtsam();
  // Smaller simulated clouds accumulate fewer samples per submap than the
  // phone recording. All registration and verification parameters are shared.
  const slam = new SlamPipeline(gtsam, { pointThreshold: replayPointThreshold });
  const frames = [];
  let batch = [];
  let poses = [];
  self.postMessage({ type: 'start', total: replayLength, width: replayWidth, height: replayHeight });
  for (const scan of roomScans()) {
    slam.addScan(scan.points);
    const frame = { points: viewingPoints(scan.points), colors: scan.colors, preview: scan.preview };
    const pose = viewingPose(slam.poseAt(frames.length, false));
    frames.push(frame);
    batch.push(frame);
    poses.push(pose);
    if (frames.length === 1 || batch.length === 5 || frames.length === replayLength) {
      self.postMessage({ type: 'batch', frames: batch, poses });
      batch = [];
      poses = [];
    }
  }
  const initial = slam.poses(false).map(viewingPose);
  const corrected = slam.poses(true).map(viewingPose);
  const maps = [fuseFrames(frames, initial), fuseFrames(frames, corrected)];
  const buffers = maps.flatMap(({ points, colors }) => [points.buffer, colors.buffer]);
  self.postMessage({
    type: 'complete', initial, corrected, maps, loop: slam.latestLoop() ?? null,
  }, buffers);
} catch (error) {
  self.postMessage({ type: 'error', message: String(error) });
}
