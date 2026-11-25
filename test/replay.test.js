import assert from 'node:assert/strict';
import test from 'node:test';
import { initGtsam } from '../src/poses.js?v=6';
import { rotationMatrix } from '../src/registration.js?v=6';
import { replayLength, replayPointThreshold, roomScans } from '../src/replay.js?v=6';
import { SlamPipeline } from '../src/slam-pipeline.js?v=6';

test('depth scans produce a verified loop when the camera revisits the room', async () => {
  const gtsam = await initGtsam();
  const slam = new SlamPipeline(gtsam, { pointThreshold: replayPointThreshold });
  let count = 0;
  for (const { points } of roomScans()) {
    slam.addScan(points);
    if (++count === 120) assert.equal(slam.loops.length, 0);
  }
  assert.equal(count, replayLength);
  assert.ok(slam.loops.length);
  for (const loop of slam.loops) {
    assert.ok(loop.to - loop.from >= 5);
    assert.ok(loop.similarity > 0.90);
    assert.ok(loop.fitness > 0.4);
  }
  const before = slam.poses(false);
  const after = slam.poses(true);
  assert.equal(before.length, count);
  assert.equal(after.length, count);
  assert.ok(after.some((pose, i) => Math.hypot(...gtsam.pose3Between(before[i], pose).translation) > 0.0005));
  // Verify that the estimated heading covers a full turn, using only poses
  // recovered by the scanner.
  let previousYaw = 0;
  let accumulatedYaw = 0;
  for (const { rotation } of after) {
    const matrix = rotationMatrix(rotation);
    const yaw = Math.atan2(matrix[2], matrix[8]);
    const difference = yaw - previousYaw;
    accumulatedYaw += Math.atan2(Math.sin(difference), Math.cos(difference));
    previousYaw = yaw;
  }
  assert.ok(Math.abs(accumulatedYaw) > 2 * Math.PI);
});

test('camera tracking follows the turn and verifies a loop', async () => {
  const gtsam = await initGtsam();
  const slam = new SlamPipeline(gtsam, { pointThreshold: 10_000, cameraTracking: true });
  for (const { points } of roomScans()) slam.addScan(points);
  assert.ok(slam.frames.length >= replayLength * 0.9);
  assert.ok(slam.loops.some(({ fitness }) => fitness > 0.4));
});
