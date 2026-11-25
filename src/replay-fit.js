import { optimizePoses } from './poses.js?v=2';
import { rotationMatrix } from './registration.js?v=2';

export function solveReplay(gtsam, { initial, ideal }) {
  const last = initial.length - 1;
  const odometry = initial.slice(1).map((pose, index) => gtsam.pose3Between(initial[index], pose));
  // This synthetic sensor drifts in position and retains a reliable heading.
  const sigmas = [0.002, 0.002, 0.002, 0.055, 0.055, 0.055];
  const loop = {
    from: 0,
    to: last,
    transform: gtsam.pose3Between(ideal[0], ideal[last]),
    noise: [0.002, 0.002, 0.002, 0.006, 0.006, 0.006],
  };
  return optimizePoses(gtsam, initial, odometry, [loop], sigmas);
}

// Average the actual projected samples within 4 cm voxels. Both views use
// the same fusion, with only the pose estimates changing.
export function fuseFrames(frames, poses) {
  const voxels = new Map();
  const size = 0.04;
  frames.forEach(({ points, colors }, frame) => {
    const m = rotationMatrix(poses[frame].rotation);
    const [tx, ty, tz] = poses[frame].translation;
    for (let i = 0; i < points.length; i += 3) {
      const x = points[i];
      const y = points[i + 1];
      const z = points[i + 2];
      const wx = m[0] * x + m[1] * y + m[2] * z + tx;
      const wy = m[3] * x + m[4] * y + m[5] * z + ty;
      const wz = m[6] * x + m[7] * y + m[8] * z + tz;
      const key = Math.floor(wx / size) + ',' + Math.floor(wy / size) + ',' + Math.floor(wz / size);
      const voxel = voxels.get(key);
      if (!voxel) {
        voxels.set(key, { x: wx, y: wy, z: wz, r: colors[i], g: colors[i + 1], b: colors[i + 2], count: 1 });
      } else {
        voxel.x += wx;
        voxel.y += wy;
        voxel.z += wz;
        voxel.r += colors[i];
        voxel.g += colors[i + 1];
        voxel.b += colors[i + 2];
        voxel.count++;
      }
    }
  });
  const points = new Float32Array(voxels.size * 3);
  const colors = new Uint8Array(points.length);
  let offset = 0;
  for (const { x, y, z, r, g, b, count } of voxels.values()) {
    points[offset] = x / count;
    points[offset + 1] = y / count;
    points[offset + 2] = z / count;
    colors[offset] = Math.round(r / count);
    colors[offset + 1] = Math.round(g / count);
    colors[offset + 2] = Math.round(b / count);
    offset += 3;
  }
  return { points, colors };
}
