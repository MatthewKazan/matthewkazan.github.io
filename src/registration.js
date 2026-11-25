import { identity } from './poses.js';

function buildTree(points, indices, depth = 0) {
  if (!indices.length) return null;
  const axis = depth % 3;
  indices.sort((a, b) => points[a + axis] - points[b + axis]);
  const middle = indices.length >> 1;
  return {
    axis,
    offset: indices[middle],
    left: buildTree(points, indices.slice(0, middle), depth + 1),
    right: buildTree(points, indices.slice(middle + 1), depth + 1),
  };
}

function nearestMatch(tree, points, x, y, z, match) {
  if (!tree) return;
  const { axis, offset, left, right } = tree;
  const dx = x - points[offset];
  const dy = y - points[offset + 1];
  const dz = z - points[offset + 2];
  const squared = dx * dx + dy * dy + dz * dz;
  if (squared < match.squared) {
    match.squared = squared;
    match.offset = offset;
  }
  const split = axis === 0 ? dx : axis === 1 ? dy : dz;
  nearestMatch(split < 0 ? left : right, points, x, y, z, match);
  if (split * split < match.squared) {
    nearestMatch(split < 0 ? right : left, points, x, y, z, match);
  }
}

function rotationMatrix({ w, x, y, z }) {
  return [
    1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w),
    2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w),
    2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y),
  ];
}

function transformPoint(matrix, x, y, z, translation) {
  return [
    matrix[0] * x + matrix[1] * y + matrix[2] * z + translation[0],
    matrix[3] * x + matrix[4] * y + matrix[5] * z + translation[1],
    matrix[6] * x + matrix[7] * y + matrix[8] * z + translation[2],
  ];
}

// Horn's quaternion fit for matched 3D points. The shift makes power iteration
// converge to the largest algebraic eigenvalue even for mostly planar scenes.
function fitRigid(matches) {
  const source = [0, 0, 0];
  const target = [0, 0, 0];
  for (const { from, to } of matches) {
    for (let axis = 0; axis < 3; axis++) {
      source[axis] += from[axis] / matches.length;
      target[axis] += to[axis] / matches.length;
    }
  }
  const covariance = new Float64Array(9);
  for (const { from, to } of matches) {
    for (let row = 0; row < 3; row++) {
      for (let col = 0; col < 3; col++) {
        covariance[row * 3 + col] += (from[row] - source[row]) * (to[col] - target[col]);
      }
    }
  }
  const [xx, xy, xz, yx, yy, yz, zx, zy, zz] = covariance;
  const eigen = [
    [xx + yy + zz, yz - zy, zx - xz, xy - yx],
    [yz - zy, xx - yy - zz, xy + yx, zx + xz],
    [zx - xz, xy + yx, -xx + yy - zz, yz + zy],
    [xy - yx, zx + xz, yz + zy, -xx - yy + zz],
  ];
  const shift = Math.max(...eigen.map((row) => row.reduce((sum, value) => sum + Math.abs(value), 0)));
  let unit = [1, 0, 0, 0];
  for (let i = 0; shift > 0 && i < 26; i++) {
    const next = eigen.map((row, a) => shift * unit[a] + row.reduce((sum, value, b) => sum + value * unit[b], 0));
    const norm = Math.hypot(...next);
    if (norm < 1e-10) break;
    unit = next.map((value) => value / norm);
  }
  const [w, x, y, z] = unit;
  const rotation = { w, x, y, z };
  const center = transformPoint(rotationMatrix(rotation), ...source, identity.translation);
  return { rotation, translation: target.map((value, axis) => value - center[axis]) };
}

// A bounded, trimmed point-to-point ICP for nearby camera views.
export function register(gtsam, source, target, seed = identity) {
  const targetStride = 3 * Math.max(1, Math.ceil(target.length / (3 * 2800)));
  const indices = [];
  for (let offset = 0; offset < target.length; offset += targetStride) indices.push(offset);
  const tree = buildTree(target, indices);
  const stride = 3 * Math.max(1, Math.ceil(source.length / (3 * 1700)));
  const samples = Math.ceil(source.length / stride);
  if (samples < 80 || indices.length < 80) return null;
  let transform = seed;
  let matches = [];
  for (let iteration = 0; iteration <= 13; iteration++) {
    matches = [];
    const radius = iteration < 4 ? 0.39 : 0.21;
    const rotation = rotationMatrix(transform.rotation);
    for (let i = 0; i < source.length; i += stride) {
      const from = transformPoint(rotation, source[i], source[i + 1], source[i + 2], transform.translation);
      const match = { squared: radius ** 2, offset: -1 };
      nearestMatch(tree, target, ...from, match);
      if (match.offset < 0) continue;
      const j = match.offset;
      matches.push({ from, to: [target[j], target[j + 1], target[j + 2]], squared: match.squared });
    }
    if (matches.length < Math.max(60, 0.33 * samples)) return null;
    if (iteration === 13) break;
    matches.sort((a, b) => a.squared - b.squared);
    const step = fitRigid(matches.slice(0, Math.floor(matches.length * 0.80)));
    transform = gtsam.pose3Compose(step, transform);
  }
  const overlap = matches.length / samples;
  const rmse = Math.sqrt(matches.reduce((sum, match) => sum + match.squared, 0) / matches.length);
  return { transform, overlap, rmse };
}

export function angleOf(pose) {
  const { w, x, y, z } = pose.rotation;
  return 2 * Math.atan2(Math.hypot(x, y, z), Math.abs(w));
}
