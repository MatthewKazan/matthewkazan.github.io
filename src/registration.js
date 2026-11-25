import { identity } from './poses.js?v=6';
import { extremeEigenvector } from './symmetric.js?v=6';

function partitionToMedian(points, indices, axis, start, end, median) {
  while (start < end) {
    const pivot = points[indices[(start + end) >> 1] + axis];
    let left = start, right = end;
    while (left <= right) {
      while (points[indices[left] + axis] < pivot) left++;
      while (points[indices[right] + axis] > pivot) right--;
      if (left <= right) {
        const value = indices[left];
        indices[left++] = indices[right];
        indices[right--] = value;
      }
    }
    if (median <= right) end = right;
    else if (median >= left) start = left;
    else break;
  }
}

export function buildTree(points, indices) {
  function build(start, end) {
    if (end - start <= 12) return { start, end, indices };
    let lowX = Infinity, lowY = Infinity, lowZ = Infinity;
    let highX = -Infinity, highY = -Infinity, highZ = -Infinity;
    for (let i = start; i < end; i++) {
      const n = indices[i];
      const x = points[n], y = points[n + 1], z = points[n + 2];
      lowX = Math.min(lowX, x); highX = Math.max(highX, x);
      lowY = Math.min(lowY, y); highY = Math.max(highY, y);
      lowZ = Math.min(lowZ, z); highZ = Math.max(highZ, z);
    }
    const rangeX = highX - lowX, rangeY = highY - lowY, rangeZ = highZ - lowZ;
    const axis = rangeX >= rangeY && rangeX >= rangeZ ? 0 : rangeY >= rangeZ ? 1 : 2;
    const middle = (start + end) >> 1;
    partitionToMedian(points, indices, axis, start, end - 1, middle);
    return { axis, split: points[indices[middle] + axis], left: build(start, middle), right: build(middle, end) };
  }
  return build(0, indices.length);
}

export function nearestMatch(tree, points, x, y, z, match) {
  if (!tree) return;
  if (tree.indices) {
    for (let i = tree.start; i < tree.end; i++) {
      const offset = tree.indices[i];
      const dx = x - points[offset];
      const dy = y - points[offset + 1];
      const dz = z - points[offset + 2];
      const squared = dx * dx + dy * dy + dz * dz;
      if (squared < match.squared) {
        match.squared = squared;
        match.offset = offset;
      }
    }
    return;
  }
  const { axis, left, right } = tree;
  const split = (axis === 0 ? x : axis === 1 ? y : z) - tree.split;
  nearestMatch(split < 0 ? left : right, points, x, y, z, match);
  if (split * split < match.squared) {
    nearestMatch(split < 0 ? right : left, points, x, y, z, match);
  }
}

export function rotationMatrix({ w, x, y, z }) {
  return [
    1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w),
    2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w),
    2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y),
  ];
}

export function transformPoint(matrix, x, y, z, translation) {
  return [
    matrix[0] * x + matrix[1] * y + matrix[2] * z + translation[0],
    matrix[3] * x + matrix[4] * y + matrix[5] * z + translation[1],
    matrix[6] * x + matrix[7] * y + matrix[8] * z + translation[2],
  ];
}

// Horn's quaternion fit minimizes the same rigid least-squares objective as
// the SVD used by Open3D's point-to-point estimator.
export function rigidFit(covariance, source, target) {
  const [xx, xy, xz, yx, yy, yz, zx, zy, zz] = covariance;
  const eigen = [
    xx + yy + zz, yz - zy, zx - xz, xy - yx,
    yz - zy, xx - yy - zz, xy + yx, zx + xz,
    zx - xz, xy + yx, -xx + yy - zz, yz + zy,
    xy - yx, zx + xz, yz + zy, -xx - yy + zz,
  ];
  const [w, x, y, z] = extremeEigenvector(eigen, 4, true);
  const rotation = { w, x, y, z };
  const center = transformPoint(rotationMatrix(rotation), ...source, identity.translation);
  return { rotation, translation: target.map((value, axis) => value - center[axis]) };
}

export function angleOf(pose) {
  const { w, x, y, z } = pose.rotation;
  return 2 * Math.atan2(Math.hypot(x, y, z), Math.abs(w));
}
