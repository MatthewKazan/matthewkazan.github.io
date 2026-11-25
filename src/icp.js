import { identity } from './poses.js?v=6';
import { nearestMatch, rigidFit } from './registration.js?v=6';
import { estimateNormals, pointTree, transformCloud, voxelDownsample } from './point-cloud.js?v=6';

function associate(source, target, tree, radiusSquared, pairs) {
  let count = 0;
  let squared = 0;
  const match = { squared: radiusSquared, offset: -1 };
  for (let i = 0; i < source.length; i += 3) {
    match.squared = radiusSquared;
    match.offset = -1;
    nearestMatch(tree, target, source[i], source[i + 1], source[i + 2], match);
    pairs[i / 3] = match.offset;
    if (match.offset === -1) continue;
    count++;
    squared += match.squared;
  }
  return { count, fitness: count / pairs.length, rmse: count ? Math.sqrt(squared / count) : 0 };
}

function pointToPoint(source, target, pairs, count, translationOnly) {
  let sx = 0, sy = 0, sz = 0;
  let tx = 0, ty = 0, tz = 0;
  const cross = new Float64Array(9);
  for (let p = 0; p < pairs.length; p++) {
    const q = pairs[p];
    if (q === -1) continue;
    const offset = p * 3;
    const x = source[offset], y = source[offset + 1], z = source[offset + 2];
    const a = target[q], b = target[q + 1], c = target[q + 2];
    sx += x; sy += y; sz += z;
    tx += a; ty += b; tz += c;
    cross[0] += x * a; cross[1] += x * b; cross[2] += x * c;
    cross[3] += y * a; cross[4] += y * b; cross[5] += y * c;
    cross[6] += z * a; cross[7] += z * b; cross[8] += z * c;
  }
  const center = [sx / count, sy / count, sz / count];
  const otherCenter = [tx / count, ty / count, tz / count];
  if (translationOnly) {
    return { rotation: identity.rotation, translation: otherCenter.map((value, axis) => value - center[axis]) };
  }
  for (let i = 0; i < 3; i++) {
    for (let j = 0; j < 3; j++) cross[i * 3 + j] -= count * center[i] * otherCenter[j];
  }
  return rigidFit(cross, center, otherCenter);
}

function solveNormalEquations(matrix, rhs) {
  const scale = Math.max(...matrix.map(Math.abs));
  for (let i = 0; i < 6; i++) {
    let pivot = i;
    for (let j = i + 1; j < 6; j++) {
      if (Math.abs(matrix[j * 6 + i]) > Math.abs(matrix[pivot * 6 + i])) pivot = j;
    }
    if (Math.abs(matrix[pivot * 6 + i]) <= scale * 1e-12) return null;
    if (i !== pivot) {
      for (let j = i; j < 6; j++) {
        const a = i * 6 + j, b = pivot * 6 + j;
        const previous = matrix[a];
        matrix[a] = matrix[b];
        matrix[b] = previous;
      }
      const previous = rhs[i];
      rhs[i] = rhs[pivot];
      rhs[pivot] = previous;
    }
    for (let row = i + 1; row < 6; row++) {
      const ratio = matrix[row * 6 + i] / matrix[i * 6 + i];
      for (let column = i + 1; column < 6; column++) {
        matrix[row * 6 + column] -= ratio * matrix[i * 6 + column];
      }
      rhs[row] -= ratio * rhs[i];
    }
  }
  for (let row = 5; row >= 0; row--) {
    for (let column = row + 1; column < 6; column++) rhs[row] -= matrix[row * 6 + column] * rhs[column];
    rhs[row] /= matrix[row * 6 + row];
  }
  return rhs;
}

function pointToPlane(source, target, pairs, normals) {
  const jacobian = new Float64Array(6);
  const normalEquations = new Float64Array(36);
  const rhs = new Float64Array(6);
  for (let p = 0; p < pairs.length; p++) {
    const q = pairs[p];
    if (q === -1) continue;
    const offset = p * 3;
    const x = source[offset], y = source[offset + 1], z = source[offset + 2];
    const nx = normals[q], ny = normals[q + 1], nz = normals[q + 2];
    const residual = (x - target[q]) * nx + (y - target[q + 1]) * ny + (z - target[q + 2]) * nz;
    jacobian[0] = y * nz - z * ny;
    jacobian[1] = z * nx - x * nz;
    jacobian[2] = x * ny - y * nx;
    jacobian[3] = nx;
    jacobian[4] = ny;
    jacobian[5] = nz;
    for (let i = 0; i < 6; i++) {
      rhs[i] -= jacobian[i] * residual;
      for (let j = i; j < 6; j++) normalEquations[i * 6 + j] += jacobian[i] * jacobian[j];
    }
  }
  for (let i = 0; i < 6; i++) {
    for (let j = 0; j < i; j++) normalEquations[i * 6 + j] = normalEquations[j * 6 + i];
  }
  const delta = solveNormalEquations(normalEquations, rhs);
  if (!delta) return identity;
  const [ax, ay, az, tx, ty, tz] = delta;
  const cx = Math.cos(ax / 2), sx = Math.sin(ax / 2);
  const cy = Math.cos(ay / 2), sy = Math.sin(ay / 2);
  const cz = Math.cos(az / 2), sz = Math.sin(az / 2);
  return {
    rotation: {
      w: cz * cy * cx + sz * sy * sx,
      x: cz * cy * sx - sz * sy * cx,
      y: cz * sy * cx + sz * cy * sx,
      z: sz * cy * cx - cz * sy * sx,
    },
    translation: [tx, ty, tz],
  };
}

// The Open3D convergence and fitness rules used by open3d_utils.py.
// With translationOnly, the seed rotation is kept and only the offset is fitted.
export function runIcp(gtsam, source, target, seed, distance, iterations, normalRadius = 0, translationOnly = false) {
  if (!source.length || !target.length) return { transform: seed, fitness: 0, rmse: 0 };
  const tree = pointTree(target);
  const normals = normalRadius ? estimateNormals(target, tree, normalRadius) : null;
  const aligned = transformCloud(source, seed);
  const pairs = new Int32Array(source.length / 3);
  const radiusSquared = distance * distance;
  let transform = seed;
  let metrics = associate(aligned, target, tree, radiusSquared, pairs);
  for (let i = 0; i < iterations && metrics.count; i++) {
    const update = normals
      ? pointToPlane(aligned, target, pairs, normals)
      : pointToPoint(aligned, target, pairs, metrics.count, translationOnly);
    transform = gtsam.pose3Compose(update, transform);
    transformCloud(aligned, update, aligned);
    const next = associate(aligned, target, tree, radiusSquared, pairs);
    const stable = Math.abs(next.fitness - metrics.fitness) < 1e-6 && Math.abs(next.rmse - metrics.rmse) < 1e-6;
    metrics = next;
    if (stable) break;
  }
  return { transform, fitness: metrics.fitness, rmse: metrics.rmse };
}

// Odometry from inferred depth drifts by up to a metre before a loop, beyond the
// reach of ICP. When the rotation is trusted (from the gyro), this tries every
// offset on a coarse grid around the seed and keeps the one with the most source
// points near the target, then refines the translation from there.
export function searchTranslation(gtsam, source, target, seed, { range = [1.5, 1, 1.5], step = 0.15 } = {}) {
  const placed = transformCloud(voxelDownsample(source, step), seed);
  const coarse = voxelDownsample(target, step);
  if (!placed.length || !coarse.length) return { transform: seed, fitness: 0, rmse: 0 };
  const tree = pointTree(coarse);
  const match = { squared: 0, offset: -1 };
  let best = { hits: -1, offset: [0, 0, 0] };
  const steps = range.map((extent) => Math.round(extent / step));
  for (let i = -steps[0]; i <= steps[0]; i++) {
    for (let j = -steps[1]; j <= steps[1]; j++) {
      for (let k = -steps[2]; k <= steps[2]; k++) {
        const dx = i * step, dy = j * step, dz = k * step;
        let hits = 0;
        for (let p = 0; p < placed.length; p += 3) {
          match.squared = step * step;
          match.offset = -1;
          nearestMatch(tree, coarse, placed[p] + dx, placed[p + 1] + dy, placed[p + 2] + dz, match);
          if (match.offset !== -1) hits++;
        }
        if (hits > best.hits) best = { hits, offset: [dx, dy, dz] };
      }
    }
  }
  let result = {
    transform: { rotation: seed.rotation, translation: seed.translation.map((value, axis) => value + best.offset[axis]) },
  };
  for (const [size, distance] of [[0.1, 0.3], [0.04, 0.12], [0.02, 0.05]]) {
    result = runIcp(gtsam, voxelDownsample(source, size), voxelDownsample(target, size),
      result.transform, distance, 100, 0, true);
  }
  return result;
}

export function runMultiscaleIcp(gtsam, source, target, seed, size, fineIterations) {
  const coarse = runIcp(gtsam, voxelDownsample(source, size * 5), voxelDownsample(target, size * 5),
    seed, size * 20, 50, size * 10);
  return runIcp(gtsam, voxelDownsample(source, size), voxelDownsample(target, size),
    coarse.transform, size * 2.5, fineIterations, size * 2);
}
