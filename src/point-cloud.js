import { buildTree, rotationMatrix } from './registration.js?v=6';
import { extremeEigenvector } from './symmetric.js?v=6';

export function transformCloud(points, pose, result = new Float64Array(points.length)) {
  const [a, b, c, d, e, f, g, h, i] = rotationMatrix(pose.rotation);
  const [tx, ty, tz] = pose.translation;
  for (let p = 0; p < points.length; p += 3) {
    const x = points[p], y = points[p + 1], z = points[p + 2];
    result[p] = a * x + b * y + c * z + tx;
    result[p + 1] = d * x + e * y + f * z + ty;
    result[p + 2] = g * x + h * y + i * z + tz;
  }
  return result;
}

export function mergeClouds(clouds) {
  const merged = new Float64Array(clouds.reduce((length, points) => length + points.length, 0));
  let offset = 0;
  for (const points of clouds) {
    merged.set(points, offset);
    offset += points.length;
  }
  return merged;
}

// Open3D anchors its voxel lattice half a voxel below the cloud's minimum.
export function voxelDownsample(points, size) {
  if (!points.length) return new Float64Array();
  let minX = Infinity, minY = Infinity, minZ = Infinity;
  let maxX = -Infinity, maxY = -Infinity;
  for (let i = 0; i < points.length; i += 3) {
    minX = Math.min(minX, points[i]);
    minY = Math.min(minY, points[i + 1]);
    minZ = Math.min(minZ, points[i + 2]);
    maxX = Math.max(maxX, points[i]);
    maxY = Math.max(maxY, points[i + 1]);
  }
  minX -= size / 2;
  minY -= size / 2;
  minZ -= size / 2;
  const width = Math.floor((maxX - minX) / size) + 1;
  const height = Math.floor((maxY - minY) / size) + 1;
  const cells = new Map();
  for (let i = 0; i < points.length; i += 3) {
    const x = points[i], y = points[i + 1], z = points[i + 2];
    const key = (Math.floor((z - minZ) / size) * height + Math.floor((y - minY) / size)) * width
      + Math.floor((x - minX) / size);
    const cell = cells.get(key);
    if (!cell) cells.set(key, [x, y, z, 1]);
    else {
      cell[0] += x;
      cell[1] += y;
      cell[2] += z;
      cell[3]++;
    }
  }
  const downsampled = new Float64Array(3 * cells.size);
  let i = 0;
  for (const [x, y, z, count] of cells.values()) {
    downsampled[i++] = x / count;
    downsampled[i++] = y / count;
    downsampled[i++] = z / count;
  }
  return downsampled;
}

export function pointTree(points) {
  const offsets = Array.from({ length: points.length / 3 }, (_, i) => i * 3);
  return buildTree(points, offsets);
}

class Neighbors {
  constructor(points, tree, maximum, radius = Infinity) {
    this.points = points;
    this.tree = tree;
    this.maximum = maximum;
    this.maximumSquared = radius * radius;
    this.offsets = new Int32Array(maximum);
    this.squared = new Float64Array(maximum);
  }

  at(offset) {
    this.count = 0;
    this.visit(this.tree, this.points[offset], this.points[offset + 1], this.points[offset + 2]);
    return this;
  }

  limit() {
    return this.count === this.maximum ? this.squared[0] : this.maximumSquared;
  }

  visit(node, x, y, z) {
    if (node.indices) {
      for (let i = node.start; i < node.end; i++) {
        const offset = node.indices[i];
        const dx = x - this.points[offset];
        const dy = y - this.points[offset + 1];
        const dz = z - this.points[offset + 2];
        const squared = dx * dx + dy * dy + dz * dz;
        if (squared < this.limit()) this.add(offset, squared);
      }
      return;
    }
    const { axis, left, right } = node;
    const split = (axis === 0 ? x : axis === 1 ? y : z) - node.split;
    this.visit(split < 0 ? left : right, x, y, z);
    if (split * split < this.limit()) this.visit(split < 0 ? right : left, x, y, z);
  }

  add(offset, squared) {
    const indices = this.offsets;
    const distances = this.squared;
    if (this.count < this.maximum) {
      let i = this.count++;
      while (i > 0) {
        const parent = (i - 1) >> 1;
        if (distances[parent] >= squared) break;
        indices[i] = indices[parent];
        distances[i] = distances[parent];
        i = parent;
      }
      indices[i] = offset;
      distances[i] = squared;
    } else {
      let i = 0;
      for (let child = 1; child < this.count; child = 2 * i + 1) {
        if (child + 1 < this.count && distances[child + 1] > distances[child]) child++;
        if (distances[child] <= squared) break;
        indices[i] = indices[child];
        distances[i] = distances[child];
        i = child;
      }
      indices[i] = offset;
      distances[i] = squared;
    }
  }
}

export function rejectStatisticalOutliers(points) {
  const count = points.length / 3;
  if (count < 2) return points;
  const neighbors = new Neighbors(points, pointTree(points), 30);
  const distances = new Float64Array(count);
  let mean = 0;
  for (let i = 0; i < count; i++) {
    neighbors.at(i * 3);
    let sum = 0;
    for (let j = 0; j < neighbors.count; j++) sum += Math.sqrt(neighbors.squared[j]);
    distances[i] = sum / neighbors.count;
    mean += distances[i];
  }
  mean /= count;
  let deviation = 0;
  for (const distance of distances) if (distance > 0) deviation += (distance - mean) ** 2;
  const cutoff = mean + 3 * Math.sqrt(deviation / (count - 1));
  const filtered = new Float64Array(points.length);
  let length = 0;
  for (let i = 0; i < count; i++) {
    if (distances[i] <= 0 || distances[i] >= cutoff) continue;
    const offset = i * 3;
    filtered[length++] = points[offset];
    filtered[length++] = points[offset + 1];
    filtered[length++] = points[offset + 2];
  }
  return filtered.subarray(0, length);
}

export function estimateNormals(points, tree, radius) {
  const neighbors = new Neighbors(points, tree, 30, radius);
  const normals = new Float64Array(points.length);
  for (let i = 0; i < points.length; i += 3) {
    neighbors.at(i);
    if (neighbors.count < 3) {
      normals[i + 2] = 1;
      continue;
    }
    let x = 0, y = 0, z = 0;
    let xx = 0, xy = 0, xz = 0, yy = 0, yz = 0, zz = 0;
    for (let n = 0; n < neighbors.count; n++) {
      const offset = neighbors.offsets[n];
      const a = points[offset], b = points[offset + 1], c = points[offset + 2];
      x += a; y += b; z += c;
      xx += a * a; xy += a * b; xz += a * c;
      yy += b * b; yz += b * c; zz += c * c;
    }
    const k = 1 / neighbors.count;
    xx -= x * x * k; xy -= x * y * k; xz -= x * z * k;
    yy -= y * y * k; yz -= y * z * k; zz -= z * z * k;
    normals.set(extremeEigenvector([xx, xy, xz, xy, yy, yz, xz, yz, zz], 3, false), i);
  }
  return normals;
}
