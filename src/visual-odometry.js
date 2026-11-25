// Frame-to-frame visual odometry for camera mode.
//
// The iPhone app registered LiDAR scans, whose depth is metric and consistent
// from frame to frame. Depth estimated from a single image is neither: its
// scale drifts by tens of percent between frames, and ICP cannot tell a scale
// error from camera motion. This module tracks image corners from the previous
// frame into the next, solves the camera motion and the new frame's depth
// scale from those tracks, and so carries one consistent scale through the
// recording. The SLAM pipeline then registers and closes loops on scans that
// agree with each other.
import { rigidFit, rotationMatrix } from './registration.js?v=6';

const levels = 3;
const radius = 4;

export function grayscale(rgba, width, height) {
  const gray = new Float32Array(width * height);
  for (let i = 0; i < gray.length; i++) {
    gray[i] = 0.299 * rgba[4 * i] + 0.587 * rgba[4 * i + 1] + 0.114 * rgba[4 * i + 2];
  }
  return gray;
}

function pyramid(gray, width, height) {
  const result = [{ data: gray, width, height }];
  for (let level = 1; level < levels; level++) {
    const previous = result[level - 1];
    const w = previous.width >> 1, h = previous.height >> 1;
    const data = new Float32Array(w * h);
    for (let y = 0; y < h; y++) {
      for (let x = 0; x < w; x++) {
        const i = 2 * y * previous.width + 2 * x;
        data[y * w + x] = 0.25 * (previous.data[i] + previous.data[i + 1] +
          previous.data[i + previous.width] + previous.data[i + previous.width + 1]);
      }
    }
    result.push({ data, width: w, height: h });
  }
  return result;
}

function sample({ data, width, height }, x, y) {
  if (x < 0 || y < 0 || x > width - 1.001 || y > height - 1.001) return NaN;
  const x0 = Math.floor(x), y0 = Math.floor(y), a = x - x0, b = y - y0, i = y0 * width + x0;
  return (data[i] * (1 - a) + data[i + 1] * a) * (1 - b) + (data[i + width] * (1 - a) + data[i + width + 1] * a) * b;
}

// Shi-Tomasi corners, at most one per grid cell, where the depth map is valid
// and smooth (corners on depth edges have unreliable depth).
function detect(image, depth, existing, cell = 12) {
  const { data, width, height } = image;
  const columns = Math.ceil(width / cell), rows = Math.ceil(height / cell);
  const taken = new Uint8Array(columns * rows);
  for (const [x, y] of existing) {
    const c = Math.floor(x / cell), r = Math.floor(y / cell);
    if (c >= 0 && r >= 0 && c < columns && r < rows) taken[r * columns + c] = 1;
  }
  const best = new Float32Array(columns * rows);
  const at = new Int32Array(columns * rows).fill(-1);
  const border = radius + 2;
  for (let y = border; y < height - border; y++) {
    for (let x = border; x < width - border; x++) {
      const cellIndex = Math.floor(y / cell) * columns + Math.floor(x / cell);
      if (taken[cellIndex]) continue;
      const i = y * width + x;
      const z = depth[i];
      if (!z) continue;
      let xx = 0, xy = 0, yy = 0, smooth = true;
      for (let dy = -2; dy <= 2; dy++) {
        for (let dx = -2; dx <= 2; dx++) {
          const j = i + dy * width + dx;
          const gx = 0.5 * (data[j + 1] - data[j - 1]);
          const gy = 0.5 * (data[j + width] - data[j - width]);
          xx += gx * gx; xy += gx * gy; yy += gy * gy;
          const neighbor = depth[j];
          if (!neighbor || Math.abs(neighbor - z) > 0.08 * z) smooth = false;
        }
      }
      if (!smooth) continue;
      const score = 0.5 * (xx + yy - Math.sqrt((xx - yy) ** 2 + 4 * xy * xy));
      if (score > best[cellIndex]) {
        best[cellIndex] = score;
        at[cellIndex] = i;
      }
    }
  }
  const found = [];
  const threshold = 50;
  for (let c = 0; c < at.length; c++) {
    if (at[c] >= 0 && best[c] > threshold) found.push([at[c] % width, Math.floor(at[c] / width)]);
  }
  return found;
}

// Pyramidal Lucas-Kanade for one point, from `from` in the previous pyramid,
// starting at `guess` in the next one.
function trackPoint(previous, next, [x, y], [gx, gy]) {
  let dx = (gx - x) / 2 ** (levels - 1), dy = (gy - y) / 2 ** (levels - 1);
  for (let level = levels - 1; level >= 0; level--) {
    const scale = 2 ** level;
    const px = x / scale, py = y / scale;
    const template = previous[level], target = next[level];
    let g11 = 0, g12 = 0, g22 = 0;
    const values = [], gradients = [];
    for (let v = -radius; v <= radius; v++) {
      for (let u = -radius; u <= radius; u++) {
        const value = sample(template, px + u, py + v);
        const ix = 0.5 * (sample(template, px + u + 1, py + v) - sample(template, px + u - 1, py + v));
        const iy = 0.5 * (sample(template, px + u, py + v + 1) - sample(template, px + u, py + v - 1));
        if (!Number.isFinite(value + ix + iy)) return null;
        values.push(value);
        gradients.push(ix, iy);
        g11 += ix * ix; g12 += ix * iy; g22 += iy * iy;
      }
    }
    const det = g11 * g22 - g12 * g12;
    if (det < 1e-6) return null;
    for (let iteration = 0; iteration < 12; iteration++) {
      let b1 = 0, b2 = 0, k = 0;
      for (let v = -radius; v <= radius; v++) {
        for (let u = -radius; u <= radius; u++, k++) {
          const error = sample(target, px + dx + u, py + dy + v) - values[k];
          if (!Number.isFinite(error)) return null;
          b1 += error * gradients[2 * k];
          b2 += error * gradients[2 * k + 1];
        }
      }
      const ux = -(g22 * b1 - g12 * b2) / det, uy = -(g11 * b2 - g12 * b1) / det;
      dx += ux; dy += uy;
      if (ux * ux + uy * uy < 0.0009) break;
    }
    if (level) { dx *= 2; dy *= 2; }
  }
  return [x + dx, y + dy];
}

const multiply = (a, b) => ({
  w: a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
  x: a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
  y: a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
  z: a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
});
const conjugate = ({ w, x, y, z }) => ({ w, x: -x, y: -y, z: -z });
const apply = (matrix, [x, y, z]) => [
  matrix[0] * x + matrix[1] * y + matrix[2] * z,
  matrix[3] * x + matrix[4] * y + matrix[5] * z,
  matrix[6] * x + matrix[7] * y + matrix[8] * z,
];
function normalize(q) {
  const n = Math.hypot(q.w, q.x, q.y, q.z);
  return { w: q.w / n, x: q.x / n, y: q.y / n, z: q.z / n };
}
function expRotation([x, y, z]) {
  const angle = Math.hypot(x, y, z);
  if (angle < 1e-12) return normalize({ w: 1, x: x / 2, y: y / 2, z: z / 2 });
  const s = Math.sin(angle / 2) / angle;
  return { w: Math.cos(angle / 2), x: x * s, y: y * s, z: z * s };
}
export function composePose(a, b) {
  const rotated = apply(rotationMatrix(a.rotation), b.translation);
  return {
    rotation: normalize(multiply(a.rotation, b.rotation)),
    translation: rotated.map((value, axis) => value + a.translation[axis]),
  };
}
export function inversePose({ rotation, translation }) {
  const inverse = conjugate(rotation);
  return { rotation: inverse, translation: apply(rotationMatrix(inverse), translation).map((v) => -v) };
}

// Similarity transform b = s R a + t from correspondences (Umeyama, via the
// Horn rotation fit already used by ICP).
function similarity(a, b, indices) {
  const ca = [0, 0, 0], cb = [0, 0, 0];
  for (const i of indices) for (let k = 0; k < 3; k++) { ca[k] += a[i][k]; cb[k] += b[i][k]; }
  for (let k = 0; k < 3; k++) { ca[k] /= indices.length; cb[k] /= indices.length; }
  const cross = new Float64Array(9);
  let spread = 0;
  for (const i of indices) {
    for (let r = 0; r < 3; r++) {
      const da = a[i][r] - ca[r];
      spread += da * da;
      for (let c = 0; c < 3; c++) cross[r * 3 + c] += da * (b[i][c] - cb[c]);
    }
  }
  if (spread < 1e-9) return null;
  const { rotation } = rigidFit(cross, ca, cb);
  const matrix = rotationMatrix(rotation);
  let dot = 0;
  for (const i of indices) {
    const ra = apply(matrix, a[i].map((value, k) => value - ca[k]));
    dot += ra[0] * (b[i][0] - cb[0]) + ra[1] * (b[i][1] - cb[1]) + ra[2] * (b[i][2] - cb[2]);
  }
  const scale = dot / spread;
  if (!(scale > 0)) return null;
  const rc = apply(matrix, ca);
  return { rotation, matrix, scale, translation: cb.map((value, k) => value - scale * rc[k]) };
}

function median(values) {
  const sorted = Float64Array.from(values).sort();
  return sorted.length ? sorted[sorted.length >> 1] : NaN;
}

// Refines the pose of the current camera from reference points X (reference
// frame) and their tracked pixels, minimizing reprojection error with Huber
// weights. With `fixedRotation` only the translation is solved.
function refinePose(X, pixels, camera, pose, fixedRotation) {
  let { rotation, translation } = pose;
  const { fx, fy, cx, cy } = camera;
  const dimension = fixedRotation ? 3 : 6;
  for (let iteration = 0; iteration < 10; iteration++) {
    const matrix = rotationMatrix(rotation);
    const H = new Float64Array(dimension * dimension), g = new Float64Array(dimension);
    for (let i = 0; i < X.length; i++) {
      const p = apply(matrix, X[i]).map((value, k) => value + translation[k]);
      if (p[2] < 0.05) continue;
      const inverse = 1 / p[2];
      const ex = fx * p[0] * inverse + cx - pixels[i][0];
      const ey = fy * p[1] * inverse + cy - pixels[i][1];
      const error = Math.hypot(ex, ey);
      const weight = error < 2 ? 1 : 2 / error;
      // Derivatives of the projection with respect to the camera-frame point.
      const du = [fx * inverse, 0, -fx * p[0] * inverse * inverse];
      const dv = [0, fy * inverse, -fy * p[1] * inverse * inverse];
      // Left perturbation: d p = v - [p]x w.
      const ju = [], jv = [];
      if (!fixedRotation) {
        ju.push(du[1] * -p[2] + du[2] * p[1], du[0] * p[2] - du[2] * p[0], -du[0] * p[1] + du[1] * p[0]);
        jv.push(dv[1] * -p[2] + dv[2] * p[1], dv[0] * p[2] - dv[2] * p[0], -dv[0] * p[1] + dv[1] * p[0]);
      }
      ju.push(...du);
      jv.push(...dv);
      for (let r = 0; r < dimension; r++) {
        g[r] += weight * (ju[r] * ex + jv[r] * ey);
        for (let c = r; c < dimension; c++) H[r * dimension + c] += weight * (ju[r] * ju[c] + jv[r] * jv[c]);
      }
    }
    for (let r = 0; r < dimension; r++) for (let c = 0; c < r; c++) H[r * dimension + c] = H[c * dimension + r];
    const step = solve(H, g.map((value) => -value), dimension);
    if (!step) break;
    const w = fixedRotation ? [0, 0, 0] : step.slice(0, 3);
    const v = fixedRotation ? step : step.slice(3);
    const delta = expRotation(w);
    rotation = normalize(multiply(delta, rotation));
    translation = apply(rotationMatrix(delta), translation).map((value, k) => value + v[k]);
    if (Math.hypot(...step) < 1e-6) break;
  }
  return { rotation, translation };
}

function solve(A, b, n) {
  const m = Float64Array.from(A), x = Float64Array.from(b);
  for (let i = 0; i < n; i++) {
    let pivot = i;
    for (let r = i + 1; r < n; r++) if (Math.abs(m[r * n + i]) > Math.abs(m[pivot * n + i])) pivot = r;
    if (Math.abs(m[pivot * n + i]) < 1e-12) return null;
    for (let c = 0; c < n; c++) [m[i * n + c], m[pivot * n + c]] = [m[pivot * n + c], m[i * n + c]];
    [x[i], x[pivot]] = [x[pivot], x[i]];
    for (let r = i + 1; r < n; r++) {
      const f = m[r * n + i] / m[i * n + i];
      for (let c = i; c < n; c++) m[r * n + c] -= f * m[i * n + c];
      x[r] -= f * x[i];
    }
  }
  for (let r = n - 1; r >= 0; r--) {
    for (let c = r + 1; c < n; c++) x[r] -= m[r * n + c] * x[c];
    x[r] /= m[r * n + r];
  }
  return Array.from(x);
}

export class VisualOdometry {
  constructor({ random = Math.random } = {}) {
    this.random = random;
    this.keyframe = null;
    this.samples = [];
    this.pose = { rotation: { w: 1, x: 0, y: 0, z: 0 }, translation: [0, 0, 0] };
    this.scale = 1;
    this.motion = null;
  }

  // gray and depth are width x height arrays; intrinsics are normalized as
  // estimateCloud returns them; orientation is an optional gyro camera-to-world
  // quaternion. Returns the camera-to-world pose and the factor to multiply this
  // frame's depth by, or ok: false when tracking failed.
  //
  // Frames are tracked against the latest keyframe rather than the previous
  // frame, so pose and scale errors only add up once per keyframe. Points that
  // stay in view keep their 3D position from keyframe to keyframe.
  process({ gray, depth, width, height, intrinsics, orientation }) {
    const camera = {
      fx: intrinsics.fx * width, fy: intrinsics.fy * height,
      cx: intrinsics.cx * width - 0.5, cy: intrinsics.cy * height - 0.5,
    };
    const images = pyramid(gray, width, height);
    const keyframe = this.keyframe;
    let result = { ok: false, tracked: 0, inliers: 0 };
    if (keyframe) result = this.track(keyframe, images, depth, width, camera, orientation);
    let motion = result.ok ? result.motion : null;
    if (keyframe && !motion) {
      // Keep moving with the gyro (or the last motion) and the last scale.
      const turn = orientation && keyframe.orientation
        ? normalize(multiply(conjugate(keyframe.orientation), orientation)) : null;
      motion = turn ? { rotation: turn, translation: this.motion?.translation ?? [0, 0, 0] } : this.motion;
    }
    if (result.ok) this.scale = result.scale;
    if (motion) this.pose = composePose(keyframe.pose, motion);
    this.motion = motion;
    const turned = motion ? 2 * Math.acos(Math.min(1, Math.abs(motion.rotation.w))) : 0;
    const moved = motion ? Math.hypot(...motion.translation) : 0;
    const needed = !keyframe || !result.ok || result.inliers < Math.max(40, 0.5 * keyframe.features.length) ||
      turned > 0.25 || moved > 0.25;
    if (needed) {
      // The chained scale is consistent from frame to frame but can wander, most
      // on blank walls where few corners track. The depth network's own metric
      // scale is noisy but does not drift, so each keyframe moves part of the way
      // back toward it, more when tracking is weak.
      const pull = Math.min(0.3, 0.03 + 4 / Math.max(1, result.inliers));
      const correction = this.scale ** -pull;
      this.scale *= correction;
      const kept = result.ok ? result.kept : [];
      const features = kept.map(({ pixel, point }) => ({ pixel, point: point.map((v) => v * correction), last: pixel }));
      for (const [x, y] of detect(images[0], depth, kept.map(({ pixel }) => pixel))) {
        const z = depth[Math.round(y) * width + Math.round(x)] * this.scale;
        if (!z) continue;
        features.push({ pixel: [x, y], last: [x, y], point: [(x - camera.cx) * z / camera.fx, (y - camera.cy) * z / camera.fy, z] });
      }
      this.keyframe = { images, features, pose: this.pose, orientation, camera };
      this.motion = { rotation: { w: 1, x: 0, y: 0, z: 0 }, translation: [0, 0, 0] };
    }
    return { ...result, pose: this.pose, scale: this.scale, keyframe: needed };
  }

  track(keyframe, images, depth, width, camera, orientation) {
    const turn = orientation && keyframe.orientation
      ? normalize(multiply(conjugate(keyframe.orientation), orientation)) : null;
    // Predict where each feature lands: the gyro turn since the keyframe, or
    // where the feature was in the previous frame.
    const predicted = turn ? { rotation: turn, translation: this.motion?.translation ?? [0, 0, 0] } : null;
    const predictedInverse = predicted && inversePose(predicted);
    const a = [], b = [], pixels = [], reference3d = [], keyframePixels = [];
    for (const feature of keyframe.features) {
      const { pixel, point } = feature;
      let guess = feature.last ?? pixel;
      if (predictedInverse) {
        const p = apply(rotationMatrix(predictedInverse.rotation), point).map((v, k) => v + predictedInverse.translation[k]);
        if (p[2] > 0.05) guess = [camera.fx * p[0] / p[2] + camera.cx, camera.fy * p[1] / p[2] + camera.cy];
      }
      feature.last = null;
      const forward = trackPoint(keyframe.images, images, pixel, guess);
      if (!forward) continue;
      const backward = trackPoint(images, keyframe.images, forward, pixel);
      if (!backward || Math.hypot(backward[0] - pixel[0], backward[1] - pixel[1]) > 1) continue;
      feature.last = forward;
      const z = depth[Math.round(forward[1]) * width + Math.round(forward[0])];
      if (!z) continue;
      a.push([(forward[0] - camera.cx) * z / camera.fx, (forward[1] - camera.cy) * z / camera.fy, z]);
      b.push(point);
      keyframePixels.push(pixel);
      pixels.push(forward);
      reference3d.push(point);
    }
    const tracked = a.length;
    if (tracked < 12) return { ok: false, tracked, inliers: 0 };

    // RANSAC over similarity transforms from the current frame's raw depth
    // points (a) to the reference frame's scaled points (b).
    let best = null, bestInliers = [];
    for (let iteration = 0; iteration < 150; iteration++) {
      const pick = new Set();
      while (pick.size < 3) pick.add(Math.floor(this.random() * tracked));
      const model = similarity(a, b, [...pick]);
      if (!model || model.scale < 0.5 || model.scale > 2) continue;
      if (turn) {
        // With motion sensors, the rotation must agree with the gyro.
        const difference = multiply(conjugate(turn), model.rotation);
        if (2 * Math.acos(Math.min(1, Math.abs(difference.w))) > 0.15) continue;
      }
      const inliers = [];
      for (let i = 0; i < tracked; i++) {
        const p = apply(model.matrix, a[i]).map((v, k) => model.scale * v + model.translation[k]);
        const error = Math.hypot(p[0] - b[i][0], p[1] - b[i][1], p[2] - b[i][2]);
        if (error < 0.03 + 0.05 * b[i][2]) inliers.push(i);
      }
      if (inliers.length > bestInliers.length) {
        best = model;
        bestInliers = inliers;
      }
    }
    if (!best || bestInliers.length < Math.max(10, 0.3 * tracked)) return { ok: false, tracked, inliers: bestInliers.length };
    const fitted = similarity(a, b, bestInliers) ?? best;

    // Reference-to-current transform from the fit, then refined on reprojection.
    // T_RC maps current-frame points into the reference frame.
    const rc = { rotation: fitted.rotation, translation: fitted.translation };
    let cr = inversePose({ rotation: rc.rotation, translation: rc.translation.map((v) => v / 1) });
    if (turn) cr = { rotation: conjugate(turn), translation: cr.translation };
    const X = bestInliers.map((i) => reference3d[i]);
    const observed = bestInliers.map((i) => pixels[i]);
    cr = refinePose(X, observed, camera, cr, !!turn);

    // The scale that makes this frame's depth agree with the tracked points.
    const matrix = rotationMatrix(cr.rotation);
    const ratios = [];
    const kept = [];
    for (const i of bestInliers) {
      const p = apply(matrix, reference3d[i]).map((v, k) => v + cr.translation[k]);
      const ex = camera.fx * p[0] / p[2] + camera.cx - pixels[i][0];
      const ey = camera.fy * p[1] / p[2] + camera.cy - pixels[i][1];
      if (Math.hypot(ex, ey) > 3) continue;
      ratios.push(p[2] / a[i][2]);
      // Carried into the next keyframe with its 3D position in this frame.
      kept.push({ pixel: pixels[i], point: p });
    }
    if (ratios.length < 10) return { ok: false, tracked, inliers: ratios.length };
    // Keep the correspondences of frames that turned enough to calibrate the
    // focal length from (see calibrateFocal).
    const angle = 2 * Math.acos(Math.min(1, Math.abs(cr.rotation.w)));
    if (angle > 0.035 && this.samples.length < 40) {
      const chosen = bestInliers.filter((_, k) => k % Math.ceil(bestInliers.length / 150) === 0);
      this.samples.push({
        from: chosen.map((i) => keyframePixels[i]),
        depth: chosen.map((i) => reference3d[i][2]),
        to: chosen.map((i) => pixels[i]),
        pose: cr, turn: turn && conjugate(turn), camera,
      });
    }
    return { ok: true, tracked, inliers: ratios.length, scale: median(ratios), motion: inversePose(cr), kept };
  }

  // The focal length the depth network reports is often 15-30% off, which
  // shortens or lengthens every turn and stretches every depth map sideways.
  // Turning frames constrain it: rotation moves points near the image edge
  // further than points near the centre, by an amount set by the focal length.
  // With gyro rotations the constraint is direct. Returns the factor to
  // multiply the focal length by, or null without enough turning frames.
  calibrateFocal() {
    if (this.samples.length < 6) return null;
    const cost = (factor) => {
      let total = 0, count = 0;
      for (const { from, depth, to, pose, turn, camera } of this.samples) {
        const f = { ...camera, fx: camera.fx * factor, fy: camera.fy * factor };
        const X = from.map(([x, y], i) => [(x - f.cx) * depth[i] / f.fx, (y - f.cy) * depth[i] / f.fy, depth[i]]);
        const start = turn ? { rotation: turn, translation: pose.translation } : pose;
        const fitted = refinePose(X, to, f, start, !!turn);
        const matrix = rotationMatrix(fitted.rotation);
        for (let i = 0; i < X.length; i++) {
          const p = apply(matrix, X[i]).map((v, k) => v + fitted.translation[k]);
          const e = Math.hypot(f.fx * p[0] / p[2] + f.cx - to[i][0], f.fy * p[1] / p[2] + f.cy - to[i][1]);
          total += e < 2 ? e * e : 4 * e - 4;
          count++;
        }
      }
      return total / count;
    };
    let best = 1, bestCost = Infinity;
    for (let factor = 0.6; factor <= 1.81; factor += 0.04) {
      const c = cost(factor);
      if (c < bestCost) { best = factor; bestCost = c; }
    }
    let low = best - 0.04, high = best + 0.04;
    for (let i = 0; i < 12; i++) {
      const m1 = low + (high - low) / 3, m2 = high - (high - low) / 3;
      if (cost(m1) < cost(m2)) high = m2; else low = m1;
    }
    return (low + high) / 2;
  }
}

// Loop closure from image features.
//
// Two submaps from the same view, minutes apart, are each a cloud of estimated
// depth whose scale and shape errors differ, so aligning the clouds with ICP
// settles wherever their errors balance, often tens of centimetres from the
// true pose. Matching the images instead gives the relative pose from the same
// kind of reprojection fit visual odometry uses, which is far more precise.
// Corners are described with BRIEF (binary intensity tests around a corner,
// compared by Hamming distance). The phone stays upright and the views of a
// loop are taken from similar distances, so no rotation or scale invariance
// is needed.

const patchRadius = 15;
const pattern = (() => {
  // A fixed set of 256 point pairs, normally distributed around the corner.
  let seed = 7;
  const random = () => ((seed = (seed * 16807) % 2147483647) / 2147483647);
  const gaussian = () => Math.sqrt(-2 * Math.log(random() + 1e-12)) * Math.cos(2 * Math.PI * random());
  const clamp = (value) => Math.max(-patchRadius, Math.min(patchRadius, Math.round(value * patchRadius / 2.5)));
  return Int8Array.from(Array.from({ length: 1024 }, () => clamp(gaussian())));
})();

function boxBlur(gray, width, height) {
  const horizontal = new Float32Array(gray.length), result = new Float32Array(gray.length);
  for (let y = 0; y < height; y++) {
    for (let x = 2; x < width - 2; x++) {
      const i = y * width + x;
      horizontal[i] = gray[i - 2] + gray[i - 1] + gray[i] + gray[i + 1] + gray[i + 2];
    }
  }
  for (let y = 2; y < height - 2; y++) {
    for (let x = 0; x < width; x++) {
      const i = y * width + x;
      result[i] = horizontal[i - 2 * width] + horizontal[i - width] + horizontal[i] + horizontal[i + width] + horizontal[i + 2 * width];
    }
  }
  return result;
}

// Describes one frame for loop matching. `depth` is multiplied by `scale`;
// intrinsics are normalized as estimateCloud returns them.
export function describeView({ gray, depth, width, height, intrinsics, scale = 1 }) {
  const camera = {
    fx: intrinsics.fx * width, fy: intrinsics.fy * height,
    cx: intrinsics.cx * width - 0.5, cy: intrinsics.cy * height - 0.5,
  };
  const smooth = boxBlur(gray, width, height);
  const border = patchRadius + 3;
  const corners = detect({ data: gray, width, height }, depth, [], 10)
    .filter(([x, y]) => x >= border && y >= border && x < width - border && y < height - border);
  const pixels = [], points = [];
  const descriptors = new Uint32Array(corners.length * 8);
  for (const [x, y] of corners) {
    const z = depth[y * width + x] * scale;
    const offset = pixels.length * 8;
    for (let bit = 0; bit < 256; bit++) {
      const a = (y + pattern[4 * bit + 1]) * width + x + pattern[4 * bit];
      const b = (y + pattern[4 * bit + 3]) * width + x + pattern[4 * bit + 2];
      if (smooth[a] < smooth[b]) descriptors[offset + (bit >> 5)] |= 1 << (bit & 31);
    }
    pixels.push([x, y]);
    points.push([(x - camera.cx) * z / camera.fx, (y - camera.cy) * z / camera.fy, z]);
  }
  return { pixels, points, descriptors: descriptors.slice(0, pixels.length * 8), camera };
}

function popcount(value) {
  value -= (value >>> 1) & 0x55555555;
  value = (value & 0x33333333) + ((value >>> 2) & 0x33333333);
  return (((value + (value >>> 4)) & 0x0f0f0f0f) * 0x01010101) >>> 24;
}

function nearest(from, i, to) {
  let best = 257, second = 257, at = -1;
  for (let j = 0; j < to.length / 8; j++) {
    let distance = 0;
    for (let k = 0; k < 8; k++) distance += popcount(from[8 * i + k] ^ to[8 * j + k]);
    if (distance < best) { second = best; best = distance; at = j; } else if (distance < second) second = distance;
  }
  return { at, best, second };
}

// Relative pose between two described views: the transform that maps points in
// view a's camera frame into view b's, with the number of verified matches, or
// null when the views do not match.
export function matchViews(a, b, random = Math.random) {
  const matches = [];
  for (let i = 0; i < a.pixels.length; i++) {
    const forward = nearest(a.descriptors, i, b.descriptors);
    if (forward.best > 64 || forward.best > 0.8 * forward.second) continue;
    if (nearest(b.descriptors, forward.at, a.descriptors).at !== i) continue;
    matches.push([i, forward.at]);
  }
  if (matches.length < 20) return null;
  const pa = matches.map(([i]) => a.points[i]), pb = matches.map(([, j]) => b.points[j]);
  let best = null, bestInliers = [];
  for (let iteration = 0; iteration < 300; iteration++) {
    const pick = new Set();
    while (pick.size < 3) pick.add(Math.floor(random() * matches.length));
    const model = similarity(pa, pb, [...pick]);
    if (!model || model.scale < 0.6 || model.scale > 1.6) continue;
    const inliers = [];
    for (let i = 0; i < matches.length; i++) {
      const p = apply(model.matrix, pa[i]).map((v, k) => model.scale * v + model.translation[k]);
      if (Math.hypot(p[0] - pb[i][0], p[1] - pb[i][1], p[2] - pb[i][2]) < 0.05 + 0.08 * pb[i][2]) inliers.push(i);
    }
    if (inliers.length > bestInliers.length) {
      best = model;
      bestInliers = inliers;
    }
  }
  if (!best || bestInliers.length < 20) return null;
  // The pose comes from reprojecting view a's points into view b's image, so
  // view b's depth errors do not enter it.
  const fitted = similarity(pa, pb, bestInliers) ?? best;
  let pose = { rotation: fitted.rotation, translation: fitted.translation };
  const X = bestInliers.map((i) => pa[i]);
  const observed = bestInliers.map((i) => b.pixels[matches[i][1]]);
  pose = refinePose(X, observed, b.camera, pose, false);
  const matrix = rotationMatrix(pose.rotation);
  let inliers = 0;
  for (let k = 0; k < X.length; k++) {
    const p = apply(matrix, X[k]).map((v, axis) => v + pose.translation[axis]);
    if (p[2] < 0.05) continue;
    const ex = b.camera.fx * p[0] / p[2] + b.camera.cx - observed[k][0];
    const ey = b.camera.fy * p[1] / p[2] + b.camera.cy - observed[k][1];
    if (Math.hypot(ex, ey) < 4) inliers++;
  }
  if (inliers < 20 || inliers < 0.3 * matches.length) return null;
  return { pose, inliers, matches: matches.length };
}
