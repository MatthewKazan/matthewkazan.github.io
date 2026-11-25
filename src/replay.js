// A cutaway lab with analytic depth, so the public demo needs no private recording.
const surfaces = [];

function box(x, y, z, width, height, length, color, pattern = '') {
  surfaces.push({
    min: [x - width / 2, y - height / 2, z - length / 2],
    max: [x + width / 2, y + height / 2, z + length / 2],
    color,
    pattern,
  });
}

const slate = [115, 157, 202];
const steel = [141, 191, 218];
const mint = [105, 232, 210];
const wood = [237, 176, 97];
const white = [202, 220, 235];

box(0, -0.09, 0, 8.35, 0.18, 6.45, [85, 129, 177], 'floor');
box(0, 1.3, -3.2, 8.35, 2.6, 0.12, [133, 151, 211], 'wall');
box(-4.16, 1.3, 0, 0.12, 2.6, 6.45, [98, 156, 198], 'wall');
box(0, 0.09, -3.11, 8.35, 0.14, 0.055, steel);
box(-4.07, 0.09, 0, 0.055, 0.14, 6.45, steel);
box(1.15, 1.62, -3.09, 3.25, 1.22, 0.05, white);
box(1.15, 1.62, -3.054, 3.12, 1.09, 0.025, slate, 'board');
box(-4.04, 1.62, 0.05, 0.13, 1.23, 2.8, mint);
box(-3.96, 1.62, 0.05, 0.035, 1.12, 2.65, [80, 183, 225], 'window');
for (const z of [-0.84, 0.05, 0.94]) box(-3.931, 1.62, z, 0.025, 1.1, 0.036, white);
box(-3.93, 1.62, 0.05, 0.026, 0.036, 2.66, white);

// Central workbench, instruments, shelving and floor markings.
box(-0.04, 0.98, -0.17, 2.55, 0.12, 1.4, wood);
for (const x of [-1.19, 1.1]) {
  for (const z of [-0.72, 0.38]) box(x, 0.46, z, 0.11, 0.92, 0.11, steel);
}
box(-0.04, 0.34, -0.17, 2.3, 0.07, 1.2, [157, 133, 111]);
box(-0.5, 1.11, -0.59, 0.72, 0.16, 0.38, slate);
box(-0.5, 1.4, -0.67, 0.87, 0.56, 0.13, steel);
box(-0.5, 1.42, -0.596, 0.77, 0.46, 0.019, [59, 218, 186], 'board');
box(-0.5, 1.06, -0.19, 0.63, 0.035, 0.21, white);
box(0.57, 1.16, 0.09, 0.47, 0.25, 0.51, [153, 187, 224]);
box(0.57, 1.2, 0.35, 0.29, 0.085, 0.019, mint);
box(0.57, 1.06, -0.48, 0.69, 0.037, 0.24, [230, 139, 112]);
box(-0.73, 0.5, 1.15, 0.68, 0.1, 0.65, [159, 141, 235]);
box(-0.73, 0.78, 1.43, 0.68, 0.6, 0.12, [159, 141, 235]);
box(-0.73, 0.22, 1.15, 0.12, 0.44, 0.12, steel);
box(-0.73, 0.045, 1.15, 0.65, 0.09, 0.51, slate);

for (const x of [-3.15, -1.76]) box(x, 0.94, -2.58, 0.09, 1.88, 0.67, steel);
for (const y of [0.13, 0.64, 1.21, 1.83]) box(-2.45, y, -2.58, 1.49, 0.07, 0.72, steel);
box(-2.78, 1.43, -2.58, 0.46, 0.35, 0.49, wood);
box(-2.13, 1.39, -2.58, 0.55, 0.26, 0.48, mint);
box(-2.63, 0.85, -2.58, 0.73, 0.35, 0.49, slate);
box(-2.13, 0.34, -2.58, 0.49, 0.34, 0.5, [177, 155, 234]);
box(3.13, 0.52, -2.62, 1.34, 1.04, 0.85, [104, 180, 201], 'drawers');
box(3.13, 1.07, -2.62, 1.45, 0.09, 0.9, white);
box(-2.83, 0.37, 1.68, 1.05, 0.74, 0.68, steel, 'drawers');
box(-2.83, 0.77, 1.68, 1.1, 0.065, 0.73, mint);
box(2.86, 0.23, 1.72, 0.81, 0.46, 0.75, [179, 143, 234], 'drawers');
box(2.86, 0.62, 1.72, 0.73, 0.32, 0.67, [110, 208, 212], 'drawers');
box(2.55, 0.01, 0.84, 1.25, 0.019, 2.65, [117, 163, 208], 'rug');

function rangeForRay(box, origin, reverseDirection, nearest) {
  let enter = 0.04;
  let exit = nearest;
  for (let axis = 0; axis < 3; axis++) {
    const low = (box.min[axis] - origin[axis]) * reverseDirection[axis];
    const high = (box.max[axis] - origin[axis]) * reverseDirection[axis];
    enter = Math.max(enter, Math.min(low, high));
    exit = Math.min(exit, Math.max(low, high));
    if (enter > exit) return Infinity;
  }
  return enter;
}

function colorAt(surface, x, y, z, column, row, frame) {
  let shade = 0.76 + 0.17 * ((column * 19 + row * 7 + frame * 11) % 21) / 20;
  const { pattern } = surface;
  if (pattern === 'floor') {
    const seam = Math.min(Math.abs(x - Math.round(x)), Math.abs(z - Math.round(z)));
    shade *= seam < 0.022 ? 1.25 : 0.85;
  } else if (pattern === 'board' || pattern === 'drawers') {
    shade *= Math.abs(y * 12 - Math.round(y * 12)) < 0.1 ? 1.31 : 0.93;
  } else if (pattern === 'window' || pattern === 'wall' || pattern === 'rug') {
    shade *= 0.85 + 0.18 * Math.sin((x + z) * 12 + y * 6) ** 2;
  } else if (Math.abs(y - surface.max[1]) < 0.005) {
    shade *= 1.1;
  }
  return surface.color.map((value) => Math.min(255, Math.round(value * shade)));
}

function cameraPose(angle, drift = 0) {
  const x = 2.88 * Math.sin(angle);
  const z = 2.11 * Math.cos(angle);
  const yaw = Math.atan2(x + 0.25, z + 0.25) + 0.18 * drift;
  return {
    yaw,
    rotation: { w: Math.cos(yaw / 2), x: 0, y: Math.sin(yaw / 2), z: 0 },
    translation: [x + 0.82 * drift, 1.48 + 0.18 * drift, z - 0.62 * drift],
  };
}

function sampleDepth(camera, index) {
  const width = 94;
  const height = 68;
  const points = new Float32Array(width * height * 3);
  const colors = new Uint8Array(points.length);
  const c = Math.cos(camera.yaw);
  const s = Math.sin(camera.yaw);
  const origin = camera.translation;
  let used = 0;

  for (let row = 0; row < height; row++) {
    for (let column = 0; column < width; column++) {
      const localX = ((column + 0.5) / width - 0.5) * 1.99;
      const localY = (0.5 - (row + 0.5) / height) * 1.49 - 0.22;
      const worldX = c * localX - s;
      const worldZ = -s * localX - c;
      const reverse = [1 / (worldX || 1e-10), 1 / (localY || 1e-10), 1 / (worldZ || 1e-10)];
      let nearest = 12;
      let surface;
      for (const candidate of surfaces) {
        const range = rangeForRay(candidate, origin, reverse, nearest);
        if (range < nearest) {
          nearest = range;
          surface = candidate;
        }
      }
      if (!surface) continue;
      points[used] = localX * nearest;
      points[used + 1] = localY * nearest;
      points[used + 2] = -nearest;
      colors.set(colorAt(surface, origin[0] + worldX * nearest, origin[1] + localY * nearest,
        origin[2] + worldZ * nearest, column, row, index), used);
      used += 3;
    }
  }
  return { points: points.subarray(0, used), colors: colors.subarray(0, used) };
}

export const replayLength = 30;

export function makeReplay() {
  const frames = [];
  const ideal = [];
  const initial = [];
  for (let i = 0; i < replayLength; i++) {
    const fraction = i / (replayLength - 1);
    const angle = 0.37 + fraction * Math.PI * 2;
    const camera = cameraPose(angle);
    ideal.push(camera);
    initial.push(cameraPose(angle, fraction ** 1.25));
    frames.push(sampleDepth(camera, i));
  }
  return { frames, ideal, initial };
}
