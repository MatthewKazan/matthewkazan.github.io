export function drawLoopOverview(canvas, before, after, loop, correction) {
  const context = canvas.getContext('2d');
  const { width, height } = canvas;
  const positions = [...before, ...after].map((pose) => pose.translation);
  const x = positions.map((position) => position[0]);
  const z = positions.map((position) => position[2]);
  const minX = Math.min(...x);
  const maxX = Math.max(...x);
  const minZ = Math.min(...z);
  const maxZ = Math.max(...z);
  const scale = Math.min((width - 31) / Math.max(0.01, maxX - minX), (height - 27) / Math.max(0.01, maxZ - minZ));
  const project = ([px, , pz]) => [
    width / 2 + (px - (minX + maxX) / 2) * scale,
    height / 2 + (pz - (minZ + maxZ) / 2) * scale,
  ];

  const older = project(before[loop.fromFrame].translation);
  const newer = project(before[loop.toFrame].translation);
  const correctedOlder = project(after[loop.fromFrame].translation);
  const correctedNewer = project(after[loop.toFrame].translation);
  context.clearRect(0, 0, width, height);
  context.lineJoin = 'round';
  context.lineCap = 'round';

  function line(points, color, opacity, dashed = false) {
    context.beginPath();
    context.moveTo(...points[0]);
    for (const point of points.slice(1)) context.lineTo(...point);
    context.strokeStyle = color;
    context.globalAlpha = opacity;
    context.lineWidth = dashed ? 3 : 4;
    context.setLineDash(dashed ? [7, 6] : []);
    context.stroke();
  }

  function marker(point, color, radius) {
    context.beginPath();
    context.arc(...point, radius, 0, 2 * Math.PI);
    context.globalAlpha = 1;
    context.fillStyle = color;
    context.fill();
  }

  const amber = '#ffc18b';
  const mint = '#85ebcf';
  line(before.map(({ translation }) => project(translation)), amber, 0.85 - 0.35 * correction, true);
  line(after.map(({ translation }) => project(translation)), mint, 0.48 + 0.52 * correction);
  const matchedA = older.map((value, axis) => value + (correctedOlder[axis] - value) * correction);
  const matchedB = newer.map((value, axis) => value + (correctedNewer[axis] - value) * correction);
  line([matchedA, matchedB], '#f2f7ff', 0.75, true);
  marker(matchedA, '#f2f7ff', 5);
  marker(matchedB, correction > 0.5 ? mint : amber, 5);
}
