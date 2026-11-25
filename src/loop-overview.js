export function drawLoopOverview(canvas, before, after, correction) {
  const context = canvas.getContext('2d');
  const { width, height } = canvas;
  const positions = [...before, ...after].map((pose) => pose.translation);
  const x = positions.map((position) => position[0]);
  const z = positions.map((position) => position[2]);
  const minX = Math.min(...x);
  const maxX = Math.max(...x);
  const minZ = Math.min(...z);
  const maxZ = Math.max(...z);
  const scale = Math.min((width - 31) / (maxX - minX), (height - 27) / (maxZ - minZ));
  const project = ([px, , pz]) => [
    width / 2 + (px - (minX + maxX) / 2) * scale,
    height / 2 + (pz - (minZ + maxZ) / 2) * scale,
  ];

  const start = project(before[0].translation);
  const driftedEnd = project(before.at(-1).translation);
  const correctedEnd = project(after.at(-1).translation);
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
  line([start, driftedEnd], amber, 0.7, true);
  marker(driftedEnd, amber, 5);
  marker(start, '#f2f7ff', 6);
  if (correction > 0) {
    marker(driftedEnd.map((value, axis) => value + (correctedEnd[axis] - value) * correction), mint, 3.7);
  }
}
