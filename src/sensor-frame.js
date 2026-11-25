// The sensor has +Z forward and +Y down. The viewer uses -Z and +Y.
export function viewingPose({ rotation: { w, x, y, z }, translation: [tx, ty, tz] }) {
  return { rotation: { w, x, y: -y, z: -z }, translation: [tx, -ty, -tz] };
}

export function viewingPoints(points) {
  const xyz = new Float32Array(points.length);
  for (let i = 0; i < xyz.length; i += 3) {
    xyz[i] = points[i];
    xyz[i + 1] = -points[i + 1];
    xyz[i + 2] = -points[i + 2];
  }
  return xyz;
}
