// Jacobi diagonalization of the small symmetric matrices used in registration.
export function extremeEigenvector(matrix, size, largest) {
  const vectors = new Float64Array(size * size);
  for (let i = 0; i < size; i++) vectors[i * size + i] = 1;
  const tolerance = Math.max(1e-20, ...matrix.map(Math.abs)) * 1e-12;
  for (let sweep = 0; sweep < 16; sweep++) {
    let converged = true;
    for (let p = 0; p < size; p++) {
      for (let q = p + 1; q < size; q++) {
        const off = matrix[p * size + q];
        if (Math.abs(off) <= tolerance) continue;
        converged = false;
        const pp = p * size + p;
        const qq = q * size + q;
        const tau = (matrix[qq] - matrix[pp]) / (2 * off);
        const t = (tau < 0 ? -1 : 1) / (Math.abs(tau) + Math.hypot(1, tau));
        const c = 1 / Math.sqrt(1 + t * t);
        const s = t * c;
        matrix[pp] -= t * off;
        matrix[qq] += t * off;
        matrix[p * size + q] = matrix[q * size + p] = 0;
        for (let i = 0; i < size; i++) {
          const ip = i * size + p;
          const iq = i * size + q;
          if (i !== p && i !== q) {
            const a = matrix[ip];
            const b = matrix[iq];
            matrix[ip] = matrix[p * size + i] = c * a - s * b;
            matrix[iq] = matrix[q * size + i] = s * a + c * b;
          }
          const a = vectors[ip];
          const b = vectors[iq];
          vectors[ip] = c * a - s * b;
          vectors[iq] = s * a + c * b;
        }
      }
    }
    if (converged) break;
  }
  let element = 0;
  for (let i = 1; i < size; i++) {
    const delta = matrix[i * size + i] - matrix[element * size + element];
    if (largest ? delta > 0 : delta < 0) element = i;
  }
  return Array.from({ length: size }, (_, i) => vectors[i * size + element]);
}
