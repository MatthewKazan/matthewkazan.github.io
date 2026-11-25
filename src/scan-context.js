// Port of ScanContext.compute_scan_context_descriptor with the project's
// radial bins, maximum Z pooling and flattened cosine similarity.
export function scanContext(points, angleBins = 60, radiusBins = 20, maxRange = 80) {
  const descriptor = new Float64Array(radiusBins * angleBins).fill(-Infinity);
  for (let i = 0; i < points.length; i += 3) {
    const x = points[i], y = points[i + 1], z = points[i + 2];
    const radius = Math.sqrt(x * x + y * y);
    if (radius >= maxRange) continue;
    const angle = (Math.atan2(y, x) + Math.PI) / (2 * Math.PI);
    const sector = Math.min(angleBins - 1, Math.floor(angle * angleBins));
    const ring = Math.min(radiusBins - 1, Math.floor(radius / maxRange * radiusBins));
    const bin = ring * angleBins + sector;
    if (z > descriptor[bin]) descriptor[bin] = z;
  }
  let norm = 0;
  for (let i = 0; i < descriptor.length; i++) {
    if (descriptor[i] === -Infinity) descriptor[i] = 0;
    norm += descriptor[i] * descriptor[i];
  }
  norm = Math.sqrt(norm) + 1e-8;
  for (let i = 0; i < descriptor.length; i++) descriptor[i] /= norm;
  return descriptor;
}

export function cosineSimilarity(a, b) {
  let dot = 0, normA = 0, normB = 0;
  for (let i = 0; i < a.length; i++) {
    dot += a[i] * b[i];
    normA += a[i] * a[i];
    normB += b[i] * b[i];
  }
  return normA > 0 && normB > 0 ? dot / Math.sqrt(normA * normB) : 0;
}
