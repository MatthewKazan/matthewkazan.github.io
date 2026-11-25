import { MogePost } from '../vendor/moge_post.js';

const ortUrl = 'https://cdn.jsdelivr.net/npm/onnxruntime-web@1.20.1/dist/';
const modelUrl = 'https://huggingface.co/Ruicheng/moge-2-vits-normal-onnx/resolve/e50ffda41565591092adea54c6ac83d6212e1e23/model.onnx';
const tokens = 352;

export function depthInputSize(width, height) {
  const aspect = width / height;
  return {
    width: 14 * Math.max(1, Math.round(Math.sqrt(tokens * aspect))),
    height: 14 * Math.max(1, Math.round(Math.sqrt(tokens / aspect))),
  };
}

export async function downloadDepthModel(progress) {
  progress('Downloading depth model…');
  const cache = await globalThis.caches?.open('moge-2-weights-v1').catch(() => null);
  const cached = await cache?.match(modelUrl).catch(() => null);
  if (cached) {
    const bytes = new Uint8Array(await cached.arrayBuffer());
    progress('Depth model downloaded');
    return bytes;
  }
  const response = await fetch(modelUrl, { credentials: 'omit', priority: 'low' });
  if (!response.ok) throw new Error('The depth model could not be downloaded.');
  // Stream into the cache alongside the download so opening Camera can reuse
  // the same request, and a return visit can reuse the downloaded weights.
  if (cache) cache.put(modelUrl, response.clone()).catch(() => {});
  const size = Number(response.headers.get('content-length'));
  let bytes;
  if (size > 0 && response.body) {
    bytes = new Uint8Array(size);
    const reader = response.body.getReader();
    let offset = 0;
    let reportedPercent = -10;
    while (true) {
      const { value, done } = await reader.read();
      if (done) break;
      bytes.set(value, offset);
      offset += value.length;
      const percent = Math.floor(100 * offset / size);
      if (percent >= reportedPercent + 10) {
        progress('Downloading depth model · ' + percent + '%');
        reportedPercent = percent;
      }
    }
    if (offset !== size) throw new Error('The depth model download was interrupted.');
  } else {
    bytes = new Uint8Array(await response.arrayBuffer());
  }

  progress('Depth model downloaded');
  return bytes;
}

export async function loadDepthModel(bytes, progress) {
  progress('Preparing depth model…');
  const ort = await import(ortUrl + 'ort.webgpu.min.mjs');
  ort.env.wasm.wasmPaths = ortUrl;
  ort.env.wasm.numThreads = 1;
  const providers = navigator.gpu ? ['webgpu', 'wasm'] : ['wasm'];
  let cause;
  for (const provider of providers) {
    try {
      const session = await ort.InferenceSession.create(bytes, {
        executionProviders: [provider],
        graphOptimizationLevel: 'all',
      });
      progress('Depth model ready');
      return { session, ort };
    } catch (error) {
      cause = error;
    }
  }
  throw new Error('This browser could not initialize the depth model.', { cause });
}

export async function estimateCloud({ session, ort }, width, height, rgba) {
  const size = width * height;
  const rgb = new Float32Array(size * 3);
  for (let p = 0; p < size; p++) {
    rgb[p] = rgba[4 * p] / 255;
    rgb[p + size] = rgba[4 * p + 1] / 255;
    rgb[p + 2 * size] = rgba[4 * p + 2] / 255;
  }
  const image = new ort.Tensor('float32', rgb, [1, 3, height, width]);
  const feeds = { [session.inputNames[0]]: image };
  if (session.inputNames.includes('num_tokens')) {
    feeds.num_tokens = new ort.Tensor('int64', BigInt64Array.of(BigInt(tokens)), []);
  }
  let outputs = {};
  try {
    outputs = await session.run(feeds);
    const output = (pattern, position) =>
      outputs[session.outputNames.find((name) => pattern.test(name)) ?? session.outputNames[position]];
    const pointsTensor = output(/point/i, 0);
    const maskTensor = output(/mask/i, 2);
    const scaleTensor = output(/scale/i, 3);
    if (pointsTensor?.data.length !== size * 3 || maskTensor?.data.length !== size || !scaleTensor) {
      throw new Error('Unexpected depth model outputs: ' + session.outputNames.join(', '));
    }
    const metric = MogePost.process({
      points: pointsTensor.data,
      mask: maskTensor.data,
      metricScale: Number(scaleTensor.data[0]),
      width,
      height,
    });
    const mask = MogePost.cleanDepthMask(metric.depth, metric.mask, width, height, 0.075, true);
    const points = new Float32Array(size * 3);
    const colors = new Uint8Array(size * 3);
    let offset = 0;
    for (let row = 0; row < height; row += 4) {
      for (let column = 0; column < width; column += 4) {
        const i = row * width + column;
        const z = metric.depth[i];
        if (!mask[i] || z < 0.16 || z > 12 || !Number.isFinite(z)) continue;
        const x = metric.points[3 * i];
        const y = metric.points[3 * i + 1];
        if (!Number.isFinite(x) || !Number.isFinite(y)) continue;
        points[offset] = x;
        points[offset + 1] = -y;
        points[offset + 2] = -z;
        colors[offset] = rgba[4 * i];
        colors[offset + 1] = rgba[4 * i + 1];
        colors[offset + 2] = rgba[4 * i + 2];
        offset += 3;
      }
    }
    return { points: points.slice(0, offset), colors: colors.slice(0, offset) };
  } finally {
    for (const tensor of Object.values(outputs)) tensor.dispose();
    for (const tensor of Object.values(feeds)) tensor.dispose();
  }
}
