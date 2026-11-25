import { MogePost } from '../vendor/moge_post.js';

const ortUrl = 'https://cdn.jsdelivr.net/npm/onnxruntime-web@1.20.1/dist/';
const modelUrl = 'https://huggingface.co/Ruicheng/moge-2-vits-normal-onnx/resolve/e50ffda41565591092adea54c6ac83d6212e1e23/model.onnx';
const modelSize = 140_852_051;
const modelCache = 'moge-2-weights-v1';
const tokens = 352;

export function depthInputSize(width, height) {
  const aspect = width / height;
  return {
    width: 14 * Math.max(1, Math.round(Math.sqrt(tokens * aspect))),
    height: 14 * Math.max(1, Math.round(Math.sqrt(tokens / aspect))),
  };
}

async function openModelCache() {
  return globalThis.caches?.open(modelCache).catch(() => null);
}

export async function removeCachedDepthModel() {
  const cache = await openModelCache();
  await cache?.delete(modelUrl).catch(() => {});
}

async function fetchDepthModel(progress) {
  progress('Downloading depth model…');
  const response = await fetch(modelUrl, { credentials: 'omit', cache: 'no-store', priority: 'low' });
  if (!response.ok || !response.body) {
    throw new Error('The depth model download failed (HTTP ' + response.status + ').');
  }
  let received = 0;
  let reportedPercent = -1;
  let lastNotice = 0;
  const body = response.body.pipeThrough(new TransformStream({
    transform(chunk, controller) {
      received += chunk.byteLength;
      if (received > modelSize) throw new Error('The depth model download has the wrong size.');
      const percent = Math.floor(100 * received / modelSize);
      const now = performance.now();
      if (percent !== reportedPercent || now - lastNotice > 2000) {
        progress('Downloading depth model · ' + percent + '%');
        reportedPercent = percent;
        lastNotice = now;
      }
      controller.enqueue(chunk);
    },
    flush() {
      if (received !== modelSize) throw new Error('The depth model download was interrupted.');
    },
  }));
  return new Response(body, { headers: { 'Content-Length': String(modelSize) } });
}

export async function downloadDepthModel(progress, refresh = false) {
  const cache = await openModelCache();
  const saved = refresh ? null : await cache?.match(modelUrl).catch(() => null);
  if (saved && Number(saved.headers.get('content-length')) === modelSize) {
    progress('Depth model downloaded');
    return saved;
  }
  if (saved) await cache.delete(modelUrl).catch(() => {});

  let response = await fetchDepthModel(progress);
  if (cache) {
    try {
      // Cache the stream directly so preloading does not hold 141 MB of model
      // weights in JavaScript while the visitor is still browsing.
      await cache.put(modelUrl, response);
      const cached = await cache.match(modelUrl);
      if (cached) {
        progress('Depth model downloaded');
        return cached;
      }
    } catch (error) {
      console.warn('Could not save the depth model in browser storage.', error);
    }
    if (response.bodyUsed || response.body.locked) response = await fetchDepthModel(progress);
  }
  const bytes = await response.arrayBuffer();
  progress('Depth model downloaded');
  return bytes;
}

// Cross-origin isolation (see coi-sw.js) allows multi-threaded WebAssembly, which
// is about 3.5 times faster than one thread on a 4-core CPU.
export function inferenceThreads(singleThread = false) {
  if (singleThread || !globalThis.crossOriginIsolated) return 1;
  return Math.max(1, Math.min(4, navigator.hardwareConcurrency || 1));
}

export async function loadDepthModel(weights, progress, forceWasm, singleThread = false) {
  progress('Reading depth model weights…');
  const bytes = new Uint8Array(weights instanceof Response ? await weights.arrayBuffer() : weights);
  if (bytes.byteLength !== modelSize) throw new Error('The saved depth model is incomplete.');
  const providers = !forceWasm && navigator.gpu ? ['webgpu', 'wasm'] : ['wasm'];
  const threads = inferenceThreads(singleThread);
  let cause;
  for (const provider of providers) {
    try {
      progress(provider === 'wasm'
        ? 'Preparing CPU depth model' + (threads > 1 ? ' on ' + threads + ' threads…' : '…')
        : 'Preparing GPU depth model…');
      const ort = await import(ortUrl + (provider === 'wasm' ? 'ort.wasm.min.mjs' : 'ort.webgpu.min.mjs'));
      ort.env.wasm.wasmPaths = ortUrl;
      ort.env.wasm.numThreads = threads;
      const session = await ort.InferenceSession.create(bytes, {
        executionProviders: [provider],
        graphOptimizationLevel: 'all',
      });
      progress('Depth model ready' + (provider === 'wasm' ? ' · CPU · ' + threads + (threads > 1 ? ' threads' : ' thread') : ' · GPU'));
      return { session, ort, provider };
    } catch (error) {
      cause = error;
    }
  }
  throw new Error('This browser could not initialize the depth model.', { cause });
}

export async function estimateCloud({ session, ort }, width, height, rgba, calibration) {
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
    const { fx, fy, cx, cy } = calibration ?? metric.intrinsics;
    const points = new Float32Array(size * 3);
    const colors = new Uint8Array(size * 3);
    let offset = 0;
    for (let row = 0; row < height; row += 4) {
      for (let column = 0; column < width; column += 4) {
        const i = row * width + column;
        const z = metric.depth[i];
        if (!mask[i] || z < 0.16 || z > 12 || !Number.isFinite(z)) continue;
        const x = ((column + 0.5) / width - cx) * z / fx;
        const y = ((row + 0.5) / height - cy) * z / fy;
        if (!Number.isFinite(x) || !Number.isFinite(y)) continue;
        points[offset] = x;
        points[offset + 1] = y;
        points[offset + 2] = z;
        colors[offset] = rgba[4 * i];
        colors[offset + 1] = rgba[4 * i + 1];
        colors[offset + 2] = rgba[4 * i + 2];
        offset += 3;
      }
    }
    return { points: points.slice(0, offset), colors: colors.slice(0, offset), intrinsics: { fx, fy, cx, cy } };
  } finally {
    for (const tensor of Object.values(outputs)) tensor.dispose();
    for (const tensor of Object.values(feeds)) tensor.dispose();
  }
}
