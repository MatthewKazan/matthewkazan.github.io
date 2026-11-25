import assert from 'node:assert/strict';
import test from 'node:test';

async function cameraFixture(t) {
  const ids = [
    'camera-preview', 'camera-prompt', 'depth-status', 'record-camera',
    'stop-camera', 'recording-rate', 'camera-frame-count', 'camera-feedback',
    'loop-comparison', 'loop-overview', 'loop-before', 'loop-after', 'loop-evidence', 'loop-shift',
  ];
  const elements = new Map(ids.map((id) => [id, new EventTarget()]));
  const video = elements.get('camera-preview');
  const track = Object.assign(new EventTarget(), { stop: t.mock.fn() });
  const stream = { getTracks: () => [track], getVideoTracks: () => [track] };
  Object.assign(video, {
    currentTime: 0, videoWidth: 640, videoHeight: 480, readyState: 2, play: async () => {},
  });
  elements.get('recording-rate').value = 'auto';
  elements.get('stop-camera').focus = () => {};

  class Canvas {
    getContext() {
      return {
        drawImage: (image) => { this.pixel = image === video ? video.currentTime : Number(image.src); },
        getImageData: () => ({ data: Uint8ClampedArray.of(this.pixel, 0, 0, 255) }),
      };
    }
    toDataURL() {
      return String(this.pixel);
    }
  }

  class CameraWorker extends EventTarget {
    constructor() {
      super();
      worker = this;
    }
    postMessage(message) {
      if (message.type === 'frame') sent.push(message.rgba[0]);
      if (message.type === 'start') this.scanId = message.id;
      if (message.type === 'preload') this.options = message;
    }
    terminate() {
      this.terminated = true;
    }
    reply(type, values = {}) {
      this.dispatchEvent(new MessageEvent('message', { data: { type, id: this.scanId, ...values } }));
    }
  }

  let worker;
  const sent = [];
  const timers = new Map();
  let lastTimer = 0;
  const document = Object.assign(new EventTarget(), {
    getElementById: (id) => elements.get(id),
    createElement: () => new Canvas(),
  });
  const globals = {
    document,
    navigator: {
      userAgent: 'Mozilla/5.0 (iPhone)',
      mediaDevices: { getUserMedia: async () => stream },
    },
    HTMLMediaElement: { HAVE_CURRENT_DATA: 2 },
    Image: class {
      naturalWidth = 280;
      naturalHeight = 210;
      async decode() {}
    },
    Worker: CameraWorker,
    setTimeout: (callback, ms) => {
      timers.set(++lastTimer, { callback, ms });
      return lastTimer;
    },
    clearTimeout: (id) => timers.delete(id),
  };
  for (const [name, value] of Object.entries(globals)) {
    const original = Object.getOwnPropertyDescriptor(globalThis, name);
    Object.defineProperty(globalThis, name, { configurable: true, value });
    t.after(() => {
      if (original) Object.defineProperty(globalThis, name, original);
      else delete globalThis[name];
    });
  }

  const { createCameraRecorder } = await import('../src/camera.js?v=13');
  const setState = t.mock.fn();
  const recorder = createCameraRecorder({
    viewer: { clear() {}, addFrame() {}, setVisible() { return 0; } },
    describePoints() {},
    setState,
  });
  const flush = () => new Promise((resolve) => setImmediate(resolve));
  const timer = (isWatchdog) => [...timers].find(([, { ms }]) => (ms > 10_000) === isWatchdog);
  const fire = (isWatchdog) => {
    const [id, { callback }] = timer(isWatchdog);
    timers.delete(id);
    callback();
  };
  const capture = (pixel) => {
    video.currentTime = pixel;
    fire(false);
  };
  const click = (id) => elements.get(id).dispatchEvent(new Event('click'));
  const counter = elements.get('camera-frame-count');

  recorder.open();
  click('record-camera');
  await flush();
  return {
    recorder, sent, track, counter, setState, capture, click, flush, fire, timer, document,
    get worker() { return worker; },
  };
}

test('sampled frames stay in order through depth startup, slow processing, and stop', async (t) => {
  const { recorder, sent, track, counter, setState, capture, click, flush, timer, worker } = await cameraFixture(t);
  assert.equal(worker.options.forceWasm, true);
  capture(10);
  capture(20);
  assert.deepEqual(sent, []);
  assert.match(counter.textContent, /2 captured · 2 queued/);

  worker.reply('ready');
  await flush();
  assert.deepEqual(sent, [10]);
  capture(30);
  capture(40);
  click('stop-camera');
  assert.equal(track.stop.mock.callCount(), 1);
  assert.equal(timer(false), undefined);
  assert.deepEqual(sent, [10]);
  assert.equal(setState.mock.calls.at(-1).arguments[0], 'Building map');

  for (let i = 0; i < 4; i++) {
    if (i % 2 === 0) {
      worker.reply('frame', { points: new Float32Array(3), colors: new Uint8Array(3), poses: [], loops: 0 });
    } else {
      worker.reply('skipped', { message: 'Test ICP rejected the depth.' });
    }
    await flush();
  }
  assert.deepEqual(sent, [10, 20, 30, 40]);
  assert.equal(counter.textContent, '4 captured · 0 queued · 2 mapped');
  assert.equal(setState.mock.calls.at(-1).arguments[0], 'Recording processed');
  recorder.stop();
});

test('a silent worker is replaced and retry can reload the model without dropping frames', async (t) => {
  const fixture = await cameraFixture(t);
  const { recorder, sent, track, counter, setState, capture, click, flush, fire, timer, document } = fixture;
  const first = fixture.worker;
  first.reply('ready');
  capture(10);
  capture(20);
  await flush();
  assert.deepEqual(sent, [10]);

  document.hidden = true;
  document.dispatchEvent(new Event('visibilitychange'));
  assert.equal(timer(true), undefined);
  document.hidden = false;
  document.dispatchEvent(new Event('visibilitychange'));
  fire(true);

  const second = fixture.worker;
  assert.equal(first.terminated, true);
  assert.notEqual(first, second);
  assert.equal(second.options.forceWasm, true);
  assert.equal(second.options.refresh, false);
  assert.equal(track.stop.mock.callCount(), 0);
  second.reply('ready');
  await flush();
  assert.deepEqual(sent, [10, 10]);
  assert.match(counter.textContent, /2 captured · 2 queued/);

  second.reply('error', { message: 'Depth runtime failed' });
  assert.equal(setState.mock.calls.at(-1).arguments[0], 'Mapping paused');
  assert.equal(track.stop.mock.callCount(), 1);
  click('record-camera');
  const third = fixture.worker;
  assert.notEqual(second, third);
  assert.equal(third.options.refresh, true);
  third.reply('ready');
  await flush();
  third.reply('skipped', { message: 'The two test frames show the same view.' });
  await flush();
  third.reply('skipped', { message: 'The two test frames show the same view.' });
  assert.deepEqual(sent, [10, 10, 10, 20]);
  assert.equal(counter.textContent, '2 captured · 0 queued · 0 mapped');
  recorder.stop();
});

test('device orientation maps to the rear camera frame used by the scans', async () => {
  const { cameraOrientation } = await import('../src/camera.js?v=13');
  const { rotationMatrix } = await import('../src/registration.js?v=6');
  const axes = (event, angle) => {
    const m = rotationMatrix(cameraOrientation(event, angle));
    return { right: [m[0], m[3], m[6]], down: [m[1], m[4], m[7]], forward: [m[2], m[5], m[8]] };
  };
  const near = (actual, expected) => actual.forEach((value, i) => assert.ok(Math.abs(value - expected[i]) < 1e-9));
  // Upright in portrait and facing north (world x east, y north, z up).
  let camera = axes({ alpha: 0, beta: 90, gamma: 0 });
  near(camera.forward, [0, 1, 0]);
  near(camera.right, [1, 0, 0]);
  near(camera.down, [0, 0, -1]);
  // Turning right lowers alpha and swings the lens toward the east.
  camera = axes({ alpha: -30, beta: 90, gamma: 0 });
  near(camera.forward, [0.5, Math.sqrt(3) / 2, 0]);
  // Flat on a table with the screen up, the lens faces the floor.
  camera = axes({ alpha: 0, beta: 0, gamma: 0 });
  near(camera.forward, [0, 0, -1]);
  near(camera.down, [0, -1, 0]);
  // With the screen turned 90 degrees, the image's right edge is the device's bottom.
  camera = axes({ alpha: 0, beta: 0, gamma: 0 }, 90);
  near(camera.right, [0, -1, 0]);
});
