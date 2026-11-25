import assert from 'node:assert/strict';
import test from 'node:test';

test('sampled frames stay in order through depth startup, slow processing, and stop', async (t) => {
  const ids = [
    'camera-preview', 'camera-prompt', 'depth-status', 'record-camera',
    'stop-camera', 'recording-rate', 'camera-frame-count', 'camera-feedback',
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
      if (message.type === 'start') workerId = message.id;
    }
    reply(type, values = {}) {
      this.dispatchEvent(new MessageEvent('message', { data: { type, id: workerId, ...values } }));
    }
  }

  let worker;
  let workerId;
  const sent = [];
  let captureTimer;
  const document = Object.assign(new EventTarget(), {
    getElementById: (id) => elements.get(id),
    createElement: () => new Canvas(),
  });
  const globals = {
    document,
    navigator: { mediaDevices: { getUserMedia: async () => stream } },
    HTMLMediaElement: { HAVE_CURRENT_DATA: 2 },
    Image: class {
      naturalWidth = 280;
      naturalHeight = 210;
      async decode() {}
    },
    Worker: CameraWorker,
    setTimeout: (capture) => { captureTimer = capture; return 1; },
    clearTimeout: () => { captureTimer = null; },
  };
  for (const [name, value] of Object.entries(globals)) {
    const original = Object.getOwnPropertyDescriptor(globalThis, name);
    Object.defineProperty(globalThis, name, { configurable: true, value });
    t.after(() => {
      if (original) Object.defineProperty(globalThis, name, original);
      else delete globalThis[name];
    });
  }

  const { createCameraRecorder } = await import('../src/camera.js?v=7');
  const setState = t.mock.fn();
  const recorder = createCameraRecorder({
    viewer: { clear() {}, addFrame() {}, setVisible() { return 0; } },
    describePoints() {},
    setState,
  });
  const flush = () => new Promise((resolve) => setImmediate(resolve));
  const capture = (pixel) => {
    video.currentTime = pixel;
    captureTimer();
  };
  const click = (id) => elements.get(id).dispatchEvent(new Event('click'));
  const counter = elements.get('camera-frame-count');

  recorder.open();
  click('record-camera');
  await flush();
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
  assert.equal(captureTimer, null);
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
});
