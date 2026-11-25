import assert from 'node:assert/strict';
import test from 'node:test';
import { downloadDepthModel } from '../src/depth.js?v=9';

test('an incomplete cached model is replaced and a requested refresh bypasses it', async (t) => {
  const size = 140_852_051;
  const block = new Uint8Array(1024 * 1024);
  let saved = new Response('partial', { headers: { 'Content-Length': '7' } });
  const cache = {
    async match() { return saved; },
    delete: t.mock.fn(async () => { saved = undefined; }),
    async put(_, response) {
      let received = 0;
      const reader = response.body.getReader();
      for (let next = await reader.read(); !next.done; next = await reader.read()) {
        received += next.value.byteLength;
      }
      assert.equal(received, size);
      saved = new Response(null, { headers: response.headers });
    },
  };
  const fetchModel = t.mock.fn(async () => {
    let remaining = size;
    return new Response(new ReadableStream({
      pull(controller) {
        const count = Math.min(remaining, block.length);
        controller.enqueue(block.subarray(0, count));
        remaining -= count;
        if (!remaining) controller.close();
      },
    }));
  });
  const globals = { caches: { async open() { return cache; } }, fetch: fetchModel };
  for (const [key, value] of Object.entries(globals)) {
    const original = Object.getOwnPropertyDescriptor(globalThis, key);
    Object.defineProperty(globalThis, key, { configurable: true, value });
    t.after(() => {
      if (original) Object.defineProperty(globalThis, key, original);
      else delete globalThis[key];
    });
  }

  const progress = t.mock.fn();
  assert.equal(await downloadDepthModel(progress), saved);
  assert.equal(cache.delete.mock.callCount(), 1);
  assert.equal(fetchModel.mock.callCount(), 1);
  assert.equal(fetchModel.mock.calls[0].arguments[1].cache, 'no-store');
  assert.equal(progress.mock.calls.at(-1).arguments[0], 'Depth model downloaded');
  assert.equal(await downloadDepthModel(progress), saved);
  assert.equal(fetchModel.mock.callCount(), 1);
  assert.equal(await downloadDepthModel(progress, true), saved);
  assert.equal(fetchModel.mock.callCount(), 2);
});
