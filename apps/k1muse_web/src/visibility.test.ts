import test from 'node:test';
import assert from 'node:assert/strict';
import { whenVisible } from './visibility';
import { WarehouseOperation } from './warehouse_operation';

function hiddenDocument() {
  const doc = Object.assign(new EventTarget(), { hidden: true });
  Object.defineProperty(globalThis, 'document', { configurable: true, value: doc });
  return doc;
}
test('hidden UI waits for a genuinely visible state, not just an event', async () => {
  const doc = hiddenDocument();
  try {
    let done = false;
    const waiting = whenVisible().then(() => { done = true; });
    doc.dispatchEvent(new Event('visibilitychange')); await Promise.resolve();
    assert.equal(done, false);
    doc.hidden = false; doc.dispatchEvent(new Event('visibilitychange')); await waiting;
    assert.equal(done, true); await whenVisible();
  } finally { Reflect.deleteProperty(globalThis, 'document'); }
});
test('warehouse polling pauses hidden and resumes the original request without mutation replay', async () => {
  const doc = hiddenDocument();
  try {
    const calls: { operation: string; args: any; requestId?: string }[] = [];
    const op = new WarehouseOperation(async (operation, args, requestId) => {
      calls.push({ operation, args, requestId });
      return { operation_id: 'original-operation', result: operation === 'warehouse_sync' ?
        { status: 'running' } : { status: 'succeeded', result: { synced_revision: 2 } } };
    });
    const result = op.start('warehouse_sync', { base_revision: 1 });
    await new Promise(resolve => setTimeout(resolve, 500));
    assert.equal(calls.length, 1);
    assert.equal(op.pending?.operationId, 'original-operation');
    doc.hidden = false; doc.dispatchEvent(new Event('visibilitychange'));
    assert.deepEqual(await result, { synced_revision: 2 });
    assert.equal(calls.length, 2); assert.equal(calls[1].operation, 'get_operation');
    assert.equal(calls[1].args.request_id, calls[0].requestId);
    assert.equal(op.pending, undefined);
  } finally { Reflect.deleteProperty(globalThis, 'document'); }
});
