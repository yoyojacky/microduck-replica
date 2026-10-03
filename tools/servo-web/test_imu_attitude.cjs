// No npm install or WebGL required: node --test test_imu_attitude.cjs
const { test } = require('node:test');
const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');
const source = fs.readFileSync(path.join(__dirname, 'imu_attitude.js'), 'utf8');
const modulePromise = import(`data:text/javascript;base64,${Buffer.from(source).toString('base64')}`);
const I = [0, 0, 0, 1], H = Math.SQRT1_2;
const packet = (overrides={}) => ({type:'imu', mode:'live', connected:true, status:'live', live:true,
  ready:1, configured:1, sample_count:1, quaternion_xyzw:I, ...overrides});
function equivalent(actual, expected) {
  const dot = actual.reduce((n,v,i) => n+v*expected[i], 0);
  assert.ok(Math.abs(Math.abs(dot)-1) < 1e-10, `${actual} != rotation ${expected}`);
}

test('default mounting rotates sensor X into body negative Z', async () => {
  const {ImuAttitude} = await modulePromise, state = new ImuAttitude();
  assert.equal(state.mountName, 'y90');
  state.receive(packet(), 0);
  state.receive(packet({sample_count:2, quaternion_xyzw:[H,0,0,H]}), 50);
  equivalent(state.target, [0,0,-H,H]);
});
test('mounting basis change preserves rotation angle on all three axes', async () => {
  const {displayQuaternion, mountQuaternion} = await modulePromise, mount = mountQuaternion('y90');
  equivalent(displayQuaternion([H,0,0,H], I, mount), [0,0,-H,H]);
  equivalent(displayQuaternion([0,H,0,H], I, mount), [0,H,0,H]);
  equivalent(displayQuaternion([0,0,H,H], I, mount), [H,0,0,H]);
});
test('first sample and explicit zero use a relative quaternion, not Euler subtraction', async () => {
  const {ImuAttitude, multiply} = await modulePromise, state = new ImuAttitude();
  state.setMount('identity');
  state.receive(packet({quaternion_xyzw:[0,H,0,H]}), 0);
  equivalent(state.target, I);
  state.receive(packet({sample_count:2, quaternion_xyzw:multiply([0,H,0,H], [H,0,0,H])}), 50);
  equivalent(state.target, [H,0,0,H]);
  assert.equal(state.zero(), true); equivalent(state.target, I);
});
test('raw orientation composes sensor-to-world with body-to-sensor', async () => {
  const {ImuAttitude} = await modulePromise, state = new ImuAttitude();
  state.receive(packet({quaternion_xyzw:[0,H,0,H]}), 0);
  assert.equal(state.showRaw(), true); equivalent(state.target, I);
  state.setMount('identity'); equivalent(state.target, [0,H,0,H]);
});
test('opposite quaternion signs and small norm error give the same display rotation', async () => {
  const {ImuAttitude} = await modulePromise, state = new ImuAttitude();
  state.receive(packet({quaternion_xyzw:[0,0,0,1.05]}), 0);
  state.receive(packet({sample_count:2, quaternion_xyzw:[-H,0,0,-H]}), 50);
  equivalent(state.target, [0,0,-H,H]);
});
test('invalid live flags, readiness, connection, mode and malformed quaternion fail closed', async () => {
  const {ImuAttitude} = await modulePromise;
  for (const invalid of [{live:false}, {status:'stale'}, {ready:0}, {configured:0}, {connected:false}, {mode:'unknown'},
    {sample_count:null}, {quaternion_xyzw:[0,0,0,0]}, {quaternion_xyzw:[0,NaN,0,1]}, {quaternion_xyzw:[0,0,0]},
    {quaternion_xyzw:[0,0,0,Infinity]}, {quaternion_xyzw:[0,0,0,2]}]) {
    const state = new ImuAttitude(); state.receive(packet(), 0); const saved = [...state.target];
    state.receive(packet(invalid), 50);
    assert.equal(state.valid, false, JSON.stringify(invalid));
    assert.deepEqual(state.target, saved); assert.equal(state.zero(), false); assert.equal(state.showRaw(), false);
  }
});
test('demo is accepted without a hardware connection and retains its mode', async () => {
  const {ImuAttitude} = await modulePromise, state = new ImuAttitude();
  state.receive(packet({mode:'demo', connected:false}), 0);
  assert.equal(state.valid, true); assert.equal(state.mode, 'demo');
});
test('no messages for 1.5 seconds freezes target and marks stale', async () => {
  const {ImuAttitude, STALE_MS} = await modulePromise, state = new ImuAttitude();
  state.receive(packet(), 0); const saved = [...state.target];
  assert.equal(state.expire(STALE_MS-1), false); assert.equal(state.valid, true);
  assert.equal(state.expire(STALE_MS), true); assert.equal(state.status, 'stale');
  assert.deepEqual(state.target, saved);
  state.receive(packet({sample_count:2, quaternion_xyzw:[H,0,0,H]}), STALE_MS+50);
  assert.equal(state.valid, true); equivalent(state.target, [0,0,-H,H]);
});
test('repeated heartbeats do not hide a stuck sampling counter', async () => {
  const {ImuAttitude} = await modulePromise, state = new ImuAttitude();
  for (let now=0; now<=1500; now+=100) state.receive(packet(), now);
  assert.equal(state.valid, false); assert.equal(state.status, 'stale');
});
test('disconnect freezes orientation while valid recovery resumes it', async () => {
  const {ImuAttitude} = await modulePromise, state = new ImuAttitude();
  state.receive(packet(), 0); state.receive(packet({sample_count:2, quaternion_xyzw:[H,0,0,H]}), 50);
  state.disconnect(); assert.equal(state.valid, false); assert.equal(state.status, 'disconnected');
  equivalent(state.target, [0,0,-H,H]);
  state.receive(packet({sample_count:3, quaternion_xyzw:I}), 100);
  assert.equal(state.valid, true); equivalent(state.target, I);
});
test('counter restart establishes a new relative baseline', async () => {
  const {ImuAttitude} = await modulePromise, state = new ImuAttitude();
  state.receive(packet({sample_count:100}), 0);
  state.receive(packet({sample_count:1, quaternion_xyzw:[H,0,0,H]}), 50);
  equivalent(state.target, I);
});
test('switching between demo and hardware cannot reuse the other source reference', async () => {
  const {ImuAttitude} = await modulePromise, state = new ImuAttitude();
  state.receive(packet({mode:'demo', connected:false}), 0);
  state.receive(packet({sample_count:100, quaternion_xyzw:[H,0,0,H]}), 50);
  assert.equal(state.mode, 'live'); equivalent(state.target, I);
});
test('Euler readout is finite through pitch singularity and handles known roll', async () => {
  const {eulerDegrees} = await modulePromise;
  assert.ok(eulerDegrees([0,H,0,H]).every(Number.isFinite));
  assert.ok(Math.abs(eulerDegrees([H,0,0,H])[0]-90) < 1e-10);
});
