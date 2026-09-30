import { InsMechanization } from '../../src/services/ekf/InsMechanization';

console.log('=== Testing Attitude Propagation in InsMechanization ===\n');

// 1. Flat Mount (Gravity along Body +Z)
console.log('--- Test 1: Flat Mount (Gravity along Body +Z) ---');
{
  const ins = new InsMechanization();
  ins.initializeAttitude({ x: 0, y: 0, z: 9.81 });
  console.log(`Initial attitude: roll=${ins.attitude.roll.toFixed(4)}, pitch=${ins.attitude.pitch.toFixed(4)}, yaw=${ins.attitude.yaw.toFixed(4)}`);

  // 90 deg clockwise turn over 2.0s around Earth vertical (body Z)
  const dt = 0.05;
  const turnRate = -Math.PI / 4.0; // -0.7854 rad/s (clockwise)
  for (let t = 0; t < 2.0; t += dt) {
    ins.predict(dt, { x: 0, y: 0, z: 9.81 }, { x: 0, y: 0, z: turnRate });
  }
  const headingDeg = (-ins.attitude.yaw * 180 / Math.PI + 360) % 360;
  console.log(`After 90° Turn: roll=${ins.attitude.roll.toFixed(4)}, pitch=${ins.attitude.pitch.toFixed(4)}, yaw=${ins.attitude.yaw.toFixed(4)} -> Heading=${headingDeg.toFixed(1)}° (Expected: ~90.0°)`);
}

// 2. Portrait Upright Mount (Gravity along Body +Y)
console.log('\n--- Test 2: Portrait Upright Mount (Gravity along Body +Y) ---');
{
  const ins = new InsMechanization();
  // Phone mounted vertically in car holder: screen facing driver, top of phone up.
  // Body Z points toward driver (North/horizontal), Body Y points up (vertical), Body X points right (East).
  // Gravity vector measured by accelerometer at rest: pointing along -Y or +Y depending on sensor convention.
  // If top of phone is Up: accel measures +9.81 along Y (since sensor measures upward reaction force).
  ins.initializeAttitude({ x: 0, y: 9.81, z: 0 });
  console.log(`Initial attitude: roll=${ins.attitude.roll.toFixed(4)}, pitch=${ins.attitude.pitch.toFixed(4)}, yaw=${ins.attitude.yaw.toFixed(4)}`);

  // Car turns 90° around true Earth vertical (which is along Body Y in portrait upright mount!)
  // In body frame, turn rate is around Body Y (gyro.y = turnRate), while gyro.z = 0.
  const dt = 0.05;
  const turnRate = -Math.PI / 4.0;
  for (let t = 0; t < 2.0; t += dt) {
    ins.predict(dt, { x: 0, y: 9.81, z: 0 }, { x: 0, y: turnRate, z: 0 });
  }
  const headingDeg = (-ins.attitude.yaw * 180 / Math.PI + 360) % 360;
  console.log(`After 90° Turn: roll=${ins.attitude.roll.toFixed(4)}, pitch=${ins.attitude.pitch.toFixed(4)}, yaw=${ins.attitude.yaw.toFixed(4)} -> Heading=${headingDeg.toFixed(1)}° (Expected: ~90.0°)`);
}

// 3. Landscape Mount (Pitch near +/-90 degrees, Gravity along Body +X)
console.log('\n--- Test 3: Landscape Mount / Pitch Near 90° (Gravity along Body +X) ---');
{
  const ins = new InsMechanization();
  // Gravity along body X: ax = 9.81, ay = 0, az = 0
  ins.initializeAttitude({ x: 9.81, y: 0, z: 0 });
  console.log(`Initial attitude: roll=${ins.attitude.roll.toFixed(4)}, pitch=${ins.attitude.pitch.toFixed(4)}, yaw=${ins.attitude.yaw.toFixed(4)}`);

  // Car turns 90° around true Earth vertical (which is along Body X in landscape mount!)
  // In body frame, turn rate is around Body X (gyro.x = turnRate), while gyro.z = 0.
  const dt = 0.05;
  const turnRate = -Math.PI / 4.0;
  for (let t = 0; t < 2.0; t += dt) {
    ins.predict(dt, { x: 9.81, y: 0, z: 0 }, { x: turnRate, y: 0, z: 0 });
  }
  const headingDeg = (-ins.attitude.yaw * 180 / Math.PI + 360) % 360;
  console.log(`After 90° Turn (rotating body X): roll=${ins.attitude.roll.toFixed(4)}, pitch=${ins.attitude.pitch.toFixed(4)}, yaw=${ins.attitude.yaw.toFixed(4)} -> Heading=${headingDeg.toFixed(1)}° (Expected: ~90.0°)`);

  // Also test if gyro.z rotation near pitch +/-90 causes singularity / invalid rotation
  const insSingularity = new InsMechanization();
  insSingularity.initializeAttitude({ x: -9.81, y: 0, z: 0 }); // pitch = +pi/2
  console.log(`\nPitch at +90° test: initial pitch=${insSingularity.attitude.pitch.toFixed(4)} rad (${(insSingularity.attitude.pitch*180/Math.PI).toFixed(1)}°)`);
  for (let t = 0; t < 2.0; t += dt) {
    insSingularity.predict(dt, { x: -9.81, y: 0, z: 0 }, { x: 0, y: 0, z: turnRate });
  }
  console.log(`After gyro.z integration at pitch=90°: roll=${insSingularity.attitude.roll}, pitch=${insSingularity.attitude.pitch}, yaw=${insSingularity.attitude.yaw}`);
  console.log(`Rotation Matrix R_b^n:`);
  console.log(insSingularity.lastRotationMatrix);
}
