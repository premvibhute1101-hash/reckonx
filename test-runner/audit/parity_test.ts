import { EkfCore as IdrEkfCore } from '../../IDR_PRO/src/core/EkfCore';
import { EkfCore as ReckonEkfCore } from '../../src/services/ekf/EkfCore';
import { Matrix } from 'ml-matrix';

interface StateComparison {
  maxPosDiff: number;
  maxVelDiff: number;
  maxAttDiff: number;
  maxAccelBiasDiff: number;
  maxGyroBiasDiff: number;
  maxCovDiagDiff: number;
  finalIdrPos: [number, number, number];
  finalReckonPos: [number, number, number];
  finalIdrVel: [number, number, number];
  finalReckonVel: [number, number, number];
  ticks: number;
}

function compareStates(idrEkf: IdrEkfCore, reckonEkf: ReckonEkfCore): {
  posDiff: number;
  velDiff: number;
  attDiff: number;
  accelBiasDiff: number;
  gyroBiasDiff: number;
  covDiagDiff: number;
} {
  const idrIns = idrEkf.getIns();
  const reckonIns = reckonEkf.getIns();

  const posDiff = Math.sqrt(
    Math.pow(idrIns.position.x - reckonIns.position.x, 2) +
    Math.pow(idrIns.position.y - reckonIns.position.y, 2) +
    Math.pow(idrIns.position.z - reckonIns.position.z, 2)
  );

  const velDiff = Math.sqrt(
    Math.pow(idrIns.velocity.x - reckonIns.velocity.x, 2) +
    Math.pow(idrIns.velocity.y - reckonIns.velocity.y, 2) +
    Math.pow(idrIns.velocity.z - reckonIns.velocity.z, 2)
  );

  const attDiff = Math.sqrt(
    Math.pow(idrIns.attitude.pitch - reckonIns.attitude.pitch, 2) +
    Math.pow(idrIns.attitude.roll - reckonIns.attitude.roll, 2) +
    Math.pow(idrIns.attitude.yaw - reckonIns.attitude.yaw, 2)
  );

  const idrX = (idrEkf as any).x as Matrix;
  const reckonX = (reckonEkf as any).x as Matrix;

  const accelBiasDiff = Math.sqrt(
    Math.pow(idrX.get(9, 0) - reckonX.get(9, 0), 2) +
    Math.pow(idrX.get(10, 0) - reckonX.get(10, 0), 2) +
    Math.pow(idrX.get(11, 0) - reckonX.get(11, 0), 2)
  );

  const gyroBiasDiff = Math.sqrt(
    Math.pow(idrX.get(12, 0) - reckonX.get(12, 0), 2) +
    Math.pow(idrX.get(13, 0) - reckonX.get(13, 0), 2) +
    Math.pow(idrX.get(14, 0) - reckonX.get(14, 0), 2)
  );

  const idrP = (idrEkf as any).P as Matrix;
  const reckonP = (reckonEkf as any).P as Matrix;

  let maxCovDiff = 0;
  for (let i = 0; i < 15; i++) {
    const diff = Math.abs(idrP.get(i, i) - reckonP.get(i, i));
    if (diff > maxCovDiff) maxCovDiff = diff;
  }

  return {
    posDiff,
    velDiff,
    attDiff,
    accelBiasDiff,
    gyroBiasDiff,
    covDiagDiff: maxCovDiff,
  };
}

// (a) Stationary IMU only, 60s
export function runTestStationary(alignBaseline: boolean = false): StateComparison {
  const idrEkf = new IdrEkfCore();
  const reckonEkf = new ReckonEkfCore();

  const dt = 0.1;
  const totalTicks = 600;

  idrEkf.getIns().initializeAttitude({ x: 0, y: 0, z: 9.81 });
  reckonEkf.getIns().initializeAttitude({ x: 0, y: 0, z: 9.81 });

  idrEkf.notifyAttitudeInitialized();
  reckonEkf.notifyAttitudeInitialized();

  let maxPos = 0, maxVel = 0, maxAtt = 0, maxAb = 0, maxGb = 0, maxP = 0;

  for (let t = 0; t < totalTicks; t++) {
    const accel: [number, number, number] = [0, 0, 9.81];
    const gyro: [number, number, number] = [0, 0, 0];

    idrEkf.predict(dt, accel, gyro);
    reckonEkf.predict(dt, accel, gyro);

    idrEkf.updateZupt();
    reckonEkf.updateZupt();

    const diff = compareStates(idrEkf, reckonEkf);
    if (diff.posDiff > maxPos) maxPos = diff.posDiff;
    if (diff.velDiff > maxVel) maxVel = diff.velDiff;
    if (diff.attDiff > maxAtt) maxAtt = diff.attDiff;
    if (diff.accelBiasDiff > maxAb) maxAb = diff.accelBiasDiff;
    if (diff.gyroBiasDiff > maxGb) maxGb = diff.gyroBiasDiff;
    if (diff.covDiagDiff > maxP) maxP = diff.covDiagDiff;
  }

  return {
    maxPosDiff: maxPos,
    maxVelDiff: maxVel,
    maxAttDiff: maxAtt,
    maxAccelBiasDiff: maxAb,
    maxGyroBiasDiff: maxGb,
    maxCovDiagDiff: maxP,
    finalIdrPos: [idrEkf.getIns().position.x, idrEkf.getIns().position.y, idrEkf.getIns().position.z],
    finalReckonPos: [reckonEkf.getIns().position.x, reckonEkf.getIns().position.y, reckonEkf.getIns().position.z],
    finalIdrVel: [idrEkf.getIns().velocity.x, idrEkf.getIns().velocity.y, idrEkf.getIns().velocity.z],
    finalReckonVel: [reckonEkf.getIns().velocity.x, reckonEkf.getIns().velocity.y, reckonEkf.getIns().velocity.z],
    ticks: totalTicks,
  };
}

// (b) Constant-velocity straight line with clean GNSS (30s)
export function runTestConstantVelocity(alignBaseline: boolean = false): StateComparison {
  const idrEkf = new IdrEkfCore();
  const reckonEkf = new ReckonEkfCore();

  const dt = 0.1;
  const totalTicks = 300;
  const speed = 10.0;

  idrEkf.getIns().initializeAttitude({ x: 0, y: 0, z: 9.81 });
  reckonEkf.getIns().initializeAttitude({ x: 0, y: 0, z: 9.81 });

  idrEkf.notifyAttitudeInitialized();
  reckonEkf.notifyAttitudeInitialized();

  if (alignBaseline) {
    // Manually seed IDR position & velocity so both start tracked
    idrEkf.getIns().position = { x: 0, y: 0, z: 0 };
    idrEkf.getIns().velocity = { x: 0, y: speed, z: 0 };
    (idrEkf as any).wasZuptActive = false;
    // Align Qvel
    (reckonEkf as any).POST_ZUPT_COOLDOWN_CYCLES = 5;
    (reckonEkf as any).lastVelQ = 0.1 * dt;
  }

  idrEkf.updateGnss([0, 0, 0], [0, speed, 0], 2.0);
  reckonEkf.updateGnss([0, 0, 0], [0, speed, 0], 2.0);

  let maxPos = 0, maxVel = 0, maxAtt = 0, maxAb = 0, maxGb = 0, maxP = 0;

  for (let t = 1; t <= totalTicks; t++) {
    const timeSec = t * dt;
    const accel: [number, number, number] = [0, 0, 9.81];
    const gyro: [number, number, number] = [0, 0, 0];

    idrEkf.predict(dt, accel, gyro);
    reckonEkf.predict(dt, accel, gyro);

    if (t % 10 === 0) {
      const gnssPos: [number, number, number] = [0, speed * timeSec, 0];
      const gnssVel: [number, number, number] = [0, speed, 0];
      idrEkf.updateGnss(gnssPos, gnssVel, 2.0);
      reckonEkf.updateGnss(gnssPos, gnssVel, 2.0);
    }

    const diff = compareStates(idrEkf, reckonEkf);
    if (diff.posDiff > maxPos) maxPos = diff.posDiff;
    if (diff.velDiff > maxVel) maxVel = diff.velDiff;
    if (diff.attDiff > maxAtt) maxAtt = diff.attDiff;
    if (diff.accelBiasDiff > maxAb) maxAb = diff.accelBiasDiff;
    if (diff.gyroBiasDiff > maxGb) maxGb = diff.gyroBiasDiff;
    if (diff.covDiagDiff > maxP) maxP = diff.covDiagDiff;
  }

  return {
    maxPosDiff: maxPos,
    maxVelDiff: maxVel,
    maxAttDiff: maxAtt,
    maxAccelBiasDiff: maxAb,
    maxGyroBiasDiff: maxGb,
    maxCovDiagDiff: maxP,
    finalIdrPos: [idrEkf.getIns().position.x, idrEkf.getIns().position.y, idrEkf.getIns().position.z],
    finalReckonPos: [reckonEkf.getIns().position.x, reckonEkf.getIns().position.y, reckonEkf.getIns().position.z],
    finalIdrVel: [idrEkf.getIns().velocity.x, idrEkf.getIns().velocity.y, idrEkf.getIns().velocity.z],
    finalReckonVel: [reckonEkf.getIns().velocity.x, reckonEkf.getIns().velocity.y, reckonEkf.getIns().velocity.z],
    ticks: totalTicks,
  };
}

// (c) Turn (90 deg) at Walking Speed (15s)
export function runTestWalkingTurn(alignBaseline: boolean = false): StateComparison {
  const idrEkf = new IdrEkfCore();
  const reckonEkf = new ReckonEkfCore();

  const dt = 0.1;
  const totalTicks = 150;
  const speed = 1.4;

  idrEkf.getIns().initializeAttitude({ x: 0, y: 0, z: 9.81 });
  reckonEkf.getIns().initializeAttitude({ x: 0, y: 0, z: 9.81 });

  idrEkf.notifyAttitudeInitialized();
  reckonEkf.notifyAttitudeInitialized();

  if (alignBaseline) {
    idrEkf.getIns().position = { x: 0, y: 0, z: 0 };
    idrEkf.getIns().velocity = { x: 0, y: speed, z: 0 };
    (idrEkf as any).wasZuptActive = false;
  }

  idrEkf.updateGnss([0, 0, 0], [0, speed, 0], 2.5);
  reckonEkf.updateGnss([0, 0, 0], [0, speed, 0], 2.5);

  let currentHeading = 0;
  let currentPosX = 0;
  let currentPosY = 0;

  let maxPos = 0, maxVel = 0, maxAtt = 0, maxAb = 0, maxGb = 0, maxP = 0;

  for (let t = 1; t <= totalTicks; t++) {
    const timeSec = t * dt;

    let turnRate = 0;
    if (timeSec > 5.0 && timeSec <= 10.0) {
      turnRate = Math.PI / 10.0;
    }
    currentHeading += turnRate * dt;

    const vx = speed * Math.sin(currentHeading);
    const vy = speed * Math.cos(currentHeading);
    currentPosX += vx * dt;
    currentPosY += vy * dt;

    const centripetalAx = turnRate * speed;
    const accel: [number, number, number] = [centripetalAx, 0, 9.81];
    const gyro: [number, number, number] = [0, 0, turnRate];

    idrEkf.predict(dt, accel, gyro);
    reckonEkf.predict(dt, accel, gyro);

    if (t % 10 === 0) {
      idrEkf.updateGnss([currentPosX, currentPosY, 0], [vx, vy, 0], 2.5);
      reckonEkf.updateGnss([currentPosX, currentPosY, 0], [vx, vy, 0], 2.5);
    }

    const diff = compareStates(idrEkf, reckonEkf);
    if (diff.posDiff > maxPos) maxPos = diff.posDiff;
    if (diff.velDiff > maxVel) maxVel = diff.velDiff;
    if (diff.attDiff > maxAtt) maxAtt = diff.attDiff;
    if (diff.accelBiasDiff > maxAb) maxAb = diff.accelBiasDiff;
    if (diff.gyroBiasDiff > maxGb) maxGb = diff.gyroBiasDiff;
    if (diff.covDiagDiff > maxP) maxP = diff.covDiagDiff;
  }

  return {
    maxPosDiff: maxPos,
    maxVelDiff: maxVel,
    maxAttDiff: maxAtt,
    maxAccelBiasDiff: maxAb,
    maxGyroBiasDiff: maxGb,
    maxCovDiagDiff: maxP,
    finalIdrPos: [idrEkf.getIns().position.x, idrEkf.getIns().position.y, idrEkf.getIns().position.z],
    finalReckonPos: [reckonEkf.getIns().position.x, reckonEkf.getIns().position.y, reckonEkf.getIns().position.z],
    finalIdrVel: [idrEkf.getIns().velocity.x, idrEkf.getIns().velocity.y, idrEkf.getIns().velocity.z],
    finalReckonVel: [reckonEkf.getIns().velocity.x, reckonEkf.getIns().velocity.y, reckonEkf.getIns().velocity.z],
    ticks: totalTicks,
  };
}

// (d) 10s GNSS outage at constant velocity (30s)
export function runTestGnssOutage(alignBaseline: boolean = false): StateComparison {
  const idrEkf = new IdrEkfCore();
  const reckonEkf = new ReckonEkfCore();

  const dt = 0.1;
  const totalTicks = 300;
  const speed = 15.0;

  idrEkf.getIns().initializeAttitude({ x: 0, y: 0, z: 9.81 });
  reckonEkf.getIns().initializeAttitude({ x: 0, y: 0, z: 9.81 });

  idrEkf.notifyAttitudeInitialized();
  reckonEkf.notifyAttitudeInitialized();

  if (alignBaseline) {
    idrEkf.getIns().position = { x: 0, y: 0, z: 0 };
    idrEkf.getIns().velocity = { x: 0, y: speed, z: 0 };
    (idrEkf as any).wasZuptActive = false;
  }

  idrEkf.updateGnss([0, 0, 0], [0, speed, 0], 2.0);
  reckonEkf.updateGnss([0, 0, 0], [0, speed, 0], 2.0);

  let maxPos = 0, maxVel = 0, maxAtt = 0, maxAb = 0, maxGb = 0, maxP = 0;

  for (let t = 1; t <= totalTicks; t++) {
    const timeSec = t * dt;
    const accel: [number, number, number] = [0, 0, 9.81];
    const gyro: [number, number, number] = [0, 0, 0];

    idrEkf.predict(dt, accel, gyro);
    reckonEkf.predict(dt, accel, gyro);

    const isOutage = timeSec > 10.0 && timeSec <= 20.0;

    if (t % 10 === 0) {
      if (!isOutage) {
        const gnssPos: [number, number, number] = [0, speed * timeSec, 0];
        const gnssVel: [number, number, number] = [0, speed, 0];
        idrEkf.updateGnss(gnssPos, gnssVel, 2.0);
        reckonEkf.updateGnss(gnssPos, gnssVel, 2.0);
      } else {
        idrEkf.updateGnss([0, 0, 0], [0, 0, 0], null);
        reckonEkf.updateGnss([0, 0, 0], [0, 0, 0], null);
      }
    }

    const diff = compareStates(idrEkf, reckonEkf);
    if (diff.posDiff > maxPos) maxPos = diff.posDiff;
    if (diff.velDiff > maxVel) maxVel = diff.velDiff;
    if (diff.attDiff > maxAtt) maxAtt = diff.attDiff;
    if (diff.accelBiasDiff > maxAb) maxAb = diff.accelBiasDiff;
    if (diff.gyroBiasDiff > maxGb) maxGb = diff.gyroBiasDiff;
    if (diff.covDiagDiff > maxP) maxP = diff.covDiagDiff;
  }

  return {
    maxPosDiff: maxPos,
    maxVelDiff: maxVel,
    maxAttDiff: maxAtt,
    maxAccelBiasDiff: maxAb,
    maxGyroBiasDiff: maxGb,
    maxCovDiagDiff: maxP,
    finalIdrPos: [idrEkf.getIns().position.x, idrEkf.getIns().position.y, idrEkf.getIns().position.z],
    finalReckonPos: [reckonEkf.getIns().position.x, reckonEkf.getIns().position.y, reckonEkf.getIns().position.z],
    finalIdrVel: [idrEkf.getIns().velocity.x, idrEkf.getIns().velocity.y, idrEkf.getIns().velocity.z],
    finalReckonVel: [reckonEkf.getIns().velocity.x, reckonEkf.getIns().velocity.y, reckonEkf.getIns().velocity.z],
    ticks: totalTicks,
  };
}

function runAll() {
  console.log('================================================================');
  console.log('1. AS-IS IMPLEMENTATION COMPARISON (Production Codebases)');
  console.log('================================================================');

  const resA = runTestStationary(false);
  console.log('\n(a) Stationary IMU Only (60s):');
  console.log(`    Max Position Diff:      ${resA.maxPosDiff.toExponential(4)} m`);
  console.log(`    Max Velocity Diff:      ${resA.maxVelDiff.toExponential(4)} m/s`);
  console.log(`    Max Attitude Diff:      ${resA.maxAttDiff.toExponential(4)} rad`);
  console.log(`    Max Accel Bias Diff:    ${resA.maxAccelBiasDiff.toExponential(4)} m/s^2`);
  console.log(`    Max Gyro Bias Diff:     ${resA.maxGyroBiasDiff.toExponential(4)} rad/s`);
  console.log(`    Max Cov Diag Diff:      ${resA.maxCovDiagDiff.toExponential(4)}`);
  console.log(`    Final IDR Pos:          [${resA.finalIdrPos.map(v => v.toFixed(4)).join(', ')}]`);
  console.log(`    Final ReckonX Pos:      [${resA.finalReckonPos.map(v => v.toFixed(4)).join(', ')}]`);

  const resB = runTestConstantVelocity(false);
  console.log('\n(b) Constant-Velocity Straight Line with Clean GNSS (30s):');
  console.log(`    Max Position Diff:      ${resB.maxPosDiff.toExponential(4)} m`);
  console.log(`    Max Velocity Diff:      ${resB.maxVelDiff.toExponential(4)} m/s`);
  console.log(`    Max Attitude Diff:      ${resB.maxAttDiff.toExponential(4)} rad`);
  console.log(`    Max Accel Bias Diff:    ${resB.maxAccelBiasDiff.toExponential(4)} m/s^2`);
  console.log(`    Max Gyro Bias Diff:     ${resB.maxGyroBiasDiff.toExponential(4)} rad/s`);
  console.log(`    Max Cov Diag Diff:      ${resB.maxCovDiagDiff.toExponential(4)}`);
  console.log(`    Final IDR Pos:          [${resB.finalIdrPos.map(v => v.toFixed(4)).join(', ')}]`);
  console.log(`    Final ReckonX Pos:      [${resB.finalReckonPos.map(v => v.toFixed(4)).join(', ')}]`);

  const resC = runTestWalkingTurn(false);
  console.log('\n(c) Turn (90 deg) at Walking Speed (15s):');
  console.log(`    Max Position Diff:      ${resC.maxPosDiff.toExponential(4)} m`);
  console.log(`    Max Velocity Diff:      ${resC.maxVelDiff.toExponential(4)} m/s`);
  console.log(`    Max Attitude Diff:      ${resC.maxAttDiff.toExponential(4)} rad`);
  console.log(`    Max Accel Bias Diff:    ${resC.maxAccelBiasDiff.toExponential(4)} m/s^2`);
  console.log(`    Max Gyro Bias Diff:     ${resC.maxGyroBiasDiff.toExponential(4)} rad/s`);
  console.log(`    Max Cov Diag Diff:      ${resC.maxCovDiagDiff.toExponential(4)}`);
  console.log(`    Final IDR Pos:          [${resC.finalIdrPos.map(v => v.toFixed(4)).join(', ')}]`);
  console.log(`    Final ReckonX Pos:      [${resC.finalReckonPos.map(v => v.toFixed(4)).join(', ')}]`);

  const resD = runTestGnssOutage(false);
  console.log('\n(d) 10s GNSS Outage at Constant Velocity (30s):');
  console.log(`    Max Position Diff:      ${resD.maxPosDiff.toExponential(4)} m`);
  console.log(`    Max Velocity Diff:      ${resD.maxVelDiff.toExponential(4)} m/s`);
  console.log(`    Max Attitude Diff:      ${resD.maxAttDiff.toExponential(4)} rad`);
  console.log(`    Max Accel Bias Diff:    ${resD.maxAccelBiasDiff.toExponential(4)} m/s^2`);
  console.log(`    Max Gyro Bias Diff:     ${resD.maxGyroBiasDiff.toExponential(4)} rad/s`);
  console.log(`    Max Cov Diag Diff:      ${resD.maxCovDiagDiff.toExponential(4)}`);
  console.log(`    Final IDR Pos:          [${resD.finalIdrPos.map(v => v.toFixed(4)).join(', ')}]`);
  console.log(`    Final ReckonX Pos:      [${resD.finalReckonPos.map(v => v.toFixed(4)).join(', ')}]`);

  console.log('\n================================================================');
  console.log('2. BASELINE ALIGNMENT COMPARISON (Seeding & Class A aligned)');
  console.log('================================================================');

  const baseB = runTestConstantVelocity(true);
  console.log('\n(b-aligned) Constant-Velocity Straight Line with Clean GNSS:');
  console.log(`    Max Position Diff:      ${baseB.maxPosDiff.toExponential(4)} m`);
  console.log(`    Max Velocity Diff:      ${baseB.maxVelDiff.toExponential(4)} m/s`);
  console.log(`    Max Attitude Diff:      ${baseB.maxAttDiff.toExponential(4)} rad`);
  console.log(`    Max Cov Diag Diff:      ${baseB.maxCovDiagDiff.toExponential(4)}`);
  console.log(`    Final IDR Pos:          [${baseB.finalIdrPos.map(v => v.toFixed(4)).join(', ')}]`);
  console.log(`    Final ReckonX Pos:      [${baseB.finalReckonPos.map(v => v.toFixed(4)).join(', ')}]`);

  const baseD = runTestGnssOutage(true);
  console.log('\n(d-aligned) 10s GNSS Outage at Constant Velocity:');
  console.log(`    Max Position Diff:      ${baseD.maxPosDiff.toExponential(4)} m`);
  console.log(`    Max Velocity Diff:      ${baseD.maxVelDiff.toExponential(4)} m/s`);
  console.log(`    Max Attitude Diff:      ${baseD.maxAttDiff.toExponential(4)} rad`);
  console.log(`    Max Cov Diag Diff:      ${baseD.maxCovDiagDiff.toExponential(4)}`);
  console.log(`    Final IDR Pos:          [${baseD.finalIdrPos.map(v => v.toFixed(4)).join(', ')}]`);
  console.log(`    Final ReckonX Pos:      [${baseD.finalReckonPos.map(v => v.toFixed(4)).join(', ')}]`);
}

runAll();
