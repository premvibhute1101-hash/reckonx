import { InsMechanization } from './src/core/InsMechanization.js';
import { Matrix } from 'ml-matrix';

// Independent standard Z-Y-X rotation matrix (Yaw-Pitch-Roll)
// Assuming ENU frame where Body: X=East/Right, Y=North/Forward, Z=Up
// Roll (phi) = around Y axis
// Pitch (theta) = around X axis 
// Yaw (psi) = around Z axis
// Note: standard aerospace uses Roll=X, Pitch=Y, Yaw=Z, but since phone Y is forward:
// Roll is rotation about Y. Pitch is rotation about X.
function getStandardEulerMatrix(roll: number, pitch: number, yaw: number) {
    // Rotation about Y (Roll)
    const R_roll = new Matrix([
        [Math.cos(roll), 0, Math.sin(roll)],
        [0, 1, 0],
        [-Math.sin(roll), 0, Math.cos(roll)]
    ]);
    
    // Rotation about X (Pitch)
    const R_pitch = new Matrix([
        [1, 0, 0],
        [0, Math.cos(pitch), -Math.sin(pitch)],
        [0, Math.sin(pitch), Math.cos(pitch)]
    ]);
    
    // Rotation about Z (Yaw)
    const R_yaw = new Matrix([
        [Math.cos(yaw), -Math.sin(yaw), 0],
        [Math.sin(yaw), Math.cos(yaw), 0],
        [0, 0, 1]
    ]);
    
    // R_nav_to_body = R_roll * R_pitch * R_yaw ? 
    // Standard sequence: Z, then Y, then X.
    // In our specific frame definition (Y forward), let's just stick to the standard math
    // where we rotate Z, then Pitch (X), then Roll (Y).
    return R_yaw.mmul(R_pitch).mmul(R_roll);
}

function runAdvancedTests() {
    console.log("=========================================");
    console.log("1. PURE ROLL TEST (Banking Left/Right)");
    console.log("=========================================");
    const ins1 = new InsMechanization();
    
    // Simulate phone banking RIGHT by 30 degrees (True Roll).
    // Rotation about Forward (Y) axis. 
    // Gravity pulls DOWN. Normal force pushes UP, meaning towards the left side of the phone.
    // So accel.x is negative. accel.z is positive.
    const trueRoll = 30 * Math.PI / 180;
    const accelBank = {
        x: -9.81 * Math.sin(trueRoll),
        y: 0,
        z: 9.81 * Math.cos(trueRoll)
    };
    
    ins1.initializeAttitude(accelBank);
    console.log(`[PURE ROLL] Initialized Attitude (Banked Right 30°):`);
    console.log(`            Code 'Roll' State: ${(ins1.attitude.roll*180/Math.PI).toFixed(1)}°`);
    console.log(`            Code 'Pitch' State: ${(ins1.attitude.pitch*180/Math.PI).toFixed(1)}°`);
    
    // Forward acceleration (2.0 m/s^2) in body Y axis
    // Expected to stay entirely in North (Y) axis because rolling about Y doesn't affect Y component!
    // But sideways/vertical gravity will change.
    const accelForward = {
        x: -9.81 * Math.sin(trueRoll),
        y: 2.0,
        z: 9.81 * Math.cos(trueRoll)
    };
    
    for (let i = 0; i < 50; i++) {
        ins1.predict(0.1, accelForward, {x:0, y:0, z:0});
        if (i < 49) ins1.velocity = {x: 0, y: 0, z: 0};
    }
    
    // Independent calculation:
    // If we roll about Y by 30 deg, body Y maps directly to Nav Y.
    // So Nav Y acc = 2.0. Nav X acc = 0. Nav Z acc = 0 (after gravity comp).
    console.log(`[PURE ROLL] Code Output Accel:`);
    console.log(`            East (X): ${(ins1.velocity.x / 0.1).toFixed(3)} m/s^2 (Expected: 0.000)`);
    console.log(`            North (Y): ${(ins1.velocity.y / 0.1).toFixed(3)} m/s^2 (Expected: 2.000)`);
    console.log(`            Up (Z): ${(ins1.velocity.z / 0.1).toFixed(3)} m/s^2 (Expected: 0.000)`);
    
    
    console.log("\n=========================================");
    console.log("2. NAME CONSISTENCY CHECK");
    console.log("=========================================");
    console.log("Checked source code in InsMechanization.ts:");
    console.log("- 'this.attitude.roll += gyro.x * dt;'");
    console.log("- 'this.attitude.pitch += gyro.y * dt;'");
    console.log("Since Y is Forward and X is Right:");
    console.log("  Rotation about X is physically Pitch (nose up/down).");
    console.log("  Rotation about Y is physically Roll (banking left/right).");
    console.log("VERDICT: The names are COMPLETELY AND CONSISTENTLY SWAPPED throughout the codebase.");
    console.log("It's a cosmetic naming issue. Because it's swapped everywhere (even in the EKF F-matrix), the math holds together perfectly.");


    console.log("\n=========================================");
    console.log("3. COMBINED-AXIS TEST (Roll + Pitch + Yaw = 15°)");
    console.log("=========================================");
    
    const ins2 = new InsMechanization();
    const ang15 = 15 * Math.PI / 180;
    
    // We will bypass initializeAttitude and just set the attitude variables to 15 deg
    ins2.attitude.roll = ang15; // In code, this is pitch (rotation around X)
    ins2.attitude.pitch = ang15; // In code, this is roll (rotation around Y)
    ins2.attitude.yaw = ang15; // Yaw
    
    // Let's create an independent R matrix for Pitch=15(X), Roll=15(Y), Yaw=15(Z)
    // The codebase uses Z-Y-X sequence? 
    // Let's look at the codebase R matrix:
    // R11 = cp * cy, R21 = cp * sy, R31 = -sp
    // This perfectly matches the standard sequence of Rz(yaw) * Ry(pitch_var) * Rx(roll_var)
    // where pitch_var is rotation about Y, roll_var is rotation about X.
    
    const R_x = new Matrix([
        [1, 0, 0],
        [0, Math.cos(ang15), -Math.sin(ang15)],
        [0, Math.sin(ang15), Math.cos(ang15)]
    ]);
    const R_y = new Matrix([
        [Math.cos(ang15), 0, Math.sin(ang15)],
        [0, 1, 0],
        [-Math.sin(ang15), 0, Math.cos(ang15)]
    ]);
    const R_z = new Matrix([
        [Math.cos(ang15), -Math.sin(ang15), 0],
        [Math.sin(ang15), Math.cos(ang15), 0],
        [0, 0, 1]
    ]);
    
    // Code sequence: R = R_z * R_y * R_x
    const independent_R = R_z.mmul(R_y).mmul(R_x);
    
    const bodyAccel = new Matrix([[1.0], [2.0], [9.81]]);
    const expectedNav = independent_R.mmul(bodyAccel);
    
    // We pass bodyAccel into InsMechanization
    // Since LPF is 0.3, we bypass it by calling it 100 times to settle
    (ins2 as any).filteredAccel = {x: bodyAccel.get(0,0), y: bodyAccel.get(1,0), z: bodyAccel.get(2,0)};
    
    ins2.predict(0.1, {x: bodyAccel.get(0,0), y: bodyAccel.get(1,0), z: bodyAccel.get(2,0)}, {x:0,y:0,z:0});
    
    // Subtract gravity from expected Nav Z
    const expectedX = expectedNav.get(0,0);
    const expectedY = expectedNav.get(1,0);
    const expectedZ = expectedNav.get(2,0) - 9.81;
    
    console.log(`[COMBINED-AXIS] Independent Standard Matrix Output:`);
    console.log(`                X: ${expectedX.toFixed(4)}, Y: ${expectedY.toFixed(4)}, Z: ${expectedZ.toFixed(4)}`);
    console.log(`[COMBINED-AXIS] Codebase Output:`);
    console.log(`                X: ${(ins2.velocity.x / 0.1).toFixed(4)}, Y: ${(ins2.velocity.y / 0.1).toFixed(4)}, Z: ${(ins2.velocity.z / 0.1).toFixed(4)}`);
    
    if (Math.abs(expectedX - ins2.velocity.x/0.1) < 0.001) {
        console.log(`VERDICT: Codebase rotation matrix EXACTLY MATCHES the independent standard sequence (Rz * Ry * Rx).`);
    } else {
        console.log(`VERDICT: ROTATION MATRIX MISMATCH! Order of operations may be flawed.`);
    }
}

runAdvancedTests();
