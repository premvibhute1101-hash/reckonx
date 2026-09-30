import { InsMechanization } from './src/core/InsMechanization.js';
import { Matrix } from 'ml-matrix';
import { EkfCore } from './src/core/EkfCore.js';

function testRotation() {
    console.log("=== EMPIRICAL ROTATION MATRIX CHECK ===");
    const ins = new InsMechanization();
    
    // Simulate phone pitched UP by 30 degrees.
    // If phone is pitched UP, and it's a standard Android (Y forward, X right, Z up),
    // and we assume ENU (X East, Y North, Z Up).
    // Let's say Body X = East, Body Y = North, Body Z = Up.
    // Pitched UP by 30 degrees (nose up).
    // Rotation is around Body X axis (which is East).
    // In Euler angles, rotation around X is 'roll' in this codebase.
    
    // Let's just set the attitude to 30 deg pitch (rotation around X)
    const angle30 = 30 * Math.PI / 180;
    
    // Wait, let's see what initializeAttitude does with accel.
    // If pitched UP 30 deg, gravity pulls DOWN.
    // So the normal force (accelerometer reading) is UP.
    // When pitched up, the Z-axis of phone points somewhat backward.
    // The Y-axis of phone points somewhat upward.
    // Accelerometer measures the UP force.
    // So accel = [0, 9.81 * sin(30), 9.81 * cos(30)]
    const accel = {
        x: 0,
        y: 9.81 * Math.sin(angle30),
        z: 9.81 * Math.cos(angle30)
    };
    
    ins.initializeAttitude(accel);
    console.log(`Initialized Attitude (from accel): roll=${(ins.attitude.roll*180/Math.PI).toFixed(1)}°, pitch=${(ins.attitude.pitch*180/Math.PI).toFixed(1)}°`);
    
    // Now simulate a forward acceleration (in phone Y axis)
    // E.g. accelerating forward at 2 m/s^2.
    // Plus gravity.
    const accelForward = {
        x: 0,
        y: 2.0 + 9.81 * Math.sin(angle30),
        z: 0.0 + 9.81 * Math.cos(angle30)
    };
    
    ins.predict(0.1, accelForward, {x:0, y:0, z:0});
    
    console.log(`After 0.1s predict:`);
    console.log(`Velocity: vx=${ins.velocity.x.toFixed(3)}, vy=${ins.velocity.y.toFixed(3)}, vz=${ins.velocity.z.toFixed(3)}`);
    // Expected: If phone is pitched up 30 degrees, and accelerates "forward" in its Y axis,
    // the true acceleration in Nav frame should be:
    // North (Y) = 2.0 * cos(30)
    // Up (Z) = 2.0 * sin(30)
    const expectedVy = (2.0 * Math.cos(angle30)) * 0.1;
    const expectedVz = (2.0 * Math.sin(angle30)) * 0.1;
    console.log(`Expected Velocity: vy=${expectedVy.toFixed(3)}, vz=${expectedVz.toFixed(3)}`);
}

function testFMatrix() {
    console.log("\n=== F-MATRIX COUPLING CHECK ===");
    // F.set(3, 7, g * dt);  // vx from pitch
    // F.set(4, 6, -g * dt); // vy from roll
    
    // In our codebase:
    // 'roll' is rotation around X (Pitch for a car).
    // 'pitch' is rotation around Y (Roll for a car).
    // Wait, if 'roll' is rotation around X, a positive 'roll' error means
    // the system thinks the nose is UP more than it is.
    // If the system thinks nose is UP, it thinks gravity is pulling in -Y direction (backward).
    // So it subtracts gravity from -Y, meaning it adds to Y.
    // So positive 'roll' error -> positive 'vy' error.
    // BUT EkfCore has: F.set(4, 6, -g * dt) which is NEGATIVE!
    // Let's verify by just logging the F matrix values manually based on the derivation.
    console.log(`If 'roll' is rotation around X, positive 'roll' means +Y acceleration from gravity? Let's trace InsMechanization.`);
}

function testBiasFeedback() {
    console.log("\n=== BIAS FEEDBACK LOOP CHECK ===");
    const ekf = new EkfCore();
    // Set a large fake bias to see if it's applied
    // Actually, I can't set this.x easily, so I'll just look at the code.
}

testRotation();
testFMatrix();
testBiasFeedback();
